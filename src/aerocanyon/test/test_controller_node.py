"""Regression coverage for controller_node's AUTO-mission control path."""
import numpy as np
import rclpy
from mavros_msgs.msg import OverrideRCIn

from aerocanyon import canyon_geometry as cg
from aerocanyon import frames
from aerocanyon.controller_node import (ENGAGE_RETRY_TICKS,
                                        LAND_TRIGGER_LOCAL_M, MODE_AUTO,
                                        QSTAB_CLIMB_ALT_M,
                                        SETPOINTS_BEFORE_OFFBOARD,
                                        ControllerNode)
from aerocanyon.rc_pwm import MODE_QSTABILIZE


class _FakeFuture:
    """A WaypointPush service future that's already done -- _tick() reads
    .done()/.result() the same tick it fires the request, matching the
    synchronous stubs the rest of this test file already uses for
    arm()/set_mode()."""

    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _FakeWaypointPushResult:
    def __init__(self, success=True, wp_transfered=4):
        self.success = success
        self.wp_transfered = wp_transfered


def _confirmed_push(mission_pushes=None):
    """A mission_client.call_async stub that reports the push succeeded
    immediately -- see _FakeFuture. Pass a list to also record requests."""
    def call_async(req):
        if mission_pushes is not None:
            mission_pushes.append(req)
        return _FakeFuture(_FakeWaypointPushResult())
    return call_async


def _run_ticks(mode, n, armed=False):
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = mode
        node.mavros_armed = armed
        arm_calls = []
        mode_calls = []
        mission_pushes = []
        import aerocanyon.controller_node as cn
        cn.arm = lambda client, value: arm_calls.append(value)
        cn.set_mode = lambda client, mode: mode_calls.append(mode)
        node.mission_client.call_async = _confirmed_push(mission_pushes)

        for _ in range(n):
            node._tick()
        node.destroy_node()
        return arm_calls, mode_calls, mission_pushes
    finally:
        rclpy.shutdown()


def test_uploads_a_mission_before_requesting_auto_and_arm():
    arm_calls, mode_calls, pushes = _run_ticks(
        'baseline', SETPOINTS_BEFORE_OFFBOARD + 1)
    assert len(pushes) == 1, 'must upload the mission exactly once'
    assert pushes[0].start_index == 0
    assert len(pushes[0].waypoints) == 4, (
        'seq 0 is a home placeholder ArduPilot always overwrites -- '
        'the real 3-item mission starts at seq 1')
    assert arm_calls == [] and mode_calls == [], (
        'must not request AUTO/arm on the same tick as the mission upload')


def test_requests_qstabilize_and_arm_once_mission_confirmed():
    arm_calls, mode_calls, _ = _run_ticks(
        'baseline', SETPOINTS_BEFORE_OFFBOARD + ENGAGE_RETRY_TICKS + 1)
    assert mode_calls == [MODE_QSTABILIZE], (
        'arms into QSTABILIZE first (climbs, then AUTO) -- gated on '
        '_mission_confirmed, not just the push having been sent, which is '
        'what actually fixed the mission-complete-instantly bug (see the '
        'module docstring above _tick)')
    assert arm_calls == [True]


def test_does_not_switch_to_auto_before_climbing_past_threshold():
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = 'baseline'
        node.mavros_armed = True
        mode_calls = []
        import aerocanyon.controller_node as cn
        cn.arm = lambda client, value: None
        cn.set_mode = lambda client, mode: mode_calls.append(mode)
        node.mission_client.call_async = _confirmed_push()

        for _ in range(5):
            node._tick()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert mode_calls == []


def test_switches_to_auto_once_climbed_past_threshold():
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = 'baseline'
        node.mavros_armed = True
        mode_calls = []
        import aerocanyon.controller_node as cn
        cn.arm = lambda client, value: None
        cn.set_mode = lambda client, mode: mode_calls.append(mode)
        node.mission_client.call_async = _confirmed_push()

        node._tick()  # enters 'climbing', captures the start altitude
        node.pos = np.array([0.0, 0.0, -QSTAB_CLIMB_ALT_M - 1.0])  # NED down: climbed past threshold
        node._tick()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert mode_calls == [MODE_AUTO]


def test_disarming_after_flying_marks_mission_complete_and_stops_re_arming():
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = 'baseline'
        node.mavros_armed = True
        arm_calls = []
        mode_calls = []
        import aerocanyon.controller_node as cn
        cn.arm = lambda client, value: arm_calls.append(value)
        cn.set_mode = lambda client, mode: mode_calls.append(mode)
        node.mission_client.call_async = _confirmed_push()

        node._tick()  # enters 'climbing'
        node.pos = np.array([0.0, 0.0, -QSTAB_CLIMB_ALT_M - 1.0])
        node._tick()  # switches to AUTO
        assert mode_calls == [MODE_AUTO]

        node.mavros_armed = False  # landed and disarmed
        for _ in range(3 * ENGAGE_RETRY_TICKS):
            node._tick()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert mode_calls == [MODE_AUTO], (
        'must not request QSTABILIZE/AUTO again after mission completion')
    assert arm_calls == [], 'must not re-arm after landing'


def test_retries_arm_request_until_engaged():
    arm_calls, _, _ = _run_ticks(
        'baseline', SETPOINTS_BEFORE_OFFBOARD + 3 * ENGAGE_RETRY_TICKS + 1)
    assert len(arm_calls) == 3


def test_stops_retrying_once_mavros_reports_armed():
    arm_calls, _, _ = _run_ticks(
        'baseline', SETPOINTS_BEFORE_OFFBOARD + 3 * ENGAGE_RETRY_TICKS + 1, armed=True)
    assert arm_calls == []


def test_treatment_mode_publishes_cbf_diagnostics_and_survives_the_tick_loop():
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = 'treatment'
        node.mavros_armed = True
        diags = []
        real_publish = node.cbf_pub.publish
        node.cbf_pub.publish = lambda msg: (diags.append(msg), real_publish(msg))[0]
        node.mission_client.call_async = _confirmed_push()
        import aerocanyon.controller_node as cn
        cn.arm = lambda client, value: None
        cn.set_mode = lambda client, mode: None
        for _ in range(SETPOINTS_BEFORE_OFFBOARD + 5):
            node._tick()
        assert len(diags) >= 1
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_yaw_to_target_publishes_yaw_only_override_when_in_auto():
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.fcu_mode = f'CMODE({MODE_AUTO})'
        entry_ned = frames.enu_to_ned(cg.CANYON_ENTRY)
        land_ned = np.array([entry_ned[0], LAND_TRIGGER_LOCAL_M, entry_ned[2]])
        node.pos = land_ned - np.array([0.0, 50.0, 0.0])  # target is due east
        node.quat = np.array([1.0, 0.0, 0.0, 0.0])  # identity -> yaw 0 (nose north)
        published = []
        node.rc_pub.publish = lambda msg: published.append(msg)
        node._yaw_to_target()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert len(published) == 1
    ch = published[0].channels
    assert ch[3] == 2000, 'target due east of a north-facing vehicle should command full right yaw'
    assert list(ch[0:3]) == [OverrideRCIn.CHAN_NOCHANGE] * 3, 'must not touch roll/pitch/throttle'


def test_yaw_to_target_does_nothing_outside_auto_mode():
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.fcu_mode = 'CMODE(18)'  # QHOVER, not AUTO
        published = []
        node.rc_pub.publish = lambda msg: published.append(msg)
        node._yaw_to_target()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert published == []


def test_accumulate_offset_zero_correction_stays_zero():
    off = ControllerNode._accumulate_offset(
        np.zeros(3), dt=1.0, current_offset=np.zeros(2), max_offset_m=3.0)
    assert np.allclose(off, [0.0, 0.0])


def test_accumulate_offset_grows_with_sustained_correction():
    off = np.zeros(2)
    for _ in range(5):
        off = ControllerNode._accumulate_offset(
            np.array([1.0, 0.0, 0.0]), dt=1.0, current_offset=off, max_offset_m=100.0)
    assert off[0] > 0.0


def test_accumulate_offset_clamps_to_max_magnitude():
    off = np.zeros(2)
    for _ in range(1000):
        off = ControllerNode._accumulate_offset(
            np.array([50.0, 0.0, 0.0]), dt=1.0, current_offset=off, max_offset_m=3.0)
    assert np.linalg.norm(off) <= 3.0 + 1e-6
