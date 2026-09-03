"""Regression coverage for controller_node's AUTO-mission control path."""
import json

import numpy as np
import pytest
import rclpy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import Waypoint as MavWaypoint
from mavros_msgs.msg import WaypointList

from aerocanyon import canyon_geometry as cg
from aerocanyon import frames
from aerocanyon.controller_node import (CRUISE_ALT_M, CRUISE_WP_SEQ,
                                        ENGAGE_RETRY_TICKS, HOME_LAT, HOME_LON,
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


def _write_mission_file(tmp_path, items):
    p = tmp_path / 'mission.json'
    p.write_text(json.dumps(items))
    return str(p)


def test_build_mission_for_map_zone_loads_and_replays_the_mission_file(tmp_path):
    items = [
        {'command': 84, 'frame': 3, 'x_lat': 44.4345, 'y_long': 26.0480,
         'z_alt': 25.0, 'autocontinue': True},
        {'command': 16, 'frame': 3, 'x_lat': 44.4348, 'y_long': 26.0490,
         'z_alt': 30.0, 'autocontinue': True},
        {'command': 85, 'frame': 3, 'x_lat': 44.4350, 'y_long': 26.0495,
         'z_alt': 0.0, 'autocontinue': True},
    ]
    mission_file = _write_mission_file(tmp_path, items)

    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.world = 'map_zone'
        node.mission_file = mission_file
        mission = node._build_mission()
        node.destroy_node()
    finally:
        rclpy.shutdown()

    assert len(mission) == 4, (
        'home placeholder (seq 0, always overwritten by ArduPilot -- see '
        'the urban_canyon mission\'s own docstring) + the 3 real items')
    assert mission[1].command == 84 and mission[1].is_current
    assert mission[2].command == 16 and not mission[2].is_current
    assert mission[3].command == 85
    assert mission[3].x_lat == pytest.approx(44.4350)
    assert mission[2].z_alt == pytest.approx(30.0), (
        'map_zone replays each item\'s own captured altitude, not the '
        'fixed CRUISE_ALT_M urban_canyon uses')


def test_build_mission_for_urban_canyon_is_unaffected():
    """world defaults to urban_canyon -- _build_mission's existing
    4-item fixed mission must be byte-for-byte the same as before this
    task's changes."""
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        mission = node._build_mission()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert len(mission) == 4
    assert mission[1].command == 84  # NAV_VTOL_TAKEOFF
    assert mission[3].command == 85  # NAV_VTOL_LAND


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


def _waypoint_list(current_seq, items):
    """items: list of (lat, lon, alt) -- builds a WaypointList the way
    MAVROS would report it, home placeholder at seq 0 included (that's
    how the real topic looks -- see WaypointList.msg)."""
    msg = WaypointList()
    msg.current_seq = current_seq
    msg.waypoints = []
    for lat, lon, alt in items:
        w = MavWaypoint()
        w.x_lat = lat
        w.y_long = lon
        w.z_alt = alt
        msg.waypoints.append(w)
    return msg


def test_active_target_defaults_to_the_fixed_urban_canyon_land_point():
    """Before any WaypointList has arrived, _active_target must match
    what _treatment_tick has always corrected -- CRUISE_WP_SEQ's own
    land-trigger point at CRUISE_ALT_M -- so every existing treatment
    test (which never publishes a WaypointList) keeps passing."""
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        target_ned, alt = node._active_target()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    entry_ned = frames.enu_to_ned(cg.CANYON_ENTRY)
    assert target_ned[0] == pytest.approx(entry_ned[0])
    assert target_ned[1] == pytest.approx(LAND_TRIGGER_LOCAL_M)
    assert alt == pytest.approx(CRUISE_ALT_M)


def test_active_target_tracks_the_fcu_reported_current_waypoint():
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node._on_mission_waypoints(_waypoint_list(
            current_seq=2,
            items=[(44.4344, 26.0478, 25.0),   # seq 0: home placeholder
                   (44.4345, 26.0480, 25.0),   # seq 1: takeoff
                   (44.4350, 26.0490, 42.0)]))  # seq 2: active cruise wp
        target_ned, alt = node._active_target()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    expected_north, expected_east = frames.latlon_to_ned(
        44.4350, 26.0490, HOME_LAT, HOME_LON)
    assert target_ned[0] == pytest.approx(expected_north)
    assert target_ned[1] == pytest.approx(expected_east)
    assert alt == pytest.approx(42.0)


def test_treatment_corrects_the_fcu_reported_current_seq_not_a_hardcoded_one():
    """Generalization for map_zone's N-waypoint missions: the correction
    push/restart must target whichever seq MAVROS reports as current,
    not the urban_canyon-only CRUISE_WP_SEQ constant."""
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = 'treatment'
        node.mavros_armed = True
        node._mission_confirmed = True
        node.wind_est = np.array([2.0, 0.0, 0.0])
        node._on_mission_waypoints(_waypoint_list(
            current_seq=5,
            items=[(44.4344, 26.0478, 25.0)] * 6))
        pushes = []
        restarts = []
        node.mission_client.call_async = _confirmed_push(pushes)
        node.set_current_client.call_async = lambda req: restarts.append(req)

        for _ in range(60):
            node._tick()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert len(pushes) == 1
    assert pushes[0].start_index == 5
    assert len(restarts) == 1
    assert restarts[0].wp_seq == 5


def test_treatment_restarts_cruise_waypoint_once_correction_push_confirms():
    """The correction push alone doesn't move the vehicle -- ArduPilot
    caches the active nav command's target in RAM and only re-reads
    mission storage when that command is restarted (AP_Mission::
    replace_cmd's own doc comment). _treatment_tick must follow every
    confirmed push with a WaypointSetCurrent on the same index to force
    that restart -- this is the fix for the bug a 49-point wind sweep
    caught live (treatment and baseline flew identically). Precondition:
    _mission_confirmed already True -- see the next test for what
    happens before that."""
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = 'treatment'
        node.mavros_armed = True
        node._mission_confirmed = True
        node.wind_est = np.array([2.0, 0.0, 0.0])  # non-zero so a correction accumulates
        pushes = []
        restarts = []
        node.mission_client.call_async = _confirmed_push(pushes)
        node.set_current_client.call_async = lambda req: restarts.append(req)

        for _ in range(60):  # > CORRECTION_UPDATE_HZ's 50-tick period, plus a tick to see the confirm
            node._tick()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert len(pushes) == 1, 'must have pushed exactly one correction by tick 60'
    assert len(restarts) == 1, 'must restart the cruise nav command once the push confirms'
    assert restarts[0].wp_seq == CRUISE_WP_SEQ


def test_treatment_does_not_push_corrections_before_initial_mission_confirms():
    """Live-caught bug: _treatment_tick runs from tick 0 regardless of
    mission state. Its first correction push landed on tick 50, before
    the initial 4-item mission upload even starts (tick
    SETPOINTS_BEFORE_OFFBOARD=100), and its second landed on the SAME
    tick as that upload -- two concurrent WaypointPush transfers on
    ArduPilot's one stateful mission-write handshake, which broke both
    ("Mission upload timeout" on the FCU, no real mission ever
    delivered). Correction pushes must wait for _mission_confirmed."""
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = 'treatment'
        node.mavros_armed = True
        node.wind_est = np.array([2.0, 0.0, 0.0])
        pushes = []
        node.mission_client.call_async = _confirmed_push(pushes)
        node.set_current_client.call_async = lambda req: None

        for _ in range(SETPOINTS_BEFORE_OFFBOARD + 1):  # past both correction-push ticks (50, 100)
            node._tick()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert len(pushes) == 1, (
        'the one push by now must be the initial mission upload, not a '
        'colliding correction push -- _mission_confirmed is only set '
        'once that upload\'s own future resolves, one tick later')


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
