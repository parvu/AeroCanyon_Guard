"""Regression coverage for controller_node's AUTO-mission control path."""
import numpy as np
import rclpy

from aerocanyon.controller_node import (ENGAGE_RETRY_TICKS, MODE_AUTO,
                                        SETPOINTS_BEFORE_OFFBOARD,
                                        ControllerNode)


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
        node.mission_client.call_async = lambda req: mission_pushes.append(req)

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
    assert len(pushes[0].waypoints) == 3
    assert arm_calls == [] and mode_calls == [], (
        'must not request AUTO/arm on the same tick as the mission upload')


def test_requests_auto_and_arm_after_the_mission_upload():
    arm_calls, mode_calls, _ = _run_ticks(
        'baseline', SETPOINTS_BEFORE_OFFBOARD + ENGAGE_RETRY_TICKS + 1)
    assert mode_calls == [MODE_AUTO]
    assert arm_calls == [True]


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
        node.mission_client.call_async = lambda req: None
        import aerocanyon.controller_node as cn
        cn.arm = lambda client, value: None
        cn.set_mode = lambda client, mode: None
        for _ in range(SETPOINTS_BEFORE_OFFBOARD + 5):
            node._tick()
        assert len(diags) >= 1
        node.destroy_node()
    finally:
        rclpy.shutdown()


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
