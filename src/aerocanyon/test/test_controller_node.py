"""Regression coverage for controller_node's MAVROS control loop."""
import numpy as np
import rclpy
from rclpy.duration import Duration

from aerocanyon.controller_node import (ENGAGE_RETRY_TICKS,
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
        import aerocanyon.controller_node as cn
        cn.arm = lambda client, value: arm_calls.append(value)
        cn.set_mode = lambda client, mode: mode_calls.append(mode)

        rc_msgs = []
        real_publish = node.rc_pub.publish
        node.rc_pub.publish = lambda msg: (rc_msgs.append(msg), real_publish(msg))[0]

        for _ in range(n):
            node._tick()
        node.destroy_node()
        return arm_calls, mode_calls, rc_msgs
    finally:
        rclpy.shutdown()


def test_tick_loop_survives_past_first_tick():
    _, _, rc_msgs = _run_ticks('baseline', SETPOINTS_BEFORE_OFFBOARD + 10)
    assert len(rc_msgs) >= 1


def test_requests_qhover_and_arm_after_setpoint_stream():
    arm_calls, mode_calls, _ = _run_ticks('baseline', SETPOINTS_BEFORE_OFFBOARD + 1)
    assert mode_calls == [18]  # rc_pwm.MODE_QHOVER
    assert arm_calls == [True]


def test_does_not_request_before_the_setpoint_stream_is_established():
    arm_calls, mode_calls, _ = _run_ticks('baseline', SETPOINTS_BEFORE_OFFBOARD)
    assert arm_calls == [] and mode_calls == []


def test_retries_arm_request_until_engaged():
    arm_calls, _, _ = _run_ticks(
        'baseline', SETPOINTS_BEFORE_OFFBOARD + 2 * ENGAGE_RETRY_TICKS + 1)
    assert len(arm_calls) == 3


def test_stops_retrying_once_mavros_reports_armed():
    arm_calls, _, _ = _run_ticks(
        'baseline', SETPOINTS_BEFORE_OFFBOARD + 2 * ENGAGE_RETRY_TICKS + 1, armed=True)
    assert arm_calls == []


def test_treatment_mode_also_survives_the_tick_loop():
    _, _, rc_msgs = _run_ticks('treatment', SETPOINTS_BEFORE_OFFBOARD + 10)
    assert len(rc_msgs) >= 1


def test_lean_from_accel_zero_is_level():
    roll, pitch = ControllerNode._lean_from_accel(
        np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    assert abs(roll) < 1e-9 and abs(pitch) < 1e-9


def test_lean_from_accel_forward_accel_pitches_forward():
    roll, pitch = ControllerNode._lean_from_accel(
        np.array([2.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    assert pitch > 0.0
    assert abs(roll) < 1e-9


def test_lean_from_accel_rightward_accel_rolls_right():
    roll, pitch = ControllerNode._lean_from_accel(
        np.array([0.0, 2.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    assert roll > 0.0
    assert abs(pitch) < 1e-9


def _run_land_ticks(n, pos, mode='baseline'):
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = mode
        node.mavros_armed = True
        node.start_time = node.get_clock().now() - Duration(seconds=30.0)
        node.pos = np.array(pos, dtype=float)
        import aerocanyon.controller_node as cn
        mode_calls = []
        cn.set_mode = lambda client, mode: mode_calls.append(mode)
        rc_msgs = []
        real_publish = node.rc_pub.publish
        node.rc_pub.publish = lambda msg: (rc_msgs.append(msg), real_publish(msg))[0]
        for _ in range(n):
            node._tick()
        result = (mode_calls, rc_msgs, node.land_requested)
        node.destroy_node()
        return result
    finally:
        rclpy.shutdown()


def test_lands_in_place_once_position_clears_the_last_tower_row():
    import aerocanyon.controller_node as controller_node
    for mode in ('baseline', 'treatment'):
        mode_calls, rc_msgs, land_requested = _run_land_ticks(
            2, pos=(0.0, controller_node.LAND_TRIGGER_LOCAL_M, 0.0), mode=mode)
        assert land_requested, f'{mode}: must request landing on clearing the tower row'
        assert mode_calls == [20], f'{mode}: must switch to QLAND exactly once'
        assert rc_msgs == [], (
            f'{mode}: must stop publishing RC overrides once handed off to QLAND')


def test_does_not_land_before_clearing_the_last_tower_row():
    import aerocanyon.controller_node as controller_node
    _, _, land_requested = _run_land_ticks(
        5, pos=(0.0, controller_node.LAND_TRIGGER_LOCAL_M - 1.0, 0.0))
    assert not land_requested
