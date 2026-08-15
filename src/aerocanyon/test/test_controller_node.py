"""Regression coverage for the arm/offboard/takeoff sequence.

controller_node previously crashed with a C-level assertion on the very
first control tick (numpy.float32 handed to a geometry_msgs/Vector3
field), which silently killed the node before it ever reached the tick
that requests offboard mode and arms the vehicle. PX4 logged nothing --
no rejected command, no error -- because the publisher was already dead.
Nothing in the test suite exercised the tick loop at all, so this went
unnoticed. These tests drive the real tick loop (no mocked physics) and
assert the vehicle_command sequence actually gets sent.
"""
import numpy as np
import rclpy
from rclpy.duration import Duration

import aerocanyon.controller_node as controller_node
from aerocanyon.controller_node import (ENGAGE_RETRY_TICKS,
                                        SETPOINTS_BEFORE_OFFBOARD,
                                        ControllerNode)
from px4_msgs.msg import VehicleCommand, VtolVehicleStatus


def _run_ticks(mode, n):
    """Run the real tick loop n times, recording every VehicleCommand the
    node would have sent and every TrajectorySetpoint it published.
    Publishing still goes through rclpy (so any serialization crash from
    earlier is still exercised), we just also intercept the send/publish
    calls to observe intent without needing a live PX4/DDS peer."""
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = mode  # override the 'baseline' default without a launch param
        sent = []
        real_send = node._send_command

        def spy_send(command, param1=0.0, param2=0.0):
            sent.append((command, param1, param2))
            real_send(command, param1, param2)

        node._send_command = spy_send

        setpoints = []
        real_publish = node.sp_pub.publish

        def spy_publish(msg):
            setpoints.append(msg)
            real_publish(msg)

        node.sp_pub.publish = spy_publish

        for _ in range(n):
            node._tick()
        node.destroy_node()
        return sent, setpoints
    finally:
        rclpy.shutdown()


def test_tick_loop_survives_past_first_tick():
    # This alone would have caught the numpy.float32/Vector3 crash: the
    # old code aborted the whole process on tick 0, so even reaching here
    # is the regression check.
    sent, _ = _run_ticks('baseline', SETPOINTS_BEFORE_OFFBOARD + 10)
    assert len(sent) >= 1


def test_requests_offboard_mode_and_arm_after_setpoint_stream():
    sent, _ = _run_ticks('baseline', SETPOINTS_BEFORE_OFFBOARD + 1)

    mode_cmds = [s for s in sent if s[0] == VehicleCommand.VEHICLE_CMD_DO_SET_MODE]
    arm_cmds = [s for s in sent if s[0] == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM]

    assert len(mode_cmds) == 1, 'must request the mode switch exactly once'
    _, custom_main_mode, custom_sub_mode = mode_cmds[0]
    assert custom_main_mode == 1.0
    assert custom_sub_mode == 6.0, 'sub-mode 6 is PX4 offboard'

    assert len(arm_cmds) == 1, 'must request arming exactly once'
    _, arm_param, _ = arm_cmds[0]
    assert arm_param == 1.0, 'param1=1 arms; 0 would disarm'


def test_does_not_request_offboard_before_the_setpoint_stream_is_established():
    # PX4 rejects an offboard switch with no prior setpoint stream -- the
    # node must not ask before SETPOINTS_BEFORE_OFFBOARD ticks have primed it.
    sent, _ = _run_ticks('baseline', SETPOINTS_BEFORE_OFFBOARD)
    assert sent == []


def test_retries_arm_request_until_engaged():
    # A single arm/offboard request can be silently rejected if PX4 hasn't
    # finished its own preflight/EKF convergence yet at exactly the tick
    # SETPOINTS_BEFORE_OFFBOARD fires. Nothing in _run_ticks ever satisfies
    # ControllerNode.armed/offboard_engaged (no PX4 is present), so if the
    # node only asked once, this would see exactly one request no matter
    # how long it ran -- it must keep asking instead.
    sent, _ = _run_ticks('baseline', SETPOINTS_BEFORE_OFFBOARD + 2 * ENGAGE_RETRY_TICKS + 1)
    arm_cmds = [s for s in sent if s[0] == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM]
    assert len(arm_cmds) == 3, 'must retry roughly once per ENGAGE_RETRY_TICKS while unengaged'


def test_yaw_points_down_the_canyons_actual_travel_direction():
    # The canyon corridor runs along Gazebo ENU +x (east, see
    # canyon_geometry.BUILDINGS), not north. A hardcoded sp.yaw=0.0
    # (north) previously pointed the nose the wrong way while the mission
    # pulled the vehicle east -- a ~90 degree mismatch that showed up as
    # the vehicle visibly snapping to the correct heading after takeoff.
    _, setpoints = _run_ticks('baseline', SETPOINTS_BEFORE_OFFBOARD + 5)
    from aerocanyon.mission import Mission
    expected_yaw = float(np.arctan2(Mission().direction[1], Mission().direction[0]))
    assert setpoints, 'tick loop must publish at least one TrajectorySetpoint'
    for sp in setpoints:
        assert abs(sp.yaw - expected_yaw) < 1e-6, (
            f'sp.yaw={sp.yaw} does not match the canyon travel direction '
            f'({expected_yaw} rad); a hardcoded yaw=0.0 (north) would fail this')
        assert abs(sp.yaw) > 1e-6, 'yaw must not silently regress to hardcoded north (0.0)'


def test_treatment_mode_also_survives_the_tick_loop():
    # Treatment mode additionally publishes CBF diagnostics; make sure that
    # path (float(np.clip(...)) etc.) doesn't have the same bug.
    sent, _ = _run_ticks('treatment', SETPOINTS_BEFORE_OFFBOARD + 10)
    assert len(sent) >= 1


def _run_ticks_already_engaged(n, elapsed_at_start_s, vel=(0.0, 0.0, 0.0),
                               pos=(0.0, 0.0, 0.0)):
    """Like _run_ticks, but fakes the vehicle as already armed + offboard
    (as if _on_status had already received confirmation) with the mission
    clock already elapsed_at_start_s into the flight and fixed measured
    velocity/position, so tests can reach the post-hold cruise phase and
    control the VTOL transition's speed gate / RTL's position gate
    without waiting on real wall-clock time or real physics."""
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.armed = True
        node.offboard_engaged = True
        node.start_time = node.get_clock().now() - Duration(seconds=elapsed_at_start_s)
        node.vel = np.array(vel, dtype=float)
        node.pos = np.array(pos, dtype=float)
        sent = []
        real_send = node._send_command

        def spy_send(command, param1=0.0, param2=0.0):
            sent.append((command, param1, param2))
            real_send(command, param1, param2)

        node._send_command = spy_send
        for _ in range(n):
            node._tick()
        node.destroy_node()
        return sent
    finally:
        rclpy.shutdown()


def test_vtol_transition_is_disabled_by_default():
    # Live testing found that requesting DO_VTOL_TRANSITION -- even gated
    # on measured forward speed reaching cruise -- caused a violent pitch
    # oscillation that dropped the vehicle to within ~2 m of the ground
    # before partially recovering. This is a real PX4 attitude/airspeed
    # tuning problem, not something a trigger-condition tweak fixes, so
    # the feature is disabled until that tuning happens. This test is the
    # safety net: it must keep failing (loudly, in review) if someone
    # flips ENABLE_VTOL_TRANSITION back on without re-verifying stability.
    sent = _run_ticks_already_engaged(5, elapsed_at_start_s=20.0, vel=(12.0, 0.0, 0.0))
    transitions = [s for s in sent if s[0] == VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION]
    assert transitions == [], (
        'ENABLE_VTOL_TRANSITION must stay False until the pitch instability '
        'seen in live testing (climb to a stall, drop to ~2m altitude) is '
        'actually fixed by tuning PX4 VTOL transition/attitude parameters')


def test_requests_vtol_transition_to_fixed_wing_after_the_hold_phase(monkeypatch):
    monkeypatch.setattr(controller_node, 'ENABLE_VTOL_TRANSITION', True)
    # elapsed starts just before hold_s (3.0s default): the transition
    # must not fire during the vertical-climb/hold phase...
    sent = _run_ticks_already_engaged(5, elapsed_at_start_s=2.9, vel=(12.0, 0.0, 0.0))
    transitions = [s for s in sent if s[0] == VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION]
    assert transitions == [], 'must not transition to fixed-wing before the hold phase ends'


def test_vtol_transition_fires_once_cruise_speed_is_reached(monkeypatch):
    monkeypatch.setattr(controller_node, 'ENABLE_VTOL_TRANSITION', True)
    # Regression: transitioning at elapsed == hold_s alone (with no speed
    # check) stalled and crashed the vehicle every time, because that
    # instant is exactly when the target has JUST started moving off the
    # pinned hold point -- horizontal speed is still ~0 then, and a VTOL
    # has no lift in fixed-wing mode without forward airspeed.
    sent = _run_ticks_already_engaged(5, elapsed_at_start_s=3.1, vel=(12.0, 0.0, 0.0))
    transitions = [s for s in sent if s[0] == VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION]
    assert len(transitions) == 1, 'must request the VTOL transition exactly once'


def test_vtol_transition_does_not_fire_at_low_speed_even_after_the_hold_phase(monkeypatch):
    monkeypatch.setattr(controller_node, 'ENABLE_VTOL_TRANSITION', True)
    sent = _run_ticks_already_engaged(5, elapsed_at_start_s=3.1, vel=(0.5, 0.0, 0.0))
    transitions = [s for s in sent if s[0] == VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION]
    assert transitions == [], 'must wait for real forward speed, not just elapsed time'


def test_vtol_transition_fires_anyway_after_the_timeout_cap_even_at_zero_speed(monkeypatch):
    monkeypatch.setattr(controller_node, 'ENABLE_VTOL_TRANSITION', True)
    # A stuck-at-zero velocity reading (e.g. the same telemetry-topic bug
    # fixed elsewhere in this project) must not withhold the transition
    # forever -- there's a generous elapsed-time cap as a fallback.
    sent = _run_ticks_already_engaged(1, elapsed_at_start_s=20.0, vel=(0.0, 0.0, 0.0))
    transitions = [s for s in sent if s[0] == VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION]
    assert len(transitions) == 1, 'must transition anyway once the timeout cap is reached'
    _, param1, _ = transitions[0]
    assert param1 == float(VtolVehicleStatus.VEHICLE_VTOL_STATE_FW)


def _run_land_ticks(n, pos, mode='baseline'):
    """Like _run_ticks_already_engaged, but also exposes the live node
    (not just the sent commands) so land tests can inspect
    node.land_requested and the actually-published setpoints."""
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = mode
        node.armed = True
        node.offboard_engaged = True
        node.start_time = node.get_clock().now() - Duration(seconds=30.0)
        node.pos = np.array(pos, dtype=float)
        sent = []
        real_send = node._send_command
        node._send_command = lambda command, param1=0.0, param2=0.0: (
            sent.append((command, param1, param2)), real_send(command, param1, param2))[0]
        setpoints = []
        real_publish = node.sp_pub.publish
        node.sp_pub.publish = lambda msg: (setpoints.append(msg), real_publish(msg))[0]

        for _ in range(n):
            node._tick()
        result = (sent, setpoints, node.land_requested)
        node.destroy_node()
        return result
    finally:
        rclpy.shutdown()


def test_lands_in_place_once_position_clears_the_canyon_exit_by_the_margin():
    # Both modes land where they are once clear of the canyon -- see
    # LAND_CLEARANCE_M's comment for why flying anywhere first (native
    # RTL, or an earlier custom fly-home-and-land design) is no longer
    # needed now that each leg gets its own fresh Gazebo/PX4 process.
    from aerocanyon.mission import Mission
    distance = Mission().distance
    for mode in ('baseline', 'treatment'):
        sent, setpoints, land_requested = _run_land_ticks(
            2, pos=(0.0, distance + controller_node.LAND_CLEARANCE_M, 0.0), mode=mode)
        land_cmds = [s for s in sent if s[0] == VehicleCommand.VEHICLE_CMD_NAV_LAND]
        assert land_requested, f'{mode}: must request landing on clearing the canyon'
        assert len(land_cmds) == 1, f'{mode}: must request landing exactly once'
        assert setpoints == [], (
            f'{mode}: must stop publishing its own setpoint stream once handed off '
            'to AUTO_LAND -- continuing would fight PX4 for control authority')


def test_does_not_land_before_clearing_the_canyon_by_the_margin():
    from aerocanyon.mission import Mission
    distance = Mission().distance
    # 1m short of the required clearance margin.
    _, _, land_requested = _run_land_ticks(5, pos=(0.0, distance + 1.0, 0.0))
    assert not land_requested, 'must not land until actually 2m clear of the canyon exit'


if __name__ == '__main__':
    # Several tests above need pytest's monkeypatch fixture (to flip
    # ENABLE_VTOL_TRANSITION on for the duration of one test), so this
    # file needs pytest as its runner rather than a plain function-call
    # list -- `pytest <this file>` is the real entry point either way.
    import sys
    import pytest as _pytest
    sys.exit(_pytest.main([__file__, '-q']))
