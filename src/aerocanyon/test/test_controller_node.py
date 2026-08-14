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
import rclpy

from aerocanyon.controller_node import SETPOINTS_BEFORE_OFFBOARD, ControllerNode
from px4_msgs.msg import VehicleCommand


def _run_ticks(mode, n):
    """Run the real tick loop n times, recording every VehicleCommand the
    node would have sent. Publishing still goes through rclpy (so any
    serialization crash from earlier is still exercised), we just also
    intercept _send_command to observe intent without needing a live
    PX4/DDS peer."""
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = mode  # override the 'baseline' default without a launch param
        sent = []
        real_send = node._send_command

        def spy(command, param1=0.0, param2=0.0):
            sent.append((command, param1, param2))
            real_send(command, param1, param2)

        node._send_command = spy
        for _ in range(n):
            node._tick()
        node.destroy_node()
        return sent
    finally:
        rclpy.shutdown()


def test_tick_loop_survives_past_first_tick():
    # This alone would have caught the numpy.float32/Vector3 crash: the
    # old code aborted the whole process on tick 0, so even reaching here
    # is the regression check.
    sent = _run_ticks('baseline', SETPOINTS_BEFORE_OFFBOARD + 10)
    assert len(sent) >= 1


def test_requests_offboard_mode_and_arm_after_setpoint_stream():
    sent = _run_ticks('baseline', SETPOINTS_BEFORE_OFFBOARD + 1)

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
    sent = _run_ticks('baseline', SETPOINTS_BEFORE_OFFBOARD)
    assert sent == []


def test_treatment_mode_also_survives_the_tick_loop():
    # Treatment mode additionally publishes CBF diagnostics; make sure that
    # path (float(np.clip(...)) etc.) doesn't have the same bug.
    sent = _run_ticks('treatment', SETPOINTS_BEFORE_OFFBOARD + 10)
    assert len(sent) >= 1


if __name__ == '__main__':
    test_tick_loop_survives_past_first_tick()
    test_requests_offboard_mode_and_arm_after_setpoint_stream()
    test_does_not_request_offboard_before_the_setpoint_stream_is_established()
    test_treatment_mode_also_survives_the_tick_loop()
    print('ok')
