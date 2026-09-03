"""One-shot diagnostic mission: arm, VTOL_TAKEOFF straight up to
ALT_M at the vehicle's CURRENT position, fly to a NAV_WAYPOINT at the
same spot (still ALT_M), then VTOL_LAND there. No canyon, no wind, no
CBF/PINN -- isolates the AUTO-mode Q-mode navigation path itself (see
controller_node.py's module docstring and
docs/superpowers/plans/ardupilot_auto_mission_qmode_tilt_bug memory)
from everything else in a real trial.

Run against an already-flying manual demo (Gazebo + SITL + MAVROS up,
see README "Fly the tricopter manually"), from the current position --
does NOT reset/respawn the vehicle. The vehicle should be armed and
landed (or at rest) when this starts; it takes over MAVROS's mission
control from there.

    python3 -m aerocanyon.vertical_test_mission [--alt 20]
"""
import argparse
import sys
import time

import rclpy
from mavros_msgs.msg import State, Waypoint
from mavros_msgs.srv import CommandBool, CommandLong, WaypointPush
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from .rc_pwm import MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

MODE_AUTO = 10  # ArduPlane custom mode number, same table controller_node.py uses


class VerticalTestMission(Node):

    def __init__(self, alt_m):
        super().__init__('vertical_test_mission')
        self.alt_m = alt_m
        self.fix = None
        self.armed = False
        self.mode = None
        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                  self._on_fix, qos_profile_sensor_data)
        self.create_subscription(State, '/mavros/state', self._on_state, 10)
        self.mission_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')

    def _on_fix(self, msg):
        self.fix = msg

    def _on_state(self, msg):
        self.armed = msg.armed
        self.mode = msg.mode

    def _spin_until(self, predicate, timeout_s, what):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            if predicate():
                return True
        self.get_logger().error(f'timed out waiting for {what}')
        return False

    def _build_mission(self):
        """Land point = current lat/lon (wherever the vehicle already
        is). The "point straight above" is the SAME lat/lon at ALT_M --
        only the altitude field differs from the land item, exactly
        like controller_node._build_mission's own entry/land pairing."""
        lat, lon = self.fix.latitude, self.fix.longitude

        def wp(command, is_current=False):
            w = Waypoint()
            w.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            w.command = command
            w.is_current = is_current
            w.autocontinue = True
            w.x_lat = lat
            w.y_long = lon
            w.z_alt = self.alt_m
            return w

        return [
            wp(16),                    # seq 0: home placeholder, overwritten by ArduPilot
            wp(84, is_current=True),   # seq 1: MAV_CMD_NAV_VTOL_TAKEOFF -- arm+climb to ALT_M here
            wp(16),                    # seq 2: MAV_CMD_NAV_WAYPOINT -- straight above the land point
            wp(85),                    # seq 3: MAV_CMD_NAV_VTOL_LAND
        ]

    def run(self):
        if not self._spin_until(lambda: self.fix is not None, 10, 'a GPS fix'):
            return False
        self.get_logger().info(
            f'current position {self.fix.latitude:.7f},{self.fix.longitude:.7f} '
            f'-> mission: takeoff/waypoint/land all here at {self.alt_m}m')

        self.mission_client.wait_for_service(timeout_sec=10)
        req = WaypointPush.Request()
        req.waypoints = self._build_mission()
        future = self.mission_client.call_async(req)
        if not self._spin_until(future.done, 10, 'mission push response'):
            return False
        if not future.result().success:
            self.get_logger().error('mission push rejected by FCU')
            return False
        self.get_logger().info(f'mission uploaded ({future.result().wp_transfered} items)')

        self.arm_client.wait_for_service(timeout_sec=10)
        arm_req = CommandBool.Request(value=True)
        self.arm_client.call_async(arm_req)
        time.sleep(1.0)

        self.cmd_client.wait_for_service(timeout_sec=10)
        mode_req = CommandLong.Request(
            command=MAV_CMD_DO_SET_MODE,
            param1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            param2=float(MODE_AUTO))
        self.cmd_client.call_async(mode_req)

        if not self._spin_until(lambda: self.armed, 10, 'arm confirmation'):
            return False
        self.get_logger().info('armed, AUTO mission engaged -- watch /mavros/state for landing/disarm')
        return True


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--alt', type=float, default=20.0,
                         help='takeoff/waypoint/land altitude, metres relative to home')
    ns = parser.parse_args(args=sys.argv[1:] if args is None else args)

    rclpy.init()
    node = VerticalTestMission(ns.alt)
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
