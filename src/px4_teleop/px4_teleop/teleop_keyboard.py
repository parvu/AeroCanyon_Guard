#!/usr/bin/env python3
"""
Keyboard Teleop for PX4 SITL (offboard velocity control)
==========================================================
Talks to PX4 over the uXRCE-DDS bridge: streams OffboardControlMode +
TrajectorySetpoint at 10 Hz (PX4 requires >2 Hz or it drops out of
offboard mode), arms, and switches to offboard mode automatically.

Velocity is commanded in the local NED frame (North-East-Down), which is
what PX4's TrajectorySetpoint.velocity expects.

Keybindings
-----------
  w / s   ->  north / south       (velocity.x)
  a / d   ->  west  / east        (velocity.y)
  i / k   ->  up    / down        (velocity.z, NED so up is negative)
  j / l   ->  yaw left / right    (yawspeed)
  Space   ->  stop (hover)
  + / -   ->  increase / decrease step size
  q       ->  quit (disarms on exit)
"""

import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

# PX4's custom main mode for offboard control (not exposed as a px4_msgs
# constant -- this is the standard literal used by every PX4 ROS2 example).
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0

BANNER = """
+------------------------------------------+
|      PX4 SITL Keyboard Teleop            |
+------------------------------------------+
|  w / s  |  North / South                 |
|  a / d  |  West  / East                  |
|  i / k  |  Up     / Down                 |
|  j / l  |  Yaw Left / Right              |
|  Space  |  STOP (hover)                  |
|  + / -  |  Increase / Decrease speed     |
|  q      |  Quit (disarms)                |
+------------------------------------------+
Speed: {speed:.2f} m/s  |  Yaw rate: {yaw:.2f} rad/s
"""


def get_key(timeout=0.05):
    """Return a single key press without blocking longer than *timeout* seconds."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class PX4TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('px4_teleop_keyboard')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        # PX4's message-versioning scheme suffixes the topic with _vN (VehicleStatus
        # is at MESSAGE_VERSION=4 on this PX4 revision) while the msg class name
        # itself stays unversioned -- check `ros2 topic list` if this drifts on
        # a different PX4 checkout.
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self._status_cb, qos)

        self.speed = 2.0
        self.yaw_rate = 0.5
        self.vx = self.vy = self.vz = self.yawspeed = 0.0

        self.armed = False
        self.offboard_active = False

        # PX4 must see a stream of setpoints for ~1s before it will accept
        # the offboard mode switch; count heartbeats and arm once past that.
        self._heartbeats = 0
        self.timer = self.create_timer(0.1, self._timer_cb)
        self.get_logger().info('PX4 keyboard teleop started. Focus this terminal and use WASD / IK / JL.')

    def _now_usec(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _status_cb(self, msg):
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.offboard_active = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_usec()
        self.offboard_pub.publish(msg)

    def _publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.velocity = [self.vx, self.vy, self.vz]
        msg.position = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.yaw = float('nan')
        msg.yawspeed = self.yawspeed
        msg.timestamp = self._now_usec()
        self.setpoint_pub.publish(msg)

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_usec()
        self.command_pub.publish(msg)

    def _arm(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def _disarm(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def _engage_offboard_mode(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD
        )

    def _timer_cb(self):
        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint()
        self._heartbeats += 1
        # PX4 needs a ~1s stream of setpoints before it will accept the offboard
        # switch, and arming can be transiently rejected (e.g. preflight checks
        # still settling) -- keep retrying at 1Hz until both are confirmed via
        # VehicleStatus rather than firing once and giving up silently.
        if self._heartbeats >= 10 and self._heartbeats % 10 == 0:
            if not self.offboard_active:
                self._engage_offboard_mode()
            if not self.armed:
                self._arm()

    def run(self):
        print(BANNER.format(speed=self.speed, yaw=self.yaw_rate))
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                key = get_key()

                if key == 'w':
                    self.vx, self.vy, self.vz = self.speed, 0.0, self.vz
                elif key == 's':
                    self.vx, self.vy = -self.speed, 0.0
                elif key == 'a':
                    self.vx, self.vy = 0.0, -self.speed
                elif key == 'd':
                    self.vx, self.vy = 0.0, self.speed
                elif key == 'i':
                    self.vz = -self.speed
                elif key == 'k':
                    self.vz = self.speed
                elif key == 'j':
                    self.yawspeed = -self.yaw_rate
                elif key == 'l':
                    self.yawspeed = self.yaw_rate
                elif key == ' ':
                    self.vx = self.vy = self.vz = self.yawspeed = 0.0
                elif key == '+':
                    self.speed = min(self.speed + 0.2, 8.0)
                    self.yaw_rate = min(self.yaw_rate + 0.05, 2.0)
                    print(f'\r  Speed: {self.speed:.2f} m/s  Yaw: {self.yaw_rate:.2f} rad/s    ', end='', flush=True)
                    continue
                elif key == '-':
                    self.speed = max(self.speed - 0.2, 0.2)
                    self.yaw_rate = max(self.yaw_rate - 0.05, 0.05)
                    print(f'\r  Speed: {self.speed:.2f} m/s  Yaw: {self.yaw_rate:.2f} rad/s    ', end='', flush=True)
                    continue
                elif key == 'q' or key == '\x03':
                    print('\nQuitting teleop, disarming.')
                    break
                else:
                    status = 'ARMED' if self.armed else 'disarmed'
                    mode = 'OFFBOARD' if self.offboard_active else 'not-offboard'
                    print(f'\r  [{status}/{mode}]  vx={self.vx:.1f} vy={self.vy:.1f} vz={self.vz:.1f} yawspeed={self.yawspeed:.1f}    ',
                          end='', flush=True)
                    continue

                status = 'ARMED' if self.armed else 'disarmed'
                mode = 'OFFBOARD' if self.offboard_active else 'not-offboard'
                print(f'\r  [{status}/{mode}]  vx={self.vx:.1f} vy={self.vy:.1f} vz={self.vz:.1f} yawspeed={self.yawspeed:.1f}    ',
                      end='', flush=True)

        except Exception as e:
            self.get_logger().error(f'Teleop error: {e}')
        finally:
            self.vx = self.vy = self.vz = self.yawspeed = 0.0
            self._publish_trajectory_setpoint()
            self._disarm()
            print()


def main(args=None):
    rclpy.init(args=args)
    node = PX4TeleopKeyboard()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
