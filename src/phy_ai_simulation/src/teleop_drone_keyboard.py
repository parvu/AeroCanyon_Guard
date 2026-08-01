#!/usr/bin/env python3
"""
Keyboard Teleop for pinn_drone
================================
Reads raw keypresses (no Enter needed) and publishes geometry_msgs/Twist
to /pinn_drone/cmd_vel, which ros_gz_bridge forwards into Gazebo Harmonic.

Keybindings
-----------
  w / s   ->  forward / backward   (linear.x)
  a / d   ->  strafe left / right  (linear.y)
  i / k   ->  ascend  / descend    (linear.z)
  j / l   ->  yaw left / right     (angular.z)
  Space   ->  stop (all zeros)
  + / -   ->  increase / decrease step size
  q       ->  quit
"""

import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
+------------------------------------------+
|      pinn_drone Keyboard Teleop          |
+------------------------------------------+
|  w / s  |  Forward  / Backward           |
|  a / d  |  Strafe Left / Right           |
|  i / k  |  Ascend   / Descend            |
|  j / l  |  Yaw Left / Right              |
|  Space  |  STOP (hover)                  |
|  + / -  |  Increase / Decrease speed     |
|  q      |  Quit                          |
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


class TeleopDroneKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_drone_keyboard')
        self.pub = self.create_publisher(Twist, '/pinn_drone/cmd_vel', 10)
        self.speed = 1.0      # linear  m/s
        self.yaw_rate = 0.5   # angular rad/s
        self.get_logger().info('Keyboard teleop started. Focus this terminal and use WASD / IK / JL.')

    def run(self):
        print(BANNER.format(speed=self.speed, yaw=self.yaw_rate))
        try:
            while rclpy.ok():
                key = get_key()
                msg = Twist()

                if key == 'w':
                    msg.linear.x = self.speed
                elif key == 's':
                    msg.linear.x = -self.speed
                elif key == 'a':
                    msg.linear.y = self.speed
                elif key == 'd':
                    msg.linear.y = -self.speed
                elif key == 'i':
                    msg.linear.z = self.speed
                elif key == 'k':
                    msg.linear.z = -self.speed
                elif key == 'j':
                    msg.angular.z = self.yaw_rate
                elif key == 'l':
                    msg.angular.z = -self.yaw_rate
                elif key == ' ':
                    pass  # all zeros -> hover stop
                elif key == '+':
                    self.speed = min(self.speed + 0.1, 5.0)
                    self.yaw_rate = min(self.yaw_rate + 0.05, 2.0)
                    print(f'\r  Speed: {self.speed:.2f} m/s  Yaw: {self.yaw_rate:.2f} rad/s    ', end='', flush=True)
                    continue
                elif key == '-':
                    self.speed = max(self.speed - 0.1, 0.1)
                    self.yaw_rate = max(self.yaw_rate - 0.05, 0.05)
                    print(f'\r  Speed: {self.speed:.2f} m/s  Yaw: {self.yaw_rate:.2f} rad/s    ', end='', flush=True)
                    continue
                elif key == 'q' or key == '\x03':  # q or Ctrl+C
                    print('\nQuitting teleop.')
                    break
                else:
                    continue  # ignore unknown keys without publishing

                self.pub.publish(msg)

                direction = {
                    'w': 'Forward', 's': 'Backward',
                    'a': 'Strafe L', 'd': 'Strafe R',
                    'i': 'Ascend', 'k': 'Descend',
                    'j': 'Yaw L', 'l': 'Yaw R',
                    ' ': 'STOP',
                }.get(key, '')
                print(f'\r  [{direction}]  lx={msg.linear.x:.1f}  ly={msg.linear.y:.1f}  lz={msg.linear.z:.1f}  az={msg.angular.z:.1f}    ',
                      end='', flush=True)

        except Exception as e:
            self.get_logger().error(f'Teleop error: {e}')
        finally:
            # Send a stop command on exit
            self.pub.publish(Twist())
            print()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopDroneKeyboard()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
