#!/usr/bin/env python3
"""control_server.py

Ported from CaveX-Explorer-Pro/web_viewer/control_server.py, adapted for
the tricopter: that version relayed browser buttons into gz-transport
StringMsg topics a ROS2 bridge node picked up. This project's manual
control is PX4 offboard velocity setpoints, which are ROS2/px4_msgs
topics directly -- so this server is an rclpy node with an embedded HTTP
server, not a gz-transport client. It only targets the README's separate
"fly it by hand" path (external Gazebo/PX4, no run_trial.py): a real
trial's controller_node is the sole source of setpoints during a leg, and
this server is not meant to run alongside one.

Run from web_viewer/, with the ROS2 workspace + PX4 DDS bridge already up
(see README "Watching a trial fly, or flying manually"):
    python3 control_server.py [port]
"""
import sys
import threading
import time
import http.server

import numpy as np
import rclpy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleAttitude, VehicleCommand)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
CONTROL_HZ = 50
# Was 1.0; raised to put full-stick climb at the target 3 m/s -- matches
# MPC_Z_VEL_MAX_UP=3.0, now set explicitly in 4022_gz_tricopter so PX4
# doesn't cap the climb below what this asks for.
MAX_VELOCITY_MPS = 3.0
MAX_YAW_RATE_RAD_S = 0.5
# Mode 2 RC sticks are proportional and self-centering -- the browser
# streams the live stick position continuously (see index.html) rather
# than a click-and-decay nudge, so there's no HOLD_S here. This is instead
# a dead-man's-switch: if the browser stops sending (closed tab, dropped
# connection, JS error), stale sticks zero out rather than freezing the
# last commanded velocity forever, matching real RC failsafe behaviour.
STICK_TIMEOUT_S = 0.3
# How long rc_bridge.py's last post has to go quiet before the browser
# falls back to showing its own virtual sticks. Longer than
# STICK_TIMEOUT_S on purpose -- flip to virtual sticks only on a real
# disconnect, not a single dropped/delayed HTTP request.
RC_PRESENT_TIMEOUT_S = 1.0

COMMAND_COMMANDS = {'arm', 'disarm', 'land', 'transition_fw', 'transition_mc'}


def resolve_stick(stick, stick_time, now, timeout):
    """Per-axis dead-man's-switch: an axis whose last update is older than
    `timeout` reads as 0.0 regardless of its last commanded value. Pure
    function -- see test_control_server.py."""
    return {
        name: (0.0 if now - stick_time[name] > timeout else stick[name])
        for name in stick
    }


def stick_to_velocity(stick, quat, scale):
    """Mode 2 stick state + current attitude -> (NED velocity [vx,vy,vz],
    yawspeed). Pure function, no ROS/rclpy needed -- see test_control_server.py.
    """
    w, x, y, z = quat
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch_v = stick['pitch'] * scale * MAX_VELOCITY_MPS
    roll_v = stick['roll'] * scale * MAX_VELOCITY_MPS
    vx = np.cos(yaw) * pitch_v - np.sin(yaw) * roll_v
    vy = np.sin(yaw) * pitch_v + np.cos(yaw) * roll_v
    vz = -stick['throttle'] * scale * MAX_VELOCITY_MPS
    yawspeed = stick['yaw'] * scale * MAX_YAW_RATE_RAD_S
    return [vx, vy, vz], yawspeed


class WebControlNode(Node):

    def __init__(self):
        super().__init__('web_control_server')
        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self._on_attitude, qos_profile_sensor_data)

        self._lock = threading.Lock()
        # Mode 2: left stick = yaw (x) + throttle/climb (y), right stick =
        # roll/strafe (x) + pitch/forward (y). Each in [-1, 1].
        #
        # Real request: the physical Tactic transmitter only decodes
        # throttle/roll/yaw reliably (see rc_bridge.py's own docstring for
        # why pitch isn't there) -- it and the browser's virtual stick
        # (index.html, pitch only) both post to /api/stick, each owning a
        # disjoint set of axes. Axes are tracked with their OWN staleness
        # timestamp, not one shared one, so one source going quiet (tab
        # closed, dongle unplugged) zeroes only ITS axes rather than
        # fighting/resetting the other source's still-live ones.
        self._stick = {'yaw': 0.0, 'throttle': 0.0, 'roll': 0.0, 'pitch': 0.0}
        self._stick_time = {'yaw': 0.0, 'throttle': 0.0, 'roll': 0.0, 'pitch': 0.0}
        self._speed_scale = 1.0
        self._hw_last_seen = 0.0
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])

        self.create_timer(1.0 / CONTROL_HZ, self._tick)

    def _on_attitude(self, msg):
        self.quat = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])

    def set_stick(self, hw=False, **axes):
        now = time.monotonic()
        with self._lock:
            for name, value in axes.items():
                self._stick[name] = max(-1.0, min(1.0, value))
                self._stick_time[name] = now
            if hw:
                self._hw_last_seen = now

    def rc_present(self):
        with self._lock:
            return time.monotonic() - self._hw_last_seen <= RC_PRESENT_TIMEOUT_S

    def apply_command(self, cmd):
        if cmd == 'speed_up':
            with self._lock:
                self._speed_scale = min(3.0, self._speed_scale + 0.25)
            return
        if cmd == 'speed_down':
            with self._lock:
                self._speed_scale = max(0.25, self._speed_scale - 0.25)
            return
        if cmd == 'arm':
            self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        elif cmd == 'disarm':
            self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        elif cmd == 'land':
            self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        elif cmd == 'transition_fw':
            # MAV_VTOL_STATE_FW = 4. controller_node's own
            # ENABLE_VTOL_TRANSITION=False only gates the AUTONOMOUS trial's
            # own logic, not PX4 itself -- this command reaches PX4 directly.
            self._send_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 4.0)
        elif cmd == 'transition_mc':
            # MAV_VTOL_STATE_MC = 3.
            self._send_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 3.0)

    def _send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def _tick(self):
        now = time.monotonic()
        with self._lock:
            stick = resolve_stick(self._stick, self._stick_time, now, STICK_TIMEOUT_S)
            scale = self._speed_scale

        # Horizontal (pitch/roll) is body-relative -- rotated by the
        # vehicle's CURRENT yaw every tick, not just when the stick moved,
        # so "forward" keeps meaning "current nose direction" even while
        # yawing. Vertical (throttle) is commanded directly in world/NED
        # Z, same as a real multicopter's manual/velocity flight mode --
        # roll/pitch shouldn't couple into commanded climb rate.
        vel, yawspeed = stick_to_velocity(stick, self.quat, scale)

        mode = OffboardControlMode()
        mode.velocity = True
        mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.mode_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.position = [float('nan')] * 3
        sp.velocity = [float(v) for v in vel]
        sp.acceleration = [float('nan')] * 3
        sp.yaw = float('nan')
        sp.yawspeed = float(yawspeed)
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.sp_pub.publish(sp)


class Handler(http.server.SimpleHTTPRequestHandler):
    node = None  # set in main()

    def do_GET(self):
        if self.path.startswith('/api/stick'):
            self._handle_stick()
        elif self.path.startswith('/api/manual'):
            self._handle_command()
        elif self.path.startswith('/api/rc_status'):
            self._handle_rc_status()
        else:
            super().do_GET()

    def _handle_stick(self):
        from urllib.parse import urlparse, parse_qs
        q = parse_qs(urlparse(self.path).query)
        # Any subset of the four axes -- the hardware bridge and the
        # browser's virtual stick each own a disjoint subset (see
        # WebControlNode's own comment on why).
        present = {k: v[0] for k, v in q.items() if k in
                   ('yaw', 'throttle', 'roll', 'pitch')}
        try:
            axes = {k: float(v) for k, v in present.items()}
        except ValueError:
            self.send_response(400)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b"yaw/throttle/roll/pitch must be numeric")
            return
        if not axes:
            self.send_response(400)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b"need at least one of yaw, throttle, roll, pitch")
            return
        # rc_bridge.py marks its own posts with hw=1 so the browser can
        # tell a physical transmitter is live and hide its virtual sticks
        # (see /api/rc_status) -- the browser's own stick never sends it.
        hw = q.get('hw', ['0'])[0] == '1'
        self.node.set_stick(hw=hw, **axes)
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def _handle_rc_status(self):
        import json
        body = json.dumps({'present': self.node.rc_present()}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_command(self):
        from urllib.parse import urlparse, parse_qs
        cmd = parse_qs(urlparse(self.path).query).get('cmd', [''])[0]
        if cmd not in COMMAND_COMMANDS and cmd not in ('speed_up', 'speed_down'):
            self.send_response(400)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(f"unknown command {cmd!r}".encode())
            return
        self.node.apply_command(cmd)
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def log_message(self, fmt, *args):
        if '/api/' in self.path:
            super().log_message(fmt, *args)


def main(args=None):
    import os
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    rclpy.init(args=args)
    node = WebControlNode()
    Handler.node = node

    server = http.server.ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"control_server ready on 0.0.0.0:{PORT} "
          f"(static files + /api/manual)")

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
