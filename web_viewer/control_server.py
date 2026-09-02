#!/usr/bin/env python3
"""control_server.py

Ported from CaveX-Explorer-Pro/web_viewer/control_server.py, adapted for
the tricopter: that version relayed browser buttons into gz-transport
StringMsg topics a ROS2 bridge node picked up. This project's manual
control goes through ROS2 topics/services directly -- so this server is
an rclpy node with an embedded HTTP server, not a gz-transport client. It
only targets the README's separate "fly it by hand" path (external
Gazebo/autopilot, no run_trial.py): a real trial's controller_node is the
sole source of setpoints during a leg, and this server is not meant to
run alongside one.

ArduPilot port (was PX4 offboard velocity setpoints on px4_msgs; this
branch talks to ArduPlane through MAVROS). Three things about ArduPlane
forced the shape of this file, all measured against a live SITL rather
than assumed:

1. **Sticks go out as RC override, not velocity setpoints.** ArduPlane's
   GUIDED mode ignores velocity entirely: its
   `GCS_MAVLINK_Plane::handle_set_position_target_local_ned`
   (ArduPlane/GCS_MAVLink_Plane.cpp) drops anything that isn't
   MAV_FRAME_LOCAL_OFFSET_NED and then uses only `packet.z` as an
   altitude *offset* -- there is no velocity path at all, unlike PX4's
   TrajectorySetpoint. Streaming TwistStamped to
   /mavros/setpoint_velocity/cmd_vel at 50 Hz for 40 s in GUIDED moved
   the airframe exactly 0.00 m. So manual flight is QHOVER +
   RC_CHANNELS_OVERRIDE (/mavros/rc/override), which is also what the
   autopilot was actually verified hovering on. Consequence: roll/pitch
   are now body-relative lean commands, not world-frame velocities, and
   the vehicle's attitude is no longer needed here (QHOVER resolves the
   body frame itself) -- hence no /mavros/local_position/pose
   subscription.

2. **Mode changes go through /mavros/cmd/command, not /mavros/set_mode.**
   This airframe heartbeats as MAV_TYPE_VTOL_TILTROTOR (21), which is
   absent from MAVROS's ArduPilot mode tables -- MAVROS logs "MODE:
   Unknown APM based FCU! Type: 21", reports every mode as "CMODE(n)",
   and /mavros/set_mode returns mode_sent=False for 'GUIDED', 'QHOVER'
   and the 'CMODE(18)' escape hatch alike. A raw COMMAND_LONG carrying
   MAV_CMD_DO_SET_MODE bypasses that table and works.

3. **MAVROS must run with system_id:=255.** ArduPilot gates RC override
   on `gcs().sysid_is_gcs(msg.sysid)` (GCS_Common.cpp
   handle_rc_channels_override) against MAV_GCS_SYSID, which defaults to
   255. MAVROS defaults to system id 1, so its overrides are silently
   dropped -- RC_CHANNELS keeps reading the SITL's own idle sticks and
   nothing moves. See the README for the launch line.

Run from web_viewer/, with Gazebo + SITL + MAVROS already up (see README
"Watching a trial fly, or flying manually"):
    python3 control_server.py [port]
"""
import pathlib
import sys
import threading
import time
import http.server

import rclpy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, CommandLong
from rclpy.node import Node

# rc_pwm.py lives in the aerocanyon ROS2 package, not on web_viewer's own
# path -- web_viewer/ is a standalone script directory (Phase 1), not a
# ROS2 package with aerocanyon as an installed dependency. An import-path
# workaround, not a proper package dependency; fine for now, flagged as a
# wart rather than solved properly here.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]
                       / 'src' / 'aerocanyon'))
from aerocanyon.rc_pwm import (MODE_QHOVER, MODE_QLAND, RC_CENTER, RC_SPAN,
                               THROTTLE_MID, THROTTLE_SPAN, arm, pwm,
                               pwm_throttle, resolve_stick, set_mode)
# RC_SPAN/THROTTLE_MID/THROTTLE_SPAN aren't used directly in this file
# anymore (rc_pwm.pwm/pwm_throttle own that math now) but stay imported
# here -- test_control_server.py imports them from this module by name.

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
CONTROL_HZ = 50
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

# transition_fw/transition_mc are deliberately absent: forward-flight
# transition is out of scope for Phase 1, and ArduPilot's trigger is
# nothing like PX4's VEHICLE_CMD_DO_VTOL_TRANSITION. index.html still has
# the buttons; they now fall through as unknown commands.
COMMAND_COMMANDS = {'arm', 'disarm', 'land'}


def stick_to_rc(stick, scale):
    """Mode 2 stick state -> (roll, pitch, throttle, yaw) PWM for RC
    channels 1-4. Pure function, no ROS/rclpy needed -- see
    test_control_server.py.

    Pitch is the one inverted axis: index.html reports "stick pushed
    forward" as +1 (up on screen) and that has to mean "fly forward",
    which on ArduPilot means nose DOWN, which is BELOW-centre PWM on
    channel 2 -- ArduPilot's un-reversed pitch channel treats high PWM as
    nose up. Roll, throttle and yaw all share ArduPilot's sign already
    (high PWM = right / climb / yaw right).

    Throttle is also the one axis with its own PWM range (THROTTLE_MID +/-
    THROTTLE_SPAN, capping at 1900 rather than the other channels' shared
    2000) -- see those constants' own comment in rc_pwm.py.
    """
    return (pwm(stick['roll'], scale), pwm(stick['pitch'], scale, invert=True),
            pwm_throttle(stick['throttle'], scale), pwm(stick['yaw'], scale))


class WebControlNode(Node):

    def __init__(self):
        super().__init__('web_control_server')
        self.rc_pub = self.create_publisher(
            OverrideRCIn, '/mavros/rc/override', 10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        # /mavros/set_mode can't be used here -- see the module docstring,
        # point 2 (MAV_TYPE 21 is missing from MAVROS's mode tables).
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')

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

        self.create_timer(1.0 / CONTROL_HZ, self._tick)

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
            # Mode first, then arm -- the same ordering the PX4 version used,
            # and the ordering the live SITL runs were verified with.
            set_mode(self.cmd_client, MODE_QHOVER)
            arm(self.arm_client, True)
        elif cmd == 'disarm':
            arm(self.arm_client, False)
        elif cmd == 'land':
            set_mode(self.cmd_client, MODE_QLAND)

    def _tick(self):
        now = time.monotonic()
        with self._lock:
            stick = resolve_stick(self._stick, self._stick_time, now, STICK_TIMEOUT_S)
            scale = self._speed_scale

        # QHOVER interprets these in the vehicle's own body frame and holds
        # altitude at centred throttle, so there's no attitude maths to do
        # here any more (the PX4 version rotated a world-frame velocity by
        # the current yaw itself). A stale/centred stick lands on 1500 on
        # every axis, i.e. hold attitude and hold altitude -- the correct
        # RC-failsafe reading of the dead-man's switch.
        roll, pitch, throttle, yaw = stick_to_rc(stick, scale)

        msg = OverrideRCIn()
        channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        channels[0:4] = [roll, pitch, throttle, yaw]
        # Channels 5-8 pinned to centre, matching the SITL runs this was
        # verified on. tricopter.parm sets FLTMODE_CH=0, explicitly
        # disabling ArduPilot's RC mode-switch, so channel 8 being pinned
        # here is never misread as a flight-mode-switch position.
        channels[4:8] = [RC_CENTER] * 4
        msg.channels = channels
        self.rc_pub.publish(msg)

        # ArduPilot drops RC overrides RC_OVERRIDE_TIME (3 s) after the last
        # one arrives, so this 50 Hz stream is what keeps manual control
        # alive; stopping the server hands the vehicle back to its own RC.


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
