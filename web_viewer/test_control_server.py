"""Smallest self-check for control_server's stick-to-velocity math --
catches a wrong sign or axis swap without needing a live ROS2/PX4
runtime. Run: python3 test_control_server.py
"""
import numpy as np

from control_server import COMMAND_COMMANDS, resolve_stick, stick_to_velocity

IDENTITY_QUAT = [1.0, 0.0, 0.0, 0.0]  # yaw = 0, nose along NED +x (north)
ZERO_STICK = {'yaw': 0.0, 'throttle': 0.0, 'roll': 0.0, 'pitch': 0.0}


def stick(**axes):
    return dict(ZERO_STICK, **axes)


# All-zero stick -> hover, no motion.
vel, yawspeed = stick_to_velocity(ZERO_STICK, IDENTITY_QUAT, 1.0)
assert vel == [0.0, 0.0, 0.0] and yawspeed == 0.0

# Full forward pitch at yaw=0 moves along NED +x (north), no sideways/vertical.
vel, yawspeed = stick_to_velocity(stick(pitch=1.0), IDENTITY_QUAT, 1.0)
assert vel[0] > 0 and abs(vel[1]) < 1e-9 and vel[2] == 0.0

# Full right roll at yaw=0 moves along NED +y (east), no forward/vertical.
vel, yawspeed = stick_to_velocity(stick(roll=1.0), IDENTITY_QUAT, 1.0)
assert abs(vel[0]) < 1e-9 and vel[1] > 0 and vel[2] == 0.0

# Full up throttle is negative NED z (up), independent of yaw/attitude.
vel, yawspeed = stick_to_velocity(stick(throttle=1.0), IDENTITY_QUAT, 1.0)
assert vel[2] < 0

# Yaw stick maps to yawspeed only, doesn't touch velocity.
vel, yawspeed = stick_to_velocity(stick(yaw=1.0), IDENTITY_QUAT, 1.0)
assert vel == [0.0, 0.0, 0.0] and yawspeed > 0

# Forward at yaw=90 deg (facing NED east) moves along +y, not +x -- proves
# horizontal motion is genuinely body-relative, not a fixed world mapping.
yaw90 = [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]  # q = [cos(y/2),0,0,sin(y/2)]
vel, _ = stick_to_velocity(stick(pitch=1.0), yaw90, 1.0)
assert vel[0] < 1e-6 and vel[1] > 0

# speed_scale scales velocity/yawspeed proportionally.
vel_1x, yaw_1x = stick_to_velocity(stick(pitch=1.0, yaw=1.0), IDENTITY_QUAT, 1.0)
vel_2x, yaw_2x = stick_to_velocity(stick(pitch=1.0, yaw=1.0), IDENTITY_QUAT, 2.0)
assert abs(vel_2x[0] - 2 * vel_1x[0]) < 1e-9
assert abs(yaw_2x - 2 * yaw_1x) < 1e-9

assert COMMAND_COMMANDS == {'arm', 'disarm', 'land'}

# resolve_stick: per-axis dead-man's-switch. Two sources (hardware bridge
# owning throttle/roll/yaw, browser owning pitch) update at different
# times -- one going stale must zero only its own axes.
live = {'yaw': 0.5, 'throttle': 0.5, 'roll': 0.5, 'pitch': 0.5}
fresh_times = {k: 10.0 for k in live}
assert resolve_stick(live, fresh_times, now=10.1, timeout=0.3) == live

stale_pitch = dict(fresh_times, pitch=5.0)  # pitch last updated 5.1s ago
resolved = resolve_stick(live, stale_pitch, now=10.1, timeout=0.3)
assert resolved['pitch'] == 0.0
assert resolved['yaw'] == 0.5 and resolved['throttle'] == 0.5 and resolved['roll'] == 0.5

print("ok")
