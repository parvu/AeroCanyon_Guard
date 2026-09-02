"""Smallest self-check for control_server's stick-to-RC math -- catches a
wrong sign or axis swap without needing a live ROS2/MAVROS/SITL runtime.
Run: python3 test_control_server.py
"""
from control_server import (COMMAND_COMMANDS, RC_CENTER, RC_SPAN,
                            resolve_stick, stick_to_rc)

ZERO_STICK = {'yaw': 0.0, 'throttle': 0.0, 'roll': 0.0, 'pitch': 0.0}


def stick(**axes):
    return dict(ZERO_STICK, **axes)


# Channel order is ArduPilot's RC1-4: roll, pitch, throttle, yaw.
ROLL, PITCH, THROTTLE, YAW = range(4)

# All-zero stick -> every channel centred. In QHOVER that is "hold
# attitude, hold altitude", which is also what a stale stick must produce.
assert stick_to_rc(ZERO_STICK, 1.0) == (RC_CENTER,) * 4

# Full right roll -> above-centre RC1 (ArduPilot: high PWM = roll right),
# nothing else moves.
out = stick_to_rc(stick(roll=1.0), 1.0)
assert out[ROLL] == RC_CENTER + RC_SPAN
assert out[PITCH] == out[THROTTLE] == out[YAW] == RC_CENTER

# Stick pushed forward (index.html reports +1) must mean FLY FORWARD, which
# on ArduPilot is nose down = BELOW centre on RC2. This is the one inverted
# axis; getting it backwards flies the vehicle away from the stick.
out = stick_to_rc(stick(pitch=1.0), 1.0)
assert out[PITCH] == RC_CENTER - RC_SPAN
assert out[ROLL] == out[THROTTLE] == out[YAW] == RC_CENTER

# Throttle up -> above-centre RC3 (QHOVER: climb).
out = stick_to_rc(stick(throttle=1.0), 1.0)
assert out[THROTTLE] == RC_CENTER + RC_SPAN
assert out[ROLL] == out[PITCH] == out[YAW] == RC_CENTER

# Yaw right -> above-centre RC4, and it must not touch the other axes.
out = stick_to_rc(stick(yaw=1.0), 1.0)
assert out[YAW] == RC_CENTER + RC_SPAN
assert out[ROLL] == out[PITCH] == out[THROTTLE] == RC_CENTER

# Negative sticks mirror around centre.
assert stick_to_rc(stick(yaw=-1.0), 1.0)[YAW] == RC_CENTER - RC_SPAN
assert stick_to_rc(stick(pitch=-1.0), 1.0)[PITCH] == RC_CENTER + RC_SPAN

# speed_scale scales deflection about centre...
assert stick_to_rc(stick(roll=1.0), 0.5)[ROLL] == RC_CENTER + RC_SPAN // 2
# ...but can never push a channel outside the 1000-2000 us band, however
# far speed_up is clicked. A PWM outside RCn_MIN/MAX is a real failsafe
# trigger on ArduPilot, not just a saturated command.
for scale in (1.0, 3.0, 100.0):
    for axis in ZERO_STICK:
        for value in (1.0, -1.0):
            for pwm in stick_to_rc(stick(**{axis: value}), scale):
                assert RC_CENTER - RC_SPAN <= pwm <= RC_CENTER + RC_SPAN, pwm

# Phase 1 is hover-only: no VTOL transition commands are wired up, so
# index.html's transition buttons fall through as unknown commands.
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
