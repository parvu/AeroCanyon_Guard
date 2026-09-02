"""Smallest self-check for control_server's stick-to-RC math -- catches a
wrong sign or axis swap without needing a live ROS2/MAVROS/SITL runtime.
Run: python3 test_control_server.py
"""
from control_server import (COMMAND_COMMANDS, RC_CENTER, RC_SPAN,
                            THROTTLE_MID, THROTTLE_SPAN,
                            resolve_stick, stick_to_rc)

ZERO_STICK = {'yaw': 0.0, 'throttle': 0.0, 'roll': 0.0, 'pitch': 0.0}


def stick(**axes):
    return dict(ZERO_STICK, **axes)


# Channel order is ArduPilot's RC1-4: roll, pitch, throttle, yaw.
ROLL, PITCH, THROTTLE, YAW = range(4)

# All-zero stick -> roll/pitch/yaw centred, throttle at its own midpoint
# (THROTTLE_MID, not RC_CENTER -- see that constant's comment). In QHOVER
# that is "hold attitude, hold altitude", which is also what a stale
# stick must produce.
out = stick_to_rc(ZERO_STICK, 1.0)
assert out[ROLL] == out[PITCH] == out[YAW] == RC_CENTER
assert out[THROTTLE] == THROTTLE_MID

# Full right roll -> above-centre RC1 (ArduPilot: high PWM = roll right),
# nothing else moves.
out = stick_to_rc(stick(roll=1.0), 1.0)
assert out[ROLL] == RC_CENTER + RC_SPAN
assert out[PITCH] == out[YAW] == RC_CENTER
assert out[THROTTLE] == THROTTLE_MID

# Stick pushed forward (index.html reports +1) must mean FLY FORWARD, which
# on ArduPilot is nose down = BELOW centre on RC2. This is the one inverted
# axis; getting it backwards flies the vehicle away from the stick.
out = stick_to_rc(stick(pitch=1.0), 1.0)
assert out[PITCH] == RC_CENTER - RC_SPAN
assert out[ROLL] == out[YAW] == RC_CENTER
assert out[THROTTLE] == THROTTLE_MID

# Throttle up -> its own max (1900, not the other channels' 2000 -- a real
# ratcheted Mode 2 throttle gimbal, no self-centering, see index.html).
out = stick_to_rc(stick(throttle=1.0), 1.0)
assert out[THROTTLE] == THROTTLE_MID + THROTTLE_SPAN
assert out[ROLL] == out[PITCH] == out[YAW] == RC_CENTER

# Yaw right -> above-centre RC4, and it must not touch the other axes.
out = stick_to_rc(stick(yaw=1.0), 1.0)
assert out[YAW] == RC_CENTER + RC_SPAN
assert out[ROLL] == out[PITCH] == RC_CENTER
assert out[THROTTLE] == THROTTLE_MID

# Negative sticks mirror around centre.
assert stick_to_rc(stick(yaw=-1.0), 1.0)[YAW] == RC_CENTER - RC_SPAN
assert stick_to_rc(stick(pitch=-1.0), 1.0)[PITCH] == RC_CENTER + RC_SPAN
assert stick_to_rc(stick(throttle=-1.0), 1.0)[THROTTLE] == THROTTLE_MID - THROTTLE_SPAN

# speed_scale scales deflection about centre...
assert stick_to_rc(stick(roll=1.0), 0.5)[ROLL] == RC_CENTER + RC_SPAN // 2
# ...but can never push a channel outside its own PWM band, however far
# speed_up is clicked. A PWM outside RCn_MIN/MAX is a real failsafe
# trigger on ArduPilot, not just a saturated command. Throttle's band is
# narrower (1000-1900) than the other three channels' (1000-2000).
for scale in (1.0, 3.0, 100.0):
    for axis in ZERO_STICK:
        for value in (1.0, -1.0):
            out = stick_to_rc(stick(**{axis: value}), scale)
            assert RC_CENTER - RC_SPAN <= out[ROLL] <= RC_CENTER + RC_SPAN
            assert RC_CENTER - RC_SPAN <= out[PITCH] <= RC_CENTER + RC_SPAN
            assert RC_CENTER - RC_SPAN <= out[YAW] <= RC_CENTER + RC_SPAN
            assert THROTTLE_MID - THROTTLE_SPAN <= out[THROTTLE] <= THROTTLE_MID + THROTTLE_SPAN

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
