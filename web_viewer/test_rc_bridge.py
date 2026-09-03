"""Smallest self-check for rc_bridge's axis normalization.
Run: python3 test_rc_bridge.py
"""
from rc_bridge import AXIS_PITCH, AXIS_ROLL, AXIS_THROTTLE, AXIS_YAW, normalize

# Center, min, max map to 0, -1, +1.
assert normalize(0) == 0.0
assert normalize(-32767) == -1.0
assert normalize(32767) == 1.0

# Halfway reads ~0.5.
assert abs(normalize(16384) - 0.5) < 0.01

# Out-of-range raw values clamp, don't overshoot [-1, 1] (real devices can
# report slightly past +/-32767).
assert normalize(-40000) == -1.0
assert normalize(40000) == 1.0

# The four axis indices are distinct -- catches a copy-paste mapping bug.
assert len({AXIS_ROLL, AXIS_PITCH, AXIS_THROTTLE, AXIS_YAW}) == 4

print("ok")
