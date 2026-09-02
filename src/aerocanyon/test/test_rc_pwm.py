"""Pure-function tests for rc_pwm's PWM mapping -- mirrors
web_viewer/test_control_server.py's existing coverage, since this is
that same logic after extraction. No rclpy needed for these three
functions (arm/set_mode need a live client and are exercised via
controller_node's own tests instead, same as control_server's manual
verification)."""
from aerocanyon.rc_pwm import (RC_CENTER, RC_SPAN, THROTTLE_MID,
                               THROTTLE_SPAN, pwm, pwm_throttle,
                               resolve_stick)


def test_pwm_centres_at_zero():
    assert pwm(0.0, 1.0) == RC_CENTER


def test_pwm_full_deflection_hits_the_span_edge():
    assert pwm(1.0, 1.0) == RC_CENTER + RC_SPAN
    assert pwm(-1.0, 1.0) == RC_CENTER - RC_SPAN


def test_pwm_invert_flips_sign():
    assert pwm(1.0, 1.0, invert=True) == RC_CENTER - RC_SPAN


def test_pwm_throttle_uses_its_own_range_not_rc_span():
    assert pwm_throttle(0.0, 1.0) == THROTTLE_MID
    assert pwm_throttle(1.0, 1.0) == THROTTLE_MID + THROTTLE_SPAN
    assert pwm_throttle(-1.0, 1.0) == THROTTLE_MID - THROTTLE_SPAN


def test_pwm_never_exceeds_its_band_however_large_the_scale():
    for scale in (1.0, 3.0, 100.0):
        assert RC_CENTER - RC_SPAN <= pwm(1.0, scale) <= RC_CENTER + RC_SPAN
        assert (THROTTLE_MID - THROTTLE_SPAN <= pwm_throttle(1.0, scale)
                <= THROTTLE_MID + THROTTLE_SPAN)


def test_resolve_stick_zeros_a_stale_axis_only():
    live = {'yaw': 0.5, 'throttle': 0.5, 'roll': 0.5, 'pitch': 0.5}
    fresh = {k: 10.0 for k in live}
    stale = dict(fresh, pitch=5.0)
    resolved = resolve_stick(live, stale, now=10.1, timeout=0.3)
    assert resolved['pitch'] == 0.0
    assert resolved['yaw'] == 0.5 and resolved['roll'] == 0.5
