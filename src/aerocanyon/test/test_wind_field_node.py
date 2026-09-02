"""Pure-function coverage for the MC-flight wind speed cap. No rclpy/gz
needed -- see wind_field_node.cap_speed's own docstring for why it's
factored out.
"""
import numpy as np

from aerocanyon.wind_field_node import MC_MAX_WIND_SPEED_MS, cap_speed


def test_cap_speed_leaves_vectors_under_the_cap_unchanged():
    v = np.array([0.5, -0.3, 0.1])
    np.testing.assert_array_equal(cap_speed(v, 2.0), v)


def test_cap_speed_clamps_magnitude_preserving_direction():
    v = np.array([8.0, 0.0, 0.0])
    out = cap_speed(v, 2.0)
    assert np.linalg.norm(out) == 2.0
    np.testing.assert_allclose(out / np.linalg.norm(out), v / np.linalg.norm(v))


def test_cap_speed_default_matches_the_2ms_mc_target():
    assert MC_MAX_WIND_SPEED_MS == 2.0


def test_cap_speed_zero_vector_stays_zero():
    v = np.zeros(3)
    np.testing.assert_array_equal(cap_speed(v, 2.0), v)
