"""Pure-function tests for the ENU/FLU (MAVROS) <-> NED/FRD (this
project's, and PX4's) convention conversion. No rclpy/ROS needed.
"""
import numpy as np
import pytest

from aerocanyon.frames import (enu_flu_quat_to_ned_frd,
                               enu_flu_rate_to_ned_frd, ned_to_latlon,
                               quat_mul)


def test_quat_mul_identity():
    q = np.array([0.7071, 0.0, 0.0, 0.7071])
    identity = np.array([1.0, 0.0, 0.0, 0.0])
    result = quat_mul(q, identity)
    np.testing.assert_allclose(result, q, atol=1e-6)


def test_quat_mul_two_90_degree_yaws_make_a_180():
    # 90 deg yaw (about ENU +z / NED -z, doesn't matter which -- pure
    # quaternion algebra) composed with itself twice is a 180 deg yaw.
    q90 = np.array([np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)])
    q180 = quat_mul(q90, q90)
    expected = np.array([np.cos(np.pi / 2), 0.0, 0.0, np.sin(np.pi / 2)])
    np.testing.assert_allclose(np.abs(q180), np.abs(expected), atol=1e-6)


def test_enu_flu_level_nose_east_matches_ned_frd_level_nose_east():
    # Physical orientation must not change, only its numeric
    # representation. "Level, nose pointing east" in ENU/FLU is a 90 deg
    # yaw about ENU +z (body FLU +x from ENU +x/east to ENU... actually
    # nose-east in ENU means body +x aligned with world +x, i.e. the
    # IDENTITY quaternion, since ENU's own +x axis IS east). In NED, east
    # is world +y, so "nose east" there is a +90 deg yaw about NED +z (a
    # quaternion with real part cos(45deg), z part sin(45deg)).
    q_enu_flu_identity = np.array([1.0, 0.0, 0.0, 0.0])
    q_ned_frd = enu_flu_quat_to_ned_frd(q_enu_flu_identity)
    expected = np.array([np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)])
    # Quaternions double-cover rotations (q and -q are the same
    # orientation) -- compare up to sign.
    assert (np.allclose(q_ned_frd, expected, atol=1e-6)
            or np.allclose(q_ned_frd, -expected, atol=1e-6))


def test_enu_flu_rate_to_ned_frd_flips_y_and_z_only():
    v = np.array([1.0, 2.0, 3.0])
    result = enu_flu_rate_to_ned_frd(v)
    np.testing.assert_allclose(result, [1.0, -2.0, -3.0])


def test_ned_to_latlon_zero_offset_returns_home():
    lat, lon = ned_to_latlon(np.array([0.0, 0.0, 0.0]), 44.0, 26.0)
    assert lat == pytest.approx(44.0, abs=1e-9)
    assert lon == pytest.approx(26.0, abs=1e-9)


def test_ned_to_latlon_north_offset_increases_latitude():
    lat, lon = ned_to_latlon(np.array([100.0, 0.0, 0.0]), 44.0, 26.0)
    assert lat > 44.0
    assert lon == pytest.approx(26.0, abs=1e-6)


def test_ned_to_latlon_east_offset_increases_longitude():
    lat, lon = ned_to_latlon(np.array([0.0, 100.0, 0.0]), 44.0, 26.0)
    assert lon > 26.0
    assert lat == pytest.approx(44.0, abs=1e-6)


def test_ned_to_latlon_matches_known_scale():
    # 1 degree of latitude is ~111,320 m -- a 111.32 m north offset should
    # read back within a small fraction of a degree of 0.001 deg latitude.
    lat, lon = ned_to_latlon(np.array([111.32, 0.0, 0.0]), 0.0, 0.0)
    assert lat == pytest.approx(0.001, rel=1e-2)
