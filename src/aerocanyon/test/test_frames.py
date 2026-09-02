"""Pure-function tests for the ENU/FLU (MAVROS) <-> NED/FRD (this
project's, and PX4's) convention conversion. No rclpy/ROS needed.
"""
import numpy as np

from aerocanyon.frames import (enu_flu_quat_to_ned_frd,
                               enu_flu_rate_to_ned_frd, quat_mul)


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
