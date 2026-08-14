import numpy as np
from aerocanyon import frames


def test_ned_enu_roundtrip():
    v = np.array([1.0, 2.0, 3.0])
    assert np.allclose(frames.enu_to_ned(frames.ned_to_enu(v)), v)


def test_ned_to_enu_swaps_axes_and_flips_down():
    # NED (north=1, east=2, down=3) -> ENU (east=2, north=1, up=-3)
    assert np.allclose(frames.ned_to_enu(np.array([1.0, 2.0, 3.0])),
                       np.array([2.0, 1.0, -3.0]))


def test_identity_quaternion_is_identity_rotation():
    assert np.allclose(frames.quat_to_rotmat(np.array([1.0, 0.0, 0.0, 0.0])),
                       np.eye(3))


def test_yaw_90_rotates_x_to_y():
    s = np.sqrt(0.5)
    R = frames.quat_to_rotmat(np.array([s, 0.0, 0.0, s]))  # +90 deg about z
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]),
                       atol=1e-9)


def test_body_z_level_points_down_in_ned():
    # Level attitude: body -z is up, so body +z is NED +down.
    assert np.allclose(frames.body_z_in_ned(np.array([1.0, 0.0, 0.0, 0.0])),
                       np.array([0.0, 0.0, 1.0]))
