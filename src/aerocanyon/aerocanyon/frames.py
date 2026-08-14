"""NED (PX4) <-> ENU (Gazebo) conversions and quaternion helpers.

Every sign flip between the two frames lives here. Do not inline one
anywhere else -- a stray flip is the single hardest bug to find in this
codebase, because the vehicle still flies, just wrongly.
"""
import numpy as np

_SWAP = np.array([[0.0, 1.0, 0.0],
                  [1.0, 0.0, 0.0],
                  [0.0, 0.0, -1.0]])


def ned_to_enu(v):
    """NED [north, east, down] -> ENU [east, north, up]."""
    return _SWAP @ np.asarray(v, dtype=float)


def enu_to_ned(v):
    """ENU [east, north, up] -> NED [north, east, down]. Self-inverse."""
    return _SWAP @ np.asarray(v, dtype=float)


def quat_to_rotmat(q):
    """Body-to-world rotation matrix from quaternion [w, x, y, z]."""
    w, x, y, z = np.asarray(q, dtype=float)
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def body_z_in_ned(q):
    """Body +z axis expressed in the NED world frame.

    Thrust acts along body -z, so thrust in NED is -T * body_z_in_ned(q).
    """
    return quat_to_rotmat(q) @ np.array([0.0, 0.0, 1.0])
