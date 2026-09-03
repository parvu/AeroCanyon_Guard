"""NED (PX4) <-> ENU (Gazebo) conversions and quaternion helpers.

Every sign flip between the two frames lives here. Do not inline one
anywhere else -- a stray flip is the single hardest bug to find in this
codebase, because the vehicle still flies, just wrongly.
"""
import math

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


def yaw_from_quat(q):
    """Heading (rad, world-NED z-axis Euler angle) from a body-to-world
    quaternion [w, x, y, z] in this project's NED/FRD convention."""
    w, x, y, z = np.asarray(q, dtype=float)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


# ENU/FLU (MAVROS/ROS convention: world East-North-Up, body
# Forward-Left-Up) <-> NED/FRD (this project's convention throughout --
# PX4's, unchanged since this port only touches where telemetry comes
# from, not how mission.py/cbf_filter.py/fo_pinn.py interpret it).
#
# Sourced from MAVROS's own internal conversion (mavros/src/lib/
# ftf_frame_conversions.cpp: NED_ENU_Q, AIRCRAFT_BASELINK_Q) rather than
# derived from scratch -- this exact sandwich transform is already
# proven correct in MAVROS's own NED-convention topics, and a
# from-scratch derivation is exactly the kind of stray-sign-flip risk
# this file's own docstring warns about.
_NED_ENU_Q = np.array([0.0, 0.70710678, 0.70710678, 0.0])       # world: 180 deg about (1,1,0)/sqrt(2)
_AIRCRAFT_BASELINK_Q = np.array([0.0, 1.0, 0.0, 0.0])            # body: 180 deg about x


def quat_mul(q1, q2):
    """Hamilton product of two [w, x, y, z] quaternions."""
    w1, x1, y1, z1 = np.asarray(q1, dtype=float)
    w2, x2, y2, z2 = np.asarray(q2, dtype=float)
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def enu_flu_quat_to_ned_frd(q):
    """MAVROS's world-ENU/body-FLU orientation quaternion [w,x,y,z] ->
    this project's world-NED/body-FRD convention."""
    return quat_mul(quat_mul(_NED_ENU_Q, np.asarray(q, dtype=float)),
                    _AIRCRAFT_BASELINK_Q)


def enu_flu_rate_to_ned_frd(v):
    """Body-frame FLU rate (angular velocity or linear acceleration,
    [x,y,z]) -> body-frame FRD. No rotation matrix needed -- this is a
    body-axis relabelling (forward stays forward; left/up flip to
    right/down), not a world-frame rotation."""
    x, y, z = np.asarray(v, dtype=float)
    return np.array([x, -y, -z])


_EARTH_RADIUS_M = 6378137.0  # WGS84 equatorial radius


def ned_to_latlon(ned, home_lat_deg, home_lon_deg):
    """NED [north, east, down] offset from a home point -> (lat, lon)
    degrees. Flat-earth/local-tangent-plane approximation -- accurate to
    sub-centimetre at this project's scale (canyon spans ~250m), the
    same approximation ArduPilot's own EKF uses internally for local NED
    <-> global conversion at this scale. `down` is unused -- altitude is
    handled separately via mission items' own relative-altitude field,
    not folded into this conversion."""
    north, east, _down = np.asarray(ned, dtype=float)
    home_lat_rad = math.radians(home_lat_deg)
    lat = home_lat_deg + math.degrees(north / _EARTH_RADIUS_M)
    lon = home_lon_deg + math.degrees(
        east / (_EARTH_RADIUS_M * math.cos(home_lat_rad)))
    return lat, lon


def latlon_to_ned(lat_deg, lon_deg, home_lat_deg, home_lon_deg):
    """(lat, lon) degrees -> NED (north, east) metre offset from a home
    point -- the exact inverse of ned_to_latlon above (same flat-earth
    approximation, solved backward)."""
    home_lat_rad = math.radians(home_lat_deg)
    north = math.radians(lat_deg - home_lat_deg) * _EARTH_RADIUS_M
    east = (math.radians(lon_deg - home_lon_deg) * _EARTH_RADIUS_M
            * math.cos(home_lat_rad))
    return north, east
