"""Control barrier function safety filter over the acceleration setpoint.

Solves  min ||u - u_des||^2  subject to the barrier constraints, so the
filter is transparent when the desired command is already safe and
minimally invasive when it is not. That transparency is what makes the
intervention figure readable: every deviation from u_des is a barrier
doing work.

Barriers:
  h1  obstacle    distance to the nearest building surface (metres)
  h2  stall       alpha_stall - angle of attack (radians) -- fixed-wing
                  only, see enable_stall below
  h3  slew        limit on how fast the commanded acceleration may change

The obstacle barrier has relative degree 2 with respect to acceleration,
so it uses the high-order form
    grad(h).u >= -k1*h - k2*hdot
The boxes make grad(h) piecewise constant, so the second-order geometry
term vanishes exactly rather than being approximated away.

enable_stall defaults to False: "stall" (loss of wing lift past a critical
angle of attack) isn't a phenomenon a multicopter has -- there's no wing.
It only applies while the vehicle is actually flying fixed-wing, i.e. when
controller_node.ENABLE_VTOL_TRANSITION is True. Verified live with it on
unconditionally: measured angle-of-attack sits near 90 degrees for the
vast majority of ordinary multicopter flight (completely normal -- the
body's "forward" axis has nothing to do with the resultant airspeed
direction when thrust, not a wing, is providing lift), so h_stall
(radians) was deeply negative almost continuously. filter() used to fold
that straight into the same h_min as h_obstacle (metres) via a bare
min() -- meters and radians combined with no unit reconciliation -- which
is why "closest approach to the barrier" printed an identical, physically
meaningless value across unrelated trials. h_obstacle and h_stall are
reported separately now specifically so that mixing can't happen again.

ponytail: SLSQP for a 3-variable QP -- measured at 2.9 ms, well inside the
20 ms control period. Move to a dedicated QP solver only if the constraint
count grows or the solve time approaches the period.
"""
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import minimize

from . import canyon_geometry as cg
from . import frames
from .constants import CONTROL_HZ


@dataclass
class CBFParams:
    alpha_stall_deg: float = 12.0
    k_obstacle: tuple = (1.0, 2.0)   # (k1 on h, k2 on hdot)
    k_stall: float = 2.0
    slew_max: float = 15.0           # m/s^2 per second
    safe_distance: float = 6.0       # standoff from building surfaces, metres
    eps: float = 1e-6


def angle_of_attack(vel_ned, wind_ned, q):
    """Angle of attack from the airspeed vector, in radians.

    Airspeed is ground velocity minus wind. Rotated into the body frame,
    alpha is the angle between the airspeed and the body x-z plane's
    forward axis. Since body z points down, positive w_body means descending
    (negative alpha), so we use -w_body in the atan2.
    """
    air_ned = np.asarray(vel_ned, dtype=float) - np.asarray(wind_ned, dtype=float)
    speed = np.linalg.norm(air_ned)
    if speed < 1.0:
        return 0.0  # below flying speed alpha is meaningless
    air_body = frames.quat_to_rotmat(q).T @ air_ned
    return float(np.arctan2(-air_body[2], max(air_body[0], 1e-6)))


class CBFFilter:

    def __init__(self, params=None, dt=1.0 / CONTROL_HZ, enable_stall=False):
        self.p = params or CBFParams()
        self.dt = float(dt)
        self.enable_stall = bool(enable_stall)
        self.last_safe = np.zeros(3)
        self.last_u = np.zeros(3)
        self._force_infeasible = False  # test hook only
        self._first_call = True  # don't apply slew limit on first call

    def _obstacle_constraint(self, pos_ned, vel_ned):
        """Return (a, b, h) for the linear constraint a . u >= b."""
        pos_enu = frames.ned_to_enu(pos_ned)
        dist, n_enu = cg.distance_and_normal(pos_enu)
        h = dist - self.p.safe_distance
        # grad(h) in NED. The normal points away from the building, i.e.
        # the direction in which h increases.
        grad = frames.enu_to_ned(n_enu)
        hdot = float(grad @ np.asarray(vel_ned, dtype=float))
        k1, k2 = self.p.k_obstacle
        return grad, -k1 * h - k2 * hdot, h

    def _stall_constraint(self, vel_ned, wind_ned, q):
        """Return (a, b, h). Alpha depends on velocity, so d(alpha)/du is
        obtained by finite difference through the velocity channel -- the
        map is smooth and 3-dimensional, so this is cheap and exact enough.
        """
        alpha_max = np.deg2rad(self.p.alpha_stall_deg)
        alpha = angle_of_attack(vel_ned, wind_ned, q)
        h = alpha_max - abs(alpha)

        grad_alpha = np.zeros(3)
        step = 1e-4
        for i in range(3):
            dv = np.zeros(3)
            dv[i] = step
            plus = angle_of_attack(np.asarray(vel_ned) + dv, wind_ned, q)
            minus = angle_of_attack(np.asarray(vel_ned) - dv, wind_ned, q)
            grad_alpha[i] = (plus - minus) / (2 * step)
        # h decreases as |alpha| grows, so grad(h) = -sign(alpha)*grad(alpha).
        grad_h = -np.sign(alpha) * grad_alpha
        return grad_h, -self.p.k_stall * h, h

    def filter(self, u_des, pos_ned, vel_ned, wind_ned, q):
        """Return (u_safe, info). All accelerations are NED, m/s^2.

        info['h_obstacle'] (metres) and info['h_stall'] (radians, None
        unless enable_stall) are kept separate deliberately -- see the
        module docstring for why combining them into one number is wrong.
        """
        u_des = np.asarray(u_des, dtype=float)
        info = {'active': False, 'h_obstacle': np.inf, 'h_stall': None, 'feasible': True}

        if not np.all(np.isfinite(u_des)):
            info['feasible'] = False
            return self.last_safe.copy(), info

        rows = []
        a_obs, b_obs, h_obs = self._obstacle_constraint(pos_ned, vel_ned)
        rows.append((a_obs, b_obs))
        info['h_obstacle'] = h_obs

        if self.enable_stall:
            a_stall, b_stall, h_stall = self._stall_constraint(vel_ned, wind_ned, q)
            if np.linalg.norm(a_stall) > self.p.eps:
                rows.append((a_stall, b_stall))
                info['h_stall'] = h_stall

        # On the first call, don't apply slew limits since we don't know the
        # true previous acceleration state. Use large but finite bounds
        # since SLSQP may have trouble with infinite bounds.
        if self._first_call:
            bounds = [(-1e3, 1e3) for _ in range(3)]
            self._first_call = False
        else:
            slew = self.p.slew_max * self.dt
            bounds = [(self.last_u[i] - slew, self.last_u[i] + slew) for i in range(3)]

        cons = [{'type': 'ineq', 'fun': (lambda u, a=a, b=b: float(a @ u - b)),
                 'jac': (lambda u, a=a: a)} for a, b in rows]

        if self._force_infeasible:
            info['feasible'] = False
            return self.last_safe.copy(), info

        res = minimize(
            lambda u: float(np.sum((u - u_des) ** 2)),
            x0=np.clip(u_des, [b[0] for b in bounds], [b[1] for b in bounds]),
            jac=lambda u: 2.0 * (u - u_des),
            bounds=bounds, constraints=cons, method='SLSQP',
            options={'maxiter': 30, 'ftol': 1e-6},
        )

        if not res.success or not np.all(np.isfinite(res.x)):
            info['feasible'] = False
            return self.last_safe.copy(), info

        u_safe = np.asarray(res.x, dtype=float)
        info['active'] = bool(np.linalg.norm(u_safe - u_des) > 1e-3)
        self.last_u = u_safe.copy()
        self.last_safe = u_safe.copy()
        return u_safe, info
