"""Train the FO-PINN wind estimator on logged trial CSVs.

Supervision is free here because we generated the wind field: the logger
records ground-truth wind alongside the state. On hardware only the
physics residual survives, which is exactly why it carries weight in the
loss rather than being decoration.
"""
import argparse
import pathlib

import numpy as np
import pandas as pd
import torch

from . import frames
from .constants import G, MASS_KG
from .fo_pinn import (STATE_KEYS, FractionalMemory, WindEstimator, total_loss)

RHO = 1.225
CD_A = 0.12  # drag area, m^2. Calibration knob -- see the note in the plan.

# The tricopter's wing halves carry their own gz-sim-lift-drag-system
# plugins in PX4-Autopilot's model.sdf (base_link, two wing-half
# surfaces), generating a real aerodynamic LIFT force -- perpendicular to
# the relative airflow -- that CD_A's isotropic drag model cannot
# represent at all (isotropic drag only ever pushes along the relative
# wind, never across it). That plugin runs continuously regardless of
# this project's own MC/FW mode flag; Gazebo has no concept of
# ENABLE_VTOL_TRANSITION. WING_CLA/WING_ALPHA_STALL/WING_CLA_STALL are
# copied from model.sdf's own <cla>/<alpha_stall>/<cla_stall> so the
# anisotropic term below reflects the same physics the simulator actually
# applies, not an independently guessed curve. WING_AREA is NOT copied
# as-is, though: model.sdf's own 2x 0.5 m^2 (from PX4's stock
# standard_vtol.sdf.jinja template, generated for a template airframe,
# never recalibrated for this specific vehicle) produced lift forces of
# 0.5-2.4x the vehicle's own weight -- disproportionate to the fairly
# controllable flight actually observed live all session. Scaled down to
# 2x 0.3 m^2 (a calibration knob, like CD_A above) alongside correcting
# MASS_KG (constants.py) to the vehicle's real simulated total mass.
#
# CARRIED OVER UNCHANGED to the tricopter, and re-verified rather than
# assumed: converting that airframe removed a rotor but touched no
# aerodynamic surface, so model.sdf still declares exactly the same four
# LiftDrag plugins (2x 0.5 m^2 wing halves + 2 small control surfaces)
# with the same cla/alpha_stall/cla_stall. Re-running the weight-fraction
# check against live tricopter telemetry reproduces the tiltrotor's
# numbers: mean wing lift 33% of vehicle weight (peak 117%, down from
# 139% on the slightly heavier tiltrotor), and mean frame drag 4% of
# weight. CD_A is arguably a touch high now -- the tricopter has one less
# rotor and shorter booms -- but the difference is far inside this knob's
# uncertainty, and trimming it without a real measurement would be false
# precision.
WING_AREA = 0.6           # m^2, both 0.3 m^2 wing-half surfaces combined
WING_CLA = 4.752798721   # lift-curve slope, per radian
WING_ALPHA_STALL = 0.3391428111  # rad (~19.4 deg)
WING_CLA_STALL = -3.85   # post-stall slope (lift collapses past alpha_stall)
# gz-sim-lift-drag-system's own post-stall behaviour past this angle isn't
# independently verifiable here (no source available in this environment,
# only the compiled plugin), and this project's own multicopter flight
# spends the majority of its time with the relative airflow nowhere near
# the wing's forward axis -- alpha near 90 degrees is the *normal* case,
# not a rare edge case (verified live). Continuing WING_CLA_STALL's linear
# extrapolation all the way out there is not physically defensible
# regardless of what the plugin does: it would put peak |CL| (~3.1 at 90
# degrees) well past the pre-stall peak (~1.6 at WING_ALPHA_STALL), which
# no real wing does -- post-stall lift plateaus, it doesn't overtake the
# pre-stall peak. Clamped a little above that pre-stall peak instead.
# ponytail: a flat clamp, not a smoothed/decaying post-stall curve --
# revisit with real force measurements if the wing ever needs to be
# trusted quantitatively deep in the post-stall region.
WING_CL_MAX = 1.8


def _wing_lift_coefficient(alpha):
    """Piecewise-linear CL(alpha) below stall (matches gz-sim-lift-drag-
    system's own model.sdf curve exactly), clamped to WING_CL_MAX beyond
    it -- see that constant's comment for why the post-stall region is
    clamped rather than extrapolated."""
    a = abs(float(alpha))
    if a <= WING_ALPHA_STALL:
        cl = WING_CLA * a
    else:
        cl = WING_CLA * WING_ALPHA_STALL + WING_CLA_STALL * (a - WING_ALPHA_STALL)
        cl = float(np.clip(cl, -WING_CL_MAX, WING_CL_MAX))
    return cl if alpha >= 0 else -cl


def wing_lift_force(vel_ned, wind_ned, q):
    """Anisotropic wing-lift force, NED newtons -- the piece the isotropic
    drag term below cannot represent. Body-frame angle of attack uses the
    same convention as cbf_filter.angle_of_attack (airspeed = velocity
    relative to the air mass, rotated into the body frame via the x-z
    plane) so the two stay consistent.
    """
    R = frames.quat_to_rotmat(q)
    airspeed_ned = np.asarray(vel_ned, dtype=float) - np.asarray(wind_ned, dtype=float)
    speed = float(np.linalg.norm(airspeed_ned))
    if speed < 1.0:
        return np.zeros(3)  # below flying speed, alpha (and lift) is meaningless

    airspeed_body = R.T @ airspeed_ned
    u, _, w = airspeed_body
    alpha = float(np.arctan2(-w, max(u, 1e-6)))
    cl = _wing_lift_coefficient(alpha)
    lift_mag = 0.5 * RHO * WING_AREA * cl * speed ** 2

    # Perpendicular to the airspeed within the body x-z plane, pointing
    # "up" (toward -z_body) for positive alpha in forward flight -- the
    # same plane cbf_filter.angle_of_attack computes alpha in.
    lift_dir_body = np.array([w, 0.0, -u])
    norm = np.linalg.norm(lift_dir_body)
    if norm < 1e-9:
        return np.zeros(3)
    lift_dir_body /= norm

    return R @ (lift_mag * lift_dir_body)


def wind_force(vel_ned, wind_ned, q):
    """Total anisotropic disturbance force, NED newtons: isotropic
    fuselage/frame drag (quadratic in the relative airflow) plus the
    wing's own lift (perpendicular to it) -- see wing_lift_force above
    for why the drag term alone can't represent the second part."""
    rel = np.asarray(wind_ned) - np.asarray(vel_ned)
    speed = np.linalg.norm(rel, axis=-1, keepdims=True)
    drag = 0.5 * RHO * CD_A * speed * rel
    return drag + wing_lift_force(vel_ned, wind_ned, q)


def thrust_from_state(accel_body, q):
    """Recover the NED thrust vector from measured body acceleration.

    The accelerometer measures specific force: everything except gravity.
    In NED that is (T + F_wind)/m, and thrust dominates, so T_ned is
    m * R(q) * a_body with the wind contribution left for the residual to
    explain. Approximate by construction -- that gap is the learning signal.
    """
    R = frames.quat_to_rotmat(q)
    return MASS_KG * (R @ np.asarray(accel_body))


def load_dataset(csv_paths, alpha=0.7, n=16):
    inputs, targets, accels, thrusts = [], [], [], []
    for path in csv_paths:
        df = pd.read_csv(path).dropna()
        # Drop the pre-arm rows: attitude is all zeros before PX4 publishes.
        df = df[(df[['qw', 'qx', 'qy', 'qz']].abs().sum(axis=1) > 0.5)]
        mem = FractionalMemory(alpha=alpha, n=n)
        for _, row in df.iterrows():
            s = np.array([float(row[k]) for k in STATE_KEYS])
            mem.push(s)
            inputs.append(np.concatenate([s, mem.features()]))

            vel = np.array([row.vx, row.vy, row.vz])
            wind = np.array([row.wind_true_n, row.wind_true_e, row.wind_true_d])
            q = np.array([row.qw, row.qx, row.qy, row.qz])
            targets.append(wind_force(vel, wind, q))

            a_body = np.array([row.ax, row.ay, row.az])
            R = frames.quat_to_rotmat(q)
            a_ned = R @ a_body
            a_ned[2] += G  # accelerometer excludes gravity; add it back
            accels.append(a_ned)
            thrusts.append(thrust_from_state(a_body, q))

    t = lambda arr: torch.tensor(np.asarray(arr), dtype=torch.float32)
    return t(inputs), t(targets), t(accels), t(thrusts)


def train(csv_paths, out_path, alpha=0.7, n=16, hidden=96,
          epochs=300, lr=1e-3, lam=0.1, val_frac=0.2, seed=0, device=None):
    torch.manual_seed(seed)
    device = device or ('cuda' if torch.cuda.is_available() else 'cpu')
    x, f_true, accel, thrust = load_dataset(csv_paths, alpha, n)
    x, f_true, accel, thrust = (t.to(device) for t in (x, f_true, accel, thrust))

    # Chronological split: the tail is held out, so we never validate on a
    # gust the network already saw the neighbours of.
    cut = int(len(x) * (1 - val_frac))
    net = WindEstimator(hidden=hidden).to(device)
    opt = torch.optim.Adam(net.parameters(), lr=lr)

    for epoch in range(epochs):
        opt.zero_grad()
        loss = total_loss(net(x[:cut]), f_true[:cut], accel[:cut],
                          thrust[:cut], lam=lam)
        loss.backward()
        opt.step()
        if epoch % 50 == 0:
            with torch.no_grad():
                v = torch.mean((net(x[cut:]) - f_true[cut:]) ** 2)
            print(f'epoch {epoch:4d}  train {float(loss):.4f}  val_mse {float(v):.4f}')

    with torch.no_grad():
        pred = net(x[cut:])
        val_mse = float(torch.mean((pred - f_true[cut:]) ** 2))
        baseline = float(torch.mean(f_true[cut:] ** 2))  # predicting zero

    torch.save({'state_dict': net.state_dict(), 'alpha': alpha,
                'n': n, 'hidden': hidden}, out_path)
    metrics = {'val_mse': val_mse, 'zero_baseline_mse': baseline,
               'skill': 1.0 - val_mse / max(baseline, 1e-9)}
    print(f'saved {out_path}')
    print(f'val_mse={val_mse:.4f}  zero-predictor={baseline:.4f}  '
          f'skill={metrics["skill"]:.3f}')
    return metrics


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csvs', nargs='+')
    ap.add_argument('--out', default=str(
        pathlib.Path(__file__).resolve().parents[1] / 'data' / 'wind_estimator.pt'))
    ap.add_argument('--alpha', type=float, default=0.7)
    # 300 (the old default) simply under-trains this data: swept on the six
    # tricopter trial legs, held-out skill runs 0.650 at 300 epochs, 0.872 at
    # 800, 0.896 at 1500, then falls back to 0.866/0.869 at 3000/5000 as it
    # starts overfitting. 1500 is the top of that curve.
    ap.add_argument('--epochs', type=int, default=1500)
    args = ap.parse_args()
    m = train(args.csvs, args.out, alpha=args.alpha, epochs=args.epochs)
    assert m['skill'] > 0.0, (
        f'model is worse than predicting zero (skill={m["skill"]:.3f}); '
        'do not deploy it')


if __name__ == '__main__':
    main()
