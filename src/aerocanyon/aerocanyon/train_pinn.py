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


def wind_force(vel_ned, wind_ned):
    """Quadratic drag force from the relative airflow, NED newtons."""
    rel = np.asarray(wind_ned) - np.asarray(vel_ned)
    speed = np.linalg.norm(rel, axis=-1, keepdims=True)
    return 0.5 * RHO * CD_A * speed * rel


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
            targets.append(wind_force(vel, wind))

            a_body = np.array([row.ax, row.ay, row.az])
            q = np.array([row.qw, row.qx, row.qy, row.qz])
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
    ap.add_argument('--epochs', type=int, default=300)
    args = ap.parse_args()
    m = train(args.csvs, args.out, alpha=args.alpha, epochs=args.epochs)
    assert m['skill'] > 0.0, (
        f'model is worse than predicting zero (skill={m["skill"]:.3f}); '
        'do not deploy it')


if __name__ == '__main__':
    main()
