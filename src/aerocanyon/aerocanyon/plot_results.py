"""Produce the two proposal figures from a pair of trial CSVs."""
import argparse
import pathlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from . import canyon_geometry as cg  # noqa: E402


def _flying(df):
    """Rows where the vehicle is actually airborne, tracking, and still
    within the canyon transit itself -- not the return leg.

    The return leg (baseline's native RTL, which doesn't reliably
    navigate home in this SITL config and can drift for hundreds of
    metres; treatment's own wind-compensated fly-home) isn't part of the
    baseline/treatment comparison being measured -- see controller_node's
    RETURN_CLEARANCE_M and History.md. Cut the data off a little past the
    canyon exit, in local NED east (== distance travelled from
    CANYON_ENTRY, see building_plot_rect below), so a few hundred metres of
    aimless post-transit wandering can't swamp the actual ~200 m transit
    in these figures."""
    exit_east = float(cg.CANYON_EXIT[0]) - float(cg.CANYON_ENTRY[0])
    transit_margin_m = 5.0
    return df[(df[['qw', 'qx', 'qy', 'qz']].abs().sum(axis=1) > 0.5)
              & (df.z < -2.0)
              & (df.y <= exit_east + transit_margin_m)]


def lateral_deviation(df):
    """RMS cross-track error. The mission runs along NED east (see
    Mission.direction / canyon_geometry.BUILDINGS -- the corridor is laid
    out along Gazebo ENU +x), so the north component IS the lateral
    deviation, not east."""
    return float(np.sqrt(np.mean(_flying(df).x ** 2)))


def building_plot_rect(b):
    """(x, y, width, height) for building b, in the same local-NED-frame
    (east, north) coordinates the trajectory plot uses.

    x/y in the trial CSV are PX4's LOCAL NED position, origin at wherever
    the EKF initialised -- which, since the vehicle spawns at CANYON_ENTRY
    (see run_trial.SPAWN_XYZ), is CANYON_ENTRY itself, not the world's
    absolute origin. Verified live: at mission start (target pinned at
    CANYON_ENTRY) the vehicle's actual logged position reads local
    east=0. canyon_geometry.BUILDINGS is in world-absolute ENU, so it has
    to be shifted by CANYON_ENTRY's own position to land in the same
    frame as the trajectory -- otherwise the
    buildings render ~90 m away from where the vehicle actually was in
    Gazebo.

    b.cx/b.cy are ENU east/north, which are numerically identical to NED
    east/north (frames.ned_to_enu only reorders components and negates
    up->down -- it never changes an east or north value), matching the
    trajectory plot's (east, north) axis order.
    """
    east0, north0 = float(cg.CANYON_ENTRY[0]), float(cg.CANYON_ENTRY[1])
    return (b.cx - east0 - b.sx / 2, b.cy - north0 - b.sy / 2, b.sx, b.sy)


def comparison_figure(base, treat, out):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5),
                                   gridspec_kw={'width_ratios': [2, 1]})

    for b in cg.BUILDINGS:
        x, y, w, h = building_plot_rect(b)
        ax1.add_patch(plt.Rectangle((x, y), w, h, color='0.75', zorder=0))

    bf, tf = _flying(base), _flying(treat)
    ax1.plot(bf.y, bf.x, label='PX4 baseline', lw=2, color='#c1443c')
    ax1.plot(tf.y, tf.x, label='FO-PINN + CBF', lw=2, color='#2b6cb0')
    ax1.axhline(0.0, ls='--', c='0.4', lw=1, label='reference path')
    ax1.set_xlabel('east (along canyon) [m]')
    ax1.set_ylabel('north (lateral) [m]')
    ax1.set_title('Canyon transit under urban wind')
    ax1.legend()
    ax1.set_aspect('equal')

    db, dt = lateral_deviation(base), lateral_deviation(treat)
    reduction = 100.0 * (db - dt) / db if db > 0 else 0.0
    ax2.bar(['baseline', 'FO-PINN + CBF'], [db, dt],
            color=['#c1443c', '#2b6cb0'])
    ax2.set_ylabel('RMS lateral deviation [m]')
    ax2.set_title(f'{reduction:.0f}% reduction')
    for i, v in enumerate((db, dt)):
        ax2.text(i, v, f'{v:.2f}', ha='center', va='bottom')

    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f'wrote {out}')
    return {'baseline_rms': db, 'treatment_rms': dt, 'reduction_pct': reduction}


def intervention_figure(treat, out):
    df = _flying(treat)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)

    ax1.plot(df.t, df.cbf_h_obstacle, color='#2b6cb0', lw=1.5,
             label='obstacle barrier value $h$')
    ax1.axhline(0.0, color='#c1443c', ls='--', lw=1.5,
                label='barrier boundary $h=0$')
    ax1.fill_between(df.t, df.cbf_h_obstacle.min(), 0.0, color='#c1443c', alpha=0.12)
    ax1.set_ylabel('$h$ [m]')
    ax1.legend(loc='upper right')
    ax1.set_title('CBF safety filter activity during canyon transit')

    ax2.fill_between(df.t, 0, df.cbf_active, step='mid',
                     color='#c1443c', alpha=0.6)
    ax2.set_ylabel('filter active')
    ax2.set_xlabel('mission time [s]')
    ax2.set_ylim(-0.05, 1.15)
    ax2.set_yticks([0, 1])

    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f'wrote {out}')
    pct = 100.0 * float(df.cbf_active.mean())
    print(f'filter active for {pct:.1f}% of the flight; '
          f'closest approach to a building: h={df.cbf_h_obstacle.min():.2f} m')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trial', default='compare')
    ap.add_argument('--trials-dir', default='trials')
    ap.add_argument('--out-dir', default='figures')
    args = ap.parse_args()

    td = pathlib.Path(args.trials_dir)
    out = pathlib.Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    base = pd.read_csv(td / f'{args.trial}_baseline.csv')
    treat = pd.read_csv(td / f'{args.trial}_treatment.csv')

    m = comparison_figure(base, treat, out / 'comparison.png')
    intervention_figure(treat, out / 'cbf_intervention.png')
    print(f"\nRMS lateral deviation: baseline {m['baseline_rms']:.2f} m, "
          f"treatment {m['treatment_rms']:.2f} m, "
          f"reduction {m['reduction_pct']:.1f}%")


if __name__ == '__main__':
    main()
