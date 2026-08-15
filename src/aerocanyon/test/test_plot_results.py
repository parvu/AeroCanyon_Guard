"""lateral_deviation() previously measured RMS of the NED east column, but
the mission travels along NED east (see mission.py / canyon_geometry.py --
the corridor runs along Gazebo ENU +x), which makes east the ALONG-track
axis, not the cross-track one. That bug reported "RMS lateral deviation"
numbers in the tens-to-hundreds of metres for a canyon whose two rows of
buildings are only ~24 m apart -- a number that large was always the
mission's own along-track progress, not a wobble. North is the correct
cross-track axis.
"""
import numpy as np
import pandas as pd

from aerocanyon.plot_results import lateral_deviation

N = 200


def _flying_df(north, east):
    return pd.DataFrame({
        't': np.linspace(0, 20, N),
        'x': north,
        'y': east,
        'z': -25.0,  # below the _flying() airborne threshold of -2.0
        'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0,
    })


def test_lateral_deviation_is_the_north_axis_not_east():
    # Large along-track (east) progress, small constant lateral (north)
    # offset -- exactly what a real canyon transit under a crosswind
    # looks like. The old buggy version would report ~115 m (RMS of the
    # east column) here instead of 3 m.
    df = _flying_df(north=np.full(N, 3.0), east=np.linspace(-90.0, 110.0, N))
    assert abs(lateral_deviation(df) - 3.0) < 1e-9


def test_lateral_deviation_ignores_along_track_progress():
    # Zero real lateral wobble: north stays at 0 the whole flight no
    # matter how far east it travels.
    df = _flying_df(north=np.zeros(N), east=np.linspace(-90.0, 110.0, N))
    assert lateral_deviation(df) < 1e-9
