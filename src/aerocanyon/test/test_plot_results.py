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

from aerocanyon import canyon_geometry as cg
from aerocanyon.plot_results import building_plot_rect, lateral_deviation

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


def test_building_plot_rect_matches_the_trajectorys_local_frame():
    # Verified live: PX4's local NED origin sits at CANYON_ENTRY (the
    # vehicle's spawn point), not the world's absolute origin -- at
    # mission start (target pinned at CANYON_ENTRY, local east=-90) the
    # vehicle's actual logged position reads local east=0. So a building
    # sitting exactly at CANYON_ENTRY's world position must plot at local
    # (0, 0), matching where the trajectory itself starts, not at its own
    # raw ENU coordinates.
    entry_building = cg.BUILDINGS[0]._replace(
        cx=float(cg.CANYON_ENTRY[0]), cy=float(cg.CANYON_ENTRY[1]))
    x, y, w, h = building_plot_rect(entry_building)
    assert abs((x + w / 2) - 0.0) < 1e-9, 'building centred on the entry must plot at local east=0'
    assert abs((y + h / 2) - 0.0) < 1e-9, 'building centred on the entry must plot at local north=0'


def test_building_plot_rect_preserves_relative_spacing_between_buildings():
    # The shift is a constant offset -- it must not distort the buildings'
    # positions relative to EACH OTHER, only move all of them together.
    rects = [building_plot_rect(b) for b in cg.BUILDINGS]
    for b, (x, y, w, h) in zip(cg.BUILDINGS, rects):
        assert w == b.sx and h == b.sy
    # tower_1 (index 2 or 3, the middle pair) sits 45m east of tower_0 in
    # world coordinates; that spacing must survive the shift unchanged.
    tower_0 = next(b for b in cg.BUILDINGS if b.name == 'tower_0_n')
    tower_1 = next(b for b in cg.BUILDINGS if b.name == 'tower_1_n')
    x0, *_ = building_plot_rect(tower_0)
    x1, *_ = building_plot_rect(tower_1)
    assert abs((x1 - x0) - (tower_1.cx - tower_0.cx)) < 1e-9
