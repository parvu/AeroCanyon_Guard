import numpy as np
from aerocanyon import canyon_geometry as cg


def test_six_buildings_forming_two_rows():
    assert len(cg.BUILDINGS) == 6
    ys = sorted({b.cy for b in cg.BUILDINGS})
    assert len(ys) == 2, "buildings should form two rows facing each other"
    assert ys[0] < 0 < ys[1], "the canyon axis should run along y = 0"


def test_point_in_canyon_centre_is_clear_of_buildings():
    d, _ = cg.distance_and_normal(np.array([0.0, 0.0, 10.0]))
    assert d > 5.0


def test_distance_shrinks_as_we_approach_a_wall():
    far, _ = cg.distance_and_normal(np.array([0.0, 0.0, 10.0]))
    near, _ = cg.distance_and_normal(np.array([0.0, 8.0, 10.0]))
    assert near < far


def test_normal_points_away_from_the_nearest_building():
    b = cg.BUILDINGS[0]
    # A point just outside the building's -y face, at mid height.
    p = np.array([b.cx, b.cy + b.sy / 2 + 1.0, b.sz / 2])
    d, n = cg.distance_and_normal(p)
    assert np.isclose(d, 1.0, atol=1e-6)
    assert np.allclose(n, np.array([0.0, 1.0, 0.0]), atol=1e-6)
    assert np.isclose(np.linalg.norm(n), 1.0)


def test_point_above_all_buildings_is_clear():
    tallest = max(b.sz for b in cg.BUILDINGS)
    d, _ = cg.distance_and_normal(np.array([0.0, 0.0, tallest + 20.0]))
    assert d > 10.0


def test_sdf_contains_every_building():
    sdf = cg.to_sdf()
    for b in cg.BUILDINGS:
        assert f'name="{b.name}"' in sdf


def test_entry_and_exit_are_on_the_canyon_axis_outside_the_buildings():
    xs = [b.cx for b in cg.BUILDINGS]
    assert cg.CANYON_ENTRY[0] < min(xs)
    assert cg.CANYON_EXIT[0] > max(xs)
    assert np.isclose(cg.CANYON_ENTRY[1], 0.0)
    assert np.isclose(cg.CANYON_EXIT[1], 0.0)
