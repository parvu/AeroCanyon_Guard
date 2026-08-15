import numpy as np
import pytest
from aerocanyon import canyon_field as cf
from aerocanyon import canyon_geometry as cg


def test_log_law_is_zero_at_roughness_height():
    assert cf.log_law(1.0, u_ref=10.0, z_ref=30.0, z0=1.0) == pytest.approx(0.0)


def test_log_law_hits_reference_speed_at_reference_height():
    assert cf.log_law(30.0, u_ref=10.0, z_ref=30.0, z0=1.0) == pytest.approx(10.0)


def test_log_law_increases_with_height():
    speeds = [cf.log_law(z) for z in (5.0, 15.0, 30.0, 60.0)]
    assert speeds == sorted(speeds)


def test_log_law_clamps_below_roughness_instead_of_going_negative():
    assert cf.log_law(0.1, z0=1.0) >= 0.0


def test_generated_grid_has_the_declared_shape():
    field, meta = cf.generate(nx=10, ny=8, nz=6)
    assert field.shape == (10, 8, 6, 3)
    assert meta['shape'] == [10, 8, 6]
    assert len(meta['origin']) == 3 and len(meta['spacing']) == 3


def test_canyon_throat_is_faster_than_open_air_at_the_same_height():
    """Channeling: flow squeezed between the towers accelerates."""
    field, meta = cf.generate(nx=40, ny=30, nz=16)
    grid = cf.WindGrid(field, meta)
    z = 20.0
    throat = np.linalg.norm(grid.at(np.array([0.0, 0.0, z])))
    open_air = np.linalg.norm(grid.at(np.array([0.0, 120.0, z])))
    assert throat > open_air


def test_lookup_clamps_outside_the_grid_rather_than_raising():
    field, meta = cf.generate(nx=10, ny=8, nz=6)
    grid = cf.WindGrid(field, meta)
    far = grid.at(np.array([1e6, 1e6, 1e6]))
    assert far.shape == (3,) and np.all(np.isfinite(far))


def test_lookup_returns_the_stored_value_at_a_cell_centre():
    field, meta = cf.generate(nx=10, ny=8, nz=6)
    grid = cf.WindGrid(field, meta)
    o, s = np.array(meta['origin']), np.array(meta['spacing'])
    p = o + s * np.array([3, 2, 4])
    assert np.allclose(grid.at(p), field[3, 2, 4], atol=1e-9)


def test_wind_inside_a_building_is_near_zero():
    field, meta = cf.generate(nx=40, ny=30, nz=16)
    grid = cf.WindGrid(field, meta)
    b = cg.BUILDINGS[0]
    inside = grid.at(np.array([b.cx, b.cy, b.sz / 2.0]))
    assert np.linalg.norm(inside) < 1.0


def test_dryden_is_zero_mean_and_reproducible():
    # This is a slow AR(1) process (tau = length_scale/airspeed): at the
    # mission's actual 15 m/s cruise speed, tau=13.3s, so 4000 steps at
    # dt=0.02 (80s) is only ~6 correlation times, and the sample mean's own
    # standard deviation works out to ~0.87 -- well above a naive 0.5
    # threshold, so that version of this test failed for something like
    # half of all seeds (confirmed live with seed=7). Using a much higher
    # airspeed here decorrelates gusts faster (shorter tau), giving far
    # more effective independent samples in the same 4000 steps: at
    # airspeed=150, tau=1.33s and the sample mean's std drops to ~0.27, so
    # a threshold of 1.0 (~3.6 sigma) is solidly non-flaky while still
    # catching a genuinely biased generator (e.g. a bug adding a constant
    # offset). airspeed here is a test-only decorrelation knob, not meant
    # to be physically realistic.
    a = cf.DrydenGust(dt=0.02, seed=7)
    b = cf.DrydenGust(dt=0.02, seed=7)
    sa = np.array([a.step(150.0) for _ in range(4000)])
    sb = np.array([b.step(150.0) for _ in range(4000)])
    assert np.allclose(sa, sb), "same seed must reproduce the same gust train"
    assert np.all(np.abs(sa.mean(axis=0)) < 1.0)
    assert np.all(sa.std(axis=0) > 0.1), "gusts must actually vary"


def test_dryden_is_correlated_in_time():
    """The whole point of Dryden over white noise: successive samples relate."""
    g = cf.DrydenGust(dt=0.02, seed=1)
    s = np.array([g.step(15.0) for _ in range(4000)])[:, 0]
    lag1 = np.corrcoef(s[:-1], s[1:])[0, 1]
    assert lag1 > 0.8
