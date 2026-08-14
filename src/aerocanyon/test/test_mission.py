import numpy as np
from aerocanyon import canyon_geometry as cg
from aerocanyon import frames
from aerocanyon.mission import Mission


def test_mission_starts_at_the_canyon_entry():
    m = Mission()
    p, done = m.target(0.0)
    assert not done
    assert np.allclose(p, frames.enu_to_ned(cg.CANYON_ENTRY), atol=1e-6)


def test_mission_ends_at_the_canyon_exit():
    m = Mission()
    p, done = m.target(1e6)
    assert done
    assert np.allclose(p, frames.enu_to_ned(cg.CANYON_EXIT), atol=1e-6)


def test_mission_is_monotonic_along_the_canyon():
    m = Mission()
    # NED north is ENU +x, so the north component must increase monotonically.
    north = [m.target(t)[0][0] for t in np.linspace(0.0, 200.0, 400)]
    assert all(b >= a - 1e-9 for a, b in zip(north, north[1:]))


def test_mission_stays_clear_of_the_buildings():
    m = Mission()
    for t in np.linspace(0.0, 200.0, 400):
        p_ned, _ = m.target(t)
        d, _ = cg.distance_and_normal(frames.ned_to_enu(p_ned))
        assert d > 2.0, f'setpoint at t={t} is {d:.2f} m from a building'


def test_mission_is_deterministic():
    a, b = Mission(), Mission()
    for t in (0.0, 12.5, 60.0, 130.0):
        assert np.allclose(a.target(t)[0], b.target(t)[0])
