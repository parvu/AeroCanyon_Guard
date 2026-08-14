import numpy as np
import pytest
from aerocanyon import canyon_geometry as cg
from aerocanyon import cbf_filter, frames

LEVEL = np.array([1.0, 0.0, 0.0, 0.0])
# Use a position outside the canyon where barrier constraints are weak
CLEAR_NED = frames.enu_to_ned(np.array([-100.0, 0.0, 25.0]))


def test_safe_command_in_open_air_passes_through_unchanged():
    f = cbf_filter.CBFFilter()
    u = np.array([0.5, 0.0, 0.0])
    out, info = f.filter(u, CLEAR_NED, np.array([12.0, 0.0, 0.0]),
                         np.zeros(3), LEVEL)
    assert np.allclose(out, u, atol=1e-3)
    assert not info['active']
    assert info['feasible']


def test_command_driving_into_a_building_is_modified():
    f = cbf_filter.CBFFilter()
    b = cg.BUILDINGS[0]
    # Just inside the canyon, next to a tower's -y face, moving toward it.
    p_enu = np.array([b.cx, b.cy - b.sy / 2 - 3.0, 25.0])
    p_ned = frames.enu_to_ned(p_enu)
    v_ned = frames.enu_to_ned(np.array([0.0, 4.0, 0.0]))   # toward the wall
    u_ned = frames.enu_to_ned(np.array([0.0, 6.0, 0.0]))   # accelerating into it
    out, info = f.filter(u_ned, p_ned, v_ned, np.zeros(3), LEVEL)
    assert info['active'], 'the filter must intervene'
    # The outward (ENU -y here) component must be less aggressive.
    assert frames.ned_to_enu(out)[1] < frames.ned_to_enu(u_ned)[1]


def test_filter_reports_the_minimum_barrier_value():
    f = cbf_filter.CBFFilter()
    _, info = f.filter(np.zeros(3), CLEAR_NED, np.zeros(3), np.zeros(3), LEVEL)
    assert np.isfinite(info['h_min'])


def test_angle_of_attack_is_zero_in_level_forward_flight():
    a = cbf_filter.angle_of_attack(np.array([15.0, 0.0, 0.0]), np.zeros(3), LEVEL)
    assert a == pytest.approx(0.0, abs=1e-6)


def test_angle_of_attack_grows_when_descending_air_hits_from_below():
    """Upward relative flow (NED -down) raises alpha."""
    a = cbf_filter.angle_of_attack(np.array([15.0, 0.0, -3.0]), np.zeros(3), LEVEL)
    assert a > 0.05


def test_wind_changes_the_angle_of_attack():
    v = np.array([15.0, 0.0, 0.0])
    still = cbf_filter.angle_of_attack(v, np.zeros(3), LEVEL)
    updraft = cbf_filter.angle_of_attack(v, np.array([0.0, 0.0, -5.0]), LEVEL)
    assert not np.isclose(still, updraft)


def test_slew_limit_caps_a_sudden_command_jump():
    f = cbf_filter.CBFFilter()
    f.filter(np.zeros(3), CLEAR_NED, np.zeros(3), np.zeros(3), LEVEL)
    huge = np.array([500.0, 0.0, 0.0])
    out, info = f.filter(huge, CLEAR_NED, np.zeros(3), np.zeros(3), LEVEL)
    assert np.linalg.norm(out) < np.linalg.norm(huge)
    assert info['active']


def test_infeasible_solve_returns_the_last_safe_command_and_says_so():
    f = cbf_filter.CBFFilter()
    safe = np.array([1.0, 0.0, 0.0])
    f.filter(safe, CLEAR_NED, np.zeros(3), np.zeros(3), LEVEL)
    f._force_infeasible = True   # test hook
    out, info = f.filter(np.array([9.0, 9.0, 9.0]), CLEAR_NED,
                         np.zeros(3), np.zeros(3), LEVEL)
    assert not info['feasible']
    assert np.allclose(out, f.last_safe)


def test_filter_never_returns_a_non_finite_command():
    f = cbf_filter.CBFFilter()
    for u in (np.zeros(3), np.array([100.0, -100.0, 50.0]), np.array([1e-9] * 3)):
        out, _ = f.filter(u, CLEAR_NED, np.array([12.0, 1.0, 0.0]),
                          np.array([5.0, 2.0, 0.0]), LEVEL)
        assert np.all(np.isfinite(out))


def test_solve_is_fast_enough_for_the_control_loop():
    import time
    f = cbf_filter.CBFFilter()
    b = cg.BUILDINGS[0]
    p_ned = frames.enu_to_ned(np.array([b.cx, b.cy - b.sy / 2 - 3.0, 25.0]))
    v_ned = frames.enu_to_ned(np.array([0.0, 4.0, 0.0]))
    u_ned = frames.enu_to_ned(np.array([0.0, 6.0, 0.0]))
    t = time.perf_counter()
    for _ in range(100):
        f.filter(u_ned, p_ned, v_ned, np.zeros(3), LEVEL)
    per_call_ms = (time.perf_counter() - t) / 100 * 1000
    assert per_call_ms < 10.0, f'{per_call_ms:.2f} ms is too slow for 50 Hz'
