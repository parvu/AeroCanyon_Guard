import numpy as np
import pytest
from aerocanyon import train_pinn

LEVEL = np.array([1.0, 0.0, 0.0, 0.0])


def test_wing_lift_coefficient_is_zero_at_zero_alpha():
    assert train_pinn._wing_lift_coefficient(0.0) == pytest.approx(0.0)


def test_wing_lift_coefficient_is_linear_below_stall():
    alpha = 0.1  # well inside WING_ALPHA_STALL
    cl = train_pinn._wing_lift_coefficient(alpha)
    assert cl == pytest.approx(train_pinn.WING_CLA * alpha)


def test_wing_lift_coefficient_is_odd():
    alpha = 0.15
    assert train_pinn._wing_lift_coefficient(-alpha) == pytest.approx(
        -train_pinn._wing_lift_coefficient(alpha))


def test_wing_lift_coefficient_continuous_at_the_stall_boundary():
    just_below = train_pinn._wing_lift_coefficient(train_pinn.WING_ALPHA_STALL - 1e-6)
    at_stall = train_pinn._wing_lift_coefficient(train_pinn.WING_ALPHA_STALL)
    assert at_stall == pytest.approx(just_below, abs=1e-4)


def test_wing_lift_coefficient_drops_past_stall():
    # cla_stall is negative -- lift collapses past the stall angle rather
    # than continuing to grow, matching a real wing (and gz-sim-lift-drag-
    # system's own model.sdf curve).
    at_stall = train_pinn._wing_lift_coefficient(train_pinn.WING_ALPHA_STALL)
    past_stall = train_pinn._wing_lift_coefficient(train_pinn.WING_ALPHA_STALL + 0.2)
    assert past_stall < at_stall


def test_wing_lift_coefficient_stays_bounded_deep_in_stall():
    # Regression: verified live that a multicopter's actual angle of
    # attack sits near 90 degrees for most ordinary flight (there's no
    # reason its forward axis should align with the resultant airspeed
    # when thrust, not a wing, is what's holding it up). The linear
    # region alone would give an absurd CL at pi/2 rad; the post-stall
    # branch must keep this finite and small instead.
    cl_90deg = train_pinn._wing_lift_coefficient(np.pi / 2)
    assert np.isfinite(cl_90deg)
    assert abs(cl_90deg) < abs(train_pinn.WING_CLA * (np.pi / 2)), (
        'post-stall branch must not just be the unclamped linear curve')


def test_wing_lift_force_is_zero_below_flying_speed():
    out = train_pinn.wing_lift_force(np.array([0.1, 0.0, 0.0]), np.zeros(3), LEVEL)
    assert np.allclose(out, 0.0)


def test_wing_lift_force_is_zero_at_zero_angle_of_attack():
    # Airspeed purely along the body forward axis: alpha=0, so no lift.
    out = train_pinn.wing_lift_force(np.array([15.0, 0.0, 0.0]), np.zeros(3), LEVEL)
    assert np.allclose(out, 0.0, atol=1e-6)


def test_wing_lift_force_is_perpendicular_to_the_airspeed():
    # Positive alpha (relative air arriving partly from below, NED -down):
    # lift must point mostly "up" (NED -z), not along the airspeed vector.
    vel = np.array([15.0, 0.0, -3.0])
    out = train_pinn.wing_lift_force(vel, np.zeros(3), LEVEL)
    assert np.linalg.norm(out) > 0.0
    airspeed = vel  # wind is zero here
    cos_angle = np.dot(out, airspeed) / (np.linalg.norm(out) * np.linalg.norm(airspeed))
    assert abs(cos_angle) < 0.1, 'lift must be ~perpendicular to the relative airflow'


def test_wind_force_includes_both_drag_and_lift():
    vel = np.array([15.0, 0.0, -3.0])
    wind = np.zeros(3)
    total = train_pinn.wind_force(vel, wind, LEVEL)
    drag_only = 0.5 * train_pinn.RHO * train_pinn.CD_A * np.linalg.norm(wind - vel) * (wind - vel)
    lift_only = train_pinn.wing_lift_force(vel, wind, LEVEL)
    assert np.allclose(total, drag_only + lift_only)
    assert np.linalg.norm(lift_only) > 0.0, 'this scenario must actually exercise the lift term'
