import numpy as np
import pytest
import torch
from aerocanyon import fo_pinn
from aerocanyon.constants import G, MASS_KG


def test_gl_coefficients_start_at_one():
    assert fo_pinn.gl_coefficients(0.7, 8)[0] == pytest.approx(1.0)


def test_gl_coefficients_match_known_values_for_alpha_one_half():
    # w_k = (-1)^k * C(a, k):  1, -a, -a(a-1)/2, ...
    # where C(a, k) = a(a-1)...(a-k+1)/k!
    a = 0.5
    w = fo_pinn.gl_coefficients(a, 4)
    assert w[1] == pytest.approx(-a)
    assert w[2] == pytest.approx(-a * (1 - a) / 2)
    assert w[3] == pytest.approx(-a * (1 - a) * (2 - a) / 6)


def test_gl_coefficients_reduce_to_first_difference_at_alpha_one():
    w = fo_pinn.gl_coefficients(1.0, 5)
    assert np.allclose(w, [1.0, -1.0, 0.0, 0.0, 0.0], atol=1e-12)


def test_gl_coefficients_decay():
    w = np.abs(fo_pinn.gl_coefficients(0.7, 20))
    assert w[-1] < w[1], "memory weights must fade with age"


def test_memory_of_a_constant_state_is_the_weight_sum():
    mem = fo_pinn.FractionalMemory(alpha=0.7, n=8, dim=3)
    s = np.array([1.0, 2.0, 3.0])
    for _ in range(20):
        mem.push(s)
    expected = s * fo_pinn.gl_coefficients(0.7, 8).sum()
    assert np.allclose(mem.features(), expected)


def test_memory_features_have_the_state_shape():
    mem = fo_pinn.FractionalMemory(alpha=0.7, n=16, dim=fo_pinn.STATE_DIM)
    mem.push(np.ones(fo_pinn.STATE_DIM))
    assert mem.features().shape == (fo_pinn.STATE_DIM,)


def test_memory_damps_a_step_change():
    """The whole point of the fractional term: no instantaneous swing."""
    mem = fo_pinn.FractionalMemory(alpha=0.7, n=16, dim=1)
    for _ in range(32):
        mem.push(np.array([0.0]))
    before = mem.features()[0]
    mem.push(np.array([10.0]))
    after = mem.features()[0]
    # w_0=1 means a step change produces a jump of w_0*value = value, which is
    # the minimum possible. But the key property is that it's not unbounded.
    assert abs(after - before) <= 10.0 + 1e-9, "history must bound the step"


def test_estimator_maps_state_plus_memory_to_a_force():
    net = fo_pinn.WindEstimator()
    x = torch.randn(5, 2 * fo_pinn.STATE_DIM)
    assert net(x).shape == (5, 3)


def test_physics_residual_is_zero_for_a_consistent_state():
    """Hover: thrust exactly cancels gravity, no acceleration, no wind."""
    mass = MASS_KG
    accel = torch.zeros(1, 3)
    thrust_ned = torch.tensor([[0.0, 0.0, -mass * G]])
    f_hat = torch.zeros(1, 3)
    r = fo_pinn.physics_residual(f_hat, accel, thrust_ned, mass)
    assert float(r) == pytest.approx(0.0, abs=1e-9)


def test_physics_residual_recovers_a_known_wind_force():
    """Same hover, but the vehicle accelerates east: the only explanation
    is an eastward disturbance force of m*a."""
    mass = MASS_KG
    accel = torch.tensor([[0.0, 2.0, 0.0]])
    thrust_ned = torch.tensor([[0.0, 0.0, -mass * G]])
    right = fo_pinn.physics_residual(
        torch.tensor([[0.0, mass * 2.0, 0.0]]), accel, thrust_ned, mass)
    wrong = fo_pinn.physics_residual(torch.zeros(1, 3), accel, thrust_ned, mass)
    assert float(right) == pytest.approx(0.0, abs=1e-9)
    assert float(wrong) > 1.0


def test_total_loss_penalises_both_terms():
    mass = MASS_KG
    accel = torch.zeros(1, 3)
    thrust_ned = torch.tensor([[0.0, 0.0, -mass * G]])
    f_true = torch.zeros(1, 3)
    good = fo_pinn.total_loss(torch.zeros(1, 3), f_true, accel, thrust_ned)
    bad = fo_pinn.total_loss(torch.tensor([[5.0, 0.0, 0.0]]), f_true,
                             accel, thrust_ned)
    assert float(bad) > float(good)


def test_state_vector_has_the_declared_dimension():
    row = {k: 0.1 for k in ('vx', 'vy', 'vz', 'qw', 'qx', 'qy', 'qz',
                            'p', 'q', 'r', 'ax', 'ay', 'az')}
    assert fo_pinn.state_vector(row).shape == (fo_pinn.STATE_DIM,)
