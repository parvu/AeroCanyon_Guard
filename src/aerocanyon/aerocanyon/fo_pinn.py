"""Fractional-order physics-informed wind estimator.

Outputs the wind disturbance FORCE, not motor commands. Predicting PWM
directly is both unlearnable from this signal and redundant with PX4's
mixer, so the network estimates the disturbance and the controller turns
that into a feedforward acceleration.

The fractional order enters as Gruenwald-Letnikov coefficients over a ring
buffer of recent states. That weighted history is concatenated to the
input, which is the Caputo memory term from Objective 1 and the mechanism
that damps control jitter at building corners: the estimate cannot swing
instantaneously because it is anchored to weighted past states.

Two loss terms:
  - supervised MSE against the ground-truth wind force
  - a physics residual from rigid-body Newton-Euler

The residual is what makes this a PINN rather than a regressor, and it is
the term that survives to hardware, where ground truth does not exist.
"""
import numpy as np
import torch
import torch.nn as nn

from .constants import G, MASS_KG

STATE_KEYS = ('vx', 'vy', 'vz', 'qw', 'qx', 'qy', 'qz',
              'p', 'q', 'r', 'ax', 'ay', 'az')
STATE_DIM = len(STATE_KEYS)


def gl_coefficients(alpha, n):
    """Gruenwald-Letnikov weights w_k = (-1)^k * binomial(alpha, k).

    Computed by the recurrence w_k = w_{k-1} * (k - 1 - alpha) / k, which
    avoids the factorial overflow the direct binomial form hits well
    before k = 100.
    """
    w = np.empty(int(n), dtype=float)
    w[0] = 1.0
    for k in range(1, int(n)):
        w[k] = w[k - 1] * (k - 1 - alpha) / k
    return w


def state_vector(row):
    """Build the 13-element state from a logged CSV row (dict or Series)."""
    return np.array([float(row[k]) for k in STATE_KEYS])


class FractionalMemory:
    """Ring buffer of past states, summarised by GL memory weights.

    features() returns sum_k w_k * state[t - k], i.e. the discrete
    fractional derivative of the state history. alpha = 1 recovers a plain
    first difference; alpha -> 0 recovers the raw current state.
    """

    def __init__(self, alpha=0.7, n=16, dim=STATE_DIM):
        self.alpha = float(alpha)
        self.n = int(n)
        self.dim = int(dim)
        self.weights = gl_coefficients(self.alpha, self.n)
        self.buffer = np.zeros((self.n, self.dim))

    def push(self, state):
        self.buffer = np.roll(self.buffer, 1, axis=0)
        self.buffer[0] = np.asarray(state, dtype=float)

    def features(self):
        return self.weights @ self.buffer

    def reset(self):
        self.buffer[:] = 0.0


class WindEstimator(nn.Module):
    """State + fractional memory -> wind disturbance force in NED (newtons)."""

    def __init__(self, hidden=96):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(2 * STATE_DIM, hidden), nn.Tanh(),
            nn.Linear(hidden, hidden), nn.Tanh(),
            nn.Linear(hidden, 3),
        )

    def forward(self, x):
        return self.net(x)


def physics_residual(f_hat, accel, thrust_ned, mass=MASS_KG):
    """Rigid-body Newton-Euler residual, mean squared over the batch.

    m*a = T + m*g + F_wind, with gravity +G on NED down. A perfect
    estimate drives this to zero.
    """
    gravity = torch.zeros_like(accel)
    gravity[:, 2] = mass * G
    return torch.mean((mass * accel - thrust_ned - gravity - f_hat) ** 2)


def total_loss(f_hat, f_true, accel, thrust_ned, lam=0.1, mass=MASS_KG):
    """Supervised MSE plus the weighted physics residual."""
    data = torch.mean((f_hat - f_true) ** 2)
    physics = physics_residual(f_hat, accel, thrust_ned, mass)
    return data + lam * physics


if __name__ == '__main__':
    # Self-check: an untrained forward pass and a finite loss.
    net = WindEstimator()
    mem = FractionalMemory()
    s = np.zeros(STATE_DIM)
    for _ in range(20):
        mem.push(s)
    x = torch.tensor(np.concatenate([s, mem.features()]),
                     dtype=torch.float32).unsqueeze(0)
    f = net(x)
    thrust = torch.tensor([[0.0, 0.0, -MASS_KG * G]])
    loss = total_loss(f, torch.zeros(1, 3), torch.zeros(1, 3), thrust)
    assert f.shape == (1, 3), f.shape
    assert torch.isfinite(loss), loss
    print(f'ok: f_hat={f.detach().numpy().round(3)} loss={float(loss):.4f}')
