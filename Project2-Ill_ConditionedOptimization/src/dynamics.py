"""Clohessy-Wiltshire dynamics and exact zero-order-hold propagation."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.linalg import expm

FloatArray = NDArray[np.float64]


def mean_motion(mu: float, reference_radius: float) -> float:
    """Return circular-orbit mean motion in radians per second."""

    if not np.isfinite(mu) or mu <= 0 or not np.isfinite(reference_radius) or reference_radius <= 0:
        raise ValueError("mu and reference_radius must be finite and positive")
    return float(np.sqrt(mu / reference_radius**3))


def continuous_matrices(n: float) -> tuple[FloatArray, FloatArray]:
    """Return continuous CW state and control matrices for positive mean motion ``n``."""

    if not np.isfinite(n) or n <= 0:
        raise ValueError("mean motion must be finite and positive")
    a = np.zeros((6, 6), dtype=float)
    a[:3, 3:] = np.eye(3)
    a[3, 0] = 3 * n**2
    a[3, 4] = 2 * n
    a[4, 3] = -2 * n
    a[5, 2] = -(n**2)
    b = np.zeros((6, 3), dtype=float)
    b[3:, :] = np.eye(3)
    return a, b


def augmented_exponential_discretization(
    n: float, sample_time: float
) -> tuple[FloatArray, FloatArray]:
    """Return exact zero-order-hold matrices from an augmented matrix exponential."""

    if not np.isfinite(sample_time) or sample_time < 0:
        raise ValueError("sample_time must be finite and nonnegative")
    a, b = continuous_matrices(n)
    augmented = np.zeros((9, 9), dtype=float)
    augmented[:6, :6] = a
    augmented[:6, 6:] = b
    transition = expm(augmented * sample_time)
    return transition[:6, :6], transition[:6, 6:]


def discretize_cw(n: float, sample_time: float) -> tuple[FloatArray, FloatArray]:
    """Return exact analytic CW zero-order-hold matrices ``Phi`` and ``Gamma``."""

    if not np.isfinite(sample_time) or sample_time < 0:
        raise ValueError("sample_time must be finite and nonnegative")
    if not np.isfinite(n) or n <= 0:
        raise ValueError("mean motion must be finite and positive")
    if sample_time == 0 or abs(n * sample_time) < 1e-4:
        return augmented_exponential_discretization(n, sample_time)

    h = sample_time
    c = np.cos(n * h)
    s = np.sin(n * h)
    phi = np.array(
        [
            [4 - 3 * c, 0, 0, s / n, 2 * (1 - c) / n, 0],
            [6 * (s - n * h), 1, 0, -2 * (1 - c) / n, (4 * s - 3 * n * h) / n, 0],
            [0, 0, c, 0, 0, s / n],
            [3 * n * s, 0, 0, c, 2 * s, 0],
            [-6 * n * (1 - c), 0, 0, -2 * s, 4 * c - 3, 0],
            [0, 0, -n * s, 0, 0, c],
        ],
        dtype=float,
    )
    gamma = np.array(
        [
            [(1 - c) / n**2, 2 * (n * h - s) / n**2, 0],
            [-2 * (n * h - s) / n**2, 4 * (1 - c) / n**2 - 1.5 * h**2, 0],
            [0, 0, (1 - c) / n**2],
            [s / n, 2 * (1 - c) / n, 0],
            [-2 * (1 - c) / n, 4 * s / n - 3 * h, 0],
            [0, 0, s / n],
        ],
        dtype=float,
    )
    return phi, gamma


def simulate(
    initial_state: ArrayLike,
    controls: ArrayLike,
    phi: ArrayLike,
    gamma: ArrayLike,
) -> FloatArray:
    """Propagate a control sequence and return states with shape ``(N+1, 6)``."""

    state0 = np.asarray(initial_state, dtype=float)
    control_array = np.asarray(controls, dtype=float)
    phi_array = np.asarray(phi, dtype=float)
    gamma_array = np.asarray(gamma, dtype=float)
    if state0.shape != (6,) or control_array.ndim != 2 or control_array.shape[1] != 3:
        raise ValueError("expected initial_state (6,) and controls (N, 3)")
    if phi_array.shape != (6, 6) or gamma_array.shape != (6, 3):
        raise ValueError("expected phi (6, 6) and gamma (6, 3)")
    if not all(
        np.all(np.isfinite(item)) for item in (state0, control_array, phi_array, gamma_array)
    ):
        raise ValueError("dynamics inputs must be finite")
    states = np.empty((control_array.shape[0] + 1, 6), dtype=float)
    states[0] = state0
    for index, control in enumerate(control_array):
        states[index + 1] = phi_array @ states[index] + gamma_array @ control
    return states
