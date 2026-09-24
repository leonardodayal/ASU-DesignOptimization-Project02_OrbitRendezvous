"""Terminal controllability and null-space constraint utilities."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

FloatArray = NDArray[np.float64]


def terminal_controllability(phi: ArrayLike, gamma: ArrayLike, num_intervals: int) -> FloatArray:
    """Return the terminal controllability matrix from formulation equation (23)."""

    phi_array = np.asarray(phi, dtype=float)
    gamma_array = np.asarray(gamma, dtype=float)
    if phi_array.shape != (6, 6) or gamma_array.shape != (6, 3):
        raise ValueError("expected phi (6, 6) and gamma (6, 3)")
    if num_intervals <= 0:
        raise ValueError("num_intervals must be positive")
    blocks = [
        np.linalg.matrix_power(phi_array, num_intervals - 1 - index) @ gamma_array
        for index in range(num_intervals)
    ]
    return np.hstack(blocks)


def numerical_rank_tolerance(matrix: ArrayLike) -> float:
    """Return the confirmed machine-epsilon singular-value rank threshold."""

    array = np.asarray(matrix, dtype=float)
    largest = np.linalg.svd(array, compute_uv=False)[0] if array.size else 0.0
    return float(max(array.shape, default=0) * np.finfo(float).eps * largest)


@dataclass(frozen=True)
class NullSpaceReduction:
    """Particular solution and orthonormal basis for exact terminal constraints."""

    particular: FloatArray
    basis: FloatArray
    rank: int
    tolerance: float
    residual: float


def terminal_null_space(
    phi: ArrayLike,
    gamma: ArrayLike,
    initial_state: ArrayLike,
    num_intervals: int,
) -> NullSpaceReduction:
    """Build ``U = U_p + Z*w`` for exact terminal rendezvous."""

    phi_array = np.asarray(phi, dtype=float)
    initial = np.asarray(initial_state, dtype=float)
    controllability = terminal_controllability(phi_array, gamma, num_intervals)
    target = -np.linalg.matrix_power(phi_array, num_intervals) @ initial
    u, singular_values, vh = np.linalg.svd(controllability, full_matrices=True)
    tolerance = numerical_rank_tolerance(controllability)
    rank = int(np.sum(singular_values > tolerance))
    if rank < 6:
        raise ValueError(f"terminal controllability matrix is rank deficient: rank={rank}")
    particular = vh[:rank].T @ ((u[:, :rank].T @ target) / singular_values[:rank])
    basis = vh[rank:].T
    residual = float(np.linalg.norm(controllability @ particular - target, ord=np.inf))
    return NullSpaceReduction(particular, basis, rank, tolerance, residual)


def maximum_control_violation(
    controls: ArrayLike,
    acceleration_limit: float | None = None,
    lower: ArrayLike | None = None,
    upper: ArrayLike | None = None,
) -> float:
    """Return maximum nonnegative violation of the selected acceleration bounds."""

    values = np.asarray(controls, dtype=float)
    if acceleration_limit is not None:
        return float(max(0.0, np.max(np.linalg.norm(values, axis=1) - acceleration_limit)))
    if lower is None or upper is None:
        raise ValueError("componentwise bounds require lower and upper")
    lower_array = np.asarray(lower, dtype=float)
    upper_array = np.asarray(upper, dtype=float)
    return float(max(0.0, np.max(lower_array - values), np.max(values - upper_array)))
