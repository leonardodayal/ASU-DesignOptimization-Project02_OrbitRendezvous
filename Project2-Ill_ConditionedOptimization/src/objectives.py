"""Quadratic objective assembly and evaluation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.linalg import block_diag

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class QuadraticObjective:
    """Coefficients for ``0.5*u.T@H@u + g.T@u + c``."""

    hessian: FloatArray
    gradient: FloatArray
    constant: float
    state_offset: FloatArray
    state_control_map: FloatArray

    def value(self, controls: ArrayLike) -> float:
        """Evaluate the condensed objective at a stacked control vector."""

        vector = np.asarray(controls, dtype=float)
        return float(0.5 * vector @ self.hessian @ vector + self.gradient @ vector + self.constant)

    def derivative(self, controls: ArrayLike) -> FloatArray:
        """Evaluate the exact condensed gradient."""

        vector = np.asarray(controls, dtype=float)
        return self.hessian @ vector + self.gradient


def state_control_mapping(
    phi: ArrayLike,
    gamma: ArrayLike,
    initial_state: ArrayLike,
    num_intervals: int,
) -> tuple[FloatArray, FloatArray]:
    """Return stacked-state affine offset and control-sensitivity matrix."""

    phi_array = np.asarray(phi, dtype=float)
    gamma_array = np.asarray(gamma, dtype=float)
    initial = np.asarray(initial_state, dtype=float)
    if phi_array.shape != (6, 6) or gamma_array.shape != (6, 3) or initial.shape != (6,):
        raise ValueError("invalid CW matrix or initial-state shape")
    if num_intervals <= 0:
        raise ValueError("num_intervals must be positive")

    offset = np.zeros(6 * (num_intervals + 1), dtype=float)
    mapping = np.zeros((6 * (num_intervals + 1), 3 * num_intervals), dtype=float)
    offset[:6] = initial
    for step in range(num_intervals):
        current = slice(6 * step, 6 * (step + 1))
        following = slice(6 * (step + 1), 6 * (step + 2))
        offset[following] = phi_array @ offset[current]
        mapping[following] = phi_array @ mapping[current]
        mapping[following, 3 * step : 3 * (step + 1)] += gamma_array
    return offset, mapping


def assemble_condensed_objective(
    phi: ArrayLike,
    gamma: ArrayLike,
    initial_state: ArrayLike,
    q_path: ArrayLike,
    q_terminal: ArrayLike,
    r_control: ArrayLike,
    sample_time: float,
    num_intervals: int,
    reference_states: ArrayLike | None = None,
) -> QuadraticObjective:
    """Assemble the exact quadratic coefficients for formulation equation (15)."""

    q = np.asarray(q_path, dtype=float)
    qf = np.asarray(q_terminal, dtype=float)
    r = np.asarray(r_control, dtype=float)
    if q.shape != (6, 6) or qf.shape != (6, 6) or r.shape != (3, 3):
        raise ValueError("invalid objective weight shape")
    if sample_time <= 0:
        raise ValueError("sample_time must be positive")
    offset, mapping = state_control_mapping(phi, gamma, initial_state, num_intervals)
    if reference_states is None:
        reference = np.zeros((num_intervals + 1, 6), dtype=float)
    else:
        reference = np.asarray(reference_states, dtype=float)
        if reference.shape != (num_intervals + 1, 6):
            raise ValueError("reference_states has the wrong shape")
    reference_vector = reference.reshape(-1)
    state_weights = block_diag(*([sample_time * q] * num_intervals + [qf]))
    control_weights = np.kron(np.eye(num_intervals), sample_time * r)
    error_offset = offset - reference_vector
    hessian = mapping.T @ state_weights @ mapping + control_weights
    hessian = (hessian + hessian.T) / 2.0
    gradient = mapping.T @ state_weights @ error_offset
    constant = float(0.5 * error_offset @ state_weights @ error_offset)
    return QuadraticObjective(hessian, gradient, constant, offset, mapping)


def objective_breakdown(
    states: ArrayLike,
    controls: ArrayLike,
    q_path: ArrayLike,
    q_terminal: ArrayLike,
    r_control: ArrayLike,
    sample_time: float,
    reference_states: ArrayLike | None = None,
) -> dict[str, float]:
    """Return terminal, path-state, control-effort, and total objective terms."""

    state_array = np.asarray(states, dtype=float)
    control_array = np.asarray(controls, dtype=float)
    q = np.asarray(q_path, dtype=float)
    qf = np.asarray(q_terminal, dtype=float)
    r = np.asarray(r_control, dtype=float)
    if state_array.shape != (control_array.shape[0] + 1, 6) or control_array.shape[1:] != (3,):
        raise ValueError("state and control trajectory shapes are inconsistent")
    reference = (
        np.zeros_like(state_array)
        if reference_states is None
        else np.asarray(reference_states, dtype=float)
    )
    errors = state_array - reference
    terminal = 0.5 * errors[-1] @ qf @ errors[-1]
    path = 0.5 * sample_time * sum(error @ q @ error for error in errors[:-1])
    effort = 0.5 * sample_time * sum(control @ r @ control for control in control_array)
    return {
        "terminal": float(terminal),
        "path_state": float(path),
        "control_effort": float(effort),
        "total": float(terminal + path + effort),
    }
