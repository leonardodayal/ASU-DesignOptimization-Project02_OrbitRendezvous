"""Single-shooting and multiple-shooting matrix construction."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy import sparse

from constraints import NullSpaceReduction, terminal_controllability, terminal_null_space
from dynamics import discretize_cw, mean_motion
from models import RendezvousConfig
from objectives import QuadraticObjective, assemble_condensed_objective

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class SingleShootingProblem:
    """Condensed quadratic and exact terminal constraint data."""

    phi: FloatArray
    gamma: FloatArray
    objective: QuadraticObjective
    terminal_matrix: FloatArray
    terminal_target: FloatArray
    reduction: NullSpaceReduction
    reduced_hessian: FloatArray
    reduced_gradient: FloatArray
    reduced_constant: float


@dataclass(frozen=True)
class MultipleShootingProblem:
    """Sparse equality-constrained quadratic matrices."""

    phi: FloatArray
    gamma: FloatArray
    hessian: sparse.csc_matrix
    gradient: FloatArray
    constant: float
    equality_matrix: sparse.csc_matrix
    equality_target: FloatArray
    num_state_variables: int


def build_single_shooting(config: RendezvousConfig) -> SingleShootingProblem:
    """Build condensed and null-space-reduced exact-rendezvous matrices."""

    config.validate()
    n = mean_motion(config.mu, config.reference_radius)
    phi, gamma = discretize_cw(n, config.sample_time)
    objective = assemble_condensed_objective(
        phi,
        gamma,
        config.initial_state,
        config.q_path,
        config.q_terminal,
        config.r_control,
        config.sample_time,
        config.num_intervals,
        config.reference_states,
    )
    terminal_matrix = terminal_controllability(phi, gamma, config.num_intervals)
    terminal_target = -np.linalg.matrix_power(phi, config.num_intervals) @ config.initial_state
    reduction = terminal_null_space(phi, gamma, config.initial_state, config.num_intervals)
    z = reduction.basis
    up = reduction.particular
    reduced_hessian = z.T @ objective.hessian @ z
    reduced_hessian = (reduced_hessian + reduced_hessian.T) / 2.0
    reduced_gradient = z.T @ (objective.hessian @ up + objective.gradient)
    reduced_constant = objective.value(up)
    return SingleShootingProblem(
        phi,
        gamma,
        objective,
        terminal_matrix,
        terminal_target,
        reduction,
        reduced_hessian,
        reduced_gradient,
        reduced_constant,
    )


def build_multiple_shooting(config: RendezvousConfig) -> MultipleShootingProblem:
    """Build the sparse exact-rendezvous multiple-shooting quadratic."""

    config.validate()
    n_intervals = config.num_intervals
    n = mean_motion(config.mu, config.reference_radius)
    phi, gamma = discretize_cw(n, config.sample_time)
    state_size = 6 * n_intervals
    control_size = 3 * n_intervals
    variable_size = state_size + control_size

    hessian = sparse.lil_matrix((variable_size, variable_size), dtype=float)
    gradient = np.zeros(variable_size, dtype=float)
    constant = 0.0
    reference = (
        np.zeros((n_intervals + 1, 6), dtype=float)
        if config.reference_states is None
        else np.asarray(config.reference_states, dtype=float)
    )
    initial_error = config.initial_state - reference[0]
    constant += 0.5 * config.sample_time * initial_error @ config.q_path @ initial_error
    for node in range(1, n_intervals + 1):
        state_slice = slice(6 * (node - 1), 6 * node)
        weight = config.q_terminal if node == n_intervals else config.sample_time * config.q_path
        hessian[state_slice, state_slice] = weight
        gradient[state_slice] = -(weight @ reference[node])
        constant += 0.5 * reference[node] @ weight @ reference[node]
    for step in range(n_intervals):
        control_slice = slice(state_size + 3 * step, state_size + 3 * (step + 1))
        hessian[control_slice, control_slice] = config.sample_time * config.r_control

    equality = sparse.lil_matrix((6 * n_intervals + 6, variable_size), dtype=float)
    target = np.zeros(6 * n_intervals + 6, dtype=float)
    for step in range(n_intervals):
        row = slice(6 * step, 6 * (step + 1))
        next_state = slice(6 * step, 6 * (step + 1))
        equality[row, next_state] = np.eye(6)
        if step == 0:
            target[row] = phi @ config.initial_state
        else:
            current_state = slice(6 * (step - 1), 6 * step)
            equality[row, current_state] = -phi
        control = slice(state_size + 3 * step, state_size + 3 * (step + 1))
        equality[row, control] = -gamma
    equality[-6:, state_size - 6 : state_size] = np.eye(6)
    return MultipleShootingProblem(
        phi,
        gamma,
        hessian.tocsc(),
        gradient,
        float(constant),
        equality.tocsc(),
        target,
        state_size,
    )
