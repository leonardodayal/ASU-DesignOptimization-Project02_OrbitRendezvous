"""Reproducible long-horizon condition number κ preflight."""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from conditioning import cond_after_diagonal_rescale, spectrum
from constraints import numerical_rank_tolerance
from dynamics import discretize_cw, mean_motion
from formulations import build_single_shooting
from models import RendezvousConfig


def cross_track_hand_check(config: RendezvousConfig, num_intervals: int = 4) -> dict[str, object]:
    """Compare analytic and numerical eigenvalues for a 2-by-2 reduced Hessian."""

    if num_intervals != 4:
        raise ValueError("the cross-track hand check requires four intervals")
    phi, gamma = discretize_cw(
        mean_motion(config.mu, config.reference_radius), config.sample_time
    )
    phi_z = phi[np.ix_([2, 5], [2, 5])]
    gamma_z = gamma[np.ix_([2, 5], [2])]
    mapping = np.zeros((2 * (num_intervals + 1), num_intervals), dtype=float)
    for step in range(num_intervals):
        current = slice(2 * step, 2 * (step + 1))
        following = slice(2 * (step + 1), 2 * (step + 2))
        mapping[following] = phi_z @ mapping[current]
        mapping[following, step : step + 1] += gamma_z

    q_z = config.q_path[np.ix_([2, 5], [2, 5])]
    qf_z = config.q_terminal[np.ix_([2, 5], [2, 5])]
    state_weights = np.zeros_like(mapping @ mapping.T)
    for node in range(num_intervals):
        state_weights[2 * node : 2 * (node + 1), 2 * node : 2 * (node + 1)] = (
            config.sample_time * q_z
        )
    state_weights[-2:, -2:] = qf_z
    hessian = (
        mapping.T @ state_weights @ mapping
        + config.sample_time * config.r_control[2, 2] * np.eye(num_intervals)
    )
    terminal_matrix = mapping[-2:]
    _, singular_values, vh = np.linalg.svd(terminal_matrix, full_matrices=True)
    rank = int(np.sum(singular_values > numerical_rank_tolerance(terminal_matrix)))
    basis = vh[rank:].T
    reduced = basis.T @ hessian @ basis
    reduced = (reduced + reduced.T) / 2.0
    if reduced.shape != (2, 2):
        raise RuntimeError("cross-track reduction did not produce a 2-by-2 matrix")
    a, b, d = reduced[0, 0], reduced[0, 1], reduced[1, 1]
    discriminant = np.sqrt((a - d) ** 2 + 4.0 * b**2)
    analytic = np.array([(a + d - discriminant) / 2.0, (a + d + discriminant) / 2.0])
    numerical = np.linalg.eigvalsh(reduced)
    relative_error = float(
        np.max(np.abs(analytic - numerical) / np.maximum(np.abs(numerical), 1.0))
    )
    return {
        "num_intervals": num_intervals,
        "reduced_hessian": reduced.tolist(),
        "analytic_eigenvalues": analytic.tolist(),
        "numerical_eigenvalues": numerical.tolist(),
        "maximum_relative_error": relative_error,
        "passed": bool(relative_error <= 100.0 * np.finfo(float).eps),
    }


def conditioning_preflight(
    config: RendezvousConfig,
    horizons: tuple[int, ...] = (15, 30, 60, 120, 240),
) -> dict[str, object]:
    """Evaluate D1/D2 inputs and the approved inactive-bound prerequisite."""

    rows: list[dict[str, float | int]] = []
    failed = False
    for horizon in horizons:
        case = replace(config, num_intervals=horizon, final_time=20.0 * horizon)
        problem = build_single_shooting(case)
        eigenvalues, condition_number = spectrum(problem.reduced_hessian)
        optimum = -np.linalg.solve(problem.reduced_hessian, problem.reduced_gradient)
        controls = (problem.reduction.particular + problem.reduction.basis @ optimum).reshape(
            horizon, 3
        )
        maximum_control = float(np.max(np.linalg.norm(controls, axis=1)))
        assert case.acceleration_limit is not None
        inactive = maximum_control <= case.acceleration_limit
        failed = failed or not inactive
        rows.append(
            {
                "num_intervals": horizon,
                "final_time_seconds": case.final_time,
                "reduced_dimension": problem.reduced_hessian.shape[0],
                "condition_number": condition_number,
                "condition_number_after_diagonal_rescaling": cond_after_diagonal_rescale(
                    problem.reduced_hessian
                ),
                "smallest_eigenvalue": float(eigenvalues[0]),
                "largest_eigenvalue": float(eigenvalues[-1]),
                "maximum_control_m_per_s2": maximum_control,
                "acceleration_bound_inactive": int(inactive),
            }
        )
    return {
        "status": "failed_criterion" if failed else "passed",
        "criterion": "acceleration bound remains inactive for the gradient-descent diagnostic",
        "automatic_retuning_performed": False,
        "cross_track_hand_check": cross_track_hand_check(config),
        "rows": rows,
    }
