"""Tests for objective and formulation equivalence."""

import autograd.numpy as anp
import numpy as np
from autograd import grad, hessian
from numpy.testing import assert_allclose

from dynamics import simulate
from formulations import build_multiple_shooting, build_single_shooting
from objectives import objective_breakdown


def test_condensed_objective_matches_trajectory(config) -> None:
    problem = build_single_shooting(config)
    controls = np.linspace(-1e-4, 1e-4, 3 * config.num_intervals).reshape(-1, 3)
    states = simulate(config.initial_state, controls, problem.phi, problem.gamma)
    components = objective_breakdown(
        states,
        controls,
        config.q_path,
        config.q_terminal,
        config.r_control,
        config.sample_time,
    )
    np.testing.assert_allclose(
        problem.objective.value(controls.reshape(-1)), components["total"], rtol=1e-11, atol=1e-10
    )


def test_null_space_enforces_terminal_rendezvous(config) -> None:
    problem = build_single_shooting(config)
    assert problem.reduction.rank == 6
    assert_allclose(
        problem.terminal_matrix @ problem.reduction.particular,
        problem.terminal_target,
        rtol=1e-10,
        atol=1e-10,
    )
    assert_allclose(problem.terminal_matrix @ problem.reduction.basis, 0.0, atol=1e-10)


def test_multiple_shooting_dimensions(config) -> None:
    problem = build_multiple_shooting(config)
    assert problem.hessian.shape == (9 * config.num_intervals, 9 * config.num_intervals)
    assert problem.equality_matrix.shape == (6 * config.num_intervals + 6, 9 * config.num_intervals)


def test_closed_form_derivatives_match_autograd(config) -> None:
    problem = build_single_shooting(config)
    matrix = anp.asarray(problem.reduced_hessian)
    linear = anp.asarray(problem.reduced_gradient)

    def objective(vector):
        return 0.5 * vector @ matrix @ vector + linear @ vector

    point = anp.linspace(-1e-4, 1e-4, matrix.shape[0])
    assert_allclose(
        grad(objective)(point),
        problem.reduced_hessian @ point + problem.reduced_gradient,
        rtol=1e-10,
        atol=1e-12,
    )
    assert_allclose(
        hessian(objective)(point),
        problem.reduced_hessian,
        rtol=1e-10,
        atol=1e-12,
    )

    point_array = np.asarray(point, dtype=float)
    direction = np.linspace(-1.0, 1.0, matrix.shape[0])
    epsilon = 1e-6
    finite_difference = (
        np.asarray(grad(objective)(point_array + epsilon * direction))
        - np.asarray(grad(objective)(point_array - epsilon * direction))
    ) / (2.0 * epsilon)
    assert_allclose(
        finite_difference,
        problem.reduced_hessian @ direction,
        rtol=1e-7,
        atol=1e-7,
    )
