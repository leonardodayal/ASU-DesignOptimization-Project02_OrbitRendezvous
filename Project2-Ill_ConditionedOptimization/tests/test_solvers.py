"""Tests for baseline and sparse-remedy solvers."""

from dataclasses import replace

import numpy as np
from numpy.testing import assert_allclose

from formulations import build_multiple_shooting, build_single_shooting
from models import Formulation, SolverStatus
from solvers import solve_reduced_gradient_descent, solve_rendezvous, solve_sparse_kkt
from verification import apply_verification


def test_sparse_kkt_satisfies_equalities(config) -> None:
    problem = build_multiple_shooting(config)
    solution = solve_sparse_kkt(problem, numerics_confirmed=True)
    assert solution.equality_residual < 1e-7
    assert solution.stationarity_residual < 1e-7


def test_reduced_gradient_descent_matches_direct_solution(config) -> None:
    unconstrained_test_config = replace(config, acceleration_limit=1.0)
    problem = build_single_shooting(unconstrained_test_config)
    solution = solve_reduced_gradient_descent(
        problem, unconstrained_test_config, numerics_confirmed=True
    )
    expected = -np.linalg.solve(problem.reduced_hessian, problem.reduced_gradient)
    assert solution.reduced_result.converged
    assert_allclose(solution.reduced_result.solution, expected, rtol=1e-6, atol=1e-8)
    assert solution.maximum_bound_violation == 0.0


def test_clarabel_solves_and_verifies_canonical_case(config) -> None:
    canonical = replace(config, num_intervals=60, final_time=1200.0)
    result = apply_verification(
        canonical,
        solve_rendezvous(canonical, numerics_confirmed=True),
    )
    assert result.status == SolverStatus.OPTIMAL
    assert result.diagnostics["verification_failures"] == []


def test_clarabel_reports_deterministic_infeasible_case(config) -> None:
    infeasible = replace(
        config,
        acceleration_limit=None,
        axis_acceleration_lower=np.zeros(3),
        axis_acceleration_upper=np.zeros(3),
    )
    result = solve_rendezvous(infeasible, numerics_confirmed=True)
    assert result.status == SolverStatus.INFEASIBLE


def test_single_and_multiple_shooting_agree(config) -> None:
    multiple = replace(config, num_intervals=30, final_time=600.0)
    single = replace(multiple, formulation=Formulation.SINGLE_SHOOTING)
    multiple_result = apply_verification(
        multiple, solve_rendezvous(multiple, numerics_confirmed=True)
    )
    single_result = apply_verification(single, solve_rendezvous(single, numerics_confirmed=True))
    assert multiple_result.status == SolverStatus.OPTIMAL
    assert single_result.status == SolverStatus.OPTIMAL
    assert_allclose(single_result.objective, multiple_result.objective, rtol=2e-7)
    assert_allclose(single_result.controls, multiple_result.controls, rtol=2e-5, atol=2e-8)
