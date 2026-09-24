"""Baseline and remedy solvers for spacecraft rendezvous."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy import sparse
from scipy.sparse.linalg import spsolve

from conditioning import GradientDescentResult, gradient_descent, spectrum
from constraints import maximum_control_violation
from dynamics import discretize_cw, mean_motion
from formulations import MultipleShootingProblem, SingleShootingProblem
from models import Formulation, RendezvousConfig, RendezvousResult, SolverStatus, TerminalMode
from objectives import objective_breakdown

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class ReducedGradientDescentSolution:
    """Control solution and reduced-space convergence history."""

    controls: FloatArray
    reduced_result: GradientDescentResult
    condition_number: float
    step_size: float
    maximum_bound_violation: float


@dataclass(frozen=True)
class SparseKktSolution:
    """One-step sparse Karush-Kuhn-Tucker solution."""

    variables: FloatArray
    multipliers: FloatArray
    objective: float
    stationarity_residual: float
    equality_residual: float
    elapsed_seconds: float


def solve_reduced_gradient_descent(
    problem: SingleShootingProblem,
    config: RendezvousConfig,
    *,
    numerics_confirmed: bool,
) -> ReducedGradientDescentSolution:
    """Run the confirmed null-space-reduced single-shooting baseline."""

    if not numerics_confirmed:
        raise PermissionError("numerical parameters require explicit confirmation")
    eigenvalues, condition_number = spectrum(problem.reduced_hessian)
    smallest = float(eigenvalues[0])
    largest = float(eigenvalues[-1])
    step = 2.0 / (largest + smallest)
    gradient_vector = problem.reduced_gradient
    hessian = problem.reduced_hessian

    def objective(w: FloatArray) -> float:
        return float(0.5 * w @ hessian @ w + gradient_vector @ w + problem.reduced_constant)

    def derivative(w: FloatArray) -> FloatArray:
        return hessian @ w + gradient_vector

    reduced = gradient_descent(
        derivative,
        np.zeros(hessian.shape[0]),
        step,
        config.solver_options.gradient_tolerance,
        config.solver_options.gradient_max_iterations,
        f=objective,
    )
    stacked = problem.reduction.particular + problem.reduction.basis @ reduced.solution
    controls = stacked.reshape(config.num_intervals, 3)
    violation = maximum_control_violation(
        controls,
        config.acceleration_limit,
        config.axis_acceleration_lower,
        config.axis_acceleration_upper,
    )
    allowed = max(
        config.numerical_tolerances.control_absolute,
        config.numerical_tolerances.control_relative
        * (
            config.acceleration_limit
            if config.acceleration_limit is not None
            else float(
                max(
                    np.max(np.abs(config.axis_acceleration_lower)),  # type: ignore[arg-type]
                    np.max(np.abs(config.axis_acceleration_upper)),  # type: ignore[arg-type]
                )
            )
        ),
    )
    if violation > allowed:
        raise RuntimeError("gradient-descent diagnostic activated the acceleration bound")
    return ReducedGradientDescentSolution(controls, reduced, condition_number, step, violation)


def solve_sparse_kkt(
    problem: MultipleShootingProblem, *, numerics_confirmed: bool
) -> SparseKktSolution:
    """Solve an equality-only multiple-shooting quadratic with one sparse Newton step."""

    if not numerics_confirmed:
        raise PermissionError("numerical parameters require explicit confirmation")
    start = perf_counter()
    zero = sparse.csc_matrix((problem.equality_matrix.shape[0], problem.equality_matrix.shape[0]))
    kkt = sparse.bmat(
        [[problem.hessian, problem.equality_matrix.T], [problem.equality_matrix, zero]],
        format="csc",
    )
    right_hand_side = np.concatenate((-problem.gradient, problem.equality_target))
    solution = np.asarray(spsolve(kkt, right_hand_side), dtype=float)
    elapsed = perf_counter() - start
    variables = solution[: problem.hessian.shape[0]]
    multipliers = solution[problem.hessian.shape[0] :]
    stationarity = (
        problem.hessian @ variables + problem.gradient + problem.equality_matrix.T @ multipliers
    )
    equality = problem.equality_matrix @ variables - problem.equality_target
    objective = (
        0.5 * variables @ problem.hessian @ variables
        + problem.gradient @ variables
        + problem.constant
    )
    return SparseKktSolution(
        variables,
        multipliers,
        float(objective),
        float(np.linalg.norm(stationarity, ord=np.inf)),
        float(np.linalg.norm(equality, ord=np.inf)),
        elapsed,
    )


def _cvxpy_status(status: str) -> SolverStatus:
    if status == "optimal":
        return SolverStatus.OPTIMAL
    if status == "optimal_inaccurate":
        return SolverStatus.FEASIBLE_NONOPTIMAL
    if status in {"infeasible", "infeasible_inaccurate"}:
        return SolverStatus.INFEASIBLE
    if status in {"unbounded", "unbounded_inaccurate"}:
        return SolverStatus.UNBOUNDED
    if status == "user_limit":
        return SolverStatus.ITERATION_LIMIT
    return SolverStatus.NUMERICAL_FAILURE


def solve_rendezvous(
    config: RendezvousConfig,
    *,
    numerics_confirmed: bool,
) -> RendezvousResult:
    """Solve the complete constrained problem with CVXPY and Clarabel."""

    config.validate()
    if not numerics_confirmed:
        raise PermissionError("numerical parameters require explicit confirmation")
    try:
        import cvxpy as cp
    except ImportError as error:
        raise RuntimeError("CVXPY and Clarabel are required for constrained solves") from error

    n = mean_motion(config.mu, config.reference_radius)
    phi, gamma = discretize_cw(n, config.sample_time)
    state_scale = np.diag(
        [config.numerical_scales.position] * 3 + [config.numerical_scales.velocity] * 3
    )
    control_scale = config.numerical_scales.control
    scaled_controls = cp.Variable((config.num_intervals, 3), name="scaled_controls")
    controls = control_scale * scaled_controls
    constraints: list[Any] = []
    if config.formulation == Formulation.MULTIPLE_SHOOTING:
        scaled_states = cp.Variable((config.num_intervals + 1, 6), name="scaled_states")
        states = scaled_states @ state_scale
        constraints.append(states[0] == config.initial_state)
        for step in range(config.num_intervals):
            constraints.append(states[step + 1] == phi @ states[step] + gamma @ controls[step])
    else:
        state_expressions: list[Any] = [config.initial_state]
        for step in range(config.num_intervals):
            state_expressions.append(
                phi @ state_expressions[-1] + gamma @ controls[step]
            )
        states = cp.vstack(state_expressions)
    if config.terminal_mode == TerminalMode.RENDEZVOUS_EXACT:
        constraints.append(states[-1] == np.zeros(6))
    elif config.terminal_mode == TerminalMode.RENDEZVOUS_TOLERANCE:
        constraints.extend(
            [
                cp.norm(states[-1, :3], 2) <= config.position_tolerance,
                cp.norm(states[-1, 3:], 2) <= config.velocity_tolerance,
            ]
        )
    else:
        constraints.append(states[-1, :3] == np.zeros(3))
        if config.position_only_velocity_policy == "bounded":
            constraints.append(cp.norm(states[-1, 3:], 2) <= config.maximum_terminal_speed)
    if config.acceleration_limit is not None:
        for step in range(config.num_intervals):
            constraints.append(cp.norm(controls[step], 2) <= config.acceleration_limit)
    else:
        constraints.extend(
            [
                controls >= config.axis_acceleration_lower,
                controls <= config.axis_acceleration_upper,
            ]
        )
    reference = (
        np.zeros((config.num_intervals + 1, 6))
        if config.reference_states is None
        else config.reference_states
    )
    terms = [0.5 * cp.quad_form(states[-1] - reference[-1], config.q_terminal)]
    for step in range(config.num_intervals):
        terms.append(
            0.5
            * config.sample_time
            * (
                cp.quad_form(states[step] - reference[step], config.q_path)
                + cp.quad_form(controls[step], config.r_control)
            )
        )
    problem = cp.Problem(cp.Minimize(sum(terms)), constraints)
    start = perf_counter()
    try:
        value = problem.solve(
            solver="CLARABEL",
            max_iter=config.solver_options.max_iterations,
            time_limit=config.solver_options.time_limit_seconds,
            tol_gap_abs=config.numerical_tolerances.solver_optimality,
            tol_gap_rel=config.numerical_tolerances.solver_optimality,
            tol_feas=config.numerical_tolerances.solver_feasibility,
        )
    except Exception as error:  # Backend exceptions are normalized into result status.
        return RendezvousResult(
            SolverStatus.NUMERICAL_FAILURE,
            f"solver failure: {error}",
            np.linspace(0.0, config.final_time, config.num_intervals + 1),
            np.empty((0, 6)),
            np.empty((0, 3)),
            solver_name="CLARABEL",
            solve_time_seconds=perf_counter() - start,
        )
    elapsed = perf_counter() - start
    status = _cvxpy_status(problem.status)
    if states.value is None or scaled_controls.value is None:
        return RendezvousResult(
            status,
            f"solver terminated with status {problem.status}",
            np.linspace(0.0, config.final_time, config.num_intervals + 1),
            np.empty((0, 6)),
            np.empty((0, 3)),
            solver_name="CLARABEL",
            iterations=getattr(problem.solver_stats, "num_iters", None),
            solve_time_seconds=elapsed,
        )
    state_values = np.asarray(states.value)
    control_values = control_scale * np.asarray(scaled_controls.value)
    components = objective_breakdown(
        state_values,
        control_values,
        config.q_path,
        config.q_terminal,
        config.r_control,
        config.sample_time,
        config.reference_states,
    )
    return RendezvousResult(
        status,
        f"solver terminated with status {problem.status}",
        np.linspace(0.0, config.final_time, config.num_intervals + 1),
        state_values,
        control_values,
        objective=float(value),
        objective_components=components,
        warnings=[
            "assumptions_unverified: circular orbit, two-body dynamics, small relative "
            "separation, negligible perturbations",
            "collision, keep-out, line-of-sight, approach-corridor, closing-speed, and "
            "plume constraints were not evaluated",
        ],
        solver_name="CLARABEL",
        iterations=getattr(problem.solver_stats, "num_iters", None),
        solve_time_seconds=elapsed,
    )
