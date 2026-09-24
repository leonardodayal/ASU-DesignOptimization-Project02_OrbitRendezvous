"""Independent post-solve verification in physical units."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from constraints import maximum_control_violation
from dynamics import discretize_cw, mean_motion
from models import RendezvousConfig, RendezvousResult, SolverStatus, TerminalMode


@dataclass(frozen=True)
class VerificationReport:
    """Independent physical and scaled feasibility checks."""

    passed: bool
    maximum_dynamics_defect: float
    maximum_scaled_dynamics_defect: float
    terminal_position_residual: float
    terminal_velocity_residual: float
    control_violation: float
    failures: tuple[str, ...]


def verify_solution(config: RendezvousConfig, result: RendezvousResult) -> VerificationReport:
    """Verify dynamics, terminal conditions, and acceleration bounds independently."""

    if result.states.shape != (config.num_intervals + 1, 6) or result.controls.shape != (
        config.num_intervals,
        3,
    ):
        return VerificationReport(
            False, np.inf, np.inf, np.inf, np.inf, np.inf, ("invalid trajectory shape",)
        )
    n = mean_motion(config.mu, config.reference_radius)
    phi, gamma = discretize_cw(n, config.sample_time)
    defects = result.states[1:] - (result.states[:-1] @ phi.T + result.controls @ gamma.T)
    maximum_defect = float(np.max(np.abs(defects)))
    scale = np.asarray(
        [config.numerical_scales.position] * 3 + [config.numerical_scales.velocity] * 3
    )
    scaled_defect = float(np.max(np.abs(defects / scale)))
    position = float(np.linalg.norm(result.states[-1, :3]))
    velocity = float(np.linalg.norm(result.states[-1, 3:]))
    violation = maximum_control_violation(
        result.controls,
        config.acceleration_limit,
        config.axis_acceleration_lower,
        config.axis_acceleration_upper,
    )
    limit = (
        config.acceleration_limit
        if config.acceleration_limit is not None
        else float(
            max(
                np.max(np.abs(config.axis_acceleration_lower)),  # type: ignore[arg-type]
                np.max(np.abs(config.axis_acceleration_upper)),  # type: ignore[arg-type]
            )
        )
    )
    allowed_control = max(
        config.numerical_tolerances.control_absolute,
        config.numerical_tolerances.control_relative * limit,
    )
    failures: list[str] = []
    if scaled_defect > config.numerical_tolerances.scaled_dynamics:
        failures.append("scaled dynamics defect")
    if config.terminal_mode == TerminalMode.RENDEZVOUS_EXACT:
        if position > config.numerical_tolerances.terminal_position:
            failures.append("terminal position")
        if velocity > config.numerical_tolerances.terminal_velocity:
            failures.append("terminal velocity")
    elif config.terminal_mode == TerminalMode.RENDEZVOUS_TOLERANCE:
        assert config.position_tolerance is not None
        assert config.velocity_tolerance is not None
        if position > config.position_tolerance:
            failures.append("terminal position")
        if velocity > config.velocity_tolerance:
            failures.append("terminal velocity")
    else:
        if position > config.numerical_tolerances.terminal_position:
            failures.append("terminal position")
        if config.position_only_velocity_policy == "bounded":
            assert config.maximum_terminal_speed is not None
            if velocity > config.maximum_terminal_speed:
                failures.append("terminal velocity bound")
    if violation > allowed_control:
        failures.append("control bound")
    return VerificationReport(
        not failures,
        maximum_defect,
        scaled_defect,
        position,
        velocity,
        violation,
        tuple(failures),
    )


def apply_verification(config: RendezvousConfig, result: RendezvousResult) -> RendezvousResult:
    """Attach diagnostics and prevent an unverified course from reporting success."""

    report = verify_solution(config, result)
    result.diagnostics.update(
        {
            "maximum_dynamics_defect": report.maximum_dynamics_defect,
            "maximum_scaled_dynamics_defect": report.maximum_scaled_dynamics_defect,
            "terminal_position_residual": report.terminal_position_residual,
            "terminal_velocity_residual": report.terminal_velocity_residual,
            "maximum_control_violation": report.control_violation,
            "terminal_speed_m_per_s": report.terminal_velocity_residual,
            "verification_failures": list(report.failures),
        }
    )
    if result.states.size:
        separation = np.linalg.norm(result.states[:, :3], axis=1)
        n = mean_motion(config.mu, config.reference_radius)
        result.diagnostics.update(
            {
                "maximum_relative_separation_ratio": float(
                    np.max(separation) / config.reference_radius
                ),
                "horizon_to_orbit_period_ratio": float(config.final_time * n / (2.0 * np.pi)),
                "integrated_acceleration_m_per_s": float(
                    config.sample_time * np.sum(np.linalg.norm(result.controls, axis=1))
                ),
            }
        )
    if (
        result.status in {SolverStatus.OPTIMAL, SolverStatus.FEASIBLE_NONOPTIMAL}
        and not report.passed
    ):
        result.status = SolverStatus.VERIFICATION_FAILED
        result.message = f"independent verification failed: {', '.join(report.failures)}"
    return result
