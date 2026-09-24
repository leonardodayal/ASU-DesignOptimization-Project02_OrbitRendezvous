"""Validated data models for rendezvous optimization."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass, field
from enum import StrEnum
from typing import Any

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]


class TerminalMode(StrEnum):
    """Supported terminal constraint modes."""

    RENDEZVOUS_EXACT = "rendezvous_exact"
    RENDEZVOUS_TOLERANCE = "rendezvous_tolerance"
    POSITION_ONLY = "position_only"


class Formulation(StrEnum):
    """Supported trajectory-optimization formulations."""

    MULTIPLE_SHOOTING = "multiple_shooting"
    SINGLE_SHOOTING = "single_shooting"


class SolverStatus(StrEnum):
    """Backend-independent solve status."""

    OPTIMAL = "optimal"
    FEASIBLE_NONOPTIMAL = "feasible_nonoptimal"
    INFEASIBLE = "infeasible"
    UNBOUNDED = "unbounded"
    ITERATION_LIMIT = "iteration_limit"
    NUMERICAL_FAILURE = "numerical_failure"
    INVALID_INPUT = "invalid_input"
    VERIFICATION_FAILED = "verification_failed"


@dataclass(frozen=True)
class NumericalScales:
    """Positive characteristic scales in SI units."""

    position: float
    velocity: float
    control: float

    def validate(self) -> None:
        for name, value in asdict(self).items():
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"numerical_scales.{name} must be finite and positive")


@dataclass(frozen=True)
class NumericalTolerances:
    """Human-confirmed solver and verification tolerances."""

    solver_feasibility: float
    solver_optimality: float
    scaled_dynamics: float
    terminal_position: float
    terminal_velocity: float
    control_absolute: float
    control_relative: float
    symmetry_relative: float
    condition_warning: float
    condition_severe: float

    def validate(self) -> None:
        for name, value in asdict(self).items():
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"numerical_tolerances.{name} must be finite and positive")
        if self.condition_severe <= self.condition_warning:
            raise ValueError("condition_severe must exceed condition_warning")


@dataclass(frozen=True)
class SolverOptions:
    """Human-confirmed iterative solver options."""

    backend: str
    max_iterations: int
    time_limit_seconds: float
    gradient_tolerance: float
    gradient_max_iterations: int

    def validate(self) -> None:
        if not self.backend:
            raise ValueError("solver_options.backend must be nonempty")
        if self.backend.upper() != "CLARABEL":
            raise ValueError("solver_options.backend must be CLARABEL")
        if self.max_iterations <= 0 or self.gradient_max_iterations <= 0:
            raise ValueError("solver iteration limits must be positive")
        if self.time_limit_seconds <= 0 or self.gradient_tolerance <= 0:
            raise ValueError("solver time and gradient tolerances must be positive")


def _array(value: Any, shape: tuple[int, ...], field_name: str) -> FloatArray:
    array = np.asarray(value, dtype=float)
    if array.shape != shape:
        raise ValueError(f"{field_name} must have shape {shape}, got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{field_name} must contain only finite values")
    return array


@dataclass(frozen=True)
class RendezvousConfig:
    """Complete, validated input for a fixed-time rendezvous problem."""

    mu: float
    reference_radius: float
    initial_state: FloatArray
    final_time: float
    num_intervals: int
    q_path: FloatArray
    q_terminal: FloatArray
    r_control: FloatArray
    numerical_scales: NumericalScales
    numerical_tolerances: NumericalTolerances
    solver_options: SolverOptions
    central_body: str = "Earth"
    orbit_regime: str = "low_earth_orbit"
    terminal_mode: TerminalMode = TerminalMode.RENDEZVOUS_EXACT
    position_tolerance: float | None = None
    velocity_tolerance: float | None = None
    position_only_velocity_policy: str | None = None
    maximum_terminal_speed: float | None = None
    acceleration_limit: float | None = None
    axis_acceleration_lower: FloatArray | None = None
    axis_acceleration_upper: FloatArray | None = None
    reference_states: FloatArray | None = None
    formulation: Formulation = Formulation.MULTIPLE_SHOOTING
    random_seed: int | None = None

    @property
    def sample_time(self) -> float:
        """Uniform zero-order-hold interval in seconds."""

        return self.final_time / self.num_intervals

    def validate(self) -> None:
        """Raise ``ValueError`` when any field violates the specification."""

        if self.central_body != "Earth" or self.orbit_regime != "low_earth_orbit":
            raise ValueError("baseline requires Earth and low_earth_orbit")
        for name in ("mu", "reference_radius", "final_time"):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if isinstance(self.num_intervals, bool) or not isinstance(self.num_intervals, int):
            raise ValueError("num_intervals must be an integer")
        if self.num_intervals <= 0:
            raise ValueError("num_intervals must be positive")
        _array(self.initial_state, (6,), "initial_state")
        self.numerical_scales.validate()
        self.numerical_tolerances.validate()
        self.solver_options.validate()

        symmetry = self.numerical_tolerances.symmetry_relative
        for name, matrix, size, require_pd in (
            ("q_path", self.q_path, 6, False),
            ("q_terminal", self.q_terminal, 6, False),
            ("r_control", self.r_control, 3, True),
        ):
            checked = _array(matrix, (size, size), name)
            if not np.allclose(checked, checked.T, rtol=symmetry, atol=0.0):
                raise ValueError(f"{name} must be symmetric")
            eigenvalues = np.linalg.eigvalsh((checked + checked.T) / 2.0)
            allowance = 100 * np.finfo(float).eps * max(1.0, float(np.linalg.norm(checked, 2)))
            if eigenvalues[0] < -allowance or (require_pd and eigenvalues[0] <= allowance):
                kind = "positive definite" if require_pd else "positive semidefinite"
                raise ValueError(f"{name} must be {kind}")

        total_limit = self.acceleration_limit is not None
        axis_limits = (
            self.axis_acceleration_lower is not None or self.axis_acceleration_upper is not None
        )
        if total_limit == axis_limits:
            raise ValueError("provide exactly one acceleration-bound model")
        if total_limit:
            acceleration_limit = self.acceleration_limit
            assert acceleration_limit is not None
            if not np.isfinite(acceleration_limit) or acceleration_limit <= 0:
                raise ValueError("acceleration_limit must be finite and positive")
        if axis_limits:
            if self.axis_acceleration_lower is None or self.axis_acceleration_upper is None:
                raise ValueError("both axis acceleration bounds are required")
            lower = _array(self.axis_acceleration_lower, (3,), "axis_acceleration_lower")
            upper = _array(self.axis_acceleration_upper, (3,), "axis_acceleration_upper")
            if np.any(lower > upper):
                raise ValueError("axis acceleration lower bounds must not exceed upper bounds")

        if self.terminal_mode == TerminalMode.RENDEZVOUS_TOLERANCE:
            if self.position_tolerance is None or self.velocity_tolerance is None:
                raise ValueError("tolerance rendezvous requires position and velocity tolerances")
            if self.position_tolerance < 0 or self.velocity_tolerance < 0:
                raise ValueError("terminal tolerances must be nonnegative")
        if self.terminal_mode == TerminalMode.POSITION_ONLY:
            if self.position_only_velocity_policy not in {"unconstrained", "bounded"}:
                raise ValueError("position_only requires an explicit terminal velocity policy")
            if self.position_only_velocity_policy == "bounded":
                if self.maximum_terminal_speed is None or self.maximum_terminal_speed < 0:
                    raise ValueError("bounded position_only requires maximum_terminal_speed")

        if self.reference_states is not None:
            _array(self.reference_states, (self.num_intervals + 1, 6), "reference_states")

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> RendezvousConfig:
        """Build a configuration from parsed YAML or JSON data."""

        values = dict(data)
        for key in (
            "mu",
            "reference_radius",
            "final_time",
            "position_tolerance",
            "velocity_tolerance",
            "maximum_terminal_speed",
            "acceleration_limit",
        ):
            if values.get(key) is not None:
                values[key] = float(values[key])
        values["num_intervals"] = int(values["num_intervals"])
        values["initial_state"] = np.asarray(values["initial_state"], dtype=float)
        for key in ("q_path", "q_terminal", "r_control", "reference_states"):
            if values.get(key) is not None:
                values[key] = np.asarray(values[key], dtype=float)
        for key in ("axis_acceleration_lower", "axis_acceleration_upper"):
            if values.get(key) is not None:
                values[key] = np.asarray(values[key], dtype=float)
        values["terminal_mode"] = TerminalMode(values.get("terminal_mode", "rendezvous_exact"))
        values["formulation"] = Formulation(values.get("formulation", "multiple_shooting"))
        scale_values = {key: float(value) for key, value in values["numerical_scales"].items()}
        tolerance_values = {
            key: float(value) for key, value in values["numerical_tolerances"].items()
        }
        solver_values = dict(values["solver_options"])
        solver_values["max_iterations"] = int(solver_values["max_iterations"])
        solver_values["gradient_max_iterations"] = int(solver_values["gradient_max_iterations"])
        solver_values["time_limit_seconds"] = float(solver_values["time_limit_seconds"])
        solver_values["gradient_tolerance"] = float(solver_values["gradient_tolerance"])
        values["numerical_scales"] = NumericalScales(**scale_values)
        values["numerical_tolerances"] = NumericalTolerances(**tolerance_values)
        values["solver_options"] = SolverOptions(**solver_values)
        config = cls(**values)
        config.validate()
        return config


@dataclass
class RendezvousResult:
    """Solver output and independent verification diagnostics."""

    status: SolverStatus
    message: str
    times: FloatArray
    states: FloatArray
    controls: FloatArray
    objective: float | None = None
    objective_components: dict[str, float] = field(default_factory=dict)
    diagnostics: dict[str, Any] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    solver_name: str | None = None
    iterations: int | None = None
    solve_time_seconds: float | None = None
