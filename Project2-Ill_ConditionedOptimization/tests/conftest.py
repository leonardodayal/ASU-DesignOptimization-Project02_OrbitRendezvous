"""Shared deterministic test configuration."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / "src"))

from dynamics import mean_motion  # noqa: E402
from models import (  # noqa: E402
    NumericalScales,
    NumericalTolerances,
    RendezvousConfig,
    SolverOptions,
)


@pytest.fixture
def config() -> RendezvousConfig:
    """Return the confirmed canonical configuration on a small test grid."""

    mu = 3.986004418e14
    radius = 6_778_137.0
    n = mean_motion(mu, radius)
    position_scale = 100.0
    velocity_scale = n * position_scale
    control_scale = 0.01
    q = np.diag([1 / position_scale**2] * 3 + [1 / velocity_scale**2] * 3)
    r = np.eye(3) / control_scale**2
    return RendezvousConfig(
        mu=mu,
        reference_radius=radius,
        initial_state=np.array([100.0, -200.0, 20.0, 0.0, 0.0, 0.0]),
        final_time=240.0,
        num_intervals=12,
        q_path=q,
        q_terminal=np.zeros((6, 6)),
        r_control=r,
        numerical_scales=NumericalScales(position_scale, velocity_scale, control_scale),
        numerical_tolerances=NumericalTolerances(
            solver_feasibility=1e-8,
            solver_optimality=1e-8,
            scaled_dynamics=1e-7,
            terminal_position=1e-5,
            terminal_velocity=1e-8,
            control_absolute=1e-10,
            control_relative=1e-7,
            symmetry_relative=1e-12,
            condition_warning=1e10,
            condition_severe=1e14,
        ),
        solver_options=SolverOptions("CLARABEL", 500, 60.0, 1e-8, 200_000),
        acceleration_limit=0.01,
    )
