"""Tests for independent post-solve verification."""

import numpy as np

from dynamics import discretize_cw, mean_motion, simulate
from models import RendezvousResult, SolverStatus
from verification import apply_verification


def test_perturbed_course_cannot_report_success(config) -> None:
    phi, gamma = discretize_cw(mean_motion(config.mu, config.reference_radius), config.sample_time)
    controls = np.zeros((config.num_intervals, 3))
    states = simulate(config.initial_state, controls, phi, gamma)
    result = RendezvousResult(
        SolverStatus.OPTIMAL,
        "unverified",
        np.linspace(0.0, config.final_time, config.num_intervals + 1),
        states,
        controls,
    )
    verified = apply_verification(config, result)
    assert verified.status == SolverStatus.VERIFICATION_FAILED
    assert "terminal position" in verified.diagnostics["verification_failures"]


def test_verification_reports_physical_interpretation_metrics(config) -> None:
    states = np.zeros((config.num_intervals + 1, 6))
    states[0] = config.initial_state
    controls = np.zeros((config.num_intervals, 3))
    result = RendezvousResult(SolverStatus.OPTIMAL, "test", np.array([]), states, controls)
    verified = apply_verification(config, result)
    assert verified.diagnostics["maximum_relative_separation_ratio"] > 0.0
    assert verified.diagnostics["horizon_to_orbit_period_ratio"] > 0.0
    assert verified.diagnostics["integrated_acceleration_m_per_s"] == 0.0
