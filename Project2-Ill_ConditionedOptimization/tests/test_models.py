"""Tests for configuration validation."""

from dataclasses import replace

import numpy as np
import pytest

from models import TerminalMode


def test_canonical_config_is_valid(config) -> None:
    config.validate()


def test_invalid_weight_and_bound_models(config) -> None:
    with pytest.raises(ValueError, match="positive definite"):
        replace(config, r_control=np.zeros((3, 3))).validate()
    with pytest.raises(ValueError, match="exactly one"):
        replace(
            config,
            axis_acceleration_lower=np.zeros(3),
            axis_acceleration_upper=np.zeros(3),
        ).validate()


def test_position_only_requires_velocity_policy(config) -> None:
    with pytest.raises(ValueError, match="velocity policy"):
        replace(config, terminal_mode=TerminalMode.POSITION_ONLY).validate()


def test_rejects_unsupported_solver_backend(config) -> None:
    invalid = replace(
        config,
        solver_options=replace(config.solver_options, backend="UNSUPPORTED"),
    )
    with pytest.raises(ValueError, match="CLARABEL"):
        invalid.validate()
