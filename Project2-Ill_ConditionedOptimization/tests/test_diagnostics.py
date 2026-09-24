"""Tests for reproducible diagnostic orchestration."""

from __future__ import annotations

import numpy as np
import pytest

from diagnostics import conditioning_preflight, cross_track_hand_check
from models import RendezvousConfig


def test_cross_track_hand_check_matches_numerical_eigenvalues(
    config: RendezvousConfig,
) -> None:
    result = cross_track_hand_check(config)
    np.testing.assert_allclose(
        result["analytic_eigenvalues"], result["numerical_eigenvalues"], rtol=1e-14
    )
    assert result["passed"] is True


def test_cross_track_hand_check_rejects_other_dimensions(config: RendezvousConfig) -> None:
    with pytest.raises(ValueError, match="four intervals"):
        cross_track_hand_check(config, num_intervals=5)


def test_conditioning_preflight_records_failed_bound_without_retuning(
    config: RendezvousConfig,
) -> None:
    payload = conditioning_preflight(config, horizons=(15,))
    assert payload["status"] == "failed_criterion"
    assert payload["automatic_retuning_performed"] is False
    rows = payload["rows"]
    assert isinstance(rows, list)
    assert rows[0]["acceleration_bound_inactive"] == 0
