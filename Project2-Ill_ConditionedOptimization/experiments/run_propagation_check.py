"""Generate the EX-01 independent propagation check."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml

from dynamics import augmented_exponential_discretization, discretize_cw, mean_motion, simulate
from models import RendezvousConfig


def run(output: Path) -> int:
    """Compare analytic propagation with an independent matrix-exponential oracle."""

    config = RendezvousConfig.from_mapping(
        yaml.safe_load(Path("data/canonical.yaml").read_text(encoding="utf-8"))
    )
    n = mean_motion(config.mu, config.reference_radius)
    phi, gamma = discretize_cw(n, config.sample_time)
    oracle_phi, oracle_gamma = augmented_exponential_discretization(n, config.sample_time)
    controls = np.zeros((config.num_intervals, 3))
    analytic = simulate(config.initial_state, controls, phi, gamma)
    oracle = simulate(config.initial_state, controls, oracle_phi, oracle_gamma)
    maximum_error = float(np.max(np.abs(analytic - oracle)))
    payload = {
        "maximum_state_error": maximum_error,
        "relative_tolerance": 1e-11,
        "absolute_tolerance": 1e-12,
        "passed": bool(np.allclose(analytic, oracle, rtol=1e-11, atol=1e-12)),
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload))
    return 0 if payload["passed"] else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output", type=Path, default=Path("outputs/report/propagation_check.json")
    )
    arguments = parser.parse_args()
    return run(arguments.output)


if __name__ == "__main__":
    raise SystemExit(main())
