"""Run the confirmed long-horizon preflight without automatic retuning."""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path

import yaml

from diagnostics import conditioning_preflight
from models import RendezvousConfig

HORIZONS = (15, 30, 60, 120, 240)


def _plot(rows: list[dict[str, float | int]], acceleration_limit: float, path: Path) -> None:
    """Render the preflight plot with caches confined to ignored output storage."""

    cache = path.parent / ".cache"
    cache.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(cache / "matplotlib"))
    os.environ.setdefault("XDG_CACHE_HOME", str(cache))
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    horizons = [int(row["num_intervals"]) for row in rows]
    raw_condition = [float(row["condition_number"]) for row in rows]
    scaled_condition = [
        float(row["condition_number_after_diagonal_rescaling"]) for row in rows
    ]
    controls = [float(row["maximum_control_m_per_s2"]) for row in rows]
    figure, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].loglog(horizons, raw_condition, "o-", label="reduced Hessian")
    axes[0].loglog(horizons, scaled_condition, "s--", label="after diagonal rescaling")
    axes[0].set_xlabel("Horizon length N")
    axes[0].set_ylabel("Condition number κ")
    axes[0].grid(True, which="both")
    axes[0].legend()
    axes[1].plot(horizons, controls, "o-", label="unconstrained optimum")
    axes[1].axhline(acceleration_limit, color="red", linestyle="--", label="limit")
    axes[1].set_xlabel("Horizon length N")
    axes[1].set_ylabel("Maximum acceleration (m/s²)")
    axes[1].grid(True)
    axes[1].legend()
    figure.tight_layout()
    figure.savefig(path)
    plt.close(figure)


def run(config_path: Path, output_dir: Path) -> int:
    """Write D1/D2 preflight data and stop if acceleration bounds become active."""

    config = RendezvousConfig.from_mapping(yaml.safe_load(config_path.read_text()))
    output_dir.mkdir(parents=True, exist_ok=True)
    payload = conditioning_preflight(config, HORIZONS)
    rows = payload["rows"]
    assert isinstance(rows, list)
    with (output_dir / "conditioning_preflight.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output_dir / "conditioning_preflight.json").write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )
    assert config.acceleration_limit is not None
    _plot(rows, config.acceleration_limit, output_dir / "conditioning_preflight.svg")
    if payload["status"] == "failed_criterion":
        print("EX-04 stopped: acceleration-bound inactive criterion failed")
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("data/canonical.yaml"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/report"))
    arguments = parser.parse_args()
    return run(arguments.config, arguments.output_dir)


if __name__ == "__main__":
    raise SystemExit(main())
