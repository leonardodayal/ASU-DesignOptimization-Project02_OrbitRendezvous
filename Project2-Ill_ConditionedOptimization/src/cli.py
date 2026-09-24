"""Command-line interface for rendezvous solves and propagation."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from diagnostics import conditioning_preflight
from dynamics import discretize_cw, mean_motion, simulate
from models import RendezvousConfig, RendezvousResult, SolverStatus
from solvers import solve_rendezvous
from verification import apply_verification


def _load_mapping(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        value = json.loads(text)
    elif path.suffix.lower() in {".yaml", ".yml"}:
        value = yaml.safe_load(text)
    else:
        raise ValueError("configuration must be JSON or YAML")
    if not isinstance(value, dict):
        raise ValueError("configuration root must be a mapping")
    return value


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if hasattr(value, "value"):
        return value.value
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items()}
    if isinstance(value, list | tuple):
        return [_jsonable(item) for item in value]
    return value


def _result_payload(config: RendezvousConfig, result: RendezvousResult) -> dict[str, Any]:
    normalized = _jsonable(asdict(config))
    canonical = json.dumps(normalized, sort_keys=True, separators=(",", ":"))
    return {
        "software_version": "0.1.0",
        "reproducibility_id": hashlib.sha256(canonical.encode()).hexdigest(),
        "metadata": {
            "model": "linear Clohessy-Wiltshire with exact zero-order hold",
            "frame": "target-centered LVLH (+x radial, +y along-track, +z orbit-normal)",
            "state_units": ["m", "m", "m", "m/s", "m/s", "m/s"],
            "control_units": ["m/s^2", "m/s^2", "m/s^2"],
            "time_origin_seconds": 0.0,
            "safety_classification": "educational analysis; not a flight command product",
        },
        "configuration": normalized,
        "result": _jsonable(asdict(result)),
    }


def _exit_code(status: SolverStatus) -> int:
    if status in {SolverStatus.OPTIMAL, SolverStatus.FEASIBLE_NONOPTIMAL}:
        return 0
    if status == SolverStatus.INVALID_INPUT:
        return 2
    if status in {SolverStatus.INFEASIBLE, SolverStatus.UNBOUNDED}:
        return 3
    if status == SolverStatus.VERIFICATION_FAILED:
        return 5
    return 4


def _write_result(output_dir: Path, config: RendezvousConfig, result: RendezvousResult) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "result.json").write_text(
        json.dumps(_result_payload(config, result), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if result.states.size:
        np.savetxt(
            output_dir / "states.csv",
            result.states,
            delimiter=",",
            header="x,y,z,vx,vy,vz",
            comments="",
        )
    if result.controls.size:
        np.savetxt(
            output_dir / "controls.csv",
            result.controls,
            delimiter=",",
            header="ux,uy,uz",
            comments="",
        )
    if not result.states.size or not result.controls.size:
        for filename in ("states.csv", "controls.csv"):
            (output_dir / filename).unlink(missing_ok=True)


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser."""

    parser = argparse.ArgumentParser(prog="orbit-rendezvous")
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command in ("solve", "diagnose", "propagate"):
        child = subparsers.add_parser(command)
        child.add_argument("config", type=Path)
        child.add_argument("--output-dir", type=Path, required=True)
        if command in {"solve", "diagnose"}:
            child.add_argument("--confirm-numerics", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the command-line application and return a documented exit code."""

    args = build_parser().parse_args(argv)
    try:
        mapping = _load_mapping(args.config)
        controls_data = mapping.pop("controls", None)
        config = RendezvousConfig.from_mapping(mapping)
        if args.command in {"solve", "diagnose"}:
            if not args.confirm_numerics:
                print("numerical parameters require --confirm-numerics")
                return 6
        if args.command == "solve":
            result = apply_verification(
                config,
                solve_rendezvous(config, numerics_confirmed=True),
            )
        elif args.command == "diagnose":
            payload = conditioning_preflight(config)
            args.output_dir.mkdir(parents=True, exist_ok=True)
            (args.output_dir / "conditioning_preflight.json").write_text(
                json.dumps(payload, indent=2) + "\n", encoding="utf-8"
            )
            rows = payload["rows"]
            assert isinstance(rows, list) and rows
            with (args.output_dir / "conditioning_preflight.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            return 5 if payload["status"] == "failed_criterion" else 0
        else:
            if controls_data is None:
                raise ValueError("propagation configuration requires controls")
            controls = np.asarray(controls_data, dtype=float)
            phi, gamma = discretize_cw(
                mean_motion(config.mu, config.reference_radius), config.sample_time
            )
            states = simulate(config.initial_state, controls, phi, gamma)
            result = RendezvousResult(
                SolverStatus.OPTIMAL,
                "propagation completed",
                np.linspace(0.0, config.final_time, config.num_intervals + 1),
                states,
                controls,
            )
        _write_result(args.output_dir, config, result)
        return _exit_code(result.status)
    except (KeyError, TypeError, ValueError) as error:
        print(f"invalid configuration: {error}")
        return 2
    except PermissionError as error:
        print(str(error))
        return 6


if __name__ == "__main__":
    raise SystemExit(main())
