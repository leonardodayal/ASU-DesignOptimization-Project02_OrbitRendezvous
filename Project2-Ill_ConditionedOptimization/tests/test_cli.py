"""Tests for command-line validation and confirmation behavior."""

import json

from cli import _jsonable, main


def test_solve_requires_explicit_confirmation() -> None:
    code = main(
        [
            "solve",
            "data/canonical.yaml",
            "--output-dir",
            "outputs/test-not-written",
        ]
    )
    assert code == 6


def test_invalid_json_returns_documented_exit_code(tmp_path) -> None:
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps({"mu": -1}), encoding="utf-8")
    code = main(
        [
            "solve",
            str(path),
            "--output-dir",
            str(tmp_path / "result"),
            "--confirm-numerics",
        ]
    )
    assert code == 2


def test_nonfinite_diagnostic_is_serialized_as_json_null() -> None:
    assert _jsonable(float("inf")) is None
