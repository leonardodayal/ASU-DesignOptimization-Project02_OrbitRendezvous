# Repository Guidelines

## Project Structure & Module Organization

This repository is currently a blank project scaffold. As implementation is added, keep numerical optimization code in `src/`, tests in `tests/`, and reproducible experiments in `experiments/`. Store small, versioned input data in `data/` and generated plots or reports in `outputs/`; do not commit large datasets or transient results. Prefer focused modules such as `src/objectives.py`, `src/solvers.py`, and `src/conditioning.py` over a single large script.

## Build, Test, and Development Commands

No build system or dependency manifest is configured yet. When adding Python code, include a `pyproject.toml` and document any new workflow in `README.md`. Recommended commands are:

- `python -m venv .venv` — create an isolated environment.
- `python -m pip install -e '.[dev]'` — install the project and development tools once extras are defined.
- `python -m pytest` — run the complete test suite.
- `python -m pytest tests/test_solvers.py -q` — run a focused test module.

Do not introduce undocumented one-off setup steps.

## Coding Style & Naming Conventions

Use four-space indentation and follow PEP 8. Name modules, functions, and variables with `snake_case`; classes with `PascalCase`; and constants with `UPPER_SNAKE_CASE`. Add type hints to public APIs and short docstrings that describe inputs, outputs, and numerical assumptions. If formatters or linters are introduced, configure them in `pyproject.toml` and apply them consistently.

## Testing Guidelines

Use `pytest` and name files `test_*.py`. Mirror source modules where practical: `src/solvers.py` should have `tests/test_solvers.py`. Cover convergence behavior, invalid inputs, reproducibility, and numerically difficult cases. Use explicit tolerances with `numpy.testing` or `pytest.approx`; avoid exact equality for floating-point results.

## Commit & Pull Request Guidelines

There is no Git history from which to infer an existing convention. Use short, imperative commit subjects, for example `Add conjugate-gradient baseline`. Keep commits scoped to one logical change. Pull requests should explain the motivation, summarize the approach, list verification commands, and note any effects on convergence, accuracy, or runtime. Include plots or result tables when experimental behavior changes, and link related issues when available.
