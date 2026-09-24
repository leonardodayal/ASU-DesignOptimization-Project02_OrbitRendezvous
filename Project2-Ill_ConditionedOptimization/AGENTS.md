# AGENTS.md — MAE598 Project 2 (Ill-Conditioned Optimization)

## Ground rules
- Never invent shorthand/compressed terms for math objects. Use the same
  standard term for the same object every time (e.g. always "condition
  number κ", never a nickname).
- Use rigorous, standard optimization terminology appropriate for an
  undergrad engineering audience — no jargon that isn't defined.
- If you don't have information you need (a threshold, a data source, a
  design choice), say so immediately. Do not fabricate values, citations,
  or numerical results.
- Before running any solve with a nontrivial threshold, tolerance, step
  size, or termination criterion, propose the value and WAIT for human
  confirmation. Do not just proceed.
- Fix all random seeds. Every numerical result must be reproducible from
  the committed code.
- Flag any operation whose cost scales badly (e.g. forming a dense Hessian
  bigger than ~5000x5000, or an eigendecomposition of a huge matrix) before
  running it, not after.
- Any new solver or diagnostic function needs at least one corresponding
  pytest test before being considered complete.

## This project
- Problem family: Family I — trajectory optimization / long-horizon control
  (sensitivity through time, vanish/explode).
- Concrete setting: spacecraft rendezvous in low Earth orbit, relative
  motion governed by the Clohessy-Wiltshire (linearized) equations about a
  circular reference orbit. Chaser spacecraft maneuvers via continuous
  thrust control to rendezvous with a target at the origin of the LVLH
  frame.
- Decision variables: control sequence u_{0:T-1} (thrust accelerations,
  R^3 per step) over horizon T. State x_k = (relative position, relative
  velocity) in R^6, propagated by the discretized CW state-transition
  matrix.
- Knob: horizon length T (and/or orbital period ratio, if used as a
  secondary knob).
- Mechanism: sensitivity of terminal/cost gradient to early controls
  compounds through T applications of the CW state-transition matrix;
  since CW dynamics are linear, the reduced Hessian w.r.t. controls can be
  derived in closed form (no autodiff needed for the Hessian itself,
  though autodiff/finite-diff should still be used to cross-check it).
- Fix: multiple shooting and/or Gauss-Newton (iLQR-style) exploiting the
  block/banded structure of the linear dynamics, vs. naive single-shooting
  gradient descent as the baseline.
- Required deliverables: problem formulation, D1–D4 diagnostics (spectrum,
  intrinsic-κ test, baseline convergence, before/after fix) — see
  project2.md in this repo for exact definitions.
- Diagnostic kit helpers (spectrum, cond_after_diagonal_rescale,
  gradient_descent, conjugate_gradient) belong in src/conditioning.py,
  adapted from project2.md — extend these rather than reinventing them.
  Add src/dynamics.py for the CW state-transition matrix and simulator.
- Each deliverable's plots/tables are written to outputs/, not committed
  ad hoc elsewhere.
- Report lives in a single Markdown file or notebook per the submission
  spec.

## Workflow
- Read HANDOFF.md at the start of every session before doing anything.
- Update HANDOFF.md at the end of every session (progress, blockers, next
  steps) before ending.
- Commit small, reviewable diffs, scoped to one logical change. Open a PR
  for teammates to review rather than pushing to main.

## Project Structure & Module Organization
Keep numerical optimization code in `src/`, tests in `tests/`, and
reproducible experiments in `experiments/`. Store small, versioned input
data in `data/`; generated plots or reports go in `outputs/` (do not
commit large datasets or transient results). Prefer focused modules —
`src/objectives.py`, `src/solvers.py`, `src/conditioning.py`,
`src/dynamics.py` — over a single large script.

## Build, Test, and Development Commands
No build system is configured yet. When adding Python code, include a
`pyproject.toml` and document any new workflow in `README.md`. Do not
introduce undocumented one-off setup steps.

- `python -m venv .venv` — create an isolated environment.
- `python -m pip install -e '.[dev]'` — install once extras are defined.
- `python -m pytest` — run the full test suite.
- `python -m pytest tests/test_solvers.py -q` — run a focused module.

## Coding Style & Naming Conventions
Four-space indentation, PEP 8. `snake_case` for modules/functions/
variables, `PascalCase` for classes, `UPPER_SNAKE_CASE` for constants.
Add type hints to public APIs and short docstrings describing inputs,
outputs, and numerical assumptions. Configure any formatters/linters in
`pyproject.toml` and apply them consistently.

## Testing Guidelines
Use `pytest`, filenames `test_*.py`, mirroring source modules where
practical (`src/solvers.py` → `tests/test_solvers.py`). Cover convergence
behavior, invalid inputs, reproducibility, and numerically difficult
cases. Use explicit tolerances (`numpy.testing`, `pytest.approx`); avoid
exact floating-point equality.

## Commit & Pull Request Guidelines
Short, imperative commit subjects (e.g. `Add conjugate-gradient
baseline`), scoped to one logical change. PRs should explain motivation,
summarize the approach, list verification commands, and note effects on
convergence, accuracy, or runtime. Include plots or result tables when
experimental behavior changes.