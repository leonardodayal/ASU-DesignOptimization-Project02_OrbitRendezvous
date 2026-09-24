# Orbit Rendezvous Optimization

Educational Python software for fixed-time spacecraft rendezvous with the linearized
Clohessy–Wiltshire equations. The project compares single-shooting gradient descent
with a sparse multiple-shooting remedy and reports named condition number κ diagnostics.
It is not flight software.

## Setup

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'
```

## Verification

```bash
python -m pytest
python -m ruff check src tests experiments
python -m mypy src
```

## Commands

Every numerical solve requires explicit confirmation of the values committed in its
configuration:

```bash
orbit-rendezvous solve data/canonical.yaml \
  --output-dir outputs/canonical \
  --confirm-numerics
```

The deterministic infeasible case is run the same way with `data/infeasible.yaml`; it
must return a nonzero exit status and must not export a nominal course.

Run the approved long-horizon preflight with:

```bash
orbit-rendezvous diagnose data/canonical.yaml \
  --output-dir outputs/report \
  --confirm-numerics
```

The confirmed canonical case activates its acceleration bound in the single-shooting
diagnostic. In accordance with the approved failure policy, the study records this fact
and stops without changing weights, bounds, horizons, or tolerances.

The YAML/JSON configuration schema is defined in
[`docs/software-specification.md`](docs/software-specification.md). Inputs declare the
Earth orbit, six-component LVLH state, fixed time grid, terminal mode, one acceleration
bound model, all objective matrices, numerical scales, tolerances, and solver limits.
`formulation` selects a genuinely condensed `single_shooting` model or a sparse
`multiple_shooting` model. Only the confirmed `CLARABEL` backend is accepted.

Successful solves write `result.json`, `states.csv`, and `controls.csv`. The JSON file
contains normalized inputs, units/frame metadata, a reproducibility hash, objective
components, solver status, and independent physical residuals. Infeasible solves write
only `result.json`, so a backend's final iterate cannot be mistaken for a course. Exit
codes are 0 for success, 2 for invalid input, 3 for infeasible/unbounded, 4 for solver
failure, 5 for failed verification or diagnostic criteria, and 6 when numerical
confirmation is absent.

The exact-propagation and full report-data experiments are also reproducible:

```bash
PYTHONPATH=src python experiments/run_propagation_check.py
PYTHONPATH=src python experiments/run_conditioning_study.py
```

The conditioning command intentionally exits nonzero for the confirmed canonical
inputs because its inactive-acceleration-bound prerequisite fails. See
[`report.md`](report.md) for the result and [`docs/dependencies.md`](docs/dependencies.md)
for the dependency review.

## Model boundary

Inputs and outputs use SI units and the target-centered LVLH frame: radial `+x`,
along-track `+y`, and orbit-normal `+z`. The model assumes a circular Earth orbit,
two-body dynamics, small relative separation, and zero-order-held acceleration. It does
not evaluate collision, keep-out, line-of-sight, plume, attitude, mass depletion, or
navigation uncertainty constraints.
