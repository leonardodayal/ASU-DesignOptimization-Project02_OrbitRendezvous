# Spacecraft Rendezvous Software Implementation Plan

## Objective

Implement the software defined by `docs/formulation.md` and
`docs/software-specification.md` as an educational Python package that:

- computes fixed-time Clohessy-Wiltshire spacecraft rendezvous courses;
- supports both condensed single shooting and sparse multiple shooting;
- independently verifies dynamics, terminal conditions, and actuator limits;
- demonstrates the Family I long-horizon ill-conditioning mechanism; and
- produces reproducible Project 2 diagnostics and a single Markdown report.

Implementation is staged so the physical model and numerical building blocks are
verified before solver and conditioning conclusions are added.

## Confirmed design decisions

### Software stack

- Python 3.11–3.13 on macOS and Linux.
- NumPy and SciPy for dense and sparse numerical operations.
- CVXPY with Clarabel for the complete constrained convex problem.
- Autograd as a development-only independent derivative oracle.
- Pytest for tests, Ruff for linting, and mypy for static type checks.
- Flat modules under `src/`, tests under `tests/`, reproducible scripts under
  `experiments/`, inputs under `data/`, and generated artifacts under `outputs/`.

### Canonical physical case

- Earth gravitational parameter:
  `mu = 3.986004418e14 m^3/s^2`.
- Circular reference radius: `6,778,137 m`.
- Initial LVLH state:
  `[100 m, -200 m, 20 m, 0 m/s, 0 m/s, 0 m/s]`.
- Fixed sample time: `h = 20 s`.
- Acceleration magnitude limit: `0.01 m/s^2`.
- Family I horizon sweep:
  `N = [15, 30, 60, 120, 240]`, with `T_f = N h`.

### Objective and scaling

- Position scale: `100 m`.
- Velocity scale: `100 n m/s`, where `n` is the reference-orbit mean motion.
- Control scale: `0.01 m/s^2`.
- Path-state matrix `Q`: diagonal inverse-square weights based on the confirmed
  position and velocity scales.
- Control matrix `R`: diagonal inverse-square weights based on the control scale.
- Terminal matrix: `Q_f = 0`, because exact terminal rendezvous is enforced as a
  constraint.
- Equation (15) of `docs/formulation.md`, including rectangular-rule factors of `h`,
  is the only baseline cost discretization.

### Numerical policy

- Solver feasibility tolerance: `1e-8`.
- Solver optimality tolerance: `1e-8`.
- Gradient-descent relative gradient tolerance: `1e-8`.
- Gradient-descent step:
  `alpha = 2 / (lambda_max + lambda_min)`.
- Gradient-descent iteration limit: `200,000`.
- Scaled dynamics verification tolerance: `1e-7`.
- Exact terminal position tolerance: `1e-5 m`.
- Exact terminal velocity tolerance: `1e-8 m/s`.
- Control-bound allowance:
  `max(1e-10 m/s^2, 1e-7 * acceleration_limit)`.
- Matrix symmetry relative tolerance: `1e-12`.
- Numerical rank threshold: standard matrix-dimension-scaled machine-epsilon singular
  value threshold.
- Condition number κ warning threshold: `1e10`.
- Severe condition number κ threshold: `1e14`.
- Clarabel iteration limit: `500`.
- Solver time limit: `60 s`.
- Dense Hessian formation or dense eigendecomposition above dimension `5000` must be
  reported and explicitly confirmed before execution.

### Constraint treatment and failure policy

- The baseline optimizer is fixed-step gradient descent on the null-space-reduced,
  exact-terminal single-shooting quadratic.
- The terminal equality is never omitted or replaced by an unconfirmed penalty.
- The baseline experiment may proceed only when its resulting controls satisfy the
  confirmed acceleration bound within the confirmed allowance.
- The structural remedy is sparse multiple shooting solved through its
  Karush–Kuhn–Tucker system and cross-checked with CVXPY/Clarabel.
- If the canonical case violates the inactive-bound prerequisite or an intrinsic
  condition number κ criterion, stop the affected experiment, report the result, and do
  not automatically change weights, limits, horizons, tolerances, or solver settings.

## Planned architecture

```text
src/
  models.py          configuration, enums, validation, and result types
  dynamics.py        CW matrices, exact zero-order-hold discretization, simulator
  objectives.py      state-control mapping, quadratic assembly, cost breakdown
  constraints.py     terminal controllability, null-space reduction, actuator checks
  formulations.py    condensed single shooting and sparse multiple shooting
  conditioning.py    spectrum, scaling, gradient descent, conjugate gradient, guards
  diagnostics.py     reproducible D1/D2 preflight and small analytical checks
  solvers.py         gradient-descent, sparse KKT, and CVXPY/Clarabel adapters
  verification.py    independent physical and scaled residual checks
  cli.py             safe configuration parsing, commands, exports, and exit codes
tests/
experiments/
data/
outputs/
```

Backend-specific behavior is isolated in `src/solvers.py`. Core dynamics, objective,
constraint, and formulation construction remain independent of CVXPY.

## Implementation stages

### Stage 1 — Project configuration and data models

1. Add `pyproject.toml` with supported Python versions, bounded dependencies, console
   entry point, test configuration, linting, and type-checking settings.
2. Define configuration enums and typed data classes.
3. Validate shapes, finite values, positive parameters, weight symmetry and curvature,
   terminal-mode requirements, mutually exclusive actuator models, solver backend, and
   reference-trajectory dimensions.
4. Add canonical and deterministic infeasible YAML configurations.

### Stage 2 — Dynamics and propagation

1. Implement mean motion and continuous forced CW matrices.
2. Implement the analytic exact zero-order-hold `Phi` and `Gamma` matrices.
3. Protect small `n h` cases with an independently computed augmented matrix
   exponential.
4. Implement deterministic state propagation for zero-order-held controls.
5. Test signs, dimensions, limiting behavior, semigroup behavior, and agreement with
   the augmented-exponential oracle.

### Stage 3 — Objective and constraints

1. Assemble the affine stacked-state mapping from the complete control sequence.
2. Assemble the closed-form condensed quadratic

   $$
   J(U)=\frac12 U^T H U+g^T U+c.
   $$

3. Implement objective-component reporting for terminal, path-state, and control terms.
4. Construct the terminal controllability matrix and exact terminal target.
5. Compute a particular terminal-feasible control and an orthonormal null-space basis,
   yielding `U = U_p + Z w`.
6. Implement total-norm and componentwise actuator-bound residual checks.
7. Cross-check the closed-form gradient and Hessian with Autograd and central finite
   differences on a small confirmed case.

### Stage 4 — Formulations and solvers

1. Implement the condensed single-shooting formulation with exact terminal
   null-space reduction.
2. Implement the multiple-shooting quadratic with sparse block dynamics equalities.
3. Implement the required fixed-step gradient-descent baseline.
4. Implement a one-step sparse Karush–Kuhn–Tucker solve for the equality-only
   multiple-shooting quadratic.
5. Implement the complete constrained CVXPY/Clarabel adapter with normalized state and
   control variables.
6. Support exact rendezvous, tolerance rendezvous, position-only interception, spherical
   acceleration limits, and componentwise acceleration limits.
7. Normalize backend termination states into common result statuses.
8. Verify that single- and multiple-shooting constrained solutions agree on a shared
   case.

### Stage 5 — Independent verification and CLI

1. Recompute physical dynamics defects independently of solver constraints.
2. Check scaled dynamics residuals, terminal position and velocity, and actuator-bound
   violations against their separately confirmed tolerances.
3. Prevent any course from reporting success when independent verification fails.
4. Report maximum relative separation, horizon/orbit-period ratio, terminal speed, and
   integrated acceleration magnitude without inventing CW-validity thresholds.
5. Implement `solve`, `diagnose`, and `propagate` commands.
6. Require `--confirm-numerics` before solves and numerical diagnostics.
7. Export normalized configuration, version, units/frame metadata, reproducibility hash,
   result status, objective components, residuals, and warnings to JSON.
8. Export state and control CSV files only when a valid trajectory exists.
9. Use distinct nonzero exit codes for invalid input, infeasibility, solver failure,
   failed verification, and missing confirmation.

### Stage 6 — Conditioning diagnostics

1. Adapt the required `project2.md` helpers in `src/conditioning.py`:
   `spectrum`, `cond_after_diagonal_rescale`, `gradient_descent`, and
   `conjugate_gradient`.
2. Add explicit guards before costly dense operations.
3. Build the reduced single-shooting Hessian for every approved horizon.
4. Record its eigenvalue extrema and condition number κ.
5. Apply symmetric Jacobi rescaling and record the resulting condition number κ.
6. Solve the unconstrained reduced quadratic only to test whether the confirmed
   acceleration bound remains inactive.
7. Add a four-interval cross-track reduction whose two-by-two eigenvalues can be checked
   analytically against the numerical eigensolver.
8. If the inactive-bound prerequisite passes, generate:
   - D1: logarithmic eigenvalue spectrum;
   - D2: condition number κ versus horizon before and after diagonal scaling;
   - D3: gradient-descent objective-gap and/or gradient-norm convergence;
   - D4: before/after evidence for the multiple-shooting remedy.
9. If the prerequisite fails, record the failed criterion and stop D3/D4 without
   retuning.

### Stage 7 — Tests, examples, and report

1. Add a corresponding Pytest test for every public solver and diagnostic function.
2. Cover invalid inputs, infeasibility, reproducibility, numerically difficult cases,
   solver-status mapping, serialization, and deliberately corrupted trajectories.
3. Use explicit floating-point tolerances and avoid exact equality for numerical
   results.
4. Add reproducible scripts for the independent propagation check and long-horizon
   conditioning study.
5. Generate report tables, JSON data, CSV data, and SVG plots under `outputs/`.
6. Write a single `report.md` with the six sections required by `project2.md`:
   motivation, formulation, mechanism, effect, remedy, and assumptions.
7. Document setup, verification, commands, schema, output interpretation, limitations,
   dependencies, and licenses.
8. Configure continuous integration for Python 3.11–3.13 on macOS and Linux.

## Verification commands

```bash
python -m pip install -e '.[dev]'
python -m ruff check src tests experiments
python -m mypy src
python -m pytest

orbit-rendezvous solve data/canonical.yaml \
  --output-dir outputs/canonical \
  --confirm-numerics

orbit-rendezvous solve data/infeasible.yaml \
  --output-dir outputs/infeasible \
  --confirm-numerics

PYTHONPATH=src python experiments/run_propagation_check.py
PYTHONPATH=src python experiments/run_conditioning_study.py
```

The infeasible example must return a nonzero exit status and export no nominal course.
The conditioning experiment may also return a documented nonzero status when an
approved prerequisite fails; that outcome is evidence of the no-retuning policy rather
than an instruction to change the case automatically.

## Completion criteria

The implementation is complete when:

- both shooting formulations and all supported terminal/actuator modes are available;
- successful courses pass independent verification;
- infeasible and failed solves cannot be mistaken for successful courses;
- analytic dynamics and derivatives agree with independent numerical or automatic
  differentiation checks;
- the required condition number κ helpers and small analytical check are tested;
- D1–D4 are produced when their approved prerequisites hold, or any failed prerequisite
  is reported transparently without retuning;
- all lint, type, and unit-test commands pass;
- numerical examples are reproducible from committed inputs and documented commands;
- generated artifacts live under `outputs/`; and
- progress, deferred work, and any repository-level blockers are recorded in
  `HANDOFF.md`.
