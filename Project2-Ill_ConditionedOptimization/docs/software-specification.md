# Spacecraft Intercept-Course Software Specification

**Status:** Revised baseline; required human confirmations are listed in Section 12.1

**Authoritative formulation:** [`docs/formulation.md`](formulation.md)

**Intended implementation:** Python library with a small command-line interface

**Safety classification:** Educational and analysis software; not flight-qualified

## 1. Purpose

The software shall calculate a dynamically feasible course for a chaser spacecraft to
intercept a target spacecraft. The baseline model is the forced
Clohessy-Wiltshire (CW) relative-motion model in a target-centered
local-vertical/local-horizontal (LVLH) frame. The
calculated course consists of a time history of relative states and piecewise-constant
commanded accelerations over a finite horizon.

The mathematical definitions, signs, matrices, and assumptions in
[`docs/formulation.md`](formulation.md) are normative. This specification defines the
software behavior around that formulation. It does not replace or modify the
mathematics.

In this document, **shall** denotes a required behavior, **should** denotes a strong
recommendation, and **may** denotes an optional behavior. Section 12 records the
binding design decisions that resolve the questions in the original draft.

## 2. Scope

### 2.1 In scope for the baseline release

- A target in a circular low Earth orbit, with Earth parameters supplied explicitly.
- A chaser represented by its six-component relative state in the target LVLH frame.
- A fixed final time and a uniform zero-order-hold control grid.
- Exact discrete propagation of the linear CW model.
- A convex quadratic objective with terminal, path-state, and control-effort terms.
- Exact or tolerance-based terminal interception constraints.
- A total acceleration-norm limit or componentwise acceleration limits.
- A structured application programming interface (API), a command-line entry point,
  and machine-readable results.
- Feasibility, residual, condition number κ, and convergence diagnostics.
- Reproducible examples and automated tests.
- A Family I long-horizon-control study in which the primary experimental variable is
  horizon length. The mechanism to investigate is whether state and control
  sensitivities propagated through time become very small (vanishing sensitivities) or
  very large (exploding sensitivities).

### 2.2 Out of scope for the baseline release

- Flight-software certification or autonomous onboard guidance.
- Noncircular target orbits, nonlinear two-body propagation, perturbations, navigation
  uncertainty, covariance propagation, and closed-loop control.
- Fuel depletion, spacecraft mass dynamics, finite burns, minimum impulse bits,
  attitude motion, thruster allocation, and plume impingement.
- Collision avoidance, keep-out zones, line-of-sight cones, and approach corridors.
- Variable-time or minimum-time interception.
- Conversion from Earth-centered inertial (ECI) states to LVLH states.

These exclusions are model boundaries, not assertions that the effects are negligible.

## 3. Terminology and coordinate convention

- **Target:** spacecraft defining the circular reference orbit and LVLH frame.
- **Chaser:** spacecraft whose commanded acceleration is optimized.
- **Course:** the returned node times, relative states, and acceleration commands.
- **Intercept:** terminal condition selected by `terminal_mode`.
- **Rendezvous:** terminal relative position and terminal relative velocity both zero
  (or within specified tolerances).
- **LVLH axes:** `+x` radially outward, `+z` along target orbit angular momentum, and
  `+y = +z x +x` along track.
- **Relative state:** `[x, y, z, x_dot, y_dot, z_dot]`, with position in metres and
  LVLH-relative velocity in metres per second.
- **Control:** `[u_x, u_y, u_z]`, the chaser acceleration resolved in LVLH, in metres per
  second squared.
- **Design decision variables:** the control sequence. In the multiple-shooting
  implementation, node states are auxiliary optimization variables constrained by the
  dynamics; they do not change the physical design-variable definition.
- **Horizon length:** the number of control intervals `N`. The formulation denotes
  physical final time by `T_f`; the software shall not use an ambiguous standalone `T`
  for both quantities. The primary study increases `N` at fixed sample time `h`, so
  `T_f = N*h` also increases.
- **Single shooting:** elimination of node states by repeated state propagation, leaving
  the control sequence as the optimization variables.
- **Multiple shooting:** optimization with node states as auxiliary variables joined by
  equality constraints for each dynamics interval. Its constraint matrices retain a
  sparse block-banded pattern: nonzero matrix blocks remain near the main diagonal.
- **Reduced Hessian:** the Hessian of the single-shooting objective with respect to the
  control sequence after eliminating the states and, when used, exact linear equality
  constraints.
- **Condition number κ:** for a symmetric positive-definite Hessian, the ratio of its
  largest eigenvalue to its smallest eigenvalue. Any generalized use based on singular
  values shall be labeled with the matrix and definition used.
- **Gauss–Newton method:** an iterative method that approximates the Hessian of a
  nonlinear least-squares objective using first derivatives of its residuals.

The software shall expose this convention in API documentation and output metadata. It
shall not accept a state without declaring or defaulting its frame and units.

## 4. User-visible inputs

The programmatic API shall accept one validated configuration object. The command-line
interface shall accept YAML and JSON configuration files. YAML shall be the primary
format used by the documented examples; TOML input is not required.

| Field | Type and units | Requirement |
|---|---|---|
| `central_body` | enum | Required and equal to `Earth` in the baseline |
| `orbit_regime` | enum | Required and equal to `low_earth_orbit`; metadata, not a numerical validity test |
| `mu` | positive float, m^3/s^2 | Gravitational parameter of the central body |
| `reference_radius` | positive float, m | Target circular-orbit radius from body center |
| `initial_state` | length-6 float vector | LVLH relative state in m and m/s |
| `final_time` | positive float, s | Fixed horizon `T_f` |
| `num_intervals` | positive integer | Number of uniform control intervals `N` |
| `terminal_mode` | enum | Optional; defaults to `rendezvous_exact`; also supports `rendezvous_tolerance` and `position_only` |
| `position_tolerance` | nonnegative float, m | Required for tolerance-based position interception |
| `velocity_tolerance` | nonnegative float, m/s | Required for tolerance-based rendezvous |
| `position_only_velocity_policy` | enum | Required with `position_only`: explicit `unconstrained` acknowledgment or `bounded` |
| `maximum_terminal_speed` | nonnegative float, m/s | Required when the position-only velocity policy is `bounded` |
| `acceleration_limit` | positive float, m/s^2 | Total 2-norm bound; mutually exclusive with axis limits |
| `axis_acceleration_lower` | length-3 vector, m/s^2 | Optional lower component limits |
| `axis_acceleration_upper` | length-3 vector, m/s^2 | Optional upper component limits |
| `q_path` | 6x6 symmetric positive-semidefinite matrix | Required path-state weight `Q` |
| `q_terminal` | 6x6 symmetric positive-semidefinite matrix | Required terminal weight `Q_f` |
| `r_control` | 3x3 symmetric positive-definite matrix | Required control weight `R` |
| `reference_states` | `(N+1)x6` array | Optional; defaults to the zero trajectory |
| `formulation` | enum | Optional `multiple_shooting` or `single_shooting`; defaults to `multiple_shooting` |
| `numerical_scales` | mapping | Required positive position, velocity, and control scales |
| `numerical_tolerances` | mapping | Required values listed in Section 6.1; human confirmation required before a solve |
| `solver_options` | mapping | Backend name, step/termination settings, and iteration/time limits; human confirmation required before a solve |
| `random_seed` | integer | Required whenever an experiment, test, solver, or diagnostic uses randomness |

The baseline shall require `central_body: Earth`, `orbit_regime: low_earth_orbit`, `mu`,
and `reference_radius`; it shall provide no implicit Earth constants. The orbit-regime
field records the intended study scope and shall not be presented as an independently
validated altitude classification. Convenience input using altitude and body radius is
deferred. The software shall not silently mix altitude and orbital radius.

All six `initial_state` components, `final_time`, `num_intervals`, all three weight
matrices, and exactly one acceleration-bound model shall be supplied explicitly. There
shall be no mission-specific default for these values.

### 4.1 Validation requirements

The software shall reject, with field-specific messages:

- non-finite values, invalid shapes, nonpositive `mu`, `reference_radius`,
  `final_time`, or acceleration limits;
- non-integral or nonpositive `num_intervals`;
- nonsymmetric weights outside a documented numerical tolerance;
- `Q` or `Q_f` with a materially negative eigenvalue, or `R` without strictly positive
  eigenvalues;
- simultaneous total-norm and componentwise acceleration limits;
- componentwise lower limits greater than upper limits;
- a reference-state array inconsistent with the grid;
- terminal tolerances missing for the selected terminal mode;
- unsupported units, frames, terminal modes, formulations, or solver options; and
- absent numerical scales or tolerances, or absent human confirmation of nontrivial
  numerical parameters before execution.

Small symmetry and definiteness roundoff shall be handled according to documented
tolerances and reported; the software shall not silently repair materially invalid
matrices.

## 5. Functional requirements

### 5.1 Model construction

- **FR-01:** The software shall calculate mean motion as
  `n = sqrt(mu / reference_radius^3)`.
- **FR-02:** It shall construct the continuous matrices `A` and `B` using equations
  (8)-(9) of the formulation.
- **FR-03:** It shall construct the zero-order-hold matrices `Phi` and `Gamma` for
  `h = final_time / num_intervals` using equations (10)-(12), or an algebraically
  equivalent matrix-exponential method.
- **FR-04:** It shall use the same `Phi` and `Gamma` for optimization, trajectory
  reconstruction, and residual checks.
- **FR-05:** It shall numerically protect the small-`n*h` expressions against
  cancellation, for example with stable power-series evaluations or a verified
  augmented matrix exponential.

### 5.2 Optimization problem

- **FR-06:** The baseline objective shall implement equation (15), including the
  factor of `h` on the rectangular-rule path and control costs.
- **FR-07:** The desired terminal state shall be zero in the baseline. The reference
  trajectory shall default to zero but may be replaced by a supplied sampled reference.
  A nonzero desired terminal state is outside the baseline API.
- **FR-08:** `rendezvous_exact` shall enforce final relative position and velocity as
  equality constraints.
- **FR-09:** `rendezvous_tolerance` shall enforce the separate Euclidean position and
  velocity bounds in equation (19).
- **FR-10:** `position_only` shall constrain final relative position exactly. It shall
  require the user to choose whether terminal velocity is unconstrained or bounded. A
  bounded policy shall require `maximum_terminal_speed`; an unconstrained policy shall
  require explicit acknowledgment. The result shall report terminal speed and warn that
  position-only interception is not rendezvous.
- **FR-11:** A total acceleration limit shall enforce `norm(u[k], 2) <= a_max` at every
  interval; component limits shall be enforced independently when selected.
- **FR-12:** The multiple-shooting formulation shall implement equation (21). The
  single-shooting formulation shall implement equations (22)-(25). Results from both
  shall agree within declared tolerances when both support the same problem.
- **FR-13:** The solver shall distinguish, as far as its backend permits, optimal,
  feasible but nonoptimal, infeasible, unbounded, iteration-limited, numerical failure,
  and invalid-input outcomes.
- **FR-14:** No course shall be labeled successful unless independent post-solve checks
  pass the configured dynamics, terminal, and control feasibility tolerances.
- **FR-15:** The baseline shall support fixed final time only. A utility may perform an
  outer sweep over fixed-time problems, but final time shall not be a decision variable
  inside the convex optimization problem.
- **FR-16:** Both multiple-shooting and single-shooting formulations shall be
  implemented. Null-space reduction is optional and shall not be required for baseline
  acceptance.
- **FR-17:** State/path constraints other than the selected terminal condition shall not
  be implemented in the baseline. Every successful result shall warn that collision,
  keep-out, line-of-sight, approach-corridor, closing-speed, and plume constraints were
  not evaluated.
- **FR-18:** The public model shall accept LVLH relative states only. ECI-to-LVLH
  conversion is outside the baseline API.
- **FR-19:** The required baseline optimization diagnostic shall be gradient descent on
  the single-shooting problem with the control sequence as its variables. Its step size,
  termination tolerance, and iteration limit shall be proposed and confirmed before it
  is run.
- **FR-20:** The required remedy shall be multiple shooting that exploits the sparse
  block-banded dynamics structure. A Gauss–Newton implementation organized in the style
  of iterative linear-quadratic regulation may be added as an extension, but it shall
  not replace the required multiple-shooting comparison.
- **FR-21:** Because the CW dynamics and quadratic cost are linear-quadratic, the
  single-shooting gradient and reduced Hessian with respect to the control sequence
  shall be derived and implemented in closed form. Automatic differentiation and
  central finite differences shall independently cross-check the implementation on
  small confirmed cases; they shall not replace the closed-form reduced Hessian.
- **FR-22:** The condition number κ study shall provide all four diagnostics required by
  `project2.md`: the Hessian eigenvalue spectrum; the intrinsic condition number κ test
  under increasing horizon length and diagonal rescaling; baseline gradient-descent
  convergence; and a before-and-after comparison with multiple shooting.
- **FR-23:** The constraint treatment used by the gradient-descent baseline shall be
  selected and confirmed before implementation. It shall not silently omit the terminal
  or acceleration constraints or introduce an unconfirmed penalty weight.

### 5.3 Results and diagnostics

The result object shall include:

- status and human-readable message;
- input configuration or an immutable normalized copy;
- mean motion, sample time, and node times;
- state array of shape `(N+1, 6)` and control array of shape `(N, 3)`;
- total objective and separate terminal, path-state, and control-effort contributions;
- maximum dynamics defect, terminal residuals, and maximum control-limit violation;
- solver name, termination reason, iterations, and elapsed solve time when available;
- formulation used and scaling applied;
- relevant estimated condition numbers κ and warnings; and
- software version and a reproducibility identifier for the configuration.

Each reported condition number κ shall name the matrix being measured. A condition
number κ for a single-shooting Hessian, reduced Hessian, equality-constraint Jacobian,
controllability matrix, or Karush-Kuhn-Tucker matrix shall not be presented simply as
“the problem condition number κ.”

The command-line interface shall return a nonzero exit status for invalid input,
infeasibility, solver failure, or failed post-solve verification. It shall serialize
the complete result as JSON. It may additionally produce state/control CSV files and
PNG or SVG diagnostic plots.

## 6. Numerical and optimization requirements

- SI units shall be used internally. User-facing non-SI conversion, if added, shall be
  explicit and tested.
- State and control variables shall be normalized before every solve using the
  user-confirmed positive position, velocity, and control scales. The specification does
  not assign numerical scale values because no such values have been supplied. All
  scales shall be recorded in the result, and both scaled and physical residuals shall
  be available.
- Scaling shall preserve the original physical problem. The result shall document scale
  factors and reconstruct outputs in SI units.
- Matrix symmetry shall be restored only for roundoff, such as `(H + H.T)/2`, and the
  pre-correction asymmetry shall be diagnosable.
- The software shall test terminal controllability and feasibility numerically and warn
  when the relevant matrix is rank deficient or nearly rank deficient. The numerical
  rank tolerance shall be supplied and confirmed before execution; this specification
  assigns no unsourced default.
- Solver feasibility tolerances and independent verification tolerances shall be
  separately configurable.
- Sparse matrices shall be retained in multiple shooting. Dense matrices may be used
  for small diagnostic problems but shall not be an accidental requirement.
- Deterministic inputs and options shall produce reproducible numerical results within
  backend and platform tolerances.
- A large condition number κ shall produce a diagnostic or warning, not an unverified
  answer. The warning threshold requires human confirmation before execution.
- If a minimum-energy equality-only analytic solution is implemented, it shall be used
  as a test oracle, not assumed valid for bounded or inequality-constrained cases.

For the associated condition number κ study, the software shall expose the
single-shooting Hessian or reduced Hessian, the terminal controllability matrix, and the
multiple-shooting equality-constraint Jacobian.
It shall support eigenvalue and singular-value diagnostics and diagonal (Jacobi)
rescaling so the intrinsic condition number κ tests in `project2.md` can be reproduced.

### 6.1 Numerical parameters requiring human confirmation

No numerical value has been supplied for the parameters below. The software and
experiments shall not invent defaults. Before any solve, the proposed values shall be
shown to a human and execution shall wait for explicit confirmation.

| Parameter | Purpose |
|---|---|
| Solver feasibility tolerance | Solver termination and feasibility classification |
| Solver optimality tolerance | Solver termination and optimality classification |
| Scaled dynamics equality tolerance | Independent dynamics-defect verification |
| Exact terminal position tolerance | Verification of exact-position modes, in metres |
| Exact terminal velocity tolerance | Verification of exact-rendezvous mode, in metres per second |
| Control-bound tolerance | Independent actuator-bound verification, in metres per second squared |
| Matrix symmetry tolerance | Validation of symmetric weight and Hessian matrices |
| Positive-semidefinite and positive-definite eigenvalue tolerances | Validation of weight-matrix curvature |
| Numerical rank tolerance | Rank and near-rank-deficiency diagnostics |
| Condition number κ warning threshold | Warning for a specifically named matrix |
| Severe condition number κ threshold | Escalated warning or refusal policy for a specifically named matrix |
| Iteration and time limits | Solver termination safeguards |
| Any step size | Required by a selected iterative diagnostic or baseline optimizer |

Tolerance-based terminal modes shall use the user-supplied mission tolerances as the
physical acceptance limits and shall still report the solver's raw residuals. Mission
tolerances do not replace separately confirmed solver and verification tolerances.

The result shall always report the maximum relative separation
`max_k(norm(r_k, 2) / reference_radius)` and the horizon-to-orbit-period ratio
`final_time / (2*pi/n)`. Because no mission-validated CW applicability thresholds are
available, the baseline shall not invent pass/fail cutoffs. It shall instead emit an
`assumptions_unverified` warning listing circular orbit, two-body dynamics, small
relative separation, and negligible perturbations.

### 6.2 Optimization implementation

The required baseline algorithm is single-shooting gradient descent, and the required
structural remedy is multiple shooting. The optimization modeling interface and backend
used to solve the multiple-shooting problem have not been supplied or approved. They
shall be selected only after a documented dependency, license, problem-class, and
platform review followed by human confirmation. An adapter boundary shall isolate
solver statuses and options. Unsupported solver/problem combinations shall fail
validation rather than silently changing constraints.

Equation (15), using rectangular-rule cost quadrature, is the only baseline cost
discretization. An exact discrete quadratic cost may be added later only as an explicitly
named alternative with separate tests; it shall not silently replace equation (15).

Controls shall always represent zero-order-held acceleration, never force or impulsive
delta-v. Reports may include descriptive total acceleration expenditure
`sum_k(norm(u_k, 2) * h)` in m/s, labeled `integrated_acceleration_magnitude`; this
derived quantity shall not change the optimization model.

Before allocating a dense Hessian with either dimension greater than approximately
5000, or before performing a dense eigenvalue decomposition of a matrix with dimension
greater than approximately 5000, the software shall report the proposed matrix,
dimensions, asymptotic computational cost, and estimated memory. It shall stop before
the operation unless a human explicitly confirms it. Methods that avoid forming the
full matrix, sparse methods, or methods that compute only selected eigenvalues should be
offered when they answer the same question.

## 7. Required examples

The implementation shall ship reproducible examples under `experiments/` and document
how to run them in `README.md`. Each example shall save its normalized inputs, solver
status, feasibility residuals, and course. Confirmed numerical values inserted into the
example configurations shall define educational examples only; they shall not become
mission defaults.

### EX-01: Unforced propagation sanity check

Given a circular Earth reference orbit and a nonzero initial relative state, set every
control to zero and compare discrete propagation against the continuous CW state
transition. This example need not solve an intercept problem. It demonstrates axis
signs, units, and exact propagation.

Expected evidence: node states agree with independent propagation to a specified
floating-point tolerance, and `Phi(0)` approaches the identity while `Gamma(0)`
approaches zero.

### EX-02: Feasible fixed-time rendezvous

The executable example shall use this schema, but its numerical values require human
confirmation before the example is run:

```yaml
central_body: Earth
orbit_regime: low_earth_orbit
mu: HUMAN_CONFIRMATION_REQUIRED
reference_radius: HUMAN_CONFIRMATION_REQUIRED
initial_state: HUMAN_CONFIRMATION_REQUIRED
final_time: HUMAN_CONFIRMATION_REQUIRED
num_intervals: HUMAN_CONFIRMATION_REQUIRED
terminal_mode: rendezvous_exact
acceleration_limit: HUMAN_CONFIRMATION_REQUIRED
q_path: HUMAN_CONFIRMATION_REQUIRED
q_terminal: HUMAN_CONFIRMATION_REQUIRED
r_control: HUMAN_CONFIRMATION_REQUIRED
numerical_scales: HUMAN_CONFIRMATION_REQUIRED
numerical_tolerances: HUMAN_CONFIRMATION_REQUIRED
solver_options: HUMAN_CONFIRMATION_REQUIRED
```

The committed executable file shall replace every placeholder with a confirmed value
and numeric array. The example shall verify terminal position, terminal velocity,
dynamics defects, and thrust bounds rather than prescribe a specific control history or
objective value before feasibility has been checked.

### EX-03: Infeasible rendezvous

Shorten the horizon or reduce available acceleration until the course is infeasible.
The program shall return an infeasible outcome, no successful course, and a useful
diagnostic. This case guards against presenting a solver's last iterate as a solution.

### EX-04: Condition number κ and long-horizon study

Run a fixed low Earth orbit scenario over increasing horizon length. The primary study
shall increase `num_intervals` while holding sample time fixed, so that `final_time`
increases. The horizon-to-orbital-period ratio may be reported as a secondary
experimental variable. Report the condition number κ of each explicitly named matrix
before and after diagonal (Jacobi) rescaling, then compare single-shooting gradient
descent with multiple shooting. This example shall produce the four diagnostics required
by `project2.md`: eigenvalue spectrum, intrinsic condition number κ test, baseline
convergence, and before-and-after remedy comparison. It shall include at least one small
case whose matrix or condition number κ is independently checked by hand.

The long-horizon sweep shall report terminal residuals, objective components, integrated
acceleration magnitude, solve time, and peak memory for every horizon. It is not a grid
refinement study because sample time is held fixed while final time changes. If grid
refinement is also studied, it shall be a separate experiment that holds final time
fixed while increasing `num_intervals`; the report shall distinguish cost-quadrature
changes from changes in the feasible control space.

## 8. Required architecture

The following separation is required unless a reviewed change preserves the same module
responsibilities:

```text
src/
  models.py          validated configuration and result types
  dynamics.py        CW matrices, exact zero-order-hold discretization, and simulator
  objectives.py      quadratic cost assembly and cost breakdown
  constraints.py     terminal and actuator constraint assembly
  formulations.py    multiple-shooting and single-shooting problem construction
  solvers.py         backend adapters and common solver-status definitions
  conditioning.py    scaling, rank, spectrum, and condition number κ diagnostics
  verification.py    independent post-solve residual checks
  cli.py             configuration loading and result export
tests/
experiments/
outputs/
```

Public functions and classes shall use type hints and concise docstrings describing
shapes, units, frames, and numerical assumptions. Core model construction shall be
independent of a particular optimization backend. Backend-specific statuses and options
shall be isolated behind adapters.

`src/conditioning.py` shall adapt and extend, rather than independently reimplement, the
helpers in `project2.md`: `spectrum`, `cond_after_diagonal_rescale`,
`gradient_descent`, and `conjugate_gradient`. The prose documentation for each helper
shall use the full mathematical term, including “condition number κ,” even where the
required function name is abbreviated. `src/dynamics.py` shall contain the CW state
transition matrices and simulator. Each helper and each public solver or diagnostic
function shall have at least one corresponding `pytest` test.

The project shall define a `pyproject.toml` with the approved runtime and development
dependencies, supported Python versions, test configuration, formatting and linting
settings, and a console entry point. The `README.md` shall document environment setup,
example commands, configuration schema, output interpretation, and known model
limitations. Python versions and supported operating systems require human confirmation
after the dependency review; this specification assigns none without that information.

The submission report shall live in one Markdown file or one notebook, as permitted by
the submission specification. All plots and tables used by the report shall be generated
reproducibly into `outputs/` and shall not be written ad hoc elsewhere.

## 9. Unit and integration test suggestions

Tests shall use `pytest`, fixed and recorded seeds wherever randomness occurs, and
explicit human-confirmed floating-point tolerances. Tests should favor physical
invariants and residuals over snapshots of solver-specific iterates. Deterministic tests
shall not introduce randomness unnecessarily.

### 9.1 Dynamics unit tests

- Verify dimensions and selected signs/entries of `A` and `B`.
- Compare `Phi` and `Gamma` with an independently computed augmented matrix exponential
  across representative `n` and `h` values.
- Verify the semigroup identity `Phi(h1 + h2) ~= Phi(h2) @ Phi(h1)`.
- Verify `Phi(0) = I`, `Gamma(0) = 0`, and the small-step limits
  `Phi ~= I + A*h`, `Gamma ~= B*h`.
- Compare one-step propagation against high-accuracy integration of the continuous ODE
  with constant control.
- Test decoupled cross-track harmonic motion and a known in-plane unforced CW solution.
- Exercise very small `n*h` to reveal catastrophic cancellation.

### 9.2 Validation unit tests

- Parameterize invalid shapes, NaN/Inf values, invalid horizons, invalid interval
  counts, inconsistent limits, and invalid reference arrays.
- Accept positive-semidefinite `Q` and `Q_f`, reject materially indefinite matrices,
  and reject singular or indefinite `R`.
- Confirm that total-norm and axis limits cannot both be active accidentally.
- Confirm all error messages identify the offending field and expected units or shape.

### 9.3 Objective and constraint unit tests

- Hand-calculate the cost of a small trajectory and verify every cost component,
  including the `h/2` factor.
- Compare analytic gradients/Hessians, if supplied, with central finite differences or
  automatic differentiation on small well-scaled cases.
- Verify the closed-form single-shooting gradient and reduced Hessian against both
  automatic differentiation and central finite differences on a small confirmed case.
- Verify stacked-state and stacked-control ordering against equations (6) and (23).
- Check terminal modes separately, including points exactly on tolerance boundaries.
- Verify both acceleration-bound models at zero, interior, boundary, and violating
  inputs.
- Compare single-shooting states from equation (22) with repeated one-step propagation.

### 9.4 Solver and end-to-end tests

- Solve a small unconstrained or equality-only minimum-energy problem and compare with
  a direct Karush-Kuhn-Tucker system or Moore-Penrose-pseudoinverse solution.
- Solve the same supported case with multiple-shooting and single-shooting formulations
  and compare physical courses, objective values, and residuals.
- Confirm the feasible example passes independent post-solve verification.
- Confirm the infeasible example does not return success or export a nominal course.
- Force iteration-limit and numerical-failure paths and verify common solver statuses and
  command-line exit codes.
- Verify a deliberately perturbed returned course fails post-solve checks.
- Run at least one case with a large condition number κ to verify scaling, warnings, and
  condition number κ diagnostics remain finite and correctly labeled.
- Verify serialized input-output round trips and reproducibility metadata.
- Verify that every solver and diagnostic function has at least one corresponding test
  and that the dense-operation preflight guard stops before an unconfirmed costly
  allocation or eigenvalue decomposition.
- Test `spectrum`, `cond_after_diagonal_rescale`, `gradient_descent`, and
  `conjugate_gradient` in `tests/test_conditioning.py`, including invalid inputs and at
  least one analytically checkable matrix.

### 9.5 Property and regression tests

- With sufficiently loose bounds, increasing available acceleration shall not turn a
  feasible case infeasible because of the model alone.
- For the minimum-energy problem, adding feasible control freedom by refining the grid
  should not increase the exact optimum solely because the feasible set was restricted;
  account for changes in the discrete cost definition when interpreting this property.
- Rotating only the decoupled cross-track sign convention consistently should preserve
  scalar cost and feasibility; inconsistent rotations should be detected in reference
  test data.
- Record regression values for model matrices and verified objective/residual scalars,
  not full solver logs or brittle iteration sequences.

## 10. Quality, safety, and operational considerations

- Every output shall state that the CW solution is valid only within its modeling
  assumptions and is not a flight command product.
- Logs and result files shall retain units, coordinate frame, time origin, and model
  version.
- Exceptions shall not be used as ordinary solver statuses, but invalid configurations
  and programmer errors should fail loudly.
- Dependencies should be minimal, pinned or bounded deliberately, and checked for
  license and security suitability under the human-confirmed dependency policy. The
  repository shall maintain a dependency and license inventory.
- Continuous integration should run unit tests, static checks, and the small examples on
  supported Python versions. Large experiments should be separately marked.
- Generated plots and transient results belong in `outputs/`; large data shall not be
  committed.
- Performance tests should track model-assembly time, solve time, and peak memory as
  `N` grows. Multiple-shooting implementation tests should guard against unintended
  dense allocation.
- Input files are untrusted data: parsers should be safe, schema validation should occur
  before numerical work, and output paths should not be derived unsafely from input.

## 11. Acceptance criteria

The baseline release is acceptable when:

1. Every resolved requirement in this specification is implemented or explicitly
   deferred in a traceable issue.
2. The configuration schema and public API declare units, frame, shapes, and defaults.
3. EX-01 through EX-04 are reproducible from documented commands.
4. Automated tests cover validation, dynamics, objective assembly, terminal and control
   constraints, solver status handling, and post-solve verification.
5. All shipped feasible examples satisfy independently checked physical residuals and
   bounds at documented tolerances.
6. Multiple-shooting and single-shooting implementations agree on at least one shared
   test case.
7. Infeasible and failed solves cannot be mistaken for successful courses.
8. Condition number κ diagnostics name their matrices and the Project 2 study
   demonstrates growth and survival under diagonal rescaling, or clearly reports that
   the chosen case does not meet the intrinsic condition number κ criterion.
9. Documentation states assumptions, limitations, installation steps, test commands,
   example commands, and output meanings.
10. Every solver and diagnostic function has at least one corresponding `pytest` test.
11. Every numerical result is reproducible from committed code, inputs, dependency
    information, and fixed recorded random seeds.
12. Costly dense operations are identified before execution and require explicit human
    confirmation as specified in Section 6.2.
13. The single-shooting gradient and reduced Hessian are implemented in closed form and
    cross-checked against automatic differentiation and central finite differences.
14. The baseline convergence diagnostic uses single-shooting gradient descent, and the
    before-and-after remedy comparison uses multiple shooting.
15. The submission report is one Markdown file or one notebook, with reproducibly
    generated plots and tables written to `outputs/`.

## 12. Design decisions and confirmations

The structural suggestions associated with the original uncertainty list are adopted as
baseline requirements. Where the repository provides no numerical value or software
selection, this specification records the required human confirmation instead of
inventing one.

| ID | Adopted decision |
|---|---|
| `DEC-01` | The software shall support selectable terminal modes and shall default to full rendezvous. `position_only` means exact position coincidence; the user shall explicitly select an unconstrained or bounded terminal-velocity policy, and the terminal speed shall be reported with a warning. |
| `DEC-02` | The baseline study shall use a circular low Earth orbit. Users shall identify Earth and low Earth orbit explicitly and shall supply `mu` and reference-orbit radius; there shall be no silent numerical Earth constants. |
| `DEC-03` | The baseline shall accept relative states already expressed in the documented LVLH frame. Inertial-state conversion is out of scope. |
| `DEC-04` | Users shall supply all six initial-state components. No mission initial state shall be assumed. |
| `DEC-05` | Final time shall be fixed within each convex solve. An optional outer parameter sweep may compare fixed final times; joint time optimization is out of scope. |
| `DEC-06` | Users shall supply both `final_time` and `num_intervals`. The primary Family I study variable shall be horizon length: EX-04 shall increase `num_intervals` at fixed sample time and shall report the horizon-to-orbital-period ratio as a possible secondary variable. |
| `DEC-07` | Tolerance terminal mode shall require explicit position and velocity tolerances. Mission tolerances and internal solver/verification tolerances shall remain distinct and shall all be reported. |
| `DEC-08` | The software shall support either a spherical acceleration limit or asymmetric componentwise limits. Exactly one bound model shall be required, with no spacecraft-specific default value. |
| `DEC-09` | Mass depletion, minimum impulse bit, duty cycle, pointing, slew, bandwidth, thruster allocation, and plume effects are excluded. Results shall warn that these effects were not evaluated. |
| `DEC-10` | Users shall supply `Q`, `Q_f`, and `R` explicitly. An automatic weight-selection helper is not part of the baseline; numerical variable scaling shall follow Section 6 without changing the physical objective. |
| `DEC-11` | The reference trajectory shall default to zero. Users may instead supply one state vector per grid node. |
| `DEC-12` | No keep-out, line-of-sight, approach-corridor, closing-speed, collision-avoidance, or other path constraints shall be present in the educational baseline. Results shall state this limitation. |
| `DEC-13` | The software shall always report relative-separation and horizon/orbit-period ratios and shall emit `assumptions_unverified`. It shall not claim a CW validity pass/fail threshold without mission-validated criteria. |
| `DEC-14` | The physical design decision variables are the control sequence. Multiple shooting and single shooting shall both be delivered and cross-checked; multiple-shooting node states are auxiliary optimization variables. Null-space reduction remains optional. |
| `DEC-15` | Single-shooting gradient descent is the required baseline algorithm, and multiple shooting is the required structural remedy. The modeling interface and backend for multiple shooting, supported Python versions, and supported operating systems require a documented dependency and license review plus human confirmation. Solver-specific behavior shall be isolated behind an adapter. |
| `DEC-16` | Rank, verification, definiteness, solver, and condition number κ parameters shall be proposed and explicitly confirmed by a human before any solve. No numerical defaults shall be fabricated; all confirmed values shall be recorded and tested. |
| `DEC-17` | Equation (15), with rectangular-rule cost, shall be the sole baseline cost discretization. Any future exact discrete cost shall be a separately named and tested option. |
| `DEC-18` | Controls shall be zero-order-held accelerations. Reports may provide integrated acceleration magnitude in m/s for interpretation, but shall not call controls forces or impulsive delta-v commands. |
| `DEC-19` | Complete results shall be written as JSON. State/control CSV and PNG/SVG diagnostic plots are optional supplemental outputs. |
| `DEC-20` | The product shall be treated as educational analysis software for the Project 2 Family I long-horizon-control and condition number κ study, not operational mission analysis or flight software. |
| `DEC-21` | Every use of randomness shall have a fixed, recorded seed. Every numerical result shall be reproducible from committed code and inputs. |
| `DEC-22` | Forming a dense Hessian with either dimension greater than approximately 5000, or applying a dense eigenvalue decomposition to a matrix with dimension greater than approximately 5000, shall be flagged before execution and shall require explicit human confirmation. |
| `DEC-23` | The single-shooting gradient and reduced Hessian with respect to the control sequence shall be implemented in closed form and cross-checked using automatic differentiation and central finite differences. |
| `DEC-24` | The required `project2.md` helpers shall be adapted in `src/conditioning.py`; the CW state-transition matrices and simulator shall be implemented in `src/dynamics.py`. Every solver and diagnostic function shall have a corresponding `pytest` test. |
| `DEC-25` | The four required diagnostics are the eigenvalue spectrum, intrinsic condition number κ test, baseline convergence, and before-and-after multiple-shooting remedy comparison. The report shall be one Markdown file or one notebook, with generated artifacts in `outputs/`. |

### 12.1 Required confirmations before implementation or execution

The following information is not present in the repository and shall not be fabricated:

- **Before implementing the solver adapter:** modeling interface, solver backend,
  supported Python versions, supported operating systems, and dependency/license policy.
- **Before implementing the gradient-descent experiment:** how terminal and acceleration
  constraints are represented in the single-shooting baseline; no constraint may be
  dropped and no penalty weight may be introduced without confirmation.
- **Before completing and running EX-01 through EX-04:** Earth gravitational parameter,
  circular reference-orbit radius, initial relative state, final time, interval count,
  acceleration bounds, objective matrices, and any reference trajectory.
- **Before every solve:** all scales, tolerances, step sizes, iteration limits, time
  limits, and termination criteria listed in Section 6.1.
- **When using position-only interception:** the terminal-velocity policy and, if
  bounded, the maximum terminal speed.
- **Before a costly dense operation:** confirmation required by Section 6.2 after its
  matrix dimensions, asymptotic cost, and estimated memory are reported.

Until these confirmations are supplied, documentation and structural code may be
developed, but no affected solve or numerical claim shall be presented as verified.

## 13. Traceability to the formulation

| Software area | Formulation equations |
|---|---|
| LVLH frame and state/control definitions | (1)-(6) |
| Mean motion and continuous CW model | (7)-(9) |
| Exact zero-order-hold discretization | (10)-(12) |
| Cost and weights | (13)-(16) |
| Initial, terminal, actuator, and path constraints | (17)-(21) |
| Single-shooting formulation and controllability | (22)-(26) |
| Convexity and problem classification | (27) |
| Source open questions resolved by Section 12 | (28) |
