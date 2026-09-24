# Spacecraft Intercept-Course Software Specification

**Status:** Draft for clarification  
**Authoritative formulation:** [`docs/formulation.md`](formulation.md)  
**Intended implementation:** Python library with a small command-line interface  
**Safety classification:** Educational and analysis software; not flight-qualified

## 1. Purpose

The software shall calculate a dynamically feasible course for a chaser spacecraft to
intercept a target spacecraft. The baseline model is the forced
Clohessy-Wiltshire (CW) relative-motion model in a target-centered LVLH frame. The
calculated course consists of a time history of relative states and piecewise-constant
commanded accelerations over a finite horizon.

The mathematical definitions, signs, matrices, and assumptions in
[`docs/formulation.md`](formulation.md) are normative. This specification defines the
software behavior around that formulation. It does not replace or modify the
mathematics.

In this document, **shall** denotes a required behavior, **should** denotes a strong
recommendation, and **may** denotes an optional behavior. Items marked `UNC-*` are open
questions whose answers may change the requirements or acceptance criteria.

## 2. Scope

### 2.1 In scope for the baseline release

- A target in a circular two-body reference orbit.
- A chaser represented by its six-component relative state in the target LVLH frame.
- A fixed final time and a uniform zero-order-hold control grid.
- Exact discrete propagation of the linear CW model.
- A convex quadratic objective with terminal, path-state, and control-effort terms.
- Exact or tolerance-based terminal interception constraints.
- A total acceleration-norm limit or componentwise acceleration limits.
- A structured programmatic API, a command-line entry point, and machine-readable
  results.
- Feasibility, residual, conditioning, and convergence diagnostics.
- Reproducible examples and automated tests.

### 2.2 Out of scope unless added after clarification

- Flight-software certification or autonomous onboard guidance.
- Noncircular target orbits, nonlinear two-body propagation, perturbations, navigation
  uncertainty, covariance propagation, and closed-loop control.
- Fuel depletion, spacecraft mass dynamics, finite burns, minimum impulse bits,
  attitude motion, thruster allocation, and plume impingement.
- Collision avoidance, keep-out zones, line-of-sight cones, and approach corridors.
- Variable-time or minimum-time interception.
- Conversion from Earth-centered inertial states to LVLH states.

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

The software shall expose this convention in API documentation and output metadata. It
shall not accept a state without declaring or defaulting its frame and units.

## 4. User-visible inputs

The programmatic API shall accept one validated configuration object. A serialized
configuration should use JSON, YAML, or TOML with the following logical fields.

| Field | Type and units | Requirement |
|---|---|---|
| `mu` | positive float, m^3/s^2 | Gravitational parameter of the central body |
| `reference_radius` | positive float, m | Target circular-orbit radius from body center |
| `initial_state` | length-6 float vector | LVLH relative state in m and m/s |
| `final_time` | positive float, s | Fixed horizon `T_f` |
| `num_intervals` | positive integer | Number of uniform control intervals `N` |
| `terminal_mode` | enum | `rendezvous_exact`, `rendezvous_tolerance`, or `position_only` |
| `position_tolerance` | nonnegative float, m | Required for tolerance-based position interception |
| `velocity_tolerance` | nonnegative float, m/s | Required for tolerance-based rendezvous |
| `acceleration_limit` | positive float, m/s^2 | Total 2-norm bound; mutually exclusive with axis limits |
| `axis_acceleration_lower` | length-3 vector, m/s^2 | Optional lower component limits |
| `axis_acceleration_upper` | length-3 vector, m/s^2 | Optional upper component limits |
| `q_path` | 6x6 symmetric PSD matrix | Path-state weight `Q` |
| `q_terminal` | 6x6 symmetric PSD matrix | Terminal weight `Q_f` |
| `r_control` | 3x3 symmetric PD matrix | Control weight `R` |
| `reference_states` | `(N+1)x6` array | Optional sampled reference trajectory |
| `formulation` | enum | `sparse` or `condensed`; see `UNC-14` |
| `solver_options` | mapping | Backend name, tolerances, and iteration/time limits |

The interface should also accept `reference_altitude` and central-body radius as a
convenience, but it shall internally form and report `reference_radius`. It shall not
silently mix altitude and orbital radius.

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
- terminal tolerances missing for the selected terminal mode; and
- unsupported units, frames, terminal modes, formulations, or solver options.

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
  cancellation, for example with stable series, `sinc`-style functions, or a verified
  augmented matrix exponential.

### 5.2 Optimization problem

- **FR-06:** The baseline objective shall implement equation (15), including the
  factor of `h` on the rectangular-rule path and control costs.
- **FR-07:** The default desired terminal state and default reference state shall be
  zero unless a different supported value is supplied.
- **FR-08:** `rendezvous_exact` shall enforce final relative position and velocity as
  equality constraints.
- **FR-09:** `rendezvous_tolerance` shall enforce the separate Euclidean position and
  velocity bounds in equation (19).
- **FR-10:** `position_only` shall constrain only final relative position. Its treatment
  of terminal velocity remains subject to `UNC-01`.
- **FR-11:** A total acceleration limit shall enforce `norm(u[k], 2) <= a_max` at every
  interval; component limits shall be enforced independently when selected.
- **FR-12:** The sparse formulation shall implement equation (21). The condensed
  formulation shall implement equations (22)-(25). Results from both shall agree within
  declared tolerances when both support the same problem.
- **FR-13:** The solver shall distinguish, as far as its backend permits, optimal,
  feasible but nonoptimal, infeasible, unbounded, iteration-limited, numerical failure,
  and invalid-input outcomes.
- **FR-14:** No course shall be labeled successful unless independent post-solve checks
  pass the configured dynamics, terminal, and control feasibility tolerances.

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
- relevant condition estimates and warnings; and
- software version and a reproducibility identifier for the configuration.

Condition estimates shall name the matrix being measured. A reported condition number
for a condensed Hessian, reduced Hessian, equality Jacobian, controllability matrix, or
KKT matrix shall not be presented simply as “the problem condition number.”

The command-line interface shall return a nonzero exit status for invalid input,
infeasibility, solver failure, or failed post-solve verification. It should serialize
the complete result and may additionally produce CSV tables and plots.

## 6. Numerical and optimization requirements

- SI units shall be used internally. User-facing non-SI conversion, if added, shall be
  explicit and tested.
- Position, velocity, acceleration, and cost scales shall be normalized before solve
  when necessary. Both scaled and physical residuals shall be available.
- Scaling shall preserve the original physical problem. The result shall document scale
  factors and reconstruct outputs in SI units.
- Matrix symmetry should be restored only for roundoff, such as `(H + H.T)/2`, and the
  pre-correction asymmetry should be diagnosable.
- The software shall test terminal controllability/feasibility numerically and warn when
  the relevant matrix is rank deficient or nearly rank deficient.
- Solver feasibility tolerances and independent verification tolerances shall be
  separately configurable.
- Sparse matrices should be retained in direct transcription. Dense matrices may be
  used for small diagnostic problems but should not be an accidental requirement.
- Deterministic inputs and options shall produce reproducible numerical results within
  backend and platform tolerances.
- Ill-conditioning shall produce a diagnostic or warning, not an unverified answer.
- If a minimum-energy equality-only analytic solution is implemented, it shall be used
  as a test oracle, not assumed valid for bounded or inequality-constrained cases.

For the associated ill-conditioning study, the software should expose the condensed or
reduced Hessian, the terminal controllability matrix, and the sparse equality Jacobian.
It should support eigenvalue/singular-value diagnostics and Jacobi rescaling so the
intrinsic-conditioning tests in `project2.md` can be reproduced.

## 7. Required examples

The implementation shall ship reproducible examples under `experiments/` and document
how to run them in `README.md`. Each example shall save its normalized inputs, solver
status, feasibility residuals, and course. Numerical values below are illustrative and
shall not become defaults until the related uncertainties are resolved.

### EX-01: Unforced propagation sanity check

Given a circular Earth reference orbit and a nonzero initial relative state, set every
control to zero and compare discrete propagation against the continuous CW state
transition. This example need not solve an intercept problem. It demonstrates axis
signs, units, and exact propagation.

Expected evidence: node states agree with independent propagation to a specified
floating-point tolerance, and `Phi(0)` approaches the identity while `Gamma(0)`
approaches zero.

### EX-02: Feasible fixed-time rendezvous

Use an explicitly labeled demonstration case such as:

```yaml
central_body: Earth  # metadata only
mu: 3.986004418e14
reference_radius: 6778137.0
initial_state: [100.0, -200.0, 20.0, 0.0, 0.0, 0.0]
final_time: 1200.0
num_intervals: 60
terminal_mode: rendezvous_exact
acceleration_limit: 0.01
q_path: zeros(6, 6)
q_terminal: zeros(6, 6)
r_control: identity(3)
```

The executable file shall contain numeric arrays rather than the illustrative
`zeros(...)` and `identity(...)` notation. The example shall verify terminal position,
terminal velocity, dynamics defects, and thrust bounds rather than prescribe a specific
control history or objective value before feasibility has been checked.

### EX-03: Infeasible rendezvous

Shorten the horizon or reduce available acceleration until the course is infeasible.
The program shall return an infeasible outcome, no successful course, and a useful
diagnostic. This case guards against presenting a solver's last iterate as a solution.

### EX-04: Conditioning study

Run a fixed physical scenario over increasing `N` and/or horizon length. Report the
condition number of each explicitly named matrix before and after coordinate/Jacobi
scaling, then compare a baseline optimizer with the selected remedy. This example shall
produce the D1-D4 evidence required by `project2.md` and include at least one small case
whose matrix or condition estimate is independently checked.

## 8. Suggested architecture

The following separation is recommended:

```text
src/
  models.py          validated configuration and result types
  dynamics.py        CW matrices and exact ZOH discretization
  objectives.py      quadratic cost assembly and cost breakdown
  constraints.py     terminal and actuator constraint assembly
  formulations.py    sparse and condensed problem construction
  solvers.py         backend adapters and normalized statuses
  conditioning.py    scaling, rank, spectrum, and condition diagnostics
  verification.py    independent post-solve residual checks
  cli.py             configuration loading and result export
tests/
experiments/
outputs/
```

Public functions and classes shall use type hints and concise docstrings describing
shapes, units, frames, and numerical assumptions. Core model construction should be
independent of a particular optimization backend. Backend-specific statuses and options
should be isolated behind adapters.

The project should define a `pyproject.toml` with runtime and development dependencies,
supported Python versions, test configuration, formatting/lint settings, and a console
entry point. The `README.md` should document environment setup, example commands,
configuration schema, output interpretation, and known model limitations.

## 9. Unit and integration test suggestions

Tests shall use `pytest`, fixed seeds where randomness is unavoidable, and explicit
floating-point tolerances. Tests should favor physical invariants and residuals over
snapshots of solver-specific iterates.

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
- Accept PSD `Q` and `Q_f`, reject materially indefinite matrices, and reject singular
  or indefinite `R`.
- Confirm that total-norm and axis limits cannot both be active accidentally.
- Confirm all error messages identify the offending field and expected units or shape.

### 9.3 Objective and constraint unit tests

- Hand-calculate the cost of a small trajectory and verify every cost component,
  including the `h/2` factor.
- Compare analytic gradients/Hessians, if supplied, with central finite differences or
  automatic differentiation on small well-scaled cases.
- Verify stacked-state and stacked-control ordering against equations (6) and (23).
- Check terminal modes separately, including points exactly on tolerance boundaries.
- Verify both acceleration-bound models at zero, interior, boundary, and violating
  inputs.
- Compare condensed states from equation (22) with repeated one-step propagation.

### 9.4 Solver and end-to-end tests

- Solve a small unconstrained/equality-only minimum-energy problem and compare with a
  direct KKT or pseudoinverse solution.
- Solve the same supported case with sparse and condensed formulations and compare
  physical courses, objective values, and residuals.
- Confirm the feasible example passes independent post-solve verification.
- Confirm the infeasible example does not return success or export a nominal course.
- Force iteration-limit and numerical-failure paths and verify normalized statuses and
  command-line exit codes.
- Verify a deliberately perturbed returned course fails post-solve checks.
- Run at least one ill-conditioned case to verify scaling, warnings, and condition
  diagnostics remain finite and correctly labeled.
- Verify serialized input-output round trips and reproducibility metadata.

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
  license/security suitability.
- Continuous integration should run unit tests, static checks, and the small examples on
  supported Python versions. Large experiments should be separately marked.
- Generated plots and transient results belong in `outputs/`; large data shall not be
  committed.
- Performance tests should track model-assembly time, solve time, and peak memory as
  `N` grows. Sparse formulation tests should guard against unintended dense allocation.
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
6. Sparse and condensed implementations agree on at least one shared test case, if both
   are selected for the baseline.
7. Infeasible and failed solves cannot be mistaken for successful courses.
8. Conditioning diagnostics name their matrices and the Project 2 study demonstrates
   growth and survival, or clearly reports that the chosen case does not meet the
   intrinsic-ill-conditioning criterion.
9. Documentation states assumptions, limitations, installation steps, test commands,
   example commands, and output meanings.

## 12. Uncertainties requiring clarification

The following questions intentionally remain open. Decisions should be recorded beside
each identifier and reflected in this specification before implementation is considered
stable.

| ID | Uncertainty / decision needed | Suggested baseline |
|---|---|---|
| `UNC-01` | Does “intercept” require position coincidence only, full zero-relative-velocity rendezvous, or selectable modes? If position-only, is terminal speed unconstrained or bounded? | Support selectable modes; default to full rendezvous for safety and consistency with the formulation. |
| `UNC-02` | Which central body and constants shall be supported? | Accept `mu` and orbital radius explicitly; provide no silent Earth default. |
| `UNC-03` | Will users supply LVLH relative states, or must the software convert inertial states and epochs? | Require LVLH state in the baseline. |
| `UNC-04` | What mission scenario supplies `initial_state`? | No default; require all six components. |
| `UNC-05` | Is final time fixed, searched over externally, or optimized jointly? | Fixed time only; an outer parameter sweep may be added without claiming a convex variable-time formulation. |
| `UNC-06` | What horizon, interval count, and convergence/refinement rule are required? | Require both `T_f` and `N`; add an example refinement study. |
| `UNC-07` | What terminal position and velocity tolerances define operational success? | Require explicit tolerances for tolerance mode; keep solver and mission tolerances distinct. |
| `UNC-08` | What are the physical actuator bounds, and is the limit spherical or componentwise/asymmetric? | Support either model, require one, and do not invent a vehicle value. |
| `UNC-09` | Must mass, propellant, duty cycle, minimum impulse bit, pointing, slew, bandwidth, or plume constraints be modeled? | Exclude and warn in the baseline. |
| `UNC-10` | How shall `Q`, `Q_f`, and `R` be chosen and nondimensionalized? | Require explicit matrices; provide a documented scale-based helper only after its policy is approved. |
| `UNC-11` | Is there a nonzero reference trajectory? | Default to zero while allowing a sampled reference. |
| `UNC-12` | Which path constraints are required: keep-out region, line of sight, approach corridor, closing speed, or collision avoidance? | None in the educational baseline; do not use it operationally. |
| `UNC-13` | What validity threshold should warn that CW linearization is inappropriate (relative distance/orbit radius, target eccentricity, or horizon)? | Always report ratios; choose thresholds only with mission guidance. |
| `UNC-14` | Which formulation is required for delivery: sparse, condensed, null-space, or multiple implementations for comparison? | Implement sparse and condensed for validation and conditioning study; null-space is optional. |
| `UNC-15` | Which optimization backend(s), licensing constraints, and supported platforms apply? | Keep a backend adapter and select an available convex solver after dependency review. |
| `UNC-16` | What numerical feasibility, rank, conditioning, and warning thresholds are acceptable? | Make them explicit configuration/profile values and establish defaults from tests. |
| `UNC-17` | Is the rectangular-rule cost mandatory, or should an exact discrete quadratic cost be offered? | Implement equation (15) first; label any exact-cost option as a distinct discretization. |
| `UNC-18` | Are controls accelerations, forces, or impulsive delta-v commands in user-facing reports? | Acceleration under zero-order hold only; integrate for descriptive delta-v without changing the model. |
| `UNC-19` | What output formats and plots are required for grading or downstream use? | JSON result plus optional CSV and PNG/SVG diagnostic plots. |
| `UNC-20` | Is this solely a Project 2 conditioning demonstration, or expected to evolve toward mission analysis? | Treat as educational analysis software unless requirements and verification scope are expanded. |

## 13. Traceability to the formulation

| Software area | Formulation equations |
|---|---|
| LVLH frame and state/control definitions | (1)-(6) |
| Mean motion and continuous CW model | (7)-(9) |
| Exact zero-order-hold discretization | (10)-(12) |
| Cost and weights | (13)-(16) |
| Initial, terminal, actuator, and path constraints | (17)-(21) |
| Condensed formulation and controllability | (22)-(26) |
| Convexity and problem classification | (27) |
| Source open questions | (28) |

