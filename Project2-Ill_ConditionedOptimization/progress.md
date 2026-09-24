# Project 2 Rubric Progress

The software implementation is substantially ahead of the submission report. Against
the rubric in `project2.md`, the current submission is estimated at **43/100**, with a
plausible grading range of **40–50**. The largest issue is that D3 and D4—the
computational core of the assignment—are explicitly deferred.

## Rubric assessment

| Rubric category | Estimated score | Current evidence | Main gap |
|---|---:|---|---|
| Problem motivation and real-world relevance | 4/10 | The spacecraft rendezvous problem is identified. | The report does not explain who makes the decision, why it matters operationally, or why optimization quality matters. |
| Formulation | 8/20 | `docs/formulation.md` contains comprehensive mathematics and the implementation follows it. | The rubric requires explicit variables, objective, constraints, and classification in the single submission report. `report.md` currently only links to the separate formulation. |
| Ill-conditioning mechanism and intrinsic condition number κ justification | 15/20 | The report identifies Family I. The reduced-Hessian condition number κ grows from `10.97` to `35,474.58` and remains large after diagonal rescaling, reaching `189,405.40`. | The report needs a mathematical derivation connecting powers of the state-transition matrix, the state-control sensitivity matrix, and the reduced-Hessian spectrum. |
| Effect demonstration: D1 and D3 | 4/20 | The report gives condition number κ values and includes an independently checked small Hessian. | There is no log-scale eigenvalue-spectrum plot, gradient-descent convergence curve, or iteration/wall-clock result at a fixed tolerance. |
| Proposed solution and before/after demonstration: D4 | 5/20 | A sparse multiple-shooting Karush–Kuhn–Tucker remedy is implemented and described. | There is no before/after convergence, effective-rate, runtime, or conditioning comparison. |
| Reproducibility | 4/5 | The project includes runnable experiments, explicit configurations, deterministic inputs, tests, dependency bounds, and generated outputs. | The GitHub Actions workflow is nested below the repository root and is therefore inactive. Dependencies are bounded but not exactly locked. |
| Presentation and clarity | 3/5 | The report is concise and organized under the required headings. | It is too abbreviated, contains almost no rendered mathematics, and delegates the required formulation to another document. |
| **Estimated total** | **43/100** |  | **Plausible range: 40–50.** |

## Standard diagnostic kit status

### D1 — Spectrum: incomplete

The report includes selected extremal eigenvalues, condition number κ values, and a
two-dimensional analytical eigenvalue check. It does not include the required
logarithmic eigenvalue-spectrum plot for the selected reduced Hessian.

### D2 — Intrinsic test: essentially complete

Horizon length is used as the structural knob. The reduced-Hessian condition number κ
grows strongly with the horizon and does not collapse under symmetric Jacobi scaling.
This is good evidence that the observed ill-conditioning is intrinsic rather than only
a unit-scaling artifact.

### D3 — Effect: missing

The single-shooting gradient-descent implementation exists, but the report contains no
executed convergence experiment, objective-gap or gradient-norm curve, or iteration and
wall-clock measurements at a fixed tolerance.

### D4 — Fix: missing

The multiple-shooting remedy is implemented, but it has not been demonstrated against
the gradient-descent baseline with common convergence metrics and tolerances.

## Current blocker

The approved diagnostic required the `0.01 m/s²` acceleration bound to remain inactive
so that unconstrained, null-space-reduced gradient descent would solve the same physical
problem without projection or a penalty. The unconstrained optimum violates that bound
at every approved horizon. Following the approved failure policy, the experiment stops
and reports the conflict without silently changing the bound, weights, horizons, or
tolerances.

Completing D3 and D4 therefore requires an explicit decision to use either:

1. a bound-aware first-order baseline, such as projected gradient descent; or
2. a separately approved diagnostic case in which the acceleration bound is inactive.

## Highest-impact remaining work

1. Resolve the acceleration-bound conflict so D3 and D4 can operate on the same
   well-defined problem.
2. Generate a logarithmic reduced-Hessian eigenvalue-spectrum plot for D1.
3. Generate objective-gap or gradient-norm convergence curves for gradient descent,
   including iterations and wall-clock time to the confirmed fixed tolerance.
4. Generate the same convergence evidence for the remedy and present the D4
   before/after comparison.
5. Move the explicit decision variables, dimensions, units, bounds, objective,
   constraints, and classification into `report.md` so it independently satisfies the
   single-report requirement.
6. Expand the Family I derivation using, for example,

   $$
   X=\mathcal A x_0+\mathcal B U,
   \qquad
   H=\mathcal B^T\bar Q\mathcal B+\bar R,
   $$

   and explain how unequal early- and late-control sensitivities widen the eigenvalue
   spectrum as the horizon grows.
7. Expand the motivation to identify the decision makers, operational consequences,
   and importance of reliable optimization.
8. Explain why multiple shooting addresses the identified mechanism: it retains local,
   sparse dynamics constraints rather than forming long products of transition matrices
   in a dense condensed sensitivity.
9. Move the CI workflow to the repository-root `.github/workflows/` directory so GitHub
   Actions discovers it.

## Overall conclusion

The repository has a strong and well-tested numerical foundation, including the
required conditioning helpers, exact dynamics, both shooting formulations, independent
verification, and a high-accuracy small-matrix hand check. The largest remaining risk
is not software correctness but missing rubric evidence. In particular, the absence of
D3 and D4 puts approximately **35–40 points** at risk if the project is submitted in its
current form.
