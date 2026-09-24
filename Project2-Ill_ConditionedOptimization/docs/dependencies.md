# Dependency and Platform Review

The confirmed baseline supports CPython 3.11–3.13 on macOS and Linux. Direct
dependencies are bounded by major version in `pyproject.toml`; the installed environment
used for the recorded results was inspected on 2026-09-23.

| Scope | Package | Installed version | Purpose | License metadata |
|---|---|---:|---|---|
| Runtime | NumPy | 2.3.5 | Dense arrays and linear algebra | BSD-3-Clause text |
| Runtime | SciPy | 1.16.3 | Matrix exponential and sparse solve | BSD-3-Clause text |
| Runtime | CVXPY | 1.9.3 | Convex modeling adapter | Apache-2.0 |
| Runtime | Clarabel | 0.11.1 | Conic solver backend | Apache-2.0 |
| Runtime | Matplotlib | 3.10.6 | Reproducible SVG plots | Matplotlib license |
| Runtime | PyYAML | 6.0.3 | Safe YAML configuration parsing | MIT |
| Development | Autograd | 1.9.1 | Independent derivative checks | MIT |
| Development | pytest | 8.4.2 | Automated tests | MIT |
| Development | mypy | 1.17.1 | Static type checks | MIT |
| Development | Ruff | 0.12.0 | Lint and formatting checks | MIT |

Setuptools and Wheel are build-system dependencies. Setuptools reports MIT license
metadata in this environment; Wheel does not expose a license field in its installed
package metadata, so its upstream license must be checked when performing a release
compliance review. This inventory is not legal advice and does not replace organization-
specific security or license approval.
