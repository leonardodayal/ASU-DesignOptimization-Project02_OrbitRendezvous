"""Condition number κ diagnostics and baseline iterative methods."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

FloatArray = NDArray[np.float64]


def _symmetric_matrix(matrix: ArrayLike) -> FloatArray:
    array = np.asarray(matrix, dtype=float)
    if array.ndim != 2 or array.shape[0] != array.shape[1]:
        raise ValueError("matrix must be square")
    if not np.all(np.isfinite(array)):
        raise ValueError("matrix must be finite")
    if not np.allclose(array, array.T, rtol=1e-12, atol=0.0):
        raise ValueError("matrix must be symmetric")
    return (array + array.T) / 2.0


def preflight_dense_operation(dimension: int, confirmed: bool = False) -> None:
    """Refuse unconfirmed dense operations above the specified dimension guard."""

    if dimension > 5000 and not confirmed:
        gibibytes = dimension * dimension * 8 / 1024**3
        raise RuntimeError(
            f"dense {dimension}x{dimension} operation requires confirmation; "
            f"one float64 matrix needs approximately {gibibytes:.2f} GiB"
        )


def spectrum(
    matrix: ArrayLike, *, large_operation_confirmed: bool = False
) -> tuple[FloatArray, float]:
    """Return ascending eigenvalues and condition number κ of a positive-definite matrix."""

    array = _symmetric_matrix(matrix)
    preflight_dense_operation(array.shape[0], large_operation_confirmed)
    eigenvalues = np.asarray(np.linalg.eigvalsh(array), dtype=np.float64)
    if eigenvalues[0] <= 0:
        raise ValueError("condition number κ requires a positive-definite matrix")
    return eigenvalues, float(eigenvalues[-1] / eigenvalues[0])


def cond_after_diagonal_rescale(
    matrix: ArrayLike, *, large_operation_confirmed: bool = False
) -> float:
    """Return condition number κ after symmetric diagonal rescaling."""

    array = _symmetric_matrix(matrix)
    preflight_dense_operation(array.shape[0], large_operation_confirmed)
    diagonal = np.diag(array)
    if np.any(diagonal <= 0):
        raise ValueError("diagonal rescaling requires a positive diagonal")
    scaled = array / np.sqrt(np.outer(diagonal, diagonal))
    return float(np.linalg.cond(scaled))


@dataclass(frozen=True)
class GradientDescentResult:
    """Gradient-descent iterate and convergence history."""

    solution: FloatArray
    iterates: FloatArray
    objectives: FloatArray
    gradient_norms: FloatArray
    converged: bool
    iterations: int


def gradient_descent(
    grad: Callable[[FloatArray], FloatArray],
    x0: ArrayLike,
    step: float,
    tol: float,
    maxit: int,
    f: Callable[[FloatArray], float] | None = None,
    x_star: ArrayLike | None = None,
) -> GradientDescentResult:
    """Run fixed-step gradient descent using a relative gradient stopping test."""

    if step <= 0 or tol <= 0 or maxit <= 0:
        raise ValueError("step, tol, and maxit must be positive")
    x = np.asarray(x0, dtype=float).copy()
    reference_norm = max(float(np.linalg.norm(grad(x))), 1.0)
    iterates: list[FloatArray] = [x.copy()]
    objectives: list[float] = []
    gradient_norms: list[float] = []
    converged = False
    for _ in range(maxit + 1):
        gradient = np.asarray(grad(x), dtype=float)
        norm = float(np.linalg.norm(gradient))
        gradient_norms.append(norm)
        if f is not None:
            objectives.append(float(f(x)))
        if norm <= tol * reference_norm:
            converged = True
            break
        if len(iterates) > maxit:
            break
        x = x - step * gradient
        iterates.append(x.copy())
    return GradientDescentResult(
        x,
        np.asarray(iterates),
        np.asarray(objectives),
        np.asarray(gradient_norms),
        converged,
        len(iterates) - 1,
    )


@dataclass(frozen=True)
class ConjugateGradientResult:
    """Conjugate-gradient solution and relative residual history."""

    solution: FloatArray
    relative_residuals: FloatArray
    converged: bool


def conjugate_gradient(
    matrix: ArrayLike,
    right_hand_side: ArrayLike,
    tol: float = 1e-8,
    maxit: int | None = None,
) -> ConjugateGradientResult:
    """Solve a symmetric positive-definite linear system by conjugate gradients."""

    a = _symmetric_matrix(matrix)
    b = np.asarray(right_hand_side, dtype=float)
    if b.shape != (a.shape[0],) or tol <= 0:
        raise ValueError("invalid right-hand side or tolerance")
    limit = a.shape[0] if maxit is None else maxit
    if limit <= 0:
        raise ValueError("maxit must be positive")
    x = np.zeros_like(b)
    residual = b.copy()
    direction = residual.copy()
    residual_square = float(residual @ residual)
    reference = max(float(np.linalg.norm(b)), 1.0)
    history: list[float] = [float(np.linalg.norm(residual) / reference)]
    converged = history[-1] <= tol
    for _ in range(limit):
        if converged:
            break
        product = a @ direction
        curvature = float(direction @ product)
        if curvature <= 0:
            raise ValueError("matrix is not positive definite")
        alpha = residual_square / curvature
        x += alpha * direction
        residual -= alpha * product
        new_square = float(residual @ residual)
        history.append(float(np.sqrt(new_square) / reference))
        converged = history[-1] <= tol
        direction = residual + (new_square / residual_square) * direction
        residual_square = new_square
    return ConjugateGradientResult(x, np.asarray(history), converged)
