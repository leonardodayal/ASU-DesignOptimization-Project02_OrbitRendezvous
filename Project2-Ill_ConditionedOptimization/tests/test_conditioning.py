"""Tests for condition number κ diagnostics and iterative methods."""

import numpy as np
import pytest
from numpy.testing import assert_allclose

from conditioning import (
    cond_after_diagonal_rescale,
    conjugate_gradient,
    gradient_descent,
    preflight_dense_operation,
    spectrum,
)


def test_spectrum_and_diagonal_rescaling() -> None:
    matrix = np.diag([1.0, 10.0, 100.0])
    eigenvalues, condition_number = spectrum(matrix)
    assert_allclose(eigenvalues, [1.0, 10.0, 100.0])
    assert condition_number == pytest.approx(100.0)
    assert cond_after_diagonal_rescale(matrix) == pytest.approx(1.0)


def test_gradient_descent_converges_on_quadratic() -> None:
    matrix = np.diag([1.0, 4.0])
    rhs = np.array([1.0, -2.0])
    result = gradient_descent(
        lambda x: matrix @ x - rhs,
        np.zeros(2),
        step=2 / 5,
        tol=1e-8,
        maxit=200,
        f=lambda x: 0.5 * x @ matrix @ x - rhs @ x,
    )
    assert result.converged
    assert_allclose(result.solution, np.linalg.solve(matrix, rhs), rtol=1e-7, atol=1e-9)


def test_conjugate_gradient_converges() -> None:
    matrix = np.array([[4.0, 1.0], [1.0, 3.0]])
    rhs = np.array([1.0, 2.0])
    result = conjugate_gradient(matrix, rhs, tol=1e-12)
    assert result.converged
    assert_allclose(result.solution, np.linalg.solve(matrix, rhs), rtol=1e-12, atol=1e-12)


def test_invalid_and_cost_guard_paths() -> None:
    with pytest.raises(ValueError, match="positive-definite"):
        spectrum(np.diag([1.0, 0.0]))
    with pytest.raises(RuntimeError, match="requires confirmation"):
        preflight_dense_operation(5001)
    preflight_dense_operation(5001, confirmed=True)
