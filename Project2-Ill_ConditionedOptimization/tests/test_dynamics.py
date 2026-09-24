"""Tests for CW model construction and propagation."""

import numpy as np
from numpy.testing import assert_allclose

from dynamics import (
    augmented_exponential_discretization,
    continuous_matrices,
    discretize_cw,
    mean_motion,
    simulate,
)


def test_continuous_matrix_signs() -> None:
    n = 0.001
    a, b = continuous_matrices(n)
    assert a.shape == (6, 6)
    assert b.shape == (6, 3)
    assert a[3, 0] == 3 * n**2
    assert a[3, 4] == 2 * n
    assert a[4, 3] == -2 * n
    assert a[5, 2] == -(n**2)
    assert_allclose(b[3:], np.eye(3))


def test_analytic_matches_augmented_exponential() -> None:
    n = mean_motion(3.986004418e14, 6_778_137.0)
    analytic_phi, analytic_gamma = discretize_cw(n, 20.0)
    oracle_phi, oracle_gamma = augmented_exponential_discretization(n, 20.0)
    assert_allclose(analytic_phi, oracle_phi, rtol=1e-11, atol=1e-12)
    assert_allclose(analytic_gamma, oracle_gamma, rtol=1e-11, atol=1e-12)


def test_zero_and_small_step_limits() -> None:
    n = 0.001
    phi, gamma = discretize_cw(n, 0.0)
    assert_allclose(phi, np.eye(6))
    assert_allclose(gamma, np.zeros((6, 3)))
    h = 1e-6
    phi, gamma = discretize_cw(n, h)
    a, b = continuous_matrices(n)
    assert_allclose(phi, np.eye(6) + h * a, rtol=1e-10, atol=1e-12)
    assert_allclose(gamma, h * b, rtol=1e-6, atol=1e-12)


def test_semigroup_and_simulation() -> None:
    n = 0.001
    phi_10, _ = discretize_cw(n, 10.0)
    phi_20, gamma_20 = discretize_cw(n, 20.0)
    assert_allclose(phi_20, phi_10 @ phi_10, rtol=1e-12, atol=1e-12)
    controls = np.zeros((3, 3))
    initial = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])
    states = simulate(initial, controls, phi_20, gamma_20)
    assert states.shape == (4, 6)
    assert_allclose(states[-1], np.linalg.matrix_power(phi_20, 3) @ initial)
