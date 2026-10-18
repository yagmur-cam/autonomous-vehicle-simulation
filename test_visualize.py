"""
test_visualize.py - Unit test suite for visualize.py
Tests the analytical geometry of compute_covariance_ellipse.
"""
import os
import pytest
import numpy as np
from visualize import compute_covariance_ellipse, plot_static_study, create_animation


def test_axis_aligned_ellipse():
    """

    Feed a diagonal covariance matrix with known unequal variances.
    The ellipse must align with the axes (angle 0 deg) and have exact 2-sigma dimensions.
    """
    # Var(x) = 9.0 -> std = 3.0 -> 2-sigma semi-axis = 6.0 -> full width = 12.0
    # Var(y) = 4.0 -> std = 2.0 -> 2-sigma semi-axis = 4.0 -> full height = 8.0
    P_pos = np.array([
        [9.0, 0.0],
        [0.0, 4.0]
    ])
    width, height, angle = compute_covariance_ellipse(P_pos, n_sigma=2.0)

    np.testing.assert_almost_equal(width, 12.0, decimal=5)
    np.testing.assert_almost_equal(height, 8.0, decimal=5)
    np.testing.assert_almost_equal(angle, 0.0, decimal=5)


def test_rotated_ellipse_orientation():
    """
    Tests that correlated x-y uncertainty produces a rotated ellipse.
    """
    # Symmetric matrix with positive covariance -> primary axis at +45 degrees
    P_pos = np.array([
        [5.0, 3.0],
        [3.0, 5.0]
    ])
    width, height, angle = compute_covariance_ellipse(P_pos, n_sigma=2.0)

    # For this matrix:
    # Eigenvalues are 8.0 and 2.0.
    # Eigenvector for lambda=8.0 is [1, 1] / sqrt(2) -> angle is 45 degrees
    expected_w = 2.0 * 2.0 * np.sqrt(8.0)
    expected_h = 2.0 * 2.0 * np.sqrt(2.0)

    np.testing.assert_almost_equal(width, expected_w, decimal=5)
    np.testing.assert_almost_equal(height, expected_h, decimal=5)
    np.testing.assert_almost_equal(angle, 45.0, decimal=5)


def test_zero_or_tiny_covariance():
    """
    Ensures numerical stability when covariance entries are zero or near-zero.
    """
    P_zero = np.zeros((2, 2))
    width, height, angle = compute_covariance_ellipse(P_zero, n_sigma=2.0)

    assert width > 0.0, "Width should be clamped above zero by numerical guard."
    assert height > 0.0, "Height should be clamped above zero by numerical guard."
    assert np.isfinite(angle)


def test_artifacts_generation(tmp_path):
    """
    Generates a tiny 15-frame run to verify static plot and GIF creation without crashing.
    """
    n_steps = 15
    truth = np.zeros((n_steps, 4))
    truth[:, 0] = np.arange(n_steps) * 1.0  # moving along x
    measurements = truth[:, :2] + 0.1
    estimates = truth.copy()
    covariances = np.tile(np.eye(4) * 2.0, (n_steps, 1, 1))

    plot_file = str(tmp_path / "test_plot.png")
    gif_file = str(tmp_path / "test_anim.gif")

    # Verify static plot generation
    plot_static_study(truth, measurements, estimates, covariances, dt=0.1, filename=plot_file)
    assert os.path.exists(plot_file), "plot_static_study failed to create output image."

    # Verify animation generation (low frame count for fast test execution)
    create_animation(truth, measurements, estimates, covariances, filename=gif_file, fps=5)
    assert os.path.exists(gif_file), "create_animation failed to create output GIF."