import numpy as np 
import pytest 
from benchmark import rmse, moving_average, nees 


def test_rmse_zero():
    a = np.array([4.0, 5.0])
    assert np.isclose(rmse(a, a), 0.0)


def test_rmse_known_value():
    estimated = [[3.0, 4.0], [0.0, 0.0]]
    true = [[0.0, 0.0], [0.0, 0.0]] #squared differences are 9, 16, 0, 0, mean of those four is 6.25
    assert rmse(np.array(estimated), np.array(true)) == pytest.approx(2.5)


def test_const_moving_avg():
    const_array = np.full((20, 2), 7.0) #twenty rows of [7.0, 7.0]
    output = moving_average(const_array, 5)
    assert np.allclose(const_array, output)


def test_moving_avg_shape(): #output shape equals input shape
    data = np.zeros((30, 2))
    output = moving_average(data, 5)
    assert output.shape == data.shape


def test_casual_does_not_look_ahead():
    step = np.zeros((20, 2))
    step[10:] = 100.0
    avgs_causal = moving_average(measurements=step, window=5, causal=True)
    avgs_centered = moving_average(measurements=step, window=5, causal=False)
    assert np.allclose(avgs_causal[9], 0.0)
    assert not np.allclose(avgs_centered[9], 0.0)


def test_nees_zero_when_perfect():
    estimates = np.zeros((50, 4))
    truth = np.zeros((50, 4))
    covariances = np.tile(np.eye(4), (50, 1, 1)) #stacks 50 copies of the identity into a (50, 4, 4)
    cov = nees(estimates, covariances, truth)
    assert cov == pytest.approx(0.0)


def test_nees_scales_with_P(): #tests the formula 
    estimates = np.ones((50, 4)) #ones, not zeros so that there is actual error
    truth = np.zeros((50, 4))
    covariances = np.tile(np.eye(4), (50, 1, 1))
    nees1 = nees(estimates, covariances, truth)
    nees4 = nees(estimates, covariances*4, truth)
    assert nees4 == pytest.approx(nees1 / 4)



