import numpy as np 
import pytest 
from benchmark import rmse, moving_average 

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




