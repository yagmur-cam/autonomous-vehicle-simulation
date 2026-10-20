import numpy as np 
import pytest 
from kalman import build_F, build_Q, build_H, run_filter


def test_F_shape():
    assert build_F(0.1).shape == (4,4)


def test_Q_syymetric():
    Q = build_Q(0.1, 3.0)
    assert np.allclose(Q, Q.T)


def test_H_ignores_velocity():
    H = build_H()
    #same position, completely different motion
    state1 = np.array([10.0, 5.0,  0.0,   0.0]) #parked at (10, 5)
    state2 = np.array([10.0, 5.0, 80.0, -60.0]) #same position but moving fast 
    assert np.allclose(H @ state1, H @ state2) #H should give [10, 5] for both, because the radar can only see where the car is, never how it’s moving


def test_output_shapes():
    estimates,covariances = run_filter(np.zeros((10, 2)), 0.1, build_Q(0.1, 1.0), np.eye(2) * 16.0)
    assert estimates.shape == (10, 4)
    assert covariances.shape == (10, 4, 4)


def test_noise_free_recovers_truth():
    dt = 0.1
    vx, n = 3.0, 100 
    #[0.0, 0.3, 0.6, … 29.7], a column of 100 positions
    measurements = np.column_stack([np.arange(n) * (vx * dt), np.zeros(n)]) #an (n, 2) array
    Q = build_Q(dt, 1.0)
    R = np.eye(2) * 1e-4 #a near perfect radar 
    est, _ = run_filter(measurements, dt, Q, R)
    #the last row is at index n - 1, and each step covers vx * dt
    final_x = (n-1) * (vx * dt) #est is the filter's own output 
    expected_final_state = np.array([final_x, 0.0, vx, 0.0])
    assert np.allclose(est[-1], expected_final_state, atol=1e-6) #a very tight tolerance


#testing K's effect 
def test_large_R_ignores_measurements():
    dt = 0.1
    #Fifty steps of a car moving 0.3 m per step in x, sitting still in y
    measurements = np.column_stack([np.arange(50) * 0.3, np.zeros(50)])
    Q = build_Q(dt, 1.0)
    est_tight, _ = run_filter(measurements, dt, Q, np.eye(2) * 1e-6) #the filter believes the radar completely
    est_loose, _ = run_filter(measurements, dt, Q, np.eye(2) * 1e6) #it believes nothing
    gap_tight = np.mean(np.abs(est_tight[:, :2] - measurements)) #just the position columns from est_tight
    gap_loose = np.mean(np.abs(est_loose[:, :2] - measurements))
    assert gap_tight < gap_loose
    
    #with R at 1e-6 the filter is being told the radar is essentially perfect, so K goes large and the estimate snaps onto every measurement. 
    #With R at 1e6 it’s told the radar is garbage, so K goes near zero and the filter coasts on its own prediction, drifting away from the dots
