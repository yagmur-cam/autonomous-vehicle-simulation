#RMSE filter vs raw, the Q/R sweep, Monte Carlo over many seeds, and NEES
#runs under various conditions, testing how well the filter works and where it breaks 
#4 experiments: baseline comparison, Q/R sweep, dropout sweep, Monte Carlo over seeds
#NEES: normalised estimation error squared, meaning "is the filter honest about its own uncetainty"
#the sweeps and monte carlo need the simulation

import numpy as np 
from kalman import run_filter
from simulator import simulate


def rmse(estimated, true):
    diff = estimated - true
    return np.sqrt(np.mean(diff**2))

def moving_average(measurements, window): #for each point, the avg of itself and its neighbours
    half_window = window // 2
    #start — i - half, but never below 0.
    #end — i + half + 1, but never past the end of the array
    avgs = []
    for i in range(len(measurements)): #for row i, you want the mean of rows i-k through i+k
        start = max(0, i-half_window)
        end = min(len(measurements), i+half_window+1)
        block = measurements[start:end]
        avgs.append(np.mean(block, axis=0)) #giving one [x, y] pair
    return np.array(avgs)


#For a 4-dimensional state, a well-calibrated filter averages about 4
def nees(estimates, covariances, truth): #measuring the error in units of the filter’s own claimed uncertainty
    errors = []
    for estimate, P, true_state in zip(estimates, covariances, truth):
        e = estimate - true_state
        errors.append(e.T @ np.linalg.inv(P) @ e)
    return np.mean(errors[10:]) #the early NEES values are near zero and drag your average down for reasons that have nothing to do with calibration.


def monte_carlo():
    pass


def baseline_comparison(truth, measurements, dt, Q, R, window=5):
    #truth is (N, 4), position and velocity 
    #measurements is (N, 2), position only 
    est, cov = run_filter(measurements, dt, Q, R)
    #raw measurements vs truth, moving average vs truth, filter vs truth
    measurement_truth = rmse(measurements, truth[:, :2])
    moving_avg_truth = rmse(moving_average(measurements, window), truth[:, :2])
    filter_truth = rmse(est[:, :2], truth[:, :2])
    return {"raw": measurement_truth, "moving_average": moving_avg_truth, "filter": filter_truth}