#six steps 
#set the parameters in place 
#simulate
#build Q and R from those parameters 
#run_filter
#call visualize()'s plotting functions, save them into figures/
#print the headline numbers from baseline_comparison 

import numpy as np 
from kalman import run_filter, build_Q
from benchmark import baseline_comparison
from simulator import simulate
from visualize import create_animation, plot_static_study

if __name__ == "__main__":

    dt = 0.1
    q = 31.62 #from the sweep 
    measurement_noise_std = 2.0 #match the simulator 
    seed = 0 
    dropout = (120, 145) #2.5 s blackout

    truth_clean, meas_clean = simulate(dt=dt, seed=seed)
    truth_gap, meas_gap = simulate(dt=dt, seed=seed, dropouts=[dropout]) #dropout takes a list of tuples

    Q = build_Q(dt, q)
    R = np.eye(2) * measurement_noise_std**2

    est_clean, cov_clean = run_filter(meas_clean, dt, Q, R)
    est_gap, cov_gap = run_filter(meas_gap, dt, Q, R)

    plot_static_study(
            truth=truth_gap, measurements=meas_gap,
            estimates=est_gap, covariances=cov_gap, dt=dt, dropouts=[dropout])

    create_animation(
            truth=truth_gap, measurements=meas_gap,
            estimates=est_gap, covariances=cov_gap)

    results = baseline_comparison(truth_clean, meas_clean, dt, Q, R)

    for name, value in results.items():
        print(f"{name:25} {value:6.3f}")