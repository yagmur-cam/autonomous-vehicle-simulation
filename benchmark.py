#RMSE filter vs raw, the Q/R sweep, Monte Carlo over many seeds, and NEES
#runs under various conditions, testing how well the filter works and where it breaks 
#4 experiments: baseline comparison, Q/R sweep, dropout sweep, Monte Carlo over seeds
#NEES: normalised estimation error squared, meaning "is the filter honest about its own uncetainty"
#the sweeps and monte carlo need the simulation

import numpy as np 
from kalman import run_filter, build_Q
from simulator import simulate


def rmse(estimated, true):
    diff = estimated - true
    return np.sqrt(np.mean(diff**2))


def moving_average(measurements, window, causal=True): #for each point, the avg of itself and its neighbours
    half_window = window // 2
    #start — i - half, but never below 0.
    #end — i + half + 1, but never past the end of the array
    avgs = []
    for i in range(len(measurements)): #for row i, you want the mean of rows i-k through i+k
        if causal:
            start = max(0, i-window+1)
            end = i+1 #the current point and the window-1 before it, doesn't look ahead
        else:
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


#returns six numbers as three pairs, one pair per method 
#for each raw, moving_avg and filter --> the mean RNMSE across all seeds which is the typical performance
#and the std deviation of that RMSE across seeds which is how much it varies run to run 
def monte_carlo(n_seeds=30, dt=0.1, q=31.62, measurement_noise_std=2.0, window=5):
    raw_values = []
    moving_avgs_centered = []
    moving_avgs_causal = []
    filter_truths = []
    Q = build_Q(dt, q)
    R = np.eye(2) * measurement_noise_std**2 #holds variances
    for seed in range(n_seeds):
        truth, measurements = simulate(dt=dt, seed=seed)
        result = baseline_comparison(truth, measurements, dt, Q, R, window)
        raw_values.append(result["raw"])
        moving_avgs_centered.append(result["moving_average_centered"])
        moving_avgs_causal.append(result["moving_average_causal"])
        filter_truths.append(result["filter"])
    return {
        "raw": (np.mean(raw_values), np.std(raw_values)), 
        "moving_average_centered": (np.mean(moving_avgs_centered), np.std(moving_avgs_centered)), 
        "moving_average_causal": (np.mean(moving_avgs_causal), np.std(moving_avgs_causal)),
        "filter": (np.mean(filter_truths), np.std(filter_truths))
    }


def baseline_comparison(truth, measurements, dt, Q, R, window=5): #returns a dict
    #truth is (N, 4), position and velocity 
    #measurements is (N, 2), position only 
    est, cov = run_filter(measurements, dt, Q, R)
    #raw measurements vs truth, moving average vs truth, filter vs truth
    measurement_truth = rmse(measurements, truth[:, :2])
    moving_avg_centered_truth = rmse(moving_average(measurements, window, causal=False), truth[:, :2])
    moving_avg_causal_truth = rmse(moving_average(measurements, window, causal=True), truth[:, :2])
    filter_truth = rmse(est[:, :2], truth[:, :2])
    return {"raw": measurement_truth, "moving_average_centered": moving_avg_centered_truth, 
            "moving_average_causal": moving_avg_causal_truth, "filter": filter_truth}


#the Q-R sweep: simulator's real noise is fixed, only the filter's beliefs change
#ask --> what happens when i tell the filter the wrong thing?
#true values are q = 0.15² = 0.0225 and r = 2.0² = 4.0
#simulate has a fixed measurement_noise_std at 2.0, while the filter's R varies 
def q_r_sweep(q_values, r_std_values, n_seeds=5):
    grid = np.zeros((len(q_values), len(r_std_values)))
    for i, q in enumerate(q_values):
        for j, r_std in enumerate(r_std_values):
            grid[i][j] = monte_carlo(n_seeds=n_seeds, dt=0.1, q=q, measurement_noise_std=r_std, window=5)["filter"][0] #the mean filter RMSE 
    return grid, q_values, r_std_values


#NEES asks, does the filter's P honestly describe its error? 
#the q that tracks best might not be the q that is calibrated 
#the RMSE-optimal tuning is not the consistency-optimal tuning,
#the simulator’s reality doesn’t change while the filter’s beliefs do
def nees_sweep(q_values, n_seeds=5, dt=0.1, measurement_noise_std=2.0):
    results = np.zeros(len(q_values))
    for i, q in enumerate(q_values):
        nees_values = []
        Q = build_Q(dt, q=q)
        R = np.eye(2) * measurement_noise_std**2 #measurement covariance
        for seed in range(n_seeds):
            truth, measurement = simulate(dt=dt, seed=seed)
            est, cov = run_filter(measurements=measurement, dt=dt, Q=Q, R=R)
            nees_values.append(nees(est, cov, truth))
        results[i] = np.mean(nees_values)
    return results 


#error growing faster than linearly with gap length
#velocity uncertainty feeds into position uncertainty and compounds
def dropout_sweep(gap_lengths, n_seeds=5, dt=0.1, q=31.62, measurement_noise_std=2.0, start=120):
    Q = build_Q(dt, q=q)
    R = np.eye(2) * measurement_noise_std**2
    errors = []
    for gap in gap_lengths:
        peaks = []
        for seed in range(n_seeds):
            truth, measurement = simulate(dt=dt, seed=seed, dropouts=[(start, start+gap)])
            est, cov = run_filter(measurement, dt=dt, Q=Q, R=R)
            #est is (N, 4): rows are timesteps, columns are state elements
            est_gap = est[start:start+gap, :2]
            truth_gap = truth[start:start+gap, :2]
            peaks.append(np.max(np.linalg.norm(est_gap - truth_gap, axis=1))) #the per-step distance, collapisng along the columns 
        errors.append(np.mean(peaks))
    return np.array(errors)



if __name__ == "__main__":
    results = monte_carlo(n_seeds=30)
    for name, (mean, std) in results.items():
        print(f"{name:16} {mean:6.3f} +/- {std:5.3f}")

    qs = np.logspace(-2, 3, 11) #0.01 up to 1000
    rs = np.array([0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0])
    grid, _, _ = q_r_sweep(qs, rs, n_seeds=3)
    i, j = np.unravel_index(np.argmin(grid), grid.shape)
    print(f"best: q={qs[i]:.4g}  r_std={rs[j]:.4g}  RMSE={grid[i, j]:.3f}")

    nees_results = nees_sweep(qs, n_seeds=3)
    for q, val in zip(qs, nees_results):
        print(f"q={q:9.4g}   NEES={val:8.3f}")

    best = np.argmin(np.abs(nees_results - 4.0))
    print(f"NEES closest to 4 at q={qs[best]:.4g}  (NEES={nees_results[best]:.3f})")

    gap_lengths = [5, 10, 20, 40, 80]
    peak_errors = dropout_sweep(gap_lengths=gap_lengths)
    for gap, err in zip(gap_lengths, peak_errors):
        print(f"gap={gap:4d} steps ({gap*0.1:.1f}s)   peak error {err:7.2f} m")

    np.savez("sweep_results.npz",
         rmse_grid=grid,
         q_values=qs,
         r_std_values=rs,
         nees_values=nees_results,
         gap_lengths=np.array(gap_lengths),
         peak_errors=peak_errors)



