# Tracking a Vehicle with a Kalman Filter

Estimating the position and velocity of a manoeuvring vehicle from noisy,
position-only radar, and measuring how well it actually works.

<img src=figures/tracking.gif width="500">

headline result in one sentence, with the number

## Overview                                          [both — write last]
What the problem is, why position-only tracking of a manoeuvring
vehicle is non-trivial. Three or four sentences.

## Results                                           [Yağmur]
### 1. Does the filter beat the simple alternatives?
    RMSE table + the causal/non-causal distinction
### 2. How should Q and R be tuned?
    the sweep, the heatmap, why optimal q is 1400x the noise parameter
### 3. Is the filter honest about its own uncertainty?
    the NEES sweep, why both criteria agree
### 4. How gracefully does it survive a radar blackout?
    the dropout curve, the t^1.5 match with theory

## How it works
### The simulated world                              [Anne]
    Motion Model and S-Curve Maneuvers
    1. In simulator.py:
    - The module builds a challenging 2D synthetic environment that forces the filter to handle the chaotic realities of physical sensors. This helps to prove the robustness of the Kalman Filter.
    - Thhe vehicle navigates a 2D Cartesian plane over a specified time horizon (e.g., 250 time steps at Δt = 0.1s). 
    Rather than a simple straight line, the ground truth is dynamically generated as an S-curve maneuver. This continuous, sweeping path forces the vehicle to undergo unmodeled lateral accelerations, rigorously testing the filter's ability to track momentum through corners without overshooting.
    - The physics of the vehicle are driven by the Discrete White Noise Acceleration (DWNA) model, which assumes that the vehicles moves at a nearly constant velocity. (maybe i could speak about the white noise)
    - The State Transition matrix (F) updates the x and y positions based on the current velocities (v_x, v_y).
    - To create this S-curve, the simulator applies a sinusodial steering input (changing the velocity vector over time) rather than driving in a complete straight line.

   


### The filter                                       [Yağmur]
    state vector, the five matrices, predict/update, NaN handling


### The figures                                      [Anne]
    what each panel of the static study shows






## Project structure

| File | Purpose | Author |
|---|---|---|
| `simulator.py` | Generates the true trajectory and the noisy radar measurements, including manoeuvres, dropouts and false echoes | Anne |
| `kalman.py` | The Kalman filter: state transition, measurement and noise matrices, predict/update cycle | Yağmur |
| `benchmark.py` | The four experiments and the metrics they use (RMSE, moving average, NEES) | Yağmur |
| `visualize.py` | The three-panel static study and the tracking animation | Anne |
| `main.py` | Wires the modules together for a single run and writes the figures | both |
| `test_kalman.py` | Unit tests for the filter | Yağmur |
| `test_benchmark.py` | Unit tests for the metrics | Yağmur |
| `test_simulator.py` | Unit tests for the simulator | Anne |
| `sweep_results.npz` | Saved sweep arrays, so plots can be redrawn without re-running | generated |
| `figures/` | Output figures and animation | generated |



## How to run it

```bash
git clone https://github.com/yagmur-cam/autonomous-vehicle-simulation.git
cd autonomous-vehicle-simulation

python -m venv .venv
source .venv/bin/activate #Windows: .\.venv\Scripts\Activate.ps1

pip install -r requirements.txt
```

| Command | What it does |
|---|---|
| `python main.py` | One tracking run with a sensor dropout. Writes both figures into `figures/`. |
| `python benchmark.py` | All four experiments. Prints the results table and saves the sweep arrays. Takes about 30 seconds. |
| `pytest` | The 23 tests. |













## Tests                                             [both]
    23 tests, what they cover, one line on the interesting ones

## Limitations                                       [both — do not skip]
    constant-velocity lag in turns, injected dropouts,
    Gaussian-by-construction noise, Cartesian not range-bearing

## Future work                                       [both]
    EKF with range/bearing, RTS smoother, Mahalanobis gating


## Authors

- **Yağmur Çam** ([@yagmur-cam](https://github.com/yagmur-cam)): Kalman filter, benchmarking and tuning experiments
- **Anne Fernando** ([@Mbmiquella](https://github.com/Mbmiquella)): simulator, visualisation and animation

Built as a self-directed project alongside our second year of Applied Computer Science and AI at Sapienza University of Rome.
