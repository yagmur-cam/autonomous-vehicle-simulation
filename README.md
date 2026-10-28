# Tracking a Vehicle with a Kalman Filter

Estimating the position and velocity of a manoeuvring vehicle from noisy, 
position-only radar, and measuring how well it actually works.

<img src="figures/tracking.gif" alt="Kalman filter tracking a vehicle" width="500">

*White dot: the real vehicle. Red dots: what the radar reports. Cyan: the filter's
estimate, with its 2σ uncertainty. The radar cuts out between 12 and 14.5 seconds,
watch the uncertainty swell, then snap back.*

**Over 30 random seeds, the tuned filter reduces position RMSE from 2.83 m to
1.30 m: a 54% improvement on the raw radar, and 40% better than a causal
moving-average baseline.**


## Overview                                          [both — write last]
What the problem is, why position-only tracking of a manoeuvring
vehicle is non-trivial. Three or four sentences.


## Results                                           [Yağmur]
### 1. Does the filter beat the simple alternatives?

Position RMSE over 30 random seeds, same trajectory and noise parameters
throughout:

| Method | RMSE (m) |
|---|---|
| Raw radar | 2.832 ± 0.106 |
| Moving average, causal (5-point) | 2.182 ± 0.125 |
| **Kalman filter** | **1.301 ± 0.101** |
| Moving average, centred (5-point) | 1.261 ± 0.085 |

The filter is **54% better than the raw radar** and **40% better than the causal
moving average**. The standard deviations matter here: the gap between the filter
and the causal baseline is 0.88 m against a spread of about 0.1 m, so it is a real
effect rather than a lucky seed.

**On the last row.** The centred moving average appears to beat the filter — and
it does, but it is not a fair comparison. A centred window averages each point
with its neighbours on *both* sides, so it uses measurements that have not arrived
yet. It is a smoother, not a filter. The Kalman filter is causal: at step *k* it
has only seen steps 0 through *k*.

We kept both rows rather than quietly dropping the one that loses. Against the
honest causal baseline the filter wins clearly; against a method that can see the
future it comes within 3%. The
like-for-like comparison would be a Kalman smoother (RTS), which is listed under
future work.


### 2. How should Q and R be tuned?
    the sweep, the heatmap, why optimal q is 1400x the noise parameter





### 3. Is the filter honest about its own uncertainty?

RMSE only tells you how far wrong the filter is. It says nothing about whether
the filter *knows* how wrong it is, and a filter that tracks well while
understating its own uncertainty is dangerous, because anything downstream will
trust it too much.

**NEES** (normalised estimation error squared) measures that. At each step it
scales the error by the filter's own covariance: `eᵀ P⁻¹ e`. For a 4-dimensional
state, a well-calibrated filter averages **4**. Much higher means overconfident;
much lower means needlessly timid.

| q | NEES |
|---|---|
| 0.01 | 1851.9 |
| 0.1 | 364.8 |
| 1 | 47.6 |
| 10 | 6.9 |
| **31.6** | **3.6** |
| 100 | 2.6 |
| 1000 | 2.3 |

At small `q` the filter is catastrophically overconfident: a NEES of 1852 against
a target of 4 means its true error is roughly **21× larger than its claimed
standard deviation**. It is tracking badly and insisting it is fine. As `q` rises
the filter admits more doubt, NEES falls through 4 near q = 31.6, and beyond
q ≈ 100 it overcorrects into claiming more uncertainty than it actually has.

**The result: both criteria pick the same tuning.** RMSE is minimised at
q = 31.62 and NEES lands closest to 4 at q = 31.62: two independent measures,
one about accuracy and one about honesty, agreeing. There was no guarantee of
this: inflating `q` to absorb deterministic steering could easily have bought
accuracy at the cost of a covariance that no longer described reality.

*Caveat: our grid steps by a factor of 3.16, so the two optima agree to within
one grid step rather than exactly. A finer sweep between 10 and 100 would
tighten that.*


### 4. How gracefully does it survive a radar blackout?
    the dropout curve, the t^1.5 match with theory


## How it works
### The simulated world                              [Anne]
    Motion Model and S-Curve Maneuvers
    1. In simulator.py:
    - The module builds a challenging 2D synthetic environment that forces the filter to handle the chaotic realities of physical sensors. This helps to prove the robustness of the Kalman Filter.
    - The vehicle navigates a 2D Cartesian plane over a specified time horizon (e.g., 250 time steps at Δt = 0.1s). 
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
