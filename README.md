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
    motion model, S-curve manoeuvres, noise, dropouts, false echoes


### The filter                                       [Yağmur]
    state vector, the five matrices, predict/update, NaN handling


### The figures                                      [Anne]
    what each panel of the static study shows

## Project structure                                 [either]
    table: file -> what it does -> who wrote it


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

## Authors                                           [either]
    who did what, two lines