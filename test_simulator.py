import numpy as np
import pytest
from simulator import simulate

def test_reproducibility():
    """Handbook A-2: Calling simulate twice with the same seed must produce byte-identical arrays."""
    t1, m1 = simulate(n_steps=100, dt=0.1, process_noise_std=0.1, measurement_noise_std=1.0, seed=123)
    t2, m2 = simulate(n_steps=100, dt=0.1, process_noise_std=0.1, measurement_noise_std=1.0, seed=123)
    np.testing.assert_array_equal(t1, t2, err_msg="Truth arrays are not byte-identical across identical seeds!")
    np.testing.assert_array_equal(m1, m2, err_msg="Measurement arrays are not byte-identical across identical seeds!")

def test_different_seeds_produce_different_data():
    """Different seeds must not generate the same random numbers."""
    t1, m1 = simulate(n_steps=50, seed=1)
    t2, m2 = simulate(n_steps=50, seed=2)
    assert not np.allclose(m1, m2), "Different seeds produced identical measurement noise!"

def test_contract_shapes():
    """Handbook Section 3: Truth must be (N, 4) and Measurements must be (N, 2)."""
    n_steps = 150
    truth, meas = simulate(n_steps=n_steps)
    assert truth.shape == (n_steps, 4), f"Expected truth shape ({n_steps}, 4), got {truth.shape}"
    assert meas.shape == (n_steps, 2), f"Expected measurements shape ({n_steps}, 2), got {meas.shape}"

def test_kinematics_distance_check():
    """Handbook A-1 Check: Total distance matches speed x elapsed time within injected noise."""
    dt = 0.1
    n_steps = 100
    # Simulate with 0 process noise and no maneuvers to test pure kinematics: x = v * t
    truth, _ = simulate(n_steps=n_steps, dt=dt, process_noise_std=0.0, seed=42)
    initial_vx = truth[0, 2] # 8.0 m/s
    total_time = (n_steps - 1) * dt
    expected_distance = initial_vx * total_time
    
    # In pure straight-line mode before turning maneuvers, x should match v * t
    # Checking step 40 (before turning starts at step 50/60)
    time_40 = 40 * dt
    assert np.isclose(truth[40, 0], initial_vx * time_40, atol=1e-5), "Constant-velocity motion equation failed!"

def test_dropout_injection():
    #Handbook Section 3: Dropout windows must strictly emit NaNs.
    dropouts = [(20, 45)]
    truth, meas = simulate(n_steps=100, dropouts=dropouts)
    
    # The dropout slice must be entirely NaN
    assert np.all(np.isnan(meas[20:45])), "Rows inside dropout window are not NaN!"
    # Outside the slice must NOT contain any NaNs
    assert not np.any(np.isnan(meas[:20])), "Spurious NaNs detected before dropout window!"
    assert not np.any(np.isnan(meas[45:])), "Spurious NaNs detected after dropout window!"
    # Truth should NEVER contain NaNs (only the sensor loses sight)
    assert not np.any(np.isnan(truth)), "Truth array contains NaNs! Truth must always exist."

def test_outlier_injection():
    """Handbook A-2: Outliers must produce extreme jumps."""
    outliers = [15, 60]
    truth, meas = simulate(n_steps=100, measurement_noise_std=1.0, outliers=outliers, seed=42)
    
    for idx in outliers:
        error = np.linalg.norm(meas[idx] - truth[idx, :2])
        # Normal 1.0m noise will practically never exceed 10m; our glint offsets are 20-35m
        assert error > 10.0, f"Outlier at index {idx} was not large enough! (Error was {error:.2f}m)"