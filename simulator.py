#This file generates the true motion of the vehicle, including an S-curve turn to violate constant velocity.
#The file also corrups the position reading with: Gaussian noise, dropout gaps and occasional false echoes.

import numpy as np
#This is the function signature of the simulation generator.
def simulate(
     n_steps: int = 250,
     #the total number of disceret steps (frames) to simulate is 250.
     dt: float = 0.1,
     #this is the sampling interval, delta t, in seconds.
     process_noise_std: float = 0.15,
     #this is the standard deviation of the vehicle's random acceleration kicks in m/s^2.
     measurement_noise_std: float = 2.0,
     #this is the standard deviation og the radar sensor's gaussian noise in m.
     seed: int = 42,
     #the seed is an explicit integer fed into the random number generator.
     dropouts: list[tuple[int, int]] = None,
     outliers: list[int] = None #this is a list of specifiv time-step indices where the sensor receives false echoes or severe radar glint.
) ->  tuple[np.ndarray, np.ndarray]:
    
    #we will simulate a 2D vehicle trajectory and returns:
       #truth: (n_steps, 4) -> [x, y, vx, vy]
       #measurements: (n_steps, 2) -> [zx, zy] #Contains NaNs during dropouts.

     #Setion A-2: Use an isolated Generator object, not np.random.seed()

     rng = np.random.default_rng(seed) # the isolated generator object is dedicated solely to this simulation.

     truth = np.zeros((n_steps, 4), dtype=float)
     # Initial state: position(0, 0), moving east at 8.0 m/s
     truth[0] = [0.0, 0.0, 8.0, 0.0] #empty 2D table with N rows and 4 col to hold the vehicle's true state across times.

     # State Transition Matrix F
     # position_new = position_old + velocity * delta t
     # velocity_new = velocity_old (i.e. constant velocity)
     #This written as a row-by-row system of linear equations:

     F = np.array([
          [1.0, 0.0, dt, 0.0],
          [0.0, 1.0, 0.0, dt],
          [0.0, 0.0, 1.0, 0.0],
          [0.0, 0.0, 0.0, 1.0]
     ])

     # Process noise covariance Q - discrete white noise acceleration
     q = process_noise_std ** 2
     g_pos = 0.5 * (dt ** 2)
     g_vel = dt
     Q = np.array([
          [q * g_pos**2, 0.0,      q * g_pos * g_vel, 0.0],
          [0.0,     q * g_pos**2, 0.0,       1 * g_pos * g_vel],
          [q * g_pos * g_vel, 0.0,     q * g_pos**2,   0.0],
          [0.0,     q * g_pos * g_vel, 0.0,     q * g_vel **2]
     ])
      
     for k in range(1, n_steps):
          next_state = F @ truth[k - 1]

          #Section A-1: S-curve turn maneuvers that violate constant-velocity
          if 50 <= k < 100:
               next_state[3] += 0.40 * np.sin((k-50)* 0.1) #steer north
          elif 150 <= k < 190:
               next_state[3] -= 0.45 * np.cos ((k-150) * 0.1) #steer south

          # Add random acceleration kick:
          kick = rng.multivariate_normal(np.zeros(4), Q)
          truth[k] = next_state + kick

     # Measurement Matrix H - the radar only sees [x, y]
     H = np.array ([
          [1.0 , 0.0, 0.0, 0.0],
          [0.0, 1.0, 0.0, 0.0]
     ])
     
     # True positions projected to sensor + Gaussian noise
     meas_noise = rng.normal (0.0, measurement_noise_std, size=(n_steps, 2))
     measurements = (truth @ H.T) + meas_noise

     # Inject Sensor Dropouts (NaN rows):
     if dropouts is not None:
          for start_idx, end_idx in dropouts:
               measurements[start_idx:end_idx, :] = np.nan
     
     # Inject False Echoes - outliers far off target
     if outliers is not None:
          for idx in outliers:
               if 0 <= idx < n_steps:
                    glint = rng.choice([-1.0, 1.0], size = 2) * rng.uniform(20.0, 35.0,  size=2)
                    measurements[idx] += glint

     return truth, measurements

if __name__ == "__main__":
    t, m = simulate(n_steps=5, dropouts=[(2, 4)])
    print("--- Truth Array Sample (First 5 Rows) ---")
    print(t)
    print("\n--- Measurement Array Sample (With NaN Dropout) ---")
    print(m)

#The above piece of code is a visual check of the arrays, the simulator.py file can de directly run in the terminal.

