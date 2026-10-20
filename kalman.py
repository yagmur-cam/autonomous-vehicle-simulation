# takes an array of noisy numbers and gives back an array of cleaned-up numbers with uncertainties attached 
#[x, y, vx, vy] ordering 

import numpy as np 

#F is the identity matrix with dt tucked into two spots, the model is constant velocity 
#given the old state, what is the new state one timestep later?
#so each row produces one element of the new state, and the entries in that row are the coefficients on the old state elements
#encodes position advances by velocity times dt, velocity unchanged
#returns a table of coefficients that works for every possible state
def build_F(dt): #the (4,4) state transition
    #new_x = 1*x + 0*y + dt*vx + 0*vy 
    #new_y = 0*x + 1*y + 0*vx + dt*vy --> The new y position is the old y, plus how fast it’s moving in the y direction, times dt
    # --the two components of velocity, carry speed and direction--
    #actual speed is sqrt(vx² + vy²), and the direction it’s pointing is atan2(vy, vx)
    #new_vx = 0*x + 0*y + 1*vx + 0*vy
    #new_vy = 0*x + 0*y + 0*vx + 1*vy
    return np.array([
        [1.0, 0.0, dt, 0.0],
        [0.0, 1.0, 0.0, dt],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])

#what does the sensor report, in terms of the state?
#radar gives you two numbers, reports position directly 
#first row is, zx equals what combination of x, y, vx, vy
def build_H(): #the (2,4) measurement matrix, a constant 
    #zx = 1*x + 0*y + 0*vx + 0*vy
    #zy = 0*x + 1*y + 0*vx + 0*vy 
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
    ])
    #last two columns are zero because the radar doesnt read velocity

#discrete white noise acceleration model
#we claimed that velocity is constant but we are wrong by this much--real cars accelerate and turn 
def build_Q(dt, q): #the (4,4) process noise, from dt and a single scalar, how the external world effect it 
    g_pos = dt**2/2
    g_vel = dt
    #Q[i][j] = q · g_i · g_j if elements i and j are driven by the same acceleration, and 0 if they aren’t
    return np.array([
        [q * g_pos**2, 0.0, q * g_pos * g_vel, 0.0],
        [0.0, q * g_pos**2, 0.0, q * g_pos * g_vel],
        [q * g_pos * g_vel, 0.0, q * g_vel**2, 0.0],
        [0.0, q * g_pos * g_vel, 0.0, q * g_vel**2],
    ])

#initialise x from the first measurement, and P deliberately large
def run_filter(measurements, dt, Q, R):
    assert measurements.ndim == 2, f"measurements must be 2D, got {measurements.ndim}D" #measurements is (200, 2)
    assert measurements.shape[1] == 2, f"measurements must have 2 columns, got {measurements.shape[1]}" #2 columns 
    assert Q.shape == (4, 4), f"Q must be 4x4, got {Q.shape}"
    assert R.shape == (2, 2), f"R must be 2x2, got {R.shape}"
    #R is measurement noise, describes the radar which reporst two numbers 
    
    F = build_F(dt)
    H = build_H()
    I = np.eye(4) #needed in the update step, 4x4 identity matrix
    
    #x and P are the filters memory
    #F, H, Q, R never change; x and P are the whole state of the machine
    #measurements is shape (N, 2), so measurements[0] is the first row, a 2-element array [zx, zy]
    x = np.array([measurements[0][0], measurements[0][1], 0.0, 0.0]) #filters current best guess, [x, y, vx, vy], gets overwritten every timestep 
    #initialized to big numbers, meaning we have no idea, the numbers will collapse as the updates arrive 
    P = np.diag([100.0, 100.0, 100.0, 100.0]) #how wrong it thinks that guess might be
    #the diagonal is the variance of each state element and the off-diagonals hold the correlations
    
    estimates = []
    covariances = []
    #predict -- maybe update -- record
    for row in measurements:
        #always predict, check whether this row is NaN, if its not then update 
        #append a copy of x and P to the two lists 
        #x becomes F times x
        #P becomes F times P times F-transpose, plus Q
        x = F @ x
        # +Q is admitting that the car is not at constant-velocity
        P = (F @ P @ F.T) + Q #uncertainty is two-dimensional, so P has to be transformed on both sides
        #the update step 
        if not np.isnan(row).any(): #only update if a measurement has arrived 
            y = row - H @ x #the innovation, we expect H @ x, gap between reality and expectation
            S = (H @ P @ H.T) + R #the innovation covariance, how surprised to be by a gap that size, position uncertainty + radar's noise 
            #works out how much of the gap to actually believe in (4,2)
            #maps a 2D measurement gap into a correction for all four state elements, including the velocities the radar never saw
            K = P @ H.T @ np.linalg.inv(S) #the Kalman gain 
            x = x + K @ y #correcting the state, gap * gain 
            P = (I - K @ H) @ P #shrinks P, it just learned something so it is less uncertain now 
        estimates.append(x.copy())
        covariances.append(P.copy())
    return np.array(estimates), np.array(covariances) #(N, 4, 4)

#The only thing in the whole filter that can ever change a velocity is K @ y

