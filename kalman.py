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
        [0.0, 1.0,0.0, 0.0],
    ])
    #last two columns are zero because the rasar doesnt read velocity


#we claimed that velocity is constant but we are wrong by this much--real cars accelerate and turn 
def build_Q(dt, q): #the (4,4) process noise, from dt and a single scalar, how the external world effect it 
    return np.array([
        [],
        [],
        [],
        [],
    ])



#build F, H, and the identity matrix once, before the loop
#initialise x from the first measurement, and P deliberately large
def run_filter(measurements, dt, Q, R):
    F = build_F(dt)
    H = build_H()
    I = np.eye(4) #needed in the update step, 4x4 identity matrix
    
    #x and P are the filters memory
    #F, H, Q, R never change; x and P are the whole state of the machine
    #measurements is shape (N, 2), so measurements[0] is the first row — a 2-element array [zx, zy]
    x = np.array([measurements[0][0], measurements[0][1], 0.0, 0.0]) #filters current best guess, [x, y, vx, vy], gets overwritten every timestep 
    #initialized to big numbers, meaning we have no idea, the numbers will collapse as the updates arrive 
    P = np.diag([100.0, 100.0, 100.0, 100.0]) #how wrong it thinks that guess might be
    #the diagonal is the variance of each state element and the off-diagonals hold the correlations
    
    estimates = []
    covariances = []
    for row in measurements:
        #always predict, check whether this row is NaN, if its not then update 
        #append a copy of x and P to the two lists 
        #x becomes F times x
        #P becomes F times P times F-transpose, plus Q
        x = F @ x
        # +Q is admitting that the car is not at constant-velocity
        P = (F @ P @ F.T) + Q #uncertainty is two-dimensional, so P has to be transformed on both sides
        estimates.append(x.copy())
        covariances.append(P.copy())
    return np.array(estimates), np.array(covariances) #(N, 4, 4)


print(build_F(0.1))
print(build_H())
print(build_F(0.1) @ np.array([0.0, 0.0, 5.0, 0.0]))

meas = np.zeros((10, 2))
est, cov = run_filter(meas, 0.1, np.eye(4) * 0.1, np.eye(2) * 16.0)
print(est[:, 0])
print(cov[:, 0, 0])