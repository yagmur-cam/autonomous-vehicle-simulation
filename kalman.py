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
    for row in measurements:
        #always predict, check whether this row is NaN, if its not then update 
        #append a copy of x and P to the two lists 
        pass



print(build_F(0.1))
print(build_H())
print(build_F(0.1) @ np.array([0.0, 0.0, 5.0, 0.0]))
