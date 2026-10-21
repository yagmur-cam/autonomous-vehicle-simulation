#six steps 
#set the parameters in place 
#simulate
#build Q and R from those parameters 
#run_filter
#call visualize()'s plotting functions, save them into figures/
#print the headline numbers from baseline_comparison 

import os 
import numpy as np 
from kalman import run_filter, build_Q
from benchmark import baseline_comparison
from simulator import simulate
from visualize import create_animation

dt = 0.1
q = 31.62 #from the sweep 
measurement_noise_std = 2.0 #match the simulator 
seed = 0 
dropout = (120, 145) #2.5 s blackout

truth, measurements = simulate(dt=dt, seed=seed, dropouts=[dropout]) #dropout takes a list of tuples

Q = build_Q(dt, q)
R = np.eye(2) * measurement_noise_std**2


