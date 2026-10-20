#six steps 
#set the parameters in place 
#simulate
#build Q and R from those parameters 
#run_filter
#call visualize()' plotting functions, save them into figures/
#print the headline numbers from baseline_comparison 

import numpy as np 
from kalman import run_filter, build_Q
from benchmark import baseline_comparison
from simulator import simulate
from visualize import create_animation