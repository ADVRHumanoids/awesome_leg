#!/usr/bin/env python3

## Script for converting mat files containing trajectories to CSV ##

import scipy.io 
import numpy as np
import csv	
from numpy import genfromtxt

mat_path = "/home/andreap/hhcm_ws/src/awesome_leg/opt_results/horizon_jump/jump_test/"
mat_name = "horizon_offline_solver_res.mat"

dump_path = mat_path

solution_names = ["q_p", "q_p_dot", "tau", "dt_opt"]

data = scipy.io.loadmat(mat_path + mat_name)

q_p = data[solution_names[0]][1:3, :] # removing passive sliding guide joint
q_p_dot = data[solution_names[1]][1:3, :]
tau = data[solution_names[2]][1:3, :]
dt_op = data[solution_names[3]]

np.savetxt(dump_path +'q_p.csv', q_p, delimiter=" ")
np.savetxt(dump_path +'q_p_dot.csv', q_p_dot, delimiter=" ")
np.savetxt(dump_path +'tau.csv', tau, delimiter=" ")
np.savetxt(dump_path +'dt_opt.csv', dt_op, delimiter=" ")

# reader = np.loadtxt(dump_path +'q_p.csv', ndmin = 2)

# print("prova")
# print(q_p)
# print("csdcsdc")
# print(reader)
