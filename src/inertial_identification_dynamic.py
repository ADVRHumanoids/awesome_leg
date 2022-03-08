#!/usr/bin/env python3

from awesome_leg_pholus_utils.logger_utilities import LogLoader, LogPlotter

from xbot_interface import xbot_interface as xbot

import numpy as np

from qpsolvers import *

from scipy import signal

from awesome_leg_pholus_utils.param_identification_utilities import *

import pinocchio

import rospy

######################### PRE-INITIALIZATIONS #########################

# Model description
urdf_path = rospy.get_param("/inertial_identification_dynamic/urdf_path")   # urdf absolute path 
urdf = open(urdf_path, "r").read() # read the URDF

matfile = rospy.get_param("/inertial_identification_dynamic/matfile_path")

# Initializing a LogLoaders for reading and using the data inside the .mat file

log_loader = LogLoader(matfile)

######################### INITIALIZATIONS #########################

# Model interface & Co (using Pinocchio)

model = pinocchio.buildModelFromXML(urdf)
data     = model.createData()

print("Pinocchio model: ", model)

#################################################

# Filtering
filter_order_acc = 8
cutoff_freq_acc = 0.05
b_acc, a_acc = signal.butter(filter_order_acc, cutoff_freq_acc) # lowpass Butterworth filter 

filter_order_tau = 8
cutoff_freq_tau = 0.008
b_tau, a_tau = signal.butter(filter_order_tau, cutoff_freq_tau) # lowpass Butterworth filter

js_time = log_loader.get_js_rel_time()
jnt_pos = log_loader.get_motors_position()
jnt_vel = log_loader.get_motors_velocity()
diff_jnt_acc = diff_mat(js_time, jnt_vel)
filtered_diff_jnt_acc = signal.filtfilt(b_acc, a_acc, diff_jnt_acc, padlen=150, axis= 1)
test_tau = log_loader.get_joints_efforts()
test_tau_filt = signal.filtfilt(b_tau, a_tau, test_tau, padlen=150, axis= 1)

q_p = jnt_pos[:, 0:-1]
q_p_dot = jnt_vel[:, 0:-1]
q_p_ddot = filtered_diff_jnt_acc
meas_tau = test_tau_filt[:, 0:-1]

print("Jnt pos:\t", q_p)
print("Jnt vel:\t", q_p_dot)
print("Jnt acc:\t", q_p_ddot)

print("Meas tau:\t", meas_tau)

#############################################

regressors = assemblePinocchioRegressor(model, data, q_p, q_p_dot, q_p_ddot)

print("Regressor hip: ", regressors[0, 1000, :])

print("Regressor knee: ", regressors[1, 1000, :])

n_active_jnts = len(q_p[:,0])
X_size = len(regressors[0, 0, :])

sigma = 0.001 * np.eye(X_size) # regularization matrix

X_guess = np.concatenate((model.inertias[1].toDynamicParameters(), model.inertias[2].toDynamicParameters(), np.array([0, 0]))) # remember Pinocchio adds an additional universe link
l_b = -1000 + np.zeros((X_size,1)).flatten() 
u_b =  1000 + np.zeros((X_size,1)).flatten() 

# link_offset = 10
# for i in range(n_active_jnts):
#     l_b[0 + i * link_offset] = 0 # mass hip
#     l_b[4 + i * link_offset] = 0 # ixx
#     l_b[6 + i * link_offset] = 0 # iyy
#     l_b[9 + i * link_offset] = 0 # izz

P, qT = compute_P_qT_inertial(regressor = regressors, tau_meas = meas_tau, sigma = sigma, X_guess = X_guess) # computing optimization matrices

QP_init= np.zeros((X_size, 1)).flatten() 

# print("qT: ", qT)
# print("P: ", P)

X = solve_qp(P = P, q = qT.T, initvals = QP_init, lb = l_b, ub = u_b) # solving QP

print("Original inertial params: ", interpret_inertial_sol2(n_active_jnts, X_guess))
print("Inertial params: ", interpret_inertial_sol2(n_active_jnts, X))

print("X_guess ", X_guess)
print("X: ", X)

error_pre_opt, abs_mean_error_pre_opt = compute_fitting_error(regressors, meas_tau, X_guess)
print("Fitting error hip before optimization ([Nm]): ", error_pre_opt[0, :])
print("Fitting error knee before optimization ([Nm]): ", error_pre_opt[1, :])
print("Fitting abs mean error hip before optimization ([Nm]): ", abs_mean_error_pre_opt[0])
print("Fitting abs mean error knee before optimization ([Nm]): ", abs_mean_error_pre_opt[1])
error, abs_mean_error = compute_fitting_error(regressors, meas_tau, X)
print("Fitting error hip after optimization ([Nm]): ", error[0, :])
print("Fitting error knee after optimization ([Nm]): ", error[1, :])
print("Fitting abs mean error hip after optimization ([Nm]): ", abs_mean_error[0])
print("Fitting abs mean error knee after optimization ([Nm]): ", abs_mean_error[1])


