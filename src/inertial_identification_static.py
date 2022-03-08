#!/usr/bin/env python3

from matplotlib.pyplot import axis
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
urdf_path = rospy.get_param("/inertial_identification_static/urdf_path")   # urdf absolute path 
urdf = open(urdf_path, "r").read() # read the URDF

matfiles = rospy.get_param("/inertial_identification_static/matfile_paths")

# Initializing a LogLoaders for reading and using the data inside the .mat file
log_loaders = []

for i in range(len(matfiles)):
    log_loaders.append(LogLoader(matfiles[i]))

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

js_times= []
diff_jnt_accs = []
jnt_positions = []
jnt_velocities = []
filtered_diff_jnt_accs = []
test_taus = []
test_taus_filt = []

for i in range(len(matfiles)):
    js_times.append(log_loaders[i].get_js_rel_time())
    jnt_positions.append(log_loaders[i].get_motors_position())
    jnt_velocities.append(log_loaders[i].get_motors_velocity())
    diff_jnt_accs.append(diff_mat(js_times[i], jnt_velocities[i]))
    filtered_diff_jnt_accs.append(signal.filtfilt(b_acc, a_acc, diff_jnt_accs[i], padlen=150, axis= 1))
    test_taus.append(log_loaders[i].get_joints_efforts())
    test_taus_filt.append(signal.filtfilt(b_tau, a_tau, test_taus[i], padlen=150, axis= 1))

avrg_static_taus = []
avrg_static_pos = []

for i in range(len(matfiles)):
    avrg_static_pos.append(np.mean(jnt_positions[i], axis = 1))
    avrg_static_taus.append(np.mean(test_taus[i], axis = 1))

q_p = rearrange_test_mat(avrg_static_pos)
q_p_dot = np.zeros((len(q_p[:, 0]), len(q_p[0, :])))
q_p_ddot = np.zeros((len(q_p[:, 0]), len(q_p[0, :])))

meas_tau = rearrange_test_mat(avrg_static_taus)

print("Jnt pos:\t", q_p)
print("Meas tau:\t", meas_tau)

#############################################

regressors = assemblePinocchioRegressor(model, data, q_p, q_p_dot, q_p_ddot)

print("Regressor hip: ", regressors[0, :, :])
print("Regressor knee: ", regressors[1, :, :])
# stacked_regressor = np.concatenate((regressors[0, :, :], regressors[1, :, :]), axis = 0)
# stacked_regressor_reduced = stacked_regressor[:, 0: 20]
# print("Stacked regressor:", stacked_regressor_reduced)
# print("Stacked regressor rank: ", np.linalg.matrix_rank(stacked_regressor_reduced))

# print("test rank", np.linalg.matrix_rank(stacked_regressor[:, (1, 2, 11, 12)]))

n_active_jnts = len(q_p[:,0])
X_size = len(regressors[0, 0, :])

sigma = 0.001 * np.eye(X_size) # regularization matrix
# sigma[0, 0] = 1000
# sigma[10, 10] = 1000

X_guess = np.concatenate((model.inertias[1].toDynamicParameters(), model.inertias[2].toDynamicParameters(), np.array([0, 0]))) # remember Pinocchio adds an additional universe link

P, qT = compute_P_qT_inertial(regressor = regressors, tau_meas = meas_tau, sigma = sigma, X_guess = X_guess) # computing optimization matrices

QP_init= np.zeros((X_size, 1)).flatten() 

# print("qT: ", qT)
# print("P: ", P)

X = solve_qp(P = P, q = qT.T, initvals = QP_init) # solving QP

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


