#!/usr/bin/env python3

from awesome_leg_pholus.logger_utilities import LogLoader, LogPlotter

import rospkg

from xbot_interface import xbot_interface as xbot

import numpy as np

from qpsolvers import *

from scipy import signal

import yaml

from awesome_leg_pholus.param_identification_utilities import *

import matplotlib as plt

import pinocchio
######################### PRE-INITIALIZATIONS #########################

# Loading actuator paramters
abs_config_path = rospkg.RosPack().get_path("awesome_leg_pholus")+"/config/"
acts_yaml_name = "actuators.yaml"
acts_config_path = abs_config_path + acts_yaml_name
with open(acts_config_path, 'r') as stream:
    acts_yaml = yaml.safe_load(stream)

# Model description
urdf_path = rospkg.RosPack().get_path("awesome_leg_pholus")+"/description/urdf/awesome_leg_param_id_backup.urdf"   # urdf absolute path 
# urdf_path = rospkg.RosPack().get_path("awesome_leg_pholus")+"/description/urdf/awesome_leg_param_id.urdf"   # urdf absolute path 
# srdf_path = rospkg.RosPack().get_path("awesome_leg_pholus")+"/description/srdf/awesome_leg_param_id.srdf"   # srdf absolute path
urdf = open(urdf_path, "r").read() # read the URDF
# srdf = open(srdf_path, "r").read() # read the URDF

# Calibration data path
matfile_path1 = rospkg.RosPack().get_path("awesome_leg_pholus")+'/tests/static_poses/pose1/'+'robot_state_log__0_2022_02_02__17_14_40.mat' 
matfile_path2 = rospkg.RosPack().get_path("awesome_leg_pholus")+'/tests/static_poses/pose2/'+'robot_state_log__0_2022_02_02__17_17_21.mat' 
matfile_path3 = rospkg.RosPack().get_path("awesome_leg_pholus")+'/tests/static_poses/pose3/'+'robot_state_log__0_2022_02_02__17_18_49.mat' 
matfile_path4 = rospkg.RosPack().get_path("awesome_leg_pholus")+'/tests/static_poses/pose4/'+'robot_state_log__0_2022_02_02__17_20_31.mat' 
matfile_path5 = rospkg.RosPack().get_path("awesome_leg_pholus")+'/tests/static_poses/pose5/'+'robot_state_log__0_2022_02_02__17_22_27.mat' 
matfile_path6 = rospkg.RosPack().get_path("awesome_leg_pholus")+'/tests/static_poses/pose6/'+'robot_state_log__0_2022_02_02__17_24_16.mat' 
matfile_path7 = rospkg.RosPack().get_path("awesome_leg_pholus")+'/tests/static_poses/pose7/'+'robot_state_log__0_2022_02_02__17_25_30.mat' 
matfile_path8 = rospkg.RosPack().get_path("awesome_leg_pholus")+'/tests/static_poses/pose8/'+'robot_state_log__0_2022_02_02__17_26_50.mat' 
matfile_path9 = rospkg.RosPack().get_path("awesome_leg_pholus")+'/tests/static_poses/pose9/'+'robot_state_log__0_2022_02_02__17_28_10.mat' 
matfile_path10 = rospkg.RosPack().get_path("awesome_leg_pholus")+'/tests/static_poses/pose10/'+'robot_state_log__0_2022_02_02__17_29_32.mat' 
matfile_path11 = rospkg.RosPack().get_path("awesome_leg_pholus")+'/tests/static_poses/pose11/'+'robot_state_log__0_2022_02_02__17_30_35.mat' 
matfile_path12 = rospkg.RosPack().get_path("awesome_leg_pholus")+'/tests/static_poses/pose12/'+'robot_state_log__0_2022_02_02__17_32_30.mat' 
matfile_path13 = rospkg.RosPack().get_path("awesome_leg_pholus")+'/tests/static_poses/pose13/'+'robot_state_log__0_2022_02_02__17_33_17.mat' 

matfiles= [matfile_path1, matfile_path2, matfile_path3, matfile_path4, matfile_path5, matfile_path6, matfile_path7, matfile_path8,
           matfile_path9, matfile_path10, matfile_path11, matfile_path12, matfile_path13]

# Initializing a LogLoaders for reading and using the data inside the .mat file
log_loaders = []

for i in range(len(matfiles)):
    log_loaders.append(LogLoader(matfiles[i]))

######################### INITIALIZATIONS #########################

# Used actuators parameters
hip_act_params = acts_yaml["actuators"]["hip"] # hip
hip_rotor_axial_MoI = hip_act_params["rotor_axial_MoI"]
hip_red_ratio = hip_act_params["red_ratio"]
hip_efficiency = hip_act_params["efficiency"]
hip_K_t = hip_act_params["K_t"]

knee_act_params = acts_yaml["actuators"]["knee"] # knee
knee_rotor_axial_MoI = knee_act_params["rotor_axial_MoI"]
knee_red_ratio = knee_act_params["red_ratio"]
knee_efficiency = knee_act_params["efficiency"]
knee_K_t = knee_act_params["K_t"]

# Model interface & Co

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
    # test_taus.append(compute_tau_over_traj_pin(model, data, jnt_positions[i][:, 0:-1], jnt_velocities[i][:, 0:-1], filtered_diff_jnt_accs[i]))
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

n_active_jnts = len(q_p[:,0])
X_size = len(regressors[0, 0, :])

sigma = 0.001 * np.eye(X_size) # "confidence" matrix

# # hip
# sigma[0,0] = 0.001  # mass
# sigma[1,1] = 100000 # mass * l_x
# sigma[2,2] = 0.001 # mass * l_y
# sigma[3,3] = 1 # mass * l_z
# sigma[9,9] = 1 # izz

# # knee
# sigma[10, 10] = 0.001 # mass
# sigma[11, 11] = 0.001 # 10000; mass * l_x
# sigma[12, 12] = 10000 # 20000; mass * l_y
# sigma[13, 13] = 1 # mass * l_z
# sigma[19, 19] = 1 # izz

# sigma[20, 20] = 1000000 # tau_tilde[0]
# sigma[21, 21] = 1000000 # tau_tilde[1]

X_guess = np.concatenate((model.inertias[1].toDynamicParameters(), model.inertias[2].toDynamicParameters(), np.array([0, 0]))) # remember Pinocchio adds an additional universe link

P, qT = compute_P_qT(regressors, meas_tau, sigma, X_guess)

QP_init= np.zeros((X_size, 1)).flatten()

# print("qT: ", qT)
# print("P: ", P)

X = solve_qp(P = P, q = qT.T, initvals = QP_init)

print("X_guess ", X_guess)
print("X: ", X)

print("Inertial params: ", interpret_sol(n_active_jnts, X))

error_pre_opt, abs_mean_error_pre_opt = compute_fitting_error(regressors, meas_tau, X_guess)
error, abs_mean_error = compute_fitting_error(regressors, meas_tau, X)

print("Fitting error hip before optimization ([Nm]): ", error_pre_opt[0, :])
print("Fitting error knee before optimization ([Nm]): ", error_pre_opt[1, :])
print("Fitting abs mean error hip before optimization ([Nm]): ", abs_mean_error_pre_opt[0])
print("Fitting abs mean error knee before optimization ([Nm]): ", abs_mean_error_pre_opt[1])

print("Fitting error hip after optimization ([Nm]): ", error[0, :])
print("Fitting error knee after optimization ([Nm]): ", error[1, :])
print("Fitting abs mean error hip after optimization ([Nm]): ", abs_mean_error[0])
print("Fitting abs mean error knee after optimization ([Nm]): ", abs_mean_error[1])


