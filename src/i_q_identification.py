#!/usr/bin/env python3

from awesome_leg_pholus_utils.logger_utilities import LogLoader, LogPlotter

import rospkg

from xbot_interface import xbot_interface as xbot

import numpy as np

from qpsolvers import *

from scipy import signal

import yaml

from awesome_leg_pholus_utils.param_identification_utilities import *

######################### PRE-INITIALIZATIONS #########################

# Loading actuator paramters
abs_config_path = rospkg.RosPack().get_path("awesome_leg_pholus")+"/config/"
acts_yaml_name = "actuators.yaml"
acts_config_path = abs_config_path + acts_yaml_name
with open(acts_config_path, 'r') as stream:
    acts_yaml = yaml.safe_load(stream)

# Model description
urdf_path = rospkg.RosPack().get_path("awesome_leg_pholus")+"/description/urdf/awesome_legv2.urdf"   # urdf absolute path (remember RBDL might have some problems parsing the first link, if it has not a d.o.f.)
srdf_path = rospkg.RosPack().get_path("awesome_leg_pholus")+"/description/srdf/awesome_legv2.srdf"   # srdf absolute path
urdf = open(urdf_path, "r").read() # read the URDF
srdf = open(srdf_path, "r").read() # read the URDF

# Calibration data path
matfile_path = rospkg.RosPack().get_path("awesome_leg_pholus")+'/test_results/fixed_hip_setup/system_identification/'+'robot_state_log__0_2022_01_26__16_49_17.mat' 

# matfile_path = rospkg.RosPack().get_path("awesome_leg_pholus")+'/test_results/fixed_hip_setup/cyclic_tests/large_excursion/faster/'+'robot_state_log__0_2022_01_26__16_50_32.mat' # faster cyclic test
# matfile_path = rospkg.RosPack().get_path("awesome_leg_pholus")+'/test_results/fixed_hip_setup/trot/fixed_hip_ample_trot/'+'robot_state_log__0_2022_01_24__17_14_01.mat' # path to the used .mat file holding the test results

# Initializing a LogLoader for reading and using the data inside the .mat file
log_loader = LogLoader(matfile_path) 

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

red_ratios=np.array([hip_red_ratio, knee_red_ratio])
efficiencies=np.array([hip_efficiency, knee_efficiency])

# Filtering
filter_order_acc = 8
cutoff_freq_acc = 0.05
b_acc, a_acc = signal.butter(filter_order_acc, cutoff_freq_acc) # lowpass Butterworth filter 

filter_order_curr = 8
cutoff_freq_curr = 0.01
b_curr, a_curr = signal.butter(filter_order_curr, cutoff_freq_curr) # lowpass Butterworth filter

filter_order_vel = 8
cutoff_freq_vel = 0.008
b_vel, a_vel = signal.butter(filter_order_vel, cutoff_freq_vel) # lowpass Butterworth filter 

filter_order_tau = 8
cutoff_freq_tau = 0.008
b_tau, a_tau = signal.butter(filter_order_tau, cutoff_freq_tau) # lowpass Butterworth filter 

# XBot model interface & Co
robot_cfg = get_xbot_cfg(urdf, srdf)
model = xbot.ModelInterface(robot_cfg)

######################### COMPUTING STUFF #########################

js_time=log_loader.get_js_rel_time()

# tau_horizon = compute_tau_over_jnt_traj(model, q_p_hor, q_p_dot_hor, q_p_ddot_hor) # effort computed using the optimized trajectory (note: possibly != the tau returned by the optimizer)

diff_jnt_acc = diff_mat(js_time, log_loader.get_motors_velocity()) # differentiated joints acceleration (from measurements)
filtered_diff_jnt_acc = signal.filtfilt(b_acc, a_acc, diff_jnt_acc, padlen=150, axis= 1) # filtered joint acceleration

jnt_pos = log_loader.get_motors_position()
jnt_vel = log_loader.get_motors_velocity()

test_tau = compute_tau_over_jnt_traj_xbot(model, jnt_pos[:, 0:-1], jnt_vel[:, 0:-1], filtered_diff_jnt_acc) # efforts computed with the measured joint trajectory
meas_tau = log_loader.get_joints_efforts()
filt_meas_tau = signal.filtfilt(b_tau, a_tau, meas_tau, padlen=150, axis= 1)

i_q_hip_time, i_q_hip_measured = log_loader.extr_from_aux(jnt_index = 0, sgnl_name = 'iq_out_fb_aux_code')
i_q_knee_time, i_q_knee_measured = log_loader.extr_from_aux(jnt_index = 1, sgnl_name = 'iq_out_fb_aux_code')
i_q_measured = [i_q_hip_measured, i_q_knee_measured]
i_q_measured_filt = signal.filtfilt(b_curr, a_curr, i_q_measured, padlen=150, axis= 1) # filtered i_q currents
i_q_meas_time = [i_q_hip_time, i_q_knee_time]

######################### OPTIMIZATION (QP) #########################
n_jnts = len(jnt_pos[:, 0])

# l_b_hip = np.array([ hip_rotor_axial_MoI, 0.01, 0.001, 0.001])
# l_b_knee = np.array([ knee_rotor_axial_MoI, 0.01, 0.001, 0.001])

# u_b_hip = np.array([ hip_rotor_axial_MoI, 0.09, 1000, 1000])
# u_b_knee = np.array([ knee_rotor_axial_MoI, 0.09, 1000, 1000])

# l_b_hip = np.array([ hip_rotor_axial_MoI, 0, 0.001, 0.001]) # rotor_MoI, K_t, K_d0, K_d1
# l_b_knee = np.array([ knee_rotor_axial_MoI, 0.045, 0.001, 0.001])

# u_b_hip = np.array([ hip_rotor_axial_MoI, 1, 1000, 1000]) # rotor_MoI, K_t, K_d0, K_d1
# u_b_knee = np.array([ knee_rotor_axial_MoI, 1, 1000, 1000])

l_b_hip = np.array([ hip_rotor_axial_MoI, hip_K_t, 0.001, 0.001]) # rotor_MoI, K_t, K_d0, K_d1
l_b_knee = np.array([ knee_rotor_axial_MoI, knee_K_t, 0.001, 0.001])

u_b_hip = np.array([ hip_rotor_axial_MoI, hip_K_t, 1000, 1000]) # rotor_MoI, K_t, K_d0, K_d1
u_b_knee = np.array([ knee_rotor_axial_MoI, knee_K_t, 1000, 1000])

N_interp_samples = 100 # number of samples over which the data is interpolated  (also equal to the number of samples of the QP)
vel_sign_threshold= 1e-1 # threshold used for computing the sign of the jnt velocities
smooth_coeff = 20

sigma_hip = 0.001 * np.eye(n_jnts + 2) # regularization matrix hip
sigma_knee = 0.1 * np.eye(n_jnts + 2) # regularization matrix knee

QP_init_hip = np.array([ hip_rotor_axial_MoI, hip_K_t, 1, 1])
QP_init_knee = np.array([ knee_rotor_axial_MoI, knee_K_t, 1, 1])
# QP_init = np.concatenate((QP_init_hip, QP_init_knee))

# P_QP, q_T_QP, _, B_QP = compute_P_qT(i_q_meas_time, js_time, i_q_measured, jnt_vel, filtered_diff_jnt_acc, log_loader.get_joints_efforts(), red_ratios, efficiencies, N_interp_samples, vel_sign_threshold)
P_QP_hip, q_T_QP_hip, A_QP_hip, B_QP_hip = compute_P_qT_i_q(sigma_hip, QP_init_hip, 0, i_q_meas_time, js_time, i_q_measured_filt, jnt_vel, filtered_diff_jnt_acc, filt_meas_tau, red_ratios, N_interp_samples, vel_sign_threshold, smooth_coeff)
P_QP_knee, q_T_QP_knee, A_QP_knee, B_QP_knee = compute_P_qT_i_q(sigma_knee, QP_init_knee, 1, i_q_meas_time, js_time, i_q_measured_filt, jnt_vel, filtered_diff_jnt_acc, filt_meas_tau, red_ratios, N_interp_samples, vel_sign_threshold, smooth_coeff)

# x = solve_qp(P = P_QP, q= q_T_QP.T, lb = np.concatenate((l_b_hip, l_b_knee), axis = 0), ub = np.concatenate((u_b_hip, u_b_knee), axis = 0), A = A_cnstr, b = b_cnstr, initvals = QP_init)
x_hip = solve_qp(P = P_QP_hip, q= q_T_QP_hip.T, lb = l_b_hip, ub = u_b_hip, initvals = QP_init_hip)
x_knee = solve_qp(P = P_QP_knee, q = q_T_QP_knee.T, lb = l_b_knee, ub = u_b_knee, initvals = QP_init_knee)

# print("X_opt: ", x)
print("X_hip: ", x_hip)
print("X_knee: ", x_knee)

# cost=compute_cost(x, P_QP, q_T_QP, B_QP)
cost_hip = compute_cost(x_hip, P_QP_hip, q_T_QP_hip, B_QP_hip)
cost_knee = compute_cost(x_knee, P_QP_knee, q_T_QP_knee, B_QP_knee)

# cost_hip_raw=compute_cost_raw(A_QP_hip, B_QP_hip, x_hip)
# cost_knee_raw=compute_cost_raw(A_QP_knee, B_QP_knee, x_knee)

# print("Cost: ", cost)
print("Cost hip: ", cost_hip)
print("Cost knee: ", cost_knee)

print("Cumulative cost: ", cost_hip + cost_knee)

hip_eff = x_hip[0] / hip_rotor_axial_MoI
knee_eff = x_knee[0] / knee_rotor_axial_MoI
J_r_hip = hip_rotor_axial_MoI
J_r_knee = knee_rotor_axial_MoI
K_t_hip = x_hip[1] / hip_eff
K_t_knee = x_knee[1] / knee_eff
K_d0_hip = x_hip[2]
K_d0_knee = x_knee[2]
K_d1_hip = x_hip[3]
K_d1_knee = x_knee[3]

print("Efficiencies: ", [hip_eff, knee_eff])
print("J_r: ", [J_r_hip, J_r_knee])
print("Kts: ", [K_t_hip, K_t_knee])
print("K_d0s: ", [K_d0_hip, K_d0_knee])
print("K_d1s: ", [K_d1_hip, K_d1_knee])

print("\n")

vel_sign_threshold_plot= vel_sign_threshold # threshold used for computing the sign of the jnt velocities associated with the plots

total_tau = test_tau + [K_d0_hip * compute_smooth_sign(jnt_vel[0, 0:-1], coeff = smooth_coeff) + K_d1_hip * jnt_vel[0, 0:-1],
                        K_d0_knee * compute_smooth_sign(jnt_vel[1, 0:-1], coeff = smooth_coeff) + K_d1_knee * jnt_vel[1, 0:-1] ]

i_q_hip_estimate = (J_r_hip * filtered_diff_jnt_acc[0,:] / hip_red_ratio + total_tau[0,:] * hip_red_ratio / hip_eff) / K_t_hip
i_q_knee_estimate = (J_r_knee * filtered_diff_jnt_acc[1,:] / knee_red_ratio + total_tau[1,:] * knee_red_ratio / knee_eff) / K_t_knee
i_q_estimate = np.array([i_q_hip_estimate, i_q_knee_estimate])
i_q_estimate_filt = signal.filtfilt(b_curr, a_curr, i_q_estimate, padlen=150, axis= 1) # filtered i_q currents

######################### PLOTTING STUFF #########################

plotter = LogPlotter(log_loader, clr_set_name="tab10") # plotter instance

n_rows1 = len(log_loader.get_joint_names()) # subplot rows
n_cols1 = 1 # subplot columns

fig = plotter.init_fig(fig_name = "current_validation") 

for joint_id in log_loader.get_joints_id_from_names(log_loader.get_joint_names()):

    joint_name = log_loader.get_joint_names_from_id([joint_id])[0]

    plotter.add_subplot(fig_name = "current_validation", row = n_rows1, column = n_cols1, index = joint_id) 
    plotter.js_plot(fig_name = "current_validation", x_data = i_q_meas_time[joint_id - 1][:], input_matrix = i_q_measured, jnt_id = joint_id, line_label = joint_name + " measured i_q", title = "Measured VS estimated i_q on " + joint_name + " joint") 
    plotter.js_plot(fig_name = "current_validation", x_data = js_time[1:len(js_time)], input_matrix = i_q_estimate, jnt_id = joint_id, line_label = joint_name + " estimated i_q", set_grid = False, add_plot = True) 
    plotter.js_plot(fig_name = "current_validation", x_data =  i_q_meas_time[joint_id - 1][:], input_matrix = i_q_measured_filt, jnt_id = joint_id, line_label = joint_name + " measured i_q (filtered)", set_grid = False, add_plot = True) 
    plotter.js_plot(fig_name = "current_validation", x_data =  js_time[1:len(js_time)], input_matrix = i_q_estimate_filt, jnt_id = joint_id, line_label = joint_name + " estimated i_q (filtered)", set_grid = False, add_plot = True) 

input() # necessary to keep all the figures open
