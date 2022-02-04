#!/usr/bin/env python3

from awesome_leg_pholus.logger_utilities import LogLoader, LogPlotter

from xbot_interface import xbot_interface as xbot

from horizon.utils import mat_storer

import numpy as np

from scipy import signal

import yaml

from awesome_leg_pholus.param_identification_utilities import *

import rospy

######################### PRE-INITIALIZATIONS #########################

urdf_path = rospy.get_param("/log_plotter/urdf_path") 
srdf_path = rospy.get_param("/log_plotter/srdf_path") 
acts_config_path = rospy.get_param("/log_plotter/actuators_config_path") 
matfile_path = rospy.get_param("/log_plotter/matfile_path") 
opt_res_path = rospy.get_param("/log_plotter/opt_results_path") 
save_fig_path = rospy.get_param("/log_plotter/save_fig_path") 
save_fig = rospy.get_param("/log_plotter/save_fig") 

# Loading actuator paramters
with open(acts_config_path, 'r') as stream:
    acts_yaml = yaml.safe_load(stream)

# Model description
urdf = open(urdf_path, "r").read() # read the URDF
srdf = open(srdf_path, "r").read() # read the URDF

# Loading optimized trajectory results
# opt_res_path = rospkg.RosPack().get_path("awesome_leg_pholus")+ "/opt_results/horizon_trot/fixed_hip"  # optimal results absolute path
# ms_loaded = mat_storer.matStorer(opt_res_path)
# solution = ms_loaded.load() # loading the solution dictionary

# Initializing logger utilities
log_loader = LogLoader(matfile_path) # initializing the LogLoader for reading and using the data inside the .mat file
plotter = LogPlotter(log_loader, clr_set_name="tab10") # plotter instance

######################### INITIALIZATIONS #########################

# Used actuators parameters
hip_act_params = acts_yaml["actuators"]["hip"] # hip
hip_rotor_axial_MoI = hip_act_params["rotor_axial_MoI"]
hip_red_ratio = hip_act_params["red_ratio"]
hip_efficiency = hip_act_params["efficiency"]
hip_K_t = hip_act_params["K_t"]
K_d0_hip = hip_act_params["K_d0"]
K_d1_hip = hip_act_params["K_d1"]

knee_act_params = acts_yaml["actuators"]["knee"] # knee
knee_rotor_axial_MoI = knee_act_params["rotor_axial_MoI"]
knee_red_ratio = knee_act_params["red_ratio"]
knee_efficiency = knee_act_params["efficiency"]
knee_K_t = knee_act_params["K_t"]
K_d0_knee = knee_act_params["K_d0"]
K_d1_knee = knee_act_params["K_d1"]

# Filtering
filter_order_acc = 8
cutoff_freq_acc = 0.05
b_acc, a_acc = signal.butter(filter_order_acc, cutoff_freq_acc) # lowpass Butterworth filter with a cutoff of 0.125 times the Nyquist frequency

filter_order_curr = 8
cutoff_freq_curr = 0.01
b_curr, a_curr = signal.butter(filter_order_curr, cutoff_freq_curr) # lowpass Butterworth filter with a cutoff of 0.125 times the Nyquist frequency

filter_order_vel = 8
cutoff_freq_vel = 0.008
b_vel, a_vel = signal.butter(filter_order_vel, cutoff_freq_vel) # lowpass Butterworth filter with a cutoff of 0.125 times the Nyquist frequency

filter_order_tau = 8
cutoff_freq_tau = 0.008
b_tau, a_tau = signal.butter(filter_order_tau, cutoff_freq_tau) # lowpass Butterworth filter 

# Horizon
# q_p_hor = solution["q_p"]
# q_p_dot_hor = solution["q_p_dot"]
# q_p_ddot_hor = solution["q_p_ddot"]
# tau_horizon= solution["tau"]
# dt = solution["dt_opt"].flatten()
# horizon_time = np.zeros([q_p_hor[0,:].size+1])
# for i in range(q_p_hor[0,:].size-1):
#     horizon_time[i+1] = horizon_time[i] + dt[i]

# XBot model interface & Co
robot_cfg = get_xbot_cfg(urdf, srdf)
model = xbot.ModelInterface(robot_cfg)

######################### NON-MODELED EFFECTS COMPENSATIONS #########################

# Compensation for unmodeled friction torque (vel-dependent)
K_d0 = np.array([K_d0_hip, K_d0_knee])
K_d1 = np.array([K_d1_hip, K_d1_knee])

######################### COMPUTING STUFF #########################
js_time=log_loader.get_js_rel_time()

# tau_horizon = compute_tau_over_jnt_traj(model, q_p_hor, q_p_dot_hor, q_p_ddot_hor) # effort computed using the optimized trajectory (note: != the tau returned by the optimizer)

diff_jnt_acc = diff_mat(js_time, log_loader.get_motors_velocity()) # differentiated joint acceleration (from measurements)
filtered_diff_jnt_acc = signal.filtfilt(b_acc, a_acc, diff_jnt_acc, padlen=150, axis= 1) # filtered joint acceleration

jnt_pos = log_loader.get_motors_position()
jnt_vel = log_loader.get_motors_velocity()

# filtered_jnt_vel = signal.filtfilt(b_vel, a_vel, log_loader.get_motors_velocity(), padlen=150, axis= 1)

smooth_coeff = 20
test_tau = compute_tau_over_jnt_traj(model, jnt_pos[:, 0:-1], jnt_vel[:, 0:-1], filtered_diff_jnt_acc) # efforts computed with the measured joint trajectory
total_tau = test_tau + [K_d0_hip * compute_smooth_sign(jnt_vel[0, 0:-1], coeff = smooth_coeff) + K_d1_hip * jnt_vel[0, 0:-1],
                        K_d0_knee * compute_smooth_sign(jnt_vel[1, 0:-1], coeff = smooth_coeff) + K_d1_knee * jnt_vel[1, 0:-1] ]

# i_q_hip_hor = (hip_rotor_axial_MoI*q_p_ddot_hor[0,:]/hip_red_ratio+tau_horizon[0,:]*hip_red_ratio/hip_efficiency)*1.0/hip_K_t
# i_q_knee_hor = (knee_rotor_axial_MoI*q_p_ddot_hor[1,:]/knee_red_ratio+tau_horizon[1,:]*knee_red_ratio/knee_efficiency)*1.0/knee_K_t

i_q_hip_estimate = (hip_rotor_axial_MoI*filtered_diff_jnt_acc[0,:]/hip_red_ratio+total_tau[0,:]*hip_red_ratio/hip_efficiency)*1.0/hip_K_t
i_q_knee_estimate = (knee_rotor_axial_MoI*filtered_diff_jnt_acc[1,:]/knee_red_ratio+total_tau[1,:]*knee_red_ratio/knee_efficiency)*1.0/knee_K_t
i_q_estimate = np.array([i_q_hip_estimate, i_q_knee_estimate])
i_q_estimate_filt = signal.filtfilt(b_curr, a_curr, i_q_estimate, padlen=150, axis= 1) # filtered i_q currents

i_q_hip_time, i_q_hip_measured=log_loader.extr_from_aux(0,'iq_out_fb_aux_code')
i_q_knee_time, i_q_knee_measured=log_loader.extr_from_aux(1,'iq_out_fb_aux_code')

i_q_measured = np.array([i_q_hip_measured, i_q_knee_measured])
i_q_measured_filt = signal.filtfilt(b_curr, a_curr, i_q_measured, padlen=150, axis= 1) # filtered i_q currents
i_q_meas_time = np.array([i_q_hip_time, i_q_knee_time])

######################### PLOTTING STUFF #########################

n_rows1 = len(log_loader.get_joint_names()) # subplot rows
n_cols1 = 1 # subplot columns

# Initializing figs
currents_fig = plotter.init_fig(fig_name = "currents") 
motor_positions_fig = plotter.init_fig(fig_name = "motor_positions") 
motor_velocities_fig = plotter.init_fig(fig_name = "motor_velocities") 
joint_efforts_fig = plotter.init_fig(fig_name = "joint_efforts") 
temperature_driver_fig = plotter.init_fig(fig_name = "temperature_driver") 
temperature_motor_fig = plotter.init_fig(fig_name = "temperature_motor") 
# horizon_torques_fig = plotter.init_fig(fig_name = "horizon_torques") 
differentiated_jnt_acc_fig = plotter.init_fig(fig_name = "differentiated_jnt_acc") 
torque_validation_fig = plotter.init_fig(fig_name = "torque_validation") 
current_validation_fig = plotter.init_fig(fig_name = "current_validation") 
# filtered_jnt_vel_fig = plotter.init_fig(fig_name = "filtered_jnt_vel") 

# Plotting
for joint_id in log_loader.get_joints_id_from_names(log_loader.get_joint_names()):

    joint_name = log_loader.get_joint_names_from_id([joint_id])[0]

    plotter.add_subplot(fig_name = "currents", row = n_rows1, column = n_cols1, index = joint_id) 
    plotter.aux_plot(fig_name = "currents", jnt_id = joint_id, title = "Aux signals on joint \""+ joint_name+"\"") 

    plotter.add_subplot(fig_name = "motor_positions", row = n_rows1, column = n_cols1, index = joint_id) 
    plotter.js_plot(fig_name = "motor_positions", input_matrix = log_loader.get_motors_position(), jnt_id = joint_id, line_label = joint_name + " motor position", title = "Motor position VS Reference (" + joint_name +")") 
    plotter.js_plot(fig_name = "motor_positions", input_matrix = log_loader.get_pos_references(), jnt_id = joint_id, line_label = joint_name + " position ref", set_grid = False, add_plot = True, ) 
    plotter.js_plot(fig_name = "motor_positions", input_matrix = log_loader.get_links_position(), jnt_id = joint_id, line_label =joint_name + " link position", set_grid = False, add_plot = True) 

    plotter.add_subplot(fig_name = "motor_velocities", row = n_rows1, column = n_cols1, index = joint_id) 
    plotter.js_plot(fig_name = "motor_velocities", input_matrix = log_loader.get_motors_velocity(), jnt_id = joint_id, line_label = joint_name + " motor velocity", title = "Motor velocity VS Reference (" + joint_name +")") 
    plotter.js_plot(fig_name = "motor_velocities", input_matrix = log_loader.get_velocity_references(), jnt_id = joint_id, line_label = joint_name + " velocity ref", set_grid = False, add_plot = True) 

    plotter.add_subplot(fig_name = "joint_efforts", row = n_rows1, column = n_cols1, index = joint_id) 
    plotter.js_plot(fig_name = "joint_efforts", input_matrix = log_loader.get_joints_efforts(), jnt_id = joint_id, line_label = joint_name + " effort", title = "Joint effort VS Reference (" + joint_name +")", draw_style = 'steps-pre') 
    plotter.js_plot(fig_name = "joint_efforts", input_matrix = log_loader.get_joints_efforts_ref(), jnt_id = joint_id, line_label = joint_name + " effort ref", set_grid = False, add_plot = True, draw_style = 'steps-pre') 

    plotter.add_subplot(fig_name = "temperature_driver", row = n_rows1, column=n_cols1, index = joint_id) 
    plotter.js_plot(fig_name = "temperature_driver", input_matrix=log_loader.get_temp_drivers(), jnt_id=joint_id, line_label = joint_name + " driver temperature", title = "Driver temperature (" + joint_name +")") 

    plotter.add_subplot(fig_name = "temperature_motor", row = n_rows1, column = n_cols1, index = joint_id) 
    plotter.js_plot(fig_name = "temperature_motor", input_matrix = log_loader.get_temp_motors(), jnt_id = joint_id, line_label = joint_name + " motor temperature", title = "Motor temeperature (" + joint_name +")") 

    # plotter.add_subplot(fig_name = "horizon_torques", row = n_rows1, column = n_cols1, index = joint_id) 
    # plotter.js_plot(fig_name = "horizon_torques", x_data=horizon_time[0:-2], input_matrix = tau_horizon, jnt_id = joint_id, line_label = joint_name + " torque", title = "Torque on " + joint_name + " (horizon)", draw_style = 'steps-pre') 

    plotter.add_subplot(fig_name = "torque_validation", row = n_rows1, column = n_cols1, index = joint_id) 
    plotter.js_plot(fig_name = "torque_validation", input_matrix = log_loader.get_joints_efforts(), jnt_id = joint_id, line_label = joint_name + " measured effort", draw_style = 'steps-pre') 
    plotter.js_plot(fig_name = "torque_validation", x_data=js_time[1:len(js_time)], input_matrix = test_tau, jnt_id = joint_id, line_label = joint_name + " computed effort", title = "Effort on " + joint_name + " (validation)", draw_style = 'steps-pre', set_grid = False, add_plot = True) 

    plotter.add_subplot(fig_name = "differentiated_jnt_acc", row = n_rows1, column = n_cols1, index = joint_id) 
    plotter.js_plot(fig_name = "differentiated_jnt_acc", x_data=js_time[1:len(js_time)], input_matrix = filtered_diff_jnt_acc, jnt_id = joint_id, line_label = joint_name + " diff jnt acc. (filtered)", title = "Differentiated joint acc. of " + joint_name + " joint") 
    plotter.js_plot(fig_name = "differentiated_jnt_acc", x_data=js_time[1:len(js_time)], input_matrix = diff_jnt_acc, jnt_id = joint_id, line_label = joint_name + " diff jnt acc. ",  set_grid = False, add_plot = True) 

    plotter.add_subplot(fig_name = "current_validation", row = n_rows1, column = n_cols1, index = joint_id) 
    plotter.js_plot(fig_name = "current_validation", x_data = i_q_meas_time[joint_id-1, :], input_matrix = i_q_measured, jnt_id = joint_id, line_label = joint_name + " measured i_q", title = "Measured VS estimated i_q on " + joint_name + " joint") 
    plotter.js_plot(fig_name = "current_validation", x_data = js_time[1:len(js_time)], input_matrix = i_q_estimate, jnt_id = joint_id, line_label = joint_name + " estimated i_q", set_grid = False, add_plot = True) 
    plotter.js_plot(fig_name = "current_validation", x_data =  i_q_meas_time[joint_id - 1, :], input_matrix = i_q_measured_filt, jnt_id = joint_id, line_label = joint_name + " measured i_q (filtered)", set_grid = False, add_plot = True) 
    plotter.js_plot(fig_name = "current_validation", x_data =  js_time[1:len(js_time)], input_matrix = i_q_estimate_filt, jnt_id = joint_id, line_label = joint_name + " estimated i_q (filtered)", set_grid = False, add_plot = True) 

    # plotter.add_subplot(fig_name = "filtered_jnt_vel", row = n_rows1, column = n_cols1, index = joint_id) 
    # plotter.js_plot(fig_name = "filtered_jnt_vel", input_matrix = filtered_jnt_vel, jnt_id = joint_id, line_label = joint_name + " jnt velocity", title = "Filtered joint velocity " + joint_name + " joint") 

if save_fig:

    currents_fig.set_size_inches(16, 12)
    motor_positions_fig.set_size_inches(16, 12)
    motor_velocities_fig.set_size_inches(16, 12)
    joint_efforts_fig.set_size_inches(16, 12)
    temperature_driver_fig.set_size_inches(16, 12)
    temperature_motor_fig.set_size_inches(16, 12)
    differentiated_jnt_acc_fig.set_size_inches(16, 12)
    torque_validation_fig.set_size_inches(16, 12)
    current_validation_fig.set_size_inches(16, 12)

    currents_fig.savefig(save_fig_path + "/" + "currents.pdf", format="pdf") 
    motor_positions_fig.savefig(save_fig_path + "/" + "motor_positions.pdf", format="pdf") 
    motor_velocities_fig.savefig(save_fig_path + "/" + "motor_velocities.pdf", format="pdf") 
    joint_efforts_fig.savefig(save_fig_path + "/" + "joint_efforts.pdf", format="pdf") 
    temperature_driver_fig.savefig(save_fig_path + "/" + "temperature_driver.pdf", format="pdf") 
    temperature_motor_fig.savefig(save_fig_path + "/" + "temperature_driver.pdf", format="pdf") 
    differentiated_jnt_acc_fig.savefig(save_fig_path + "/" + "differentiated_jnt_acc.pdf", format="pdf") 
    torque_validation_fig.savefig(save_fig_path + "/" + "torque_validation.pdf", format="pdf") 
    current_validation_fig.savefig(save_fig_path + "/" + "current_validation.pdf", format="pdf") 

input() # necessary to keep all the figures open
