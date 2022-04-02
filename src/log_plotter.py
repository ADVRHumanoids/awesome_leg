#!/usr/bin/env python3

from awesome_leg_pholus_utils.logger_utilities import LogLoader, LogPlotter

from xbot_interface import xbot_interface as xbot

# from horizon.utils import mat_storer

import numpy as np

from scipy import signal

import yaml

from awesome_leg_pholus_utils.param_identification_utilities import *

import rospy

######################### PRE-INITIALIZATIONS #########################

urdf_path = rospy.get_param("/log_plotter/urdf_path") 
srdf_path = rospy.get_param("/log_plotter/srdf_path") 
acts_config_path = rospy.get_param("/log_plotter/actuators_config_path") 
matfile_path = rospy.get_param("/log_plotter/matfile_path") 
save_fig_path = rospy.get_param("/log_plotter/save_fig_path") 
save_fig = rospy.get_param("/log_plotter/save_fig") 

# Loading actuator paramters
with open(acts_config_path, 'r') as stream:
    acts_yaml = yaml.safe_load(stream)

# Model description
urdf = open(urdf_path, "r").read() # read the URDF
srdf = open(srdf_path, "r").read() # read the URDF

# Initializing logger utilities and useful variables
log_loader = LogLoader(matfile_path) # initializing the LogLoader for reading and using the data inside the .mat file
plotter = LogPlotter(log_loader, clr_set_name = "tab10") # plotter instance

jnt_names = log_loader.get_joint_names()
n_jnts = len(jnt_names)

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
cutoff_freq_acc = 0.08
b_acc, a_acc = signal.butter(filter_order_acc, cutoff_freq_acc) # lowpass Butterworth filter with a cutoff of 0.125 times the Nyquist frequency

filter_order_curr = 8
cutoff_freq_curr = 0.01
b_curr, a_curr = signal.butter(filter_order_curr, cutoff_freq_curr) # lowpass Butterworth filter with a cutoff of 0.125 times the Nyquist frequency

filter_order_aux = 8
cutoff_freq_aux = 0.05
b_aux, a_aux = signal.butter(filter_order_aux, cutoff_freq_aux) # lowpass Butterworth filter with a cutoff of 0.125 times the Nyquist frequency

filter_order_vel = 8
cutoff_freq_vel = 0.008
b_vel, a_vel = signal.butter(filter_order_vel, cutoff_freq_vel) # lowpass Butterworth filter with a cutoff of 0.125 times the Nyquist frequency

filter_order_tau = 8
cutoff_freq_tau = 0.008
b_tau, a_tau = signal.butter(filter_order_tau, cutoff_freq_tau) # lowpass Butterworth filter 

# XBot model interface & Co
robot_cfg = get_xbot_cfg(urdf, srdf)
model = xbot.ModelInterface(robot_cfg)

######################### NON-MODELED EFFECTS COMPENSATIONS #########################

# Compensation for unmodeled friction torque (vel-dependent)
K_d0 = np.array([K_d0_hip, K_d0_knee])
K_d1 = np.array([K_d1_hip, K_d1_knee])

######################### COMPUTING STUFF #########################
js_time = log_loader.get_js_rel_time()

diff_jnt_acc = diff_mat(js_time, log_loader.get_motors_velocity()) # differentiated joint accelerations (from measurements)
filtered_diff_jnt_acc = signal.filtfilt(b_acc, a_acc, diff_jnt_acc, padlen=150, axis= 1) # filtered joint acceleration

jnt_pos = log_loader.get_motors_position()
jnt_vel = log_loader.get_motors_velocity()

smooth_coeff = 20
test_tau = compute_tau_over_jnt_traj_xbot(model, jnt_pos[:, 0:-1], jnt_vel[:, 0:-1], filtered_diff_jnt_acc) #efforts computed with the measured joint trajectory

total_tau = test_tau + np.dot(K_d0, compute_smooth_sign(jnt_vel[:, 0:-1], coeff = smooth_coeff)) + np.dot(K_d1, jnt_vel[:, 0:-1]) # fictitious total torque (estimated + friction effects)

i_q_estimate = (hip_rotor_axial_MoI*filtered_diff_jnt_acc/hip_red_ratio+total_tau*hip_red_ratio/hip_efficiency)*1.0/hip_K_t
i_q_estimate_filt = signal.filtfilt(b_curr, a_curr, i_q_estimate, padlen=150, axis= 1) # filtered i_q currents

# Auxiliary data

aux_signal_names = log_loader.get_aux_signal_names()
aux_signal_codes = log_loader.get_aux_signal_codes(aux_signal_names)
aux_signal_number = len(aux_signal_codes) # number of auxiliary signals

aux_times = [] # list, because the rows will in general have different number of cols
aux_val = [] # aux values
aux_val_filt = [] # filtered aux values 

for jnt in range(0, n_jnts):

    for aux_name in aux_signal_names:

        aux_code = log_loader.get_aux_signal_code(aux_name)

        code_index = aux_code - 1 # converting to 0-based indexing

        times, vals = log_loader.extr_from_aux(jnt, aux_signal_names[code_index])

        aux_times.append(times)
        aux_val.append(vals)

        if not ("ref" in aux_name): # not a reference signal --> filter it

            filt_val = signal.filtfilt(b_aux, a_aux, vals, padlen=150, axis= 0) 

            aux_val_filt.append(filt_val) 

        else: # reference signal --> append raw values

            aux_val_filt.append(vals) 

######################### PLOTTING STUFF #########################

n_cols_subplot = 1 
n_rows_subplot = n_jnts

# Initializing figs
aux_fig = plotter.init_fig(fig_name = "aux") 
aux_fig_filtered = plotter.init_fig(fig_name = "aux_filtered") 
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
joint_ids = log_loader.get_joints_id_from_names(jnt_names)

for joint_id in joint_ids:

    joint_name = log_loader.get_joint_names_from_id([joint_id])[0]

    plotter.add_subplot(fig_name = "aux", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    plotter.aux_plot(fig_name = "aux", jnt_id = joint_id, title = "Unfiltered aux signals on joint (" + joint_name +")") 

    plotter.add_subplot(fig_name = "aux_filtered", row = n_rows_subplot, column = n_cols_subplot, index = joint_id)
    
    j = 0 # only used to check when adding the first plot
    for aux_name in aux_signal_names:

        aux_code = log_loader.get_aux_signal_code(aux_name)
        code_index = aux_code - 1 #converting to 0-based indexing

        if j == 0: # first plot --> add title and grid
            plot_title = "Filtered aux signals (" + joint_name +")"
            set_grid = True
            add_plot = False
            j=+1

        else:
            plot_title = None 
            set_grid = False
            add_plot = True
        
        plotter.vect_plot(fig_name = "aux_filtered", x_data = aux_times[code_index + (joint_id - 1) * aux_signal_number], input_vector = aux_val_filt[code_index + (joint_id - 1) * aux_signal_number], title = plot_title, line_label = aux_name.replace('_aux_code', ''), set_grid = set_grid, add_plot = add_plot)
    
    plotter.add_subplot(fig_name = "motor_positions", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    plotter.js_plot(fig_name = "motor_positions", input_matrix = log_loader.get_motors_position(), jnt_id = joint_id, line_label = joint_name + " motor position", title = "Motor position VS Reference (" + joint_name +")") 
    plotter.js_plot(fig_name = "motor_positions", input_matrix = log_loader.get_pos_references(), jnt_id = joint_id, line_label = joint_name + " position ref", set_grid = False, add_plot = True) 
    plotter.js_plot(fig_name = "motor_positions", input_matrix = log_loader.get_links_position(), jnt_id = joint_id, line_label =joint_name + " link position", set_grid = False, add_plot = True) 

    plotter.add_subplot(fig_name = "motor_velocities", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    plotter.js_plot(fig_name = "motor_velocities", input_matrix = log_loader.get_motors_velocity(), jnt_id = joint_id, line_label = joint_name + " motor velocity", title = "Motor velocity VS Reference (" + joint_name +")") 
    plotter.js_plot(fig_name = "motor_velocities", input_matrix = log_loader.get_velocity_references(), jnt_id = joint_id, line_label = joint_name + " velocity ref", set_grid = False, add_plot = True) 

    plotter.add_subplot(fig_name = "joint_efforts", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    plotter.js_plot(fig_name = "joint_efforts", input_matrix = log_loader.get_joints_efforts_ref(), jnt_id = joint_id, line_label = joint_name + " effort ref", title = "Joint effort VS Reference (" + joint_name +")", draw_style = 'steps-pre') 
    plotter.js_plot(fig_name = "joint_efforts", input_matrix = log_loader.get_joints_efforts(), jnt_id = joint_id, line_label = joint_name + " effort", set_grid = False, add_plot = True, draw_style = 'steps-pre') 

    plotter.add_subplot(fig_name = "temperature_driver", row = n_rows_subplot, column=n_cols_subplot, index = joint_id) 
    plotter.js_plot(fig_name = "temperature_driver", input_matrix=log_loader.get_temp_drivers(), jnt_id=joint_id, line_label = joint_name + " driver temperature", title = "Driver temperature (" + joint_name +")") 

    plotter.add_subplot(fig_name = "temperature_motor", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    plotter.js_plot(fig_name = "temperature_motor", input_matrix = log_loader.get_temp_motors(), jnt_id = joint_id, line_label = joint_name + " motor temperature", title = "Motor temeperature (" + joint_name +")") 

    plotter.add_subplot(fig_name = "torque_validation", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    plotter.js_plot(fig_name = "torque_validation", input_matrix = log_loader.get_joints_efforts(), jnt_id = joint_id, line_label = joint_name + " measured effort", draw_style = 'steps-pre') 
    plotter.js_plot(fig_name = "torque_validation", x_data=js_time[1:len(js_time)], input_matrix = test_tau, jnt_id = joint_id, line_label = joint_name + " computed effort", title = "Effort on " + joint_name + " (validation)", draw_style = 'steps-pre', set_grid = False, add_plot = True) 

    plotter.add_subplot(fig_name = "differentiated_jnt_acc", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    plotter.js_plot(fig_name = "differentiated_jnt_acc", x_data=js_time[1:len(js_time)], input_matrix = filtered_diff_jnt_acc, jnt_id = joint_id, line_label = joint_name + " diff jnt acc. (filtered)", title = "Differentiated joint acc. of " + joint_name + " joint") 
    plotter.js_plot(fig_name = "differentiated_jnt_acc", x_data=js_time[1:len(js_time)], input_matrix = diff_jnt_acc, jnt_id = joint_id, line_label = joint_name + " diff jnt acc. ",  set_grid = False, add_plot = True) 

    plotter.add_subplot(fig_name = "current_validation", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    # plotter.js_plot(fig_name = "current_validation", x_data = i_q_meas_time[joint_id-1, :], input_matrix = i_q_measured, jnt_id = joint_id, line_label = joint_name + " measured i_q", title = "Measured VS estimated i_q on " + joint_name + " joint") 
    plotter.js_plot(fig_name = "current_validation", x_data = js_time[1:len(js_time)], input_matrix = i_q_estimate, jnt_id = joint_id, line_label = joint_name + " estimated i_q", set_grid = False, add_plot = True) 
    # plotter.js_plot(fig_name = "current_validation", x_data =  i_q_meas_time[joint_id - 1, :], input_matrix = i_q_measured_filt, jnt_id = joint_id, line_label = joint_name + " measured i_q (filtered)", set_grid = False, add_plot = True) 
    plotter.js_plot(fig_name = "current_validation", x_data =  js_time[1:len(js_time)], input_matrix = i_q_estimate_filt, jnt_id = joint_id, line_label = joint_name + " estimated i_q (filtered)", set_grid = False, add_plot = True) 

if save_fig:

    aux_fig.set_size_inches(16, 12)
    motor_positions_fig.set_size_inches(16, 12)
    motor_velocities_fig.set_size_inches(16, 12)
    joint_efforts_fig.set_size_inches(16, 12)
    temperature_driver_fig.set_size_inches(16, 12)
    temperature_motor_fig.set_size_inches(16, 12)
    differentiated_jnt_acc_fig.set_size_inches(16, 12)
    torque_validation_fig.set_size_inches(16, 12)
    current_validation_fig.set_size_inches(16, 12)

    aux_fig.savefig(save_fig_path + "/" + "currents.pdf", format="pdf") 
    motor_positions_fig.savefig(save_fig_path + "/" + "motor_positions.pdf", format="pdf") 
    motor_velocities_fig.savefig(save_fig_path + "/" + "motor_velocities.pdf", format="pdf") 
    joint_efforts_fig.savefig(save_fig_path + "/" + "joint_efforts.pdf", format="pdf") 
    temperature_driver_fig.savefig(save_fig_path + "/" + "temperature_driver.pdf", format="pdf") 
    temperature_motor_fig.savefig(save_fig_path + "/" + "temperature_driver.pdf", format="pdf") 
    differentiated_jnt_acc_fig.savefig(save_fig_path + "/" + "differentiated_jnt_acc.pdf", format="pdf") 
    torque_validation_fig.savefig(save_fig_path + "/" + "torque_validation.pdf", format="pdf") 
    current_validation_fig.savefig(save_fig_path + "/" + "current_validation.pdf", format="pdf") 

input() # necessary to keep all the figures open
