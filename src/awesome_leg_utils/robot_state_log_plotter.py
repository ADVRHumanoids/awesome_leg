#!/usr/bin/env python3

from logger_utilities import LogLoader, LogPlotter

from xbot_interface import xbot_interface as xbot

# from horizon.utils import mat_storer

import numpy as np

from scipy import signal

import yaml

import rospy

######################### PRE-INITIALIZATIONS #########################

matfile_path = rospy.get_param("/robot_state_log_plotter/matfile_path") 
save_fig_path = rospy.get_param("/robot_state_log_plotter/save_fig_path") 
save_fig = rospy.get_param("/robot_state_log_plotter/save_fig") 

# Initializing logger utilities and useful variables
log_loader = LogLoader(matfile_path) # initializing the LogLoader for reading and using the data inside the .mat file
plotter = LogPlotter(log_loader, clr_set_name = "tab10") # plotter instance

jnt_names = log_loader.get_joints_names()
joint_ids = log_loader.get_joints_id_from_names(jnt_names)
n_jnts = len(jnt_names)

js_time = log_loader.get_js_rel_time()
jnt_pos = log_loader.get_motors_position()
jnt_vel = log_loader.get_motors_velocity()

# auxiliary data
aux_signal_names = log_loader.get_aux_signal_names()
aux_signal_codes = log_loader.get_aux_signal_codes(aux_signal_names)
aux_signal_number = len(aux_signal_codes) # number of auxiliary signals

aux_times = [None] * (n_jnts * aux_signal_number) # list, because the rows will in general have different number of cols
aux_val = [None] * (n_jnts * aux_signal_number) # aux values
aux_val_postproc = [None] * (n_jnts * aux_signal_number) # post-processed aux values (in particular, filtered)

i_q_meas = [None] * (n_jnts) # measured i_q 
i_q_meas_filt = [None] * (n_jnts) # filtered measured i_q
i_q_meas_time = [None] * (n_jnts) # i_q time vector

# Filtering
filter_order_acc = 8
cutoff_freq_acc = 0.08
b_acc, a_acc = signal.butter(filter_order_acc, cutoff_freq_acc) # lowpass Butterworth filter with a cutoff of 0.125 times the Nyquist frequency

filter_order_curr = 8
cutoff_freq_curr = 0.01
b_curr, a_curr = signal.butter(filter_order_curr, cutoff_freq_curr) # lowpass Butterworth filter with a cutoff of 0.125 times the Nyquist frequency

filter_order_aux = 8
cutoff_freq_aux = 0.08
b_aux, a_aux = signal.butter(filter_order_aux, cutoff_freq_aux) # lowpass Butterworth filter with a cutoff of 0.125 times the Nyquist frequency

filter_order_vel = 8
cutoff_freq_vel = 0.008
b_vel, a_vel = signal.butter(filter_order_vel, cutoff_freq_vel) # lowpass Butterworth filter with a cutoff of 0.125 times the Nyquist frequency

filter_order_tau = 8
cutoff_freq_tau = 0.008
b_tau, a_tau = signal.butter(filter_order_tau, cutoff_freq_tau) # lowpass Butterworth filter 

# building the objects which will hold aux data
for joint_id in joint_ids: 

    joint_index = joint_id - 1 # converting to 0-based indexing

    for aux_name in aux_signal_names:

        times, vals = log_loader.extr_from_aux(joint_id, aux_name)

        code_index = log_loader.get_aux_signal_code(aux_name) - 1

        aux_times[code_index + joint_index * aux_signal_number] = times
        aux_val[code_index + joint_index * aux_signal_number] = vals

        if (not ("ref" in aux_name)): # not a reference signal --> filter it

            # filt_val = signal.filtfilt(b_aux, a_aux, vals) 
        
            aux_val_postproc[code_index + joint_index * aux_signal_number] = vals

            if ("iq" in aux_name): # also assign iq data to separate container
                
                i_q_meas[joint_index] = vals
                i_q_meas_filt[joint_index] = vals
                i_q_meas_time[joint_index] = times

        else: # reference signal --> append raw values
            
            aux_val_postproc[code_index + joint_index * aux_signal_number] = vals

# computing the tip cartesian impedance produced by the used joint impedance
log_stiffness = log_loader.get_joints_stiffness()
log_damping = log_loader.get_joints_damping()
motor_positions = log_loader.get_links_position()
n_log_samples = log_stiffness.shape[1]

######################### PLOTTING STUFF #########################

n_rows_subplot = n_jnts

n_cols_subplot = 1 

# Initializing figs
aux_fig = plotter.init_fig(fig_name = "aux_raw") 
aux_fig_filtered = plotter.init_fig(fig_name = "aux_filtered") 
motor_positions_fig = plotter.init_fig(fig_name = "motor_positions") 
motor_velocities_fig = plotter.init_fig(fig_name = "motor_velocities") 
joint_efforts_fig = plotter.init_fig(fig_name = "raw_joint_efforts") 
temperature_driver_fig = plotter.init_fig(fig_name = "temperature_driver") 
temperature_motor_fig = plotter.init_fig(fig_name = "temperature_motor") 
differentiated_jnt_acc_fig = plotter.init_fig(fig_name = "differentiated_jnt_acc") 
torque_validation_fig = plotter.init_fig(fig_name = "torque_validation") 
current_validation_fig = plotter.init_fig(fig_name = "current_validation") 
jnt_impedance_fig = plotter.init_fig(fig_name = "jnt_impedance") 

# Plotting

for joint_id in joint_ids:

    joint_name = log_loader.get_joints_names_from_id([joint_id])[0]

    # raw aux signals

    plotter.add_subplot(fig_name = "aux_raw", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    plotter.aux_plot(fig_name = "aux_raw", jnt_id = joint_id, title = "Unfiltered aux signals on joint (" + joint_name +")") 

    # post-processed aux signals

    plotter.add_subplot(fig_name = "aux_filtered", row = n_rows_subplot, column = n_cols_subplot, index = joint_id)
    
    first_plot = True # only used to check when adding the first plot
    for aux_name in aux_signal_names:

        aux_code = log_loader.get_aux_signal_code(aux_name)
        code_index = aux_code - 1 #converting to 0-based indexing

        if first_plot: # first plot --> add title and grid

            plot_title = "Filtered aux signals (" + joint_name +")"
            set_grid = True
            add_plot = False

            first_plot = False

        else:

            plot_title = None 
            set_grid = False
            add_plot = True
        
        plotter.vect_plot(fig_name = "aux_filtered", x_data = aux_times[code_index + (joint_id - 1) * aux_signal_number],\
            input_vector = aux_val_postproc[code_index + (joint_id - 1) * aux_signal_number],\
            title = plot_title, line_label = aux_name.replace('_aux_code', ''), set_grid = set_grid, add_plot = add_plot)
    
    # joint positions and velocities
    
    plotter.add_subplot(fig_name = "motor_positions", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    plotter.js_plot(fig_name = "motor_positions", input_matrix = log_loader.get_motors_position(), jnt_id = joint_id,\
        line_label = joint_name + " motor position", title = "Motor position VS Reference (" + joint_name +")") 
    plotter.js_plot(fig_name = "motor_positions", input_matrix = log_loader.get_pos_references(),\
        jnt_id = joint_id, line_label = joint_name + " position ref", set_grid = False, add_plot = True) 
    
    plotter.add_subplot(fig_name = "motor_velocities", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    plotter.js_plot(fig_name = "motor_velocities", input_matrix = log_loader.get_motors_velocity(),\
        jnt_id = joint_id, line_label = joint_name + " motor velocity", title = "Motor velocity VS Reference (" + joint_name +")") 
    plotter.js_plot(fig_name = "motor_velocities", input_matrix = log_loader.get_velocity_references(),\
        jnt_id = joint_id, line_label = joint_name + " velocity ref", set_grid = False, add_plot = True) 

    # raw joint efforts
        
    plotter.add_subplot(fig_name = "raw_joint_efforts", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    plotter.js_plot(fig_name = "raw_joint_efforts", input_matrix = log_loader.get_joints_efforts_ref(),\
        jnt_id = joint_id, line_label = joint_name + " effort ref", title = "Unfiltered joint effort VS Reference (" + joint_name +")", draw_style = 'steps-pre') 
    plotter.js_plot(fig_name = "raw_joint_efforts", input_matrix = log_loader.get_joints_efforts(),\
        jnt_id = joint_id, line_label = joint_name + " effort", set_grid = False, add_plot = True, draw_style = 'steps-pre') 

    # raw joint efforts vs estimated ones (via inverse dynamics)

    plotter.add_subplot(fig_name = "torque_validation", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    plotter.js_plot(fig_name = "torque_validation", input_matrix = log_loader.get_joints_efforts(),\
        jnt_id = joint_id, line_label = joint_name + " measured effort", draw_style = 'steps-pre') 

    # temperature data

    plotter.add_subplot(fig_name = "temperature_driver", row = n_rows_subplot, column=n_cols_subplot, index = joint_id) 
    plotter.js_plot(fig_name = "temperature_driver", input_matrix=log_loader.get_temp_drivers(),\
        jnt_id=joint_id, line_label = joint_name + " driver temperature", title = "Driver temperature (" + joint_name +")") 

    plotter.add_subplot(fig_name = "temperature_motor", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    plotter.js_plot(fig_name = "temperature_motor", input_matrix = log_loader.get_temp_motors(),\
        jnt_id = joint_id, line_label = joint_name + " motor temperature", title = "Motor temeperature (" + joint_name +")") 

    if (len(aux_signal_names) != 0):

        plotter.add_subplot(fig_name = "current_validation", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
        # unfiltered data
        plotter.vect_plot(fig_name = "current_validation", x_data = i_q_meas_time[joint_id - 1], \
            input_vector = i_q_meas[joint_id - 1], title = "Measured VS estimated i_q on " + joint_name + " joint", \
            line_label = joint_name + " measured i_q", set_grid = True, add_plot = False)
        # filtered data
        plotter.vect_plot(fig_name = "current_validation", x_data = i_q_meas_time[joint_id - 1], \
            input_vector = i_q_meas_filt[joint_id - 1], line_label = joint_name + " measured i_q (filtered)", set_grid = False, add_plot = True)

        pass

    plotter.add_subplot(fig_name = "jnt_impedance", row = n_rows_subplot, column = n_cols_subplot, index = joint_id) 
    plotter.js_plot(fig_name = "jnt_impedance", x_data = js_time, input_matrix = log_stiffness, jnt_id = joint_id, \
        line_label = joint_name + " joint stiffness", title = "Impedance setpoint of joint " + joint_name + " joint") 
    plotter.js_plot(fig_name = "jnt_impedance", x_data = js_time, input_matrix = log_damping, jnt_id = joint_id,\
        line_label = joint_name + " joint damping ",  set_grid = False, add_plot = True) 


plotter.show_plots()    

if save_fig: # save figures

    if (len(aux_signal_names) != 0):

        aux_fig.set_size_inches(16, 12)
        current_validation_fig.set_size_inches(16, 12)
        aux_fig.savefig(save_fig_path + "/" + "currents.pdf", format="pdf") 
        current_validation_fig.savefig(save_fig_path + "/" + "current_validation.pdf", format="pdf") 

    motor_positions_fig.set_size_inches(16, 12)
    motor_velocities_fig.set_size_inches(16, 12)
    joint_efforts_fig.set_size_inches(16, 12)
    temperature_driver_fig.set_size_inches(16, 12)
    temperature_motor_fig.set_size_inches(16, 12)
    differentiated_jnt_acc_fig.set_size_inches(16, 12)
    torque_validation_fig.set_size_inches(16, 12)
    
    motor_positions_fig.savefig(save_fig_path + "/" + "motor_positions.pdf", format="pdf") 
    motor_velocities_fig.savefig(save_fig_path + "/" + "motor_velocities.pdf", format="pdf") 
    joint_efforts_fig.savefig(save_fig_path + "/" + "joint_efforts.pdf", format="pdf") 
    temperature_driver_fig.savefig(save_fig_path + "/" + "temperature_driver.pdf", format="pdf") 
    temperature_motor_fig.savefig(save_fig_path + "/" + "temperature_driver.pdf", format="pdf") 
    differentiated_jnt_acc_fig.savefig(save_fig_path + "/" + "differentiated_jnt_acc.pdf", format="pdf") 
    torque_validation_fig.savefig(save_fig_path + "/" + "torque_validation.pdf", format="pdf") 
