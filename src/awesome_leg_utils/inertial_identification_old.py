#!/usr/bin/env python3

from awesome_leg_utils.logger_utilities import LogLoader, LogPlotter

import rospkg

from xbot_interface import xbot_interface as xbot

import numpy as np

from qpsolvers import *

from scipy import signal

import yaml

from awesome_leg_utils.param_identification_utilities import *

import matplotlib as plt

######################### PRE-INITIALIZATIONS #########################

# Loading actuator paramters
abs_config_path = rospkg.RosPack().get_path("awesome_leg")+"/config/"
acts_yaml_name = "actuators.yaml"
acts_config_path = abs_config_path + acts_yaml_name
with open(acts_config_path, 'r') as stream:
    acts_yaml = yaml.safe_load(stream)

# Model description
urdf_path = rospkg.RosPack().get_path("awesome_leg")+"/description/urdf/awesome_legv2.urdf"   # urdf absolute path (remember RBDL might have some problems parsing the first link, if it has not a d.o.f.)
srdf_path = rospkg.RosPack().get_path("awesome_leg")+"/description/srdf/awesome_legv2.srdf"   # srdf absolute path
urdf = open(urdf_path, "r").read() # read the URDF
srdf = open(srdf_path, "r").read() # read the URDF

# Calibration data path
matfile_path1 = rospkg.RosPack().get_path("awesome_leg")+'/test_results/fixed_hip_setup/static_poses/pose1/'+'robot_state_log__0_2022_02_02__17_14_40.mat' 
matfile_path2 = rospkg.RosPack().get_path("awesome_leg")+'/test_results/fixed_hip_setup/static_poses/pose2/'+'robot_state_log__0_2022_02_02__17_17_21.mat' 
matfile_path3 = rospkg.RosPack().get_path("awesome_leg")+'/test_results/fixed_hip_setup/static_poses/pose3/'+'robot_state_log__0_2022_02_02__17_18_49.mat' 
matfile_path4 = rospkg.RosPack().get_path("awesome_leg")+'/test_results/fixed_hip_setup/static_poses/pose4/'+'robot_state_log__0_2022_02_02__17_20_31.mat' 
matfile_path5 = rospkg.RosPack().get_path("awesome_leg")+'/test_results/fixed_hip_setup/static_poses/pose5/'+'robot_state_log__0_2022_02_02__17_22_27.mat' 
matfile_path6 = rospkg.RosPack().get_path("awesome_leg")+'/test_results/fixed_hip_setup/static_poses/pose6/'+'robot_state_log__0_2022_02_02__17_24_16.mat' 
matfile_path7 = rospkg.RosPack().get_path("awesome_leg")+'/test_results/fixed_hip_setup/static_poses/pose7/'+'robot_state_log__0_2022_02_02__17_25_30.mat' 
matfile_path8 = rospkg.RosPack().get_path("awesome_leg")+'/test_results/fixed_hip_setup/static_poses/pose8/'+'robot_state_log__0_2022_02_02__17_26_50.mat' 
matfile_path9 = rospkg.RosPack().get_path("awesome_leg")+'/test_results/fixed_hip_setup/static_poses/pose9/'+'robot_state_log__0_2022_02_02__17_28_10.mat' 
matfile_path10 = rospkg.RosPack().get_path("awesome_leg")+'/test_results/fixed_hip_setup/static_poses/pose10/'+'robot_state_log__0_2022_02_02__17_29_32.mat' 
matfile_path11 = rospkg.RosPack().get_path("awesome_leg")+'/test_results/fixed_hip_setup/static_poses/pose11/'+'robot_state_log__0_2022_02_02__17_30_35.mat' 
matfile_path12 = rospkg.RosPack().get_path("awesome_leg")+'/test_results/fixed_hip_setup/static_poses/pose12/'+'robot_state_log__0_2022_02_02__17_32_30.mat' 
matfile_path13 = rospkg.RosPack().get_path("awesome_leg")+'/test_results/fixed_hip_setup/static_poses/pose13/'+'robot_state_log__0_2022_02_02__17_33_17.mat' 

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
xbot_model = xbot.ModelInterface(robot_cfg)

g = 9.81
m1 = 3
m2 = 5
l_hip = 0.2
######################### COMPUTING STUFF #########################

js_times= []
diff_jnt_accs = []
jnt_positions = []
jnt_velocities = []
filtered_diff_jnt_accs = []
test_taus = []

for i in range(len(matfiles)):
    js_times.append(log_loaders[i].get_js_rel_time())
    jnt_positions.append(log_loaders[i].get_motors_position())
    jnt_velocities.append(log_loaders[i].get_motors_velocity())
    diff_jnt_accs.append(diff_mat(js_times[i], jnt_velocities[i]))
    filtered_diff_jnt_accs.append(signal.filtfilt(b_acc, a_acc, diff_jnt_accs[i], padlen=150, axis= 1))
    test_taus.append(compute_tau_over_jnt_traj(xbot_model, jnt_positions[i][:, 0:-1], jnt_velocities[i][:, 0:-1], filtered_diff_jnt_accs[i]))

avrg_static_taus = []
avrg_static_pos = []

for i in range(len(matfiles)):
    avrg_static_taus.append(np.mean(test_taus[i], axis = 1))
    avrg_static_pos.append(np.mean(jnt_positions[i], axis = 1))

static_taus = rearrange_static_test_mat(avrg_static_taus)
static_pos = rearrange_static_test_mat(avrg_static_pos)

l_CoMs = compute_l_CoM(g, m1, m2, l_hip, static_pos, static_taus)

print(static_taus)
print(static_pos)