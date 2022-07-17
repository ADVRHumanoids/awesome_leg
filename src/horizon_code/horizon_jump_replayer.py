#!/usr/bin/env python3

########################## IMPORTS ##########################

import rospkg

import yaml

from horizon.utils import mat_storer

import matplotlib.pyplot as plt

import numpy as np

import casadi_kin_dyn

from horizon.ros.replay_trajectory import *

import subprocess
########################## INITIALIZATIONS & Co ##########################

# Loading some paramters directly from YAML
abs_horizon_config_path = rospkg.RosPack().get_path("awesome_leg")+"/config/horizon/"
abs_launch_path = rospkg.RosPack().get_path("awesome_leg")+"/launch/"
urdf_base_path = rospkg.RosPack().get_path("awesome_leg")+"/description/urdf/"

config_file_name = "horizon_replayer.yaml"
config_path = abs_horizon_config_path + config_file_name
with open(config_path, 'r') as stream:
    horizon_yaml = yaml.safe_load(stream)

opt_results = rospkg.RosPack().get_path("awesome_leg") + "/" + horizon_yaml["horizon"]["horizon_replayer"]["opt_results_rel_path"]  # optimal results absolute pathù

urdf_path = urdf_base_path + horizon_yaml["horizon"]["horizon_replayer"]["urdf_name"]  # URDF absolute path
urdf_file = open(urdf_path, "r").read()
urdf_awesome_leg = casadi_kin_dyn.py3casadi_kin_dyn.CasadiKinDyn(urdf_file)
joint_names = urdf_awesome_leg.joint_names()

ms_loaded = mat_storer.matStorer(opt_results)

solution=ms_loaded.load() # loading the solution dictionary

q_p=solution["q_p"]
GRF=solution["f_contact"]
dt=solution["dt_opt"].flatten()

subprocess.Popen(["roslaunch", abs_launch_path + "rviz_standalone_complete.launch", 'gazebo_model_type:=sliding_hip', 'use_joint_publisher_gui:=false'])
rospy.loginfo("leg jump replay visualization on RViz started")

joint_names.remove("universe")  # removing the "universe joint"
# rpl_traj = replay_trajectory(dt[0], joint_names, q_p, GRF, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, urdf_awesome_leg)  # replaying the (resampled) trajectory
rpl_traj = replay_trajectory(dt[0], joint_names, q_p)  # replaying the (resampled) trajectory
rpl_traj.sleep(1.)
rpl_traj.replay(is_floating_base = False)