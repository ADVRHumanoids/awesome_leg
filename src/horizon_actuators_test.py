#!/usr/bin/env python3

###############################################
import rospy
import rospkg

from xbot_msgs.msg import JointCommand

from horizon.utils import mat_storer

import numpy as np
###############################################
# csi-> reduction ratio
# eta-> harmonic drive efficiency
# i_d=[Izz omega_dot+tau*csi/eta]*1/K_t

rospackage=rospkg.RosPack()
ms_loaded = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/sim_results/horizon_offline_solver.mat")
ms_test = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/sim_results/horizon_actuators_test.mat")
solution=ms_loaded.load() # loading the solution dictionary

q_p_dot=solution["q_p_dot"]
q_p_ddot=solution["q_p_ddot"]
tau=solution["tau"]

n_nodes = rospy.get_param("/horizon/problem/n_nodes") # number of optimization nodes (remember to run the optimized node before, otherwise the parameter server will not be populated)
hip_cntrl_mode = rospy.get_param("/horizon/joints/hip_joint/control_mode") # control mode (position, velocity, effort)

# hip actuator
hip_axial_MoI=rospy.get_param("/actuators/hip/axial_MoI")
hip_mass=rospy.get_param("/actuators/hip/mass")
hip_red_ratio=rospy.get_param("/actuators/hip/red_ratio")
hip_efficiency=rospy.get_param("/actuators/hip/efficiency")
hip_K_t=rospy.get_param("/actuators/hip/K_t")
hip_K_bmf=rospy.get_param("/actuators/hip/K_bmf")
hip_R=rospy.get_param("/actuators/hip/R")
hip_tau_max_br=rospy.get_param("/actuators/hip/tau_max_br")
hip_tau_max_ar=rospy.get_param("/actuators/hip/tau_max_ar")
hip_tau_peak_br=rospy.get_param("/actuators/hip/tau_peak_br")
hip_tau_peak_ar=rospy.get_param("/actuators/hip/tau_peak_ar")
hip_I_max=rospy.get_param("/actuators/hip/I_max")
hip_I_peak=rospy.get_param("/actuators/hip/I_peak")
hip_omega_max_nl_bf=rospy.get_param("/actuators/hip/omega_max_nl_bf")
hip_omega_max_nl_af=rospy.get_param("/actuators/hip/omega_max_nl_af")
hip_omega_max_nl_af44=rospy.get_param("/actuators/hip/omega_max_nl_af44")
hip_omega_max_nl_af112=rospy.get_param("/actuators/hip/omega_max_nl_af112")

# knee actuator
knee_axial_MoI=rospy.get_param("/actuators/knee/axial_MoI")
knee_mass=rospy.get_param("/actuators/knee/mass")
knee_red_ratio=rospy.get_param("/actuators/knee/red_ratio")
knee_efficiency=rospy.get_param("/actuators/knee/efficiency")
knee_K_t=rospy.get_param("/actuators/knee/K_t")
knee_K_bmf=rospy.get_param("/actuators/knee/K_bmf")
knee_R=rospy.get_param("/actuators/knee/R")
knee_tau_max_br=rospy.get_param("/actuators/knee/tau_max_br")
knee_tau_max_ar=rospy.get_param("/actuators/knee/tau_max_ar")
knee_tau_peak_br=rospy.get_param("/actuators/knee/tau_peak_br")
knee_tau_peak_ar=rospy.get_param("/actuators/knee/tau_peak_ar")
knee_I_max=rospy.get_param("/actuators/knee/I_max")
knee_I_peak=rospy.get_param("/actuators/knee/I_peak")
knee_omega_max_nl_bf=rospy.get_param("/actuators/knee/omega_max_nl_bf")
knee_omega_max_nl_af=rospy.get_param("/actuators/knee/omega_max_nl_af")
knee_omega_max_nl_af44=rospy.get_param("/actuators/knee/omega_max_nl_af44")
knee_omega_max_nl_af112=rospy.get_param("/actuators/knee/omega_max_nl_af112")

i_d_est_hip=np.zeros([n_nodes-1])
i_d_est_knee=np.zeros([n_nodes-1])
was_max_vel_exceeded_hip=np.zeros([n_nodes-1])
was_max_vel_exceeded_knee=np.zeros([n_nodes-1])

for i in range(n_nodes-1): 

    i_d_est_hip[i]=(hip_axial_MoI*q_p_ddot[0,i+1]+tau[0,i]*hip_red_ratio/hip_efficiency)*1.0/hip_K_t
    i_d_est_knee[i]=(knee_axial_MoI*q_p_ddot[1,i+1]+tau[1,i]*knee_red_ratio/knee_efficiency)*1.0/knee_K_t

    if  q_p_dot[0,i+1]>hip_omega_max_nl_af44 : was_max_vel_exceeded_hip[i]=1
    else: was_max_vel_exceeded_hip[i]=0
    if  q_p_dot[1,i+1]>knee_omega_max_nl_af44 : was_max_vel_exceeded_knee[i]=1
    else: was_max_vel_exceeded_knee[i]=0

actuators_debug_test_hip={"i_d_est":i_d_est_hip,"was_max_vel_exceeded":was_max_vel_exceeded_hip}
actuators_debug_test_knee={"i_d_est":i_d_est_knee,"was_max_vel_exceeded":was_max_vel_exceeded_knee}
actuators_debug_test={"hip":actuators_debug_test_hip, "knee":actuators_debug_test_knee}

ms_test.store(actuators_debug_test) # saving solution data to file

print(actuators_debug_test)


