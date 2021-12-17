#!/usr/bin/env python3

###############################################
import rospy
import rospkg

from xbot_msgs.msg import JointCommand
from xbot_interface import xbot_interface as xbot

from horizon.utils import mat_storer

import numpy as np

import warnings
###############################################
rospackage=rospkg.RosPack()

urdf_rel_path = rospy.get_param("/horizon/urdf_relative_path")  # urdf relative path (wrt to the package)
media_rel_path = rospy.get_param("/horizon/media_relative_path")  # media relative path (wrt to the package)
opt_res_rel_path = rospy.get_param("/horizon/opt_results_rel_path")  # optimal results relative path (wrt to the package)

# xbot2 control modes codes (bitmask, p-v-e-k-d, where p is the LSB)
pos_cntrl_code=1
vel_cntrl_code=2
eff_cntrl_code=4
stiff_cntrl_code=8
damp_cntrl_code=16

task_type = rospy.get_param("/horizon/task_type")  # task type

if task_type=="jump":

    is_adaptive_dt = rospy.get_param("horizon/horizon_solver/is_adaptive_dt")  # if true, use an adaptive dt
    is_single_dt = rospy.get_param("horizon/horizon_solver/is_single_dt")  # if true (and if addaptive dt is enable), use only one dt over the entire opt. horizon 

    if is_adaptive_dt:
        if is_single_dt:
            n_nodes = rospy.get_param("horizon/horizon_solver/variable_dt/single_dt/problem_settings/n_nodes") # number of optimization nodes (remember to run the optimized node before, otherwise the parameter server will not be populated)
        else:
            n_nodes = rospy.get_param("horizon/horizon_solver/variable_dt/multiple_dt/problem_settings/n_nodes") # number of optimization nodes (remember to run the optimized node before, otherwise the parameter server will not be populated)
    else:
        n_nodes = rospy.get_param("horizon/horizon_solver/constant_dt/problem_settings/n_nodes") # number of optimization nodes (remember to run the optimized node before, otherwise the parameter server will not be populated)

    if is_adaptive_dt:
        if is_single_dt:
            ms = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+opt_res_rel_path+"/single_dt/horizon_offline_solver.mat")
        else:
            ms= mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+opt_res_rel_path+"/multiple_dt/horizon_offline_solver.mat")
    else:
        ms = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+opt_res_rel_path+"/fixed_dt/horizon_offline_solver.mat")

elif task_type=="trot":

    n_nodes = rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/n_nodes") # number of optimization nodes (remember to run the optimized node before, otherwise the parameter server will not be populated)
    ms = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+opt_res_rel_path+"/first_version/horizon_offline_solver.mat")


solution=ms.load() # loading the solution dictionary

q_p=solution["q_p"]
q_p_dot=solution["q_p_dot"]
q_p_ddot=solution["q_p_ddot"]
tau=solution["tau"]
f_contact=solution["f_contact"]
solution_time=solution["sol_time"]
dt=solution["dt_opt"].flatten() 

time_vector = np.zeros(dt.size+1)
for i in range(dt.size):
    time_vector[i+1] = time_vector[i] + dt[i]

hip_cntrl_mode = rospy.get_param("/horizon/joints/hip_joint/control_mode") # control mode (position, velocity, effort)
knee_cntrl_mode = rospy.get_param("/horizon/joints/knee_joint/control_mode") # control mode (position, velocity, effort)
hip_joint_stffnss = rospy.get_param("/horizon/joints/hip_joint/stiffness") # hip joint stiffness setting (to be used by Xbot)
knee_joint_stffnss = rospy.get_param("/horizon/joints/knee_joint/stiffness") # knee joint stiffness setting (to be used by Xbot)
hip_joint_damp = rospy.get_param("/horizon/joints/hip_joint/damping") # hip joint damping setting (to be used by Xbot)
knee_joint_damp = rospy.get_param("/horizon/joints/knee_joint/damping") # knee joint damping setting (to be used by Xbot)

joint_command=JointCommand() # initializing object for holding the joint command
joint_command.stiffness=[hip_joint_stffnss,knee_joint_stffnss]
joint_command.damping=[hip_joint_damp,knee_joint_damp]

ctrl_mode=[pos_cntrl_code+eff_cntrl_code+stiff_cntrl_code+damp_cntrl_code,
           pos_cntrl_code+eff_cntrl_code+stiff_cntrl_code+damp_cntrl_code] # default is set to position control + torque feedforwars+ stiffness and damping setting

if hip_cntrl_mode=="position":
    ctrl_mode[0]=pos_cntrl_code+eff_cntrl_code+stiff_cntrl_code+damp_cntrl_code
elif hip_cntrl_mode=="velocity":
    ctrl_mode[0]=vel_cntrl_code+eff_cntrl_code+stiff_cntrl_code+damp_cntrl_code
elif hip_cntrl_mode=="effort":
    ctrl_mode[0]=eff_cntrl_code+stiff_cntrl_code+damp_cntrl_code
else: warnings.warn("\nInvalid hip control mode. Please check your configuration file.\n Allowed control modes: \"position\",\t\"velocity\",\t\"effort\"\n")

if knee_cntrl_mode=="position":
    ctrl_mode[1]=pos_cntrl_code+eff_cntrl_code+stiff_cntrl_code+damp_cntrl_code
elif knee_cntrl_mode=="velocity":
    ctrl_mode[1]=vel_cntrl_code+eff_cntrl_code+stiff_cntrl_code+damp_cntrl_code
elif knee_cntrl_mode=="effort":
    ctrl_mode=eff_cntrl_code+stiff_cntrl_code+damp_cntrl_code
else: warnings.warn("\nInvalid hip control mode. The control mode will be set to position.\n Allowed control modes: \"position\",\t\"velocity\",\t\"effort\"\n")

joint_command.ctrl_mode=ctrl_mode

pub_iterator=0 # used to slide thorugh the solution


###############################################

def pack_xbot2_message():
    if pub_iterator<=(n_nodes-1): # continue sliding and publishing until the end of the solution is reached
        joint_command.name=["hip_pitch_1","knee_pitch_1"]
        joint_command.position=[q_p[0,pub_iterator],q_p[1,pub_iterator]]
        joint_command.velocity=[q_p_dot[0,pub_iterator],q_p_dot[1,pub_iterator]]
        joint_command.effort=[tau[0,pub_iterator-1],tau[1,pub_iterator-1]]
        print(pub_iterator)

def horizon_pub():
    global pub_iterator
    pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size=10)
    rospy.init_node('horizon_publisher', anonymous=True)

    while not rospy.is_shutdown(): 
        if pub_iterator>(n_nodes-1):
            pub_iterator=0 # replaying trajectory after end
            rospy.sleep(2) # sleep between replayed trajectories
        pub_iterator=pub_iterator+1 # incrementing publishing counter

        rate = rospy.Rate(1/dt[pub_iterator-1])
        print(1/dt[pub_iterator-1])
        pack_xbot2_message()
        pub.publish(joint_command)  
        rate.sleep()

if __name__ == '__main__':
    try:
        horizon_pub()
    except rospy.ROSInterruptException:
        pass
