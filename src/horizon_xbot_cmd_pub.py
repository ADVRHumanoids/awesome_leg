#!/usr/bin/env python3

####################### IMPORTS #######################

import rospy
import rospkg

from xbot_msgs.msg import JointCommand
from xbot_msgs.msg import JointState
# from xbot_interface import xbot_interface as xbot

from horizon.utils import mat_storer

import numpy as np

####################### LOADING PARAMETERS FROM THE ROS PARAMETER SERVER #######################

wait_until_initial_pose= rospy.get_param("/horizon/xbot_command_pub/approaching_traj/wait_until_initial_pose") # used 

## Paths parameters
rospackage=rospkg.RosPack() # getting absolute ros package path
opt_res_rel_path = rospy.get_param("/horizon/opt_results_rel_path")  # optimal results relative path (wrt to the package)
task_type = rospy.get_param("/horizon/task_type")  # task type

## Based on the task_type, load the right solution from the auxiliary folder
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
            ms = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/"+opt_res_rel_path+"/single_dt/horizon_offline_solver.mat")
        else:
            ms= mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/"+opt_res_rel_path+"/multiple_dt/horizon_offline_solver.mat")
    else:
        ms = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/"+opt_res_rel_path+"/fixed_dt/horizon_offline_solver.mat")

elif task_type=="trot":

    n_nodes = rospy.get_param("horizon/horizon_solver/problem_settings/n_nodes") # number of optimization nodes (remember to run the optimized node before, otherwise the parameter server will not be populated)
    ms = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/"+opt_res_rel_path+"/horizon_offline_solver.mat")
    
    n_int_approach_traj=rospy.get_param("/horizon/xbot_command_pub/approaching_traj/n_intervals")
    T_exec_approach=rospy.get_param("/horizon/xbot_command_pub/approaching_traj/T_execution")
    traj_exec_standby_time=rospy.get_param("/horizon/xbot_command_pub/approaching_traj/standby_time")

## Loading the solution dictionary, based on the selected task_type 
solution=ms.load() 

q_p=solution["q_p"]
q_p_dot=solution["q_p_dot"]
q_p_ddot=solution["q_p_ddot"]
tau=solution["tau"]
f_contact=solution["f_contact"]
solution_time=solution["sol_time"]
dt=solution["dt_opt"].flatten() 

time_vector = np.zeros(dt.size+1) # time vector used by the trajectory publisher
for i in range(dt.size):
    time_vector[i+1] = time_vector[i] + dt[i]

hip_cntrl_mode = rospy.get_param("/horizon/xbot_command_pub/joint_cntrl/hip_joint/control_mode") # hip control mode (bitmask, p-v-e-k-d->[1,2,4,8,16], where p is the LSB)
knee_cntrl_mode = rospy.get_param("/horizon/xbot_command_pub/joint_cntrl/knee_joint/control_mode") # knee control mode (bitmask, p-v-e-k-d->[1,2,4,8,16], where p is the LSB)
hip_joint_stffnss = rospy.get_param("/horizon/xbot_command_pub/joint_cntrl/hip_joint/stiffness") # hip joint stiffness setting (to be used by Xbot, if impedance cntrl is enabled)
knee_joint_stffnss = rospy.get_param("/horizon/xbot_command_pub/joint_cntrl/knee_joint/stiffness") # knee joint stiffness setting (to be used by Xbot, if impedance cntrl is enabled)
hip_joint_damp = rospy.get_param("/horizon/xbot_command_pub/joint_cntrl/hip_joint/damping") # hip joint damping setting (to be used by Xbot, if impedance cntrl is enabled)
knee_joint_damp = rospy.get_param("/horizon/xbot_command_pub/joint_cntrl/knee_joint/damping") # knee joint damping setting (to be used by Xbot, if impedance cntrl is enabled)

joint_command=JointCommand() # initializing object for holding the joint command
joint_command.stiffness=[hip_joint_stffnss,knee_joint_stffnss] # if impedance cntrl is not enabled, xbot simply ignores this setting
joint_command.damping=[hip_joint_damp,knee_joint_damp] # if impedance cntrl is not enabled, xbot simply ignores this setting

ctrl_mode=[hip_cntrl_mode, knee_cntrl_mode] # cntrl mode vector

joint_command.ctrl_mode=ctrl_mode
joint_command.name=["hip_pitch_1","knee_pitch_1"]

pub_iterator=0 # used to slide through the solution
is_q_init_reached=1 # when 1, "horizon_initial_pose_pub()" does not run

####################### FUNCTIONS #######################

def pack_xbot2_message():
            
    if pub_iterator<=(n_nodes-1): # continue sliding and publishing until the end of the solution is reached
        
        joint_command.position=[q_p[0,pub_iterator],q_p[1,pub_iterator]]
        joint_command.velocity=[q_p_dot[0,pub_iterator],q_p_dot[1,pub_iterator]]
        joint_command.effort=[tau[0,pub_iterator-1],tau[1,pub_iterator-1]]

        print("Publishing trajectory sample n.:\t"+str(pub_iterator)+","+"\n") # print a simple debug message
        print("with publish rate:\t"+str(1/dt[pub_iterator-1])+"\n") # print a simple debug message

def xbot_cmd_publisher():
    global pub_iterator # using the corresponding variable initialized in the global scope

    xbot_cmd_pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size=10) # publish on xbotcore/command

    rospy.init_node('horizon_xbot_publisher', anonymous=True) # initialize a publisher node (anonymous, so potentially another node of the same type can run concurrently)

    while not rospy.is_shutdown(): 
        print("\n")
        print(wait_until_initial_pose)
        print("\n")
        if wait_until_initial_pose:
            is_initial_pose_reached = rospy.get_param("/horizon/xbot_command_pub/approaching_traj/is_initial_pose_reached") # used 
            print("\n")
            print(is_initial_pose_reached)
            print("\n")
            if is_initial_pose_reached:
                if pub_iterator>(n_nodes-1):
                    pub_iterator=0 # replaying trajectory after end
                    # rospy.sleep(2) # sleep between replayed trajectories
                pub_iterator=pub_iterator+1 # incrementing publishing counter
                rate = rospy.Rate(1/dt[pub_iterator-1]) # potentially, a trajectory with variable dt can be provided
                pack_xbot2_message() # copy the optimized trajectory to xbot command object 
                xbot_cmd_pub.publish(joint_command) # publish the commands
                rate.sleep() # wait
        else:
            if pub_iterator>(n_nodes-1):
                    pub_iterator=0 # replaying trajectory after end
                    # rospy.sleep(2) # sleep between replayed trajectories
            pub_iterator=pub_iterator+1 # incrementing publishing counter
            rate = rospy.Rate(1/dt[pub_iterator-1]) # potentially, a trajectory with variable dt can be provided
            pack_xbot2_message() # copy the optimized trajectory to xbot command object 
            xbot_cmd_pub.publish(joint_command) # publish the commands
            rate.sleep() # wait

def current_q_p_assigner(joints_state): # callback function which reads the joint_states and updates current_q_p
    global current_q_p 
    current_q_p=joints_state.motor_position # assigning the state to the global variable
    
###############################################

if __name__ == '__main__':
    try:
        
        xbot_cmd_publisher()

    except rospy.ROSInterruptException:
        pass
