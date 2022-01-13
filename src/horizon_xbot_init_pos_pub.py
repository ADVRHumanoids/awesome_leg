#!/usr/bin/env python3

####################### IMPORTS #######################

from pickle import FALSE
import rospy
import rospkg

from xbot_msgs.msg import JointCommand
from xbot_msgs.msg import JointState

from horizon.utils import mat_storer

import numpy as np

####################### SETTING PARAMETERS ON THE ROS PARAMETER SERVER #######################

rospy.set_param("/horizon/xbot_command_pub/approaching_traj/is_initial_pose_reached",False)# used to tell other nodes if the approach procedure has finished

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

q_p=solution["q_p"] # only used to extract the first traj. sample
dt=T_exec_approach/n_int_approach_traj

time_vector = np.zeros(n_int_approach_traj+1)
for i in range(n_int_approach_traj):
    time_vector[i+1] = time_vector[i] + dt

joint_command=JointCommand() # initializing object for holding the joint command
ctrl_mode=[1, 1] 

joint_command.ctrl_mode=ctrl_mode
joint_command.name=["hip_pitch_1","knee_pitch_1"]

pub_iterator=0 # used to slide through the solution

q_p_init_target=[q_p[0,0],q_p[1,0]]
current_q_p=[0,0] # currently measured joint positions (to be used by "horizon_initial_pose_pub")

####################### FUNCTIONS #######################

def generate_Peisekah_trajectory(n_int_approach_traj, T_exec_approach, q_p_init_target, q_p_current):    

    common_part_traj= (126*np.power(time_vector/T_exec_approach,5)-420*np.power(time_vector/T_exec_approach,6)+540*np.power(time_vector/T_exec_approach,7)-315*np.power(time_vector/T_exec_approach,8)+70*np.power(time_vector/T_exec_approach,9))
    
    common_part_traj=np.transpose(np.tile(common_part_traj, (2,1)))

    q_p_approach_traj=np.tile(q_p_current,(n_int_approach_traj+1,1))+np.multiply(np.tile((q_p_init_target-q_p_current),(n_int_approach_traj+1,1)),common_part_traj)

    return q_p_approach_traj

def pack_xbot2_message(q_p_approach_traj):
            
    if pub_iterator<=(n_int_approach_traj-1): # continue sliding and publishing until the end of the solution is reached
        
        joint_command.position=[q_p_approach_traj[pub_iterator,0],q_p_approach_traj[pub_iterator,1]]

        print("Publishing initialization trajectory sample n.:\t"+str(pub_iterator)+","+"\n") # print a simple debug message
        print("with publish rate:\t"+str(1/dt)+"\n") # print a simple debug message

def xbot_cmd_publisher():
    global pub_iterator # using the corresponding variable initialized in the global scope
    global current_q_p # using the corresponding variable initialized in the global scope
    
    xbot_cmd_pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size=10) # publish on xbotcore/command
    rospy.Subscriber('/xbotcore/joint_states', JointState, current_q_p_assigner) # subscribe at xbotcore/joint_states

    rospy.init_node('horizon_xbot_publisher', anonymous=True) # initialize a publisher node (anonymous, so potentially another node of the same type can run concurrently)

    while not rospy.is_shutdown(): 
        if pub_iterator==0: # read the current joint trajectory and use it as the initialization
            q_p_approach_traj=generate_Peisekah_trajectory(n_int_approach_traj, T_exec_approach, np.array(q_p_init_target), current_q_p)

        if pub_iterator>(n_nodes-1): # finished playing the approach trajectory, set the topic message and shutdown node
            rospy.set_param("horizon/xbot_command_pub/approaching_traj/is_initial_pose_reached",True)
            rospy.signal_shutdown("Finished publishing approaching trajectory")
        pub_iterator=pub_iterator+1 # incrementing publishing counter

        rate = rospy.Rate(1/dt) # potentially, a trajectory with variable dt can be provided

        pack_xbot2_message(q_p_approach_traj) # copy the optimized trajectory to xbot command object 

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
