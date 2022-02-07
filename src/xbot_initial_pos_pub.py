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

n_int_approach_traj=rospy.get_param("xbot_initial_pos_pub/approaching_traj/n_intervals")
T_exec_approach=rospy.get_param("xbot_initial_pos_pub/approaching_traj/T_execution")
traj_exec_standby_time=rospy.get_param("xbot_initial_pos_pub/approaching_traj/standby_time")
q_p_target=rospy.get_param("xbot_initial_pos_pub/approaching_traj/q_p_target")

dt=T_exec_approach/n_int_approach_traj

time_vector = np.zeros(n_int_approach_traj+1)
for i in range(n_int_approach_traj):
    time_vector[i+1] = time_vector[i] + dt

joint_command=JointCommand() # initializing object for holding the joint command
ctrl_mode=[1, 1] 

joint_command.ctrl_mode=ctrl_mode
joint_command.name=["hip_pitch_1","knee_pitch_1"]

pub_iterator= -1 # used to slide through the solution
flip_traj= False

current_q_p=[0,0] # (initialization). Currently measured joint positions (to be used by "horizon_initial_pose_pub")

####################### FUNCTIONS #######################

def generate_Peisekah_trajectory(n_int_approach_traj, T_exec_approach, q_p_target, q_p_current):    

    common_part_traj= (126*np.power(time_vector/T_exec_approach,5)-420*np.power(time_vector/T_exec_approach,6)+540*np.power(time_vector/T_exec_approach,7)-315*np.power(time_vector/T_exec_approach,8)+70*np.power(time_vector/T_exec_approach,9))
    
    common_part_traj=np.transpose(np.tile(common_part_traj, (2,1)))

    q_p_approach_traj=np.tile(q_p_current,(n_int_approach_traj+1,1))+np.multiply(np.tile((q_p_target-q_p_current),(n_int_approach_traj+1,1)),common_part_traj)

    return q_p_approach_traj

def pack_xbot2_message(q_p_approach_traj):
            
    if pub_iterator<=(n_int_approach_traj-1): # continue sliding and publishing until the end of the solution is reached
        
        joint_command.position=[q_p_approach_traj[pub_iterator,0],q_p_approach_traj[pub_iterator,1]]

        print("Publishing initialization trajectory sample n.:\t"+str(pub_iterator)+","+"\n") # print a simple debug message
        print("with publish rate:\t"+str(1/dt)+"\n") # print a simple debug message

def xbot_cmd_publisher():
    global pub_iterator # using the corresponding variable initialized in the global scope
    global current_q_p # using the corresponding variable initialized in the global scope
    global q_p_target

    xbot_cmd_pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size=10) # publish on xbotcore/command
    rospy.Subscriber('/xbotcore/joint_states', JointState, current_q_p_assigner) # subscribe at xbotcore/joint_states

    rospy.init_node('horizon_xbot_publisher', anonymous=False) # initialize a publisher node (not anonymous, so another node of the same type cannot run concurrently)

    while not rospy.is_shutdown(): 
        
        if pub_iterator==-1: # read the current joint position and use it as the initialization
            
            q_p_approach_traj=generate_Peisekah_trajectory(n_int_approach_traj, T_exec_approach, np.array(q_p_target), current_q_p)

        if pub_iterator>(n_int_approach_traj-1): # finished playing the approach trajectory, set the topic message and shutdown node

            rospy.sleep(traj_exec_standby_time) # wait before going back to the initial position and starting over again
            
            rospy.signal_shutdown("Finished publishing approaching trajectory")

        else:

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
