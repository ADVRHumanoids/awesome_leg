#!/usr/bin/env python3

####################### IMPORTS #######################

import rospy

from xbot_msgs.msg import JointCommand
from xbot_msgs.msg import JointState

import numpy as np

import pinocchio as pin

#########################################################

class HorizonInitialXbotCmdPub:

    def __init__(self):

        self.xbot_cmd_pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size=10) # publish on xbotcore/command
        self.xbot_state_sub = rospy.Subscriber('/xbotcore/joint_states', JointState, self.current_q_p_assigner) # subscribe at xbotcore/joint_states
        
        self.urdf_path = rospy.get_param("xbot_initial_pos_pub/urdf_path")
        self.urdf = open(self.urdf_path, "r").read() # read the URDF
        self.pin_model = pin.buildModelFromXML(self.urdf)
        self.pin_model_data = self.pin_model.createData()
        self.n_jnts = self.pin_model.nq
        self.joint_names = []
        [self.joint_names.append((self.pin_model.names[i])) for i in range(1, self.n_jnts+1)]

        self.n_int_approach_traj = rospy.get_param("xbot_initial_pos_pub/approaching_traj/n_intervals")
        self.T_exec_approach = rospy.get_param("xbot_initial_pos_pub/approaching_traj/T_execution")
        self.traj_exec_standby_time = rospy.get_param("xbot_initial_pos_pub/approaching_traj/standby_time")
        self.q_p_target = rospy.get_param("xbot_initial_pos_pub/approaching_traj/q_p_target")
    
        self.q_p_approach_traj = np.zeros((self.n_int_approach_traj, self.n_jnts))

        self.dt = self.T_exec_approach / self.n_int_approach_traj
        self.time_vector = np.zeros(self.n_int_approach_traj + 1)
        for i in range(self.n_int_approach_traj):
            self.time_vector[i+1] = self.time_vector[i] + self.dt

        self.joint_command = JointCommand() # initializing object for holding the joint command
        self.joint_command.ctrl_mode = rospy.get_param("/xbot_initial_pos_pub/approaching_traj/ctrl_mode")  
        self.joint_command.name = self.joint_names  
        self.joint_command.stiffness = rospy.get_param("/xbot_initial_pos_pub/approaching_traj/stiffness")  
        self.joint_command.damping = rospy.get_param("/xbot_initial_pos_pub/approaching_traj/damping")  

        self.pub_iterator = -1 # used to slide through the solution

        self.current_q_p = np.zeros((self.n_jnts, 1)).flatten() # (initialization). Currently measured joint positions (to be used by "horizon_initial_pose_pub")

    def generate_Peisekah_trajectory(self):    

        common_term_traj= (126 * np.power(self.time_vector/self.T_exec_approach, 5) - 420 * np.power(self.time_vector/self.T_exec_approach, 6) + 540 * np.power(self.time_vector/self.T_exec_approach, 7) - 315 * np.power(self.time_vector/self.T_exec_approach, 8) + 70 * np.power(self.time_vector/self.T_exec_approach, 9))
        
        common_term_traj = np.transpose(np.tile(common_term_traj, (self.n_jnts, 1) )) # replicating the common term along the joint axes

        self.q_p_approach_traj = np.tile(self.current_q_p, (self.n_int_approach_traj + 1, 1)) + np.multiply(np.tile((self.q_p_target - self.current_q_p), (self.n_int_approach_traj + 1, 1)), common_term_traj)

    def pack_xbot2_message(self):
            
        if self.pub_iterator <= (self.n_int_approach_traj - 1): # continue sliding and publishing until the end of the solution is reached
            
            position_command = np.zeros((self.n_jnts, 1)).flatten()

            for i in range(self.n_jnts):

                position_command[i] = self.q_p_approach_traj[self.pub_iterator, i]

            self.joint_command.position = np.ndarray.tolist(position_command)

            print("Publishing initialization trajectory sample n.:\t" + str(self.pub_iterator + 1) + "," + "\n") # print a simple debug message
            print("with a rate of :\t" + str(1/self.dt) + "Hz \n") # print a simple debug message

    def xbot_cmd_publisher(self):

        self.rate = rospy.Rate(1/self.dt) # potentially, a trajectory with variable dt can be provided
        
        while not rospy.is_shutdown(): 
            
            if self.pub_iterator == -1: # read the current joint position and use it as the initialization
                
                self.generate_Peisekah_trajectory()

            if self.pub_iterator > (self.n_int_approach_traj - 1): # finished playing the approach trajectory, set the topic message and shutdown node

                rospy.sleep(self.traj_exec_standby_time) # wait before going back to the initial position and starting over again

                rospy.loginfo("Finished publishing approaching trajectory")
                rospy.signal_shutdown("Finished publishing approaching trajectory")

            else:

                self.pub_iterator = self.pub_iterator + 1 # incrementing publishing counter

                self.pack_xbot2_message() # copy the optimized trajectory to xbot command object 

                self.xbot_cmd_pub.publish(self.joint_command) # publish the commands

                self.rate.sleep() # wait
                
    def current_q_p_assigner(self, joints_state): # callback function which reads the joint_states and updates current_q_p

        self.current_q_p = np.array(joints_state.motor_position) # assigning the state to the global variable
    

if __name__ == '__main__':

    xbot_init_pos_pub = HorizonInitialXbotCmdPub()

    rospy.init_node('xbot_init_pos_pub', anonymous = False) # initialize a publisher node (not anonymous, so another node of the same type cannot run concurrently)
    
    try: # start publishing the commands

        xbot_init_pos_pub.xbot_cmd_publisher()

    except rospy.ROSInterruptException:
        
        pass

