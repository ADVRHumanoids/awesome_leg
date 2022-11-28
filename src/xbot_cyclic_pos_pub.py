#!/usr/bin/env python3

####################### IMPORTS #######################

import rospy

from xbot_msgs.msg import JointCommand
from xbot_msgs.msg import JointState

import numpy as np

import pinocchio as pin

#########################################################

class CyclicPosTester:

    def __init__(self):

        self.xbot_cmd_pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size=10) # publish on xbotcore/command
        self.xbot_state_sub = rospy.Subscriber('/xbotcore/joint_states', JointState, self.current_q_p_assigner) # subscribe at xbotcore/joint_states

        self.urdf_path = rospy.get_param("cyclic_test/urdf_path")

        self.urdf = open(self.urdf_path, "r").read() # read the URDF
        self.pin_model = pin.buildModelFromXML(self.urdf)
        self.pin_model_data = self.pin_model.createData()
        self.n_jnts = self.pin_model.nq
        self.joint_names = []
        [self.joint_names.append((self.pin_model.names[i])) for i in range(1, self.n_jnts + 1)]


        self.PI = []
        [self.PI.append( self.pin_model.inertias[i].toDynamicParameters() ) for i in range(1, self.n_jnts + 1)] # reading inertial parameters from the provided URDF (used to compute torques)
        self.PI = np.array(self.PI).flatten()

        self.n_int_approach_traj=rospy.get_param("cyclic_test/approaching_traj/n_intervals")
        self.T_exec_approach=rospy.get_param("cyclic_test/approaching_traj/T_execution")

        self.traj_exec_standby_time=rospy.get_param("cyclic_test/approaching_traj/standby_time")
        self.q_p_target=rospy.get_param("cyclic_test/approaching_traj/q_p_target")

        self.dt=self.T_exec_approach/self.n_int_approach_traj

        self.time_vector = np.zeros(self.n_int_approach_traj + 1)
        for i in range(self.n_int_approach_traj):
            self.time_vector[i+1] = self.time_vector[i] + self.dt

        self.joint_command = JointCommand() # initializing object for holding the joint command
        self.joint_command.ctrl_mode = rospy.get_param("/cyclic_test/approaching_traj/ctrl_mode")  
        self.joint_command.name = self.joint_names  
        self.joint_command.stiffness = rospy.get_param("/cyclic_test/approaching_traj/stiffness")  
        self.joint_command.damping = rospy.get_param("/cyclic_test/approaching_traj/damping") 

        self.pub_iterator = -1 # used to slide through the solution
        self.flip_traj = False

        self.current_q_p = np.zeros((self.n_jnts, 1)).flatten() 
        self.traj_q_p = np.zeros((self.n_jnts, self.n_int_approach_traj + 1))
        self.traj_q_p_dot = np.zeros((self.n_jnts, self.n_int_approach_traj + 1))
        self.traj_q_p_ddot = np.zeros((self.n_jnts, self.n_int_approach_traj + 1))

        self.tau = np.zeros((self.n_jnts, len(self.time_vector)))

    def generate_Peisekah_trajectory(self):    

        lambd = self.time_vector/self.T_exec_approach
        
        common_term_q_p = np.transpose(np.tile( (126 * np.power(lambd, 5) - 420 * np.power(lambd, 6) + 540 * np.power(lambd, 7) - 315 * np.power(lambd, 8) + 70 * np.power(lambd, 9)), (self.n_jnts, 1) )) # replicating the common term along the joint axes
        common_term_q_p_dot = np.transpose(np.tile( ( 630 /self.T_exec_approach  * np.power((lambd - 1), 4) * np.power(lambd, 4) ), (self.n_jnts, 1) ))
        common_term_q_p_ddot = np.transpose(np.tile( ( 2520 /np.power(self.T_exec_approach, 2)  * np.power((lambd - 1), 3) * np.power(lambd, 3) * (2 * lambd - 1) ), (self.n_jnts, 1) ))

        self.traj_q_p = np.transpose(np.tile(self.current_q_p, (self.n_int_approach_traj + 1, 1)) + np.multiply(np.tile((self.q_p_target - self.current_q_p), (self.n_int_approach_traj + 1, 1)), common_term_q_p))
        self.traj_q_p_dot = np.transpose(np.multiply(np.tile((self.q_p_target - self.current_q_p), (self.n_int_approach_traj + 1, 1)), common_term_q_p_dot))
        self.traj_q_p_ddot = np.transpose(np.multiply(np.tile((self.q_p_target - self.current_q_p), (self.n_int_approach_traj + 1, 1)), common_term_q_p_ddot))

        self.assemblePinocchioRegressor()
        
        for i in range(self.n_jnts):
            self.tau[i, :] = np.dot(self.regressors[i, :, :], self.PI)

    def pack_xbot2_message(self):
            
        if self.pub_iterator < self.n_int_approach_traj: # continue sliding and publishing until the end of the solution is reached
            
            position_command = np.zeros((self.n_jnts, 1)).flatten()
            velocity_command = np.zeros((self.n_jnts, 1)).flatten()
            effort_command = np.zeros((self.n_jnts, 1)).flatten()
            
            for i in range(self.n_jnts):

                position_command[i] = self.traj_q_p[i, self.pub_iterator]
                velocity_command[i] = self.traj_q_p_dot[i, self.pub_iterator]
                effort_command[i] = self.tau[i, self.pub_iterator - 1]

            self.joint_command.position = np.ndarray.tolist(position_command) # tolist necessary, since the joint command field requires a list
            self.joint_command.velocity = np.ndarray.tolist(velocity_command)
            self.joint_command.effort = np.ndarray.tolist(effort_command)
            
            rospy.loginfo("Publishing command sample n.:\t" + str(self.pub_iterator + 1) + "," + "\n") # print a simple debug message
            rospy.loginfo("with a rate of :\t" + str(1/self.dt) + "Hz \n") # print a simple debug message

    def xbot_cmd_publisher(self):

        self.rate = rospy.Rate(1/self.dt) # potentially, a trajectory with variable dt can be provided
        
        while not rospy.is_shutdown(): 
            
            if self.pub_iterator == -1: # read the current joint position and use it as the initialization
                
                self.generate_Peisekah_trajectory()

            if self.pub_iterator > (self.n_int_approach_traj - 1): # finished playing the approach trajectory, set the topic message and shutdown node

                self.pub_iterator = 0 # reset iterator

                self.traj_q_p = np.flip(self.traj_q_p, axis = 1) # flip trajectory
                self.traj_q_p_dot = np.flip(self.traj_q_p_dot, axis = 1) # flip trajectory
                self.traj_q_p_ddot = np.flip(self.traj_q_p_ddot, axis = 1) # flip trajectory

                rospy.sleep(self.traj_exec_standby_time) # wait before going back to the initial position and starting over again

            else:

                self.pub_iterator = self.pub_iterator + 1 # incrementing publishing counter

                self.pack_xbot2_message() # copy the optimized trajectory to xbot command object 

                self.xbot_cmd_pub.publish(self.joint_command) # publish the commands

                self.rate.sleep() # wait
                
    def current_q_p_assigner(self, joints_state): # callback function which reads the joint_states and updates current_q_p

        self.current_q_p = np.array(joints_state.motor_position) # assigning the state to the global variable
    
    def assemblePinocchioRegressor(self):

        """
        Assemble the total inverse dynamics regressor for a robot.
        
        """

        n_jnts = len(self.traj_q_p[:, 0])
        n_samples = len(self.traj_q_p[0, :])
        n_inertia_params = len(self.pin_model.inertias[0].toDynamicParameters())
        self.regressors = np.zeros((n_jnts, n_samples, n_jnts * n_inertia_params))

        for i in range(n_samples):
            regressor_aux = pin.computeJointTorqueRegressor(self.pin_model, self.pin_model_data, self.traj_q_p[:, i], self.traj_q_p_dot[:, i], self.traj_q_p_ddot[:, i])
            for j in range(n_jnts):
                self.regressors[j, i, :] = regressor_aux[j, :]

    #     return regressors

    # def compute_efforts_pin(self):
        
    #     self.tau = np.zeros((self.n_jnts, 1)).flatten()

if __name__ == '__main__':

    cyclic_pos_tester = CyclicPosTester()

    rospy.init_node('cyclic_pos_tester', anonymous = False) # initialize a publisher node (not anonymous, so another node of the same type cannot run concurrently)
    
    try: # start publishing the commands

        cyclic_pos_tester.xbot_cmd_publisher()

    except rospy.ROSInterruptException:
        
        pass

