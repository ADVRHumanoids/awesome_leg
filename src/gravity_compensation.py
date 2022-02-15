#!/usr/bin/env python3

# No dependence upon the chosen task type ---> standardized horizon solution format

# No explicit dependence upon the urdf 

####################### IMPORTS #######################

import rospy

from xbot_msgs.msg import JointCommand
from xbot_msgs.msg import JointState

import numpy as np

import pinocchio as pin

#########################################################

class GravityCompensator:

    def __init__(self):

        self.xbot_cmd_pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size = 10) # publish on xbotcore/command
        self.xbot_state_sub = rospy.Subscriber('/xbotcore/joint_states', JointState, self.current_q_p_assigner) # subscribe at xbotcore/joint_states

        self.urdf_path = rospy.get_param("gravity_compensator/urdf_path")
        self.urdf = open(self.urdf_path, "r").read() # read the URDF
        self.pin_model = pin.buildModelFromXML(self.urdf)
        self.pin_model_data = self.pin_model.createData()
        self.n_jnts = self.pin_model.nq
        self.joint_names = []
        [self.joint_names.append((self.pin_model.names[i])) for i in range(1, self.n_jnts + 1)]
        
        self.dt = rospy.get_param("/gravity_compensator/dt") 

        self.PI = rospy.get_param("/gravity_compensator/PI") 
        self.torque_bias = rospy.get_param("/gravity_compensator/torque_bias") 

        self.joint_command = JointCommand() # initializing object for holding the joint command
        self.joint_command.ctrl_mode = rospy.get_param("/gravity_compensator/ctrl_mode") 
        self.joint_command.stiffness = rospy.get_param("/gravity_compensator/stiffness")  
        self.joint_command.damping = rospy.get_param("/gravity_compensator/damping") 
        self.joint_command.name = self.joint_names

        self.current_q_p = np.zeros((self.n_jnts, 1)).flatten() # currently measured joint positions (to be used by "horizon_initial_pose_pub")

        self.tau = np.zeros((self.n_jnts, 1)).flatten()

    def pack_xbot2_message(self):

        self.joint_command.effort = np.ndarray.tolist(self.tau)

        rospy.loginfo("Publishing gravity compensation torques with rate: "+ str(1/self.dt) +"Hz"+"\n") # print a simple debug message
        rospy.loginfo("Published torque: " + str(self.tau))

    def xbot_cmd_publisher(self):
        
        self.rate = rospy.Rate(1/self.dt)

        while not rospy.is_shutdown():
            
            self.compute_efforts_pin()

            self.pack_xbot2_message() # copy the optimized trajectory to xbot command object 

            self.xbot_cmd_pub.publish(self.joint_command) # publish the commands

            self.rate.sleep() # wait

    def current_q_p_assigner(self, joints_state): # callback function which reads the joint_states and updates current_q_p

        self.current_q_p = np.array(joints_state.motor_position) # assigning the state to the global variable

    def compute_efforts_pin(self):

        self.tau = np.dot(pin.computeJointTorqueRegressor(self.pin_model, self.pin_model_data, self.current_q_p, np.zeros((self.n_jnts, 1)), np.zeros((self.n_jnts, 1))), self.PI) + self.torque_bias 

if __name__ == '__main__':

    gravity_compensator = GravityCompensator()

    rospy.init_node('gravity_compensator', anonymous = False) # initialize a publisher node (not anonymous, so another node of the same type cannot run concurrently)
    try: # start publishing the commands

        gravity_compensator.xbot_cmd_publisher()

    except rospy.ROSInterruptException:
        
        pass

