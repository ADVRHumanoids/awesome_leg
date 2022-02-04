#!/usr/bin/env python3
####################### IMPORTS #######################
import rospy

import rospkg

from xbot_msgs.msg import JointCommand
from xbot_msgs.msg import JointState

import numpy as np

from xbot_interface import xbot_interface as xbot

from awesome_leg_pholus.param_identification_utilities import get_xbot_cfg

import pinocchio

####################### SETTING PARAMETERS ON THE ROS PARAMETER SERVER #######################

dt = rospy.get_param("/gravity_compensator/dt") 
urdf_path = rospy.get_param("/gravity_compensator/urdf_path") 
PI = rospy.get_param("/gravity_compensator/PI") 
torque_bias = rospy.get_param("/gravity_compensator/torque_bias") 

# srdf_path = rospy.get_param("/gravity_compensator/srdf_path") 

joint_command=JointCommand() # initializing object for holding the joint command
ctrl_mode=[4, 4] # for gravity compensation, use a simple torque control mode 

joint_command.ctrl_mode=ctrl_mode
joint_command.name=["hip_pitch_1","knee_pitch_1"]

# Model description
urdf = open(urdf_path, "r").read() # read the URDF
# srdf = open(srdf_path, "r").read() # read the URDF

# XBot model interface & Co
# robot_cfg = get_xbot_cfg(urdf, srdf)
# model = xbot.ModelInterface(robot_cfg)

model = pinocchio.buildModelFromXML(urdf)
data = model.createData()

current_q_p=[0, 0] # currently measured joint positions (to be used by "horizon_initial_pose_pub")

####################### FUNCTIONS #######################

def compute_grav_comp_efforts_xbot2(model, q_p, q_p_dot, q_p_ddot):

    n_joints = len(q_p)
    tau = np.zeros((n_joints, 1))

    model.setJointPosition(q_p)
    model.setJointVelocity(q_p_dot)
    model.setJointAcceleration(q_p_ddot)
    model.update()

    tau = model.computeInverseDynamics()
    
    return tau

def compute_efforts_pin(model, data, q_p, q_p_dot, q_p_ddot, PI, torque_bias):

    tau = np.dot(pinocchio.computeJointTorqueRegressor(model, data, q_p, q_p_dot, q_p_ddot), PI) + torque_bias 
    print(tau)
    return tau

def pack_xbot2_message(tau):
            
    joint_command.effort=[tau[0], tau[1]]

    print("Publishing gravity compensation torques with rate: "+ str(1/dt) +"Hz"+"\n") # print a simple debug message
    print("Published torque: ", tau)
def xbot_cmd_publisher():

    global current_q_p # using the corresponding variable initialized in the global scope
    
    xbot_cmd_pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size = 10) # publish on xbotcore/command
    rospy.Subscriber('/xbotcore/joint_states', JointState, current_q_p_assigner) # subscribe at xbotcore/joint_states

    rospy.init_node('horizon_xbot_publisher', anonymous = False) # initialize a publisher node (not anonymous, so another node of the same type cannot run concurrently)

    while not rospy.is_shutdown(): 

        rate = rospy.Rate(1/dt)

        tau = compute_efforts_pin(model, data, np.array(current_q_p), np.array([0, 0]), np.array([0, 0]), PI, torque_bias)

        pack_xbot2_message(tau) # copy the optimized trajectory to xbot command object 

        xbot_cmd_pub.publish(joint_command) # publish the commands

        rate.sleep() # wait

def current_q_p_assigner(joints_state): # callback function which reads the joint_states and updates current_q_p

    global current_q_p 

    current_q_p = joints_state.motor_position # assigning the state to the global variable
    
###############################################

if __name__ == '__main__':

    try:
        xbot_cmd_publisher()

    except rospy.ROSInterruptException:
        pass
