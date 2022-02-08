#!/usr/bin/env python3

# No dependence upon the chosen task type ---> standardized horizon solution format

# No explicit dependence upon the urdf 

####################### IMPORTS #######################

import rospy

from xbot_msgs.msg import JointCommand
from xbot_msgs.msg import JointState

from horizon.utils import mat_storer

import numpy as np

import pinocchio as pin

#########################################################

class HorizonXbotInitPosPub:

    def __init__(self):

        rospy.set_param("/horizon/xbot_command_pub/approaching_traj/is_initial_pose_reached", False) # used to tell other nodes if the approach procedure has finished

        self.xbot_cmd_pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size = 10) # publish on xbotcore/command
        self.xbot_state_sub = rospy.Subscriber('/xbotcore/joint_states', JointState, self.current_q_p_assigner) # subscribe at xbotcore/joint_states

        self.urdf_path = rospy.get_param("horizon_xbot_cmd_pub/urdf_path")
        self.urdf = open(self.urdf_path, "r").read() # read the URDF
        self.pin_model = pin.buildModelFromXML(self.urdf)
        self.pin_model_data = self.pin_model.createData()
        self.n_jnts = self.pin_model.nq
        self.joint_names = []
        [self.joint_names.append((self.pin_model.names[i])) for i in range(1, self.n_jnts + 1)]

        self.n_int_approach_traj=rospy.get_param("/horizon/xbot_command_pub/approaching_traj/n_intervals")
        self.T_exec_approach=rospy.get_param("/horizon/xbot_command_pub/approaching_traj/T_execution")
        self.traj_exec_standby_time=rospy.get_param("/horizon/xbot_command_pub/approaching_traj/standby_time")

        ## Paths parameters
        self.opt_res_path = rospy.get_param("horizon/opt_results_path")  # optimal results relative path (wrt to the package)
        self.task_type = rospy.get_param("/horizon/task_type")  # task type

        ## Based on the task_type, load the right solution from the auxiliary folder
        if self.task_type == "jump":

            self.is_adaptive_dt = rospy.get_param("horizon/horizon_solver/is_adaptive_dt")  # if true, use an adaptive dt
            self.is_single_dt = rospy.get_param("horizon/horizon_solver/is_single_dt")  # if true (and if addaptive dt is enable), use only one dt over the entire opt. horizon 

            if self.is_adaptive_dt:
                if self.is_single_dt:
                    ms = mat_storer.matStorer(self.opt_res_path+"/single_dt/horizon_offline_solver.mat")
                else:
                    ms = mat_storer.matStorer(self.opt_res_path+"/multiple_dt/horizon_offline_solver.mat")
            else:
                ms = mat_storer.matStorer(self.opt_res_path+"/fixed_dt/horizon_offline_solver.mat")

        elif self.task_type=="trot":

            ms = mat_storer.matStorer(self.opt_res_path+"/horizon_offline_solver.mat")
            
        ## Loading the solution dictionary, based on the selected task_type 
        self.solution = ms.load() 

        self.q_p = self.solution["q_p"] # only used to extract the first traj. sample
        self.dt = self.T_exec_approach/self.n_int_approach_traj

        self.time_vector = np.zeros(self.n_int_approach_traj + 1)
        for i in range(self.n_int_approach_traj):
            self.time_vector[i+1] = self.time_vector[i] + self.dt

        self.joint_command = JointCommand() # initializing object for holding the joint command
        self.joint_command.ctrl_mode = rospy.get_param("/horizon_xbot_cmd_pub/horizon_trajectory/ctrl_mode")  
        self.joint_command.name = self.joint_names 
        self.joint_command.stiffness = rospy.get_param("/horizon_xbot_cmd_pub/horizon_trajectory/stiffness")  
        self.joint_command.damping = rospy.get_param("/horizon_xbot_cmd_pub/horizon_trajectory/damping")  

        self.pub_iterator = 0 # used to slide through the solution

        self.q_p_init_target = self.q_p[:, 0]
        self.current_q_p = np.zeros((self.n_jnts, 1)).flatten() 
    
    def generate_Peisekah_trajectory(self):    

        common_part_traj= (126 * np.power(self.time_vector/self.T_exec_approach, 5) - 420 * np.power(self.time_vector/self.T_exec_approach, 6) + 540 * np.power(self.time_vector/self.T_exec_approach, 7) - 315 * np.power(self.time_vector/self.T_exec_approach, 8) + 70 * np.power(self.time_vector/self.T_exec_approach, 9))
        
        common_part_traj = np.transpose(np.tile(common_part_traj, (2, 1) ))

        self.q_p_approach_traj = np.tile(self.current_q_p, (self.n_int_approach_traj + 1, 1)) + np.multiply(np.tile((self.q_p_init_target - self.current_q_p),(self.n_int_approach_traj + 1, 1)), common_part_traj)

    def pack_xbot2_message(self):
            
        if self.pub_iterator <= (self.n_int_approach_traj - 1): # continue sliding and publishing until the end of the solution is reached
            
            position_command = np.zeros((self.n_jnts, 1)).flatten()

            for i in range(self.n_jnts):

                position_command[i] = self.q_p_approach_traj[self.pub_iterator, i]

            self.joint_command.position = np.ndarray.tolist(position_command)

            print("Publishing initialization trajectory sample n.:\t" + str(self.pub_iterator + 1) + "," + "\n") # print a simple debug message
            print("with a rate of :\t" + str(1/self.dt) + "Hz \n") # print a simple debug message
            
    def xbot_cmd_publisher(self):

        while not rospy.is_shutdown(): 

            if self.pub_iterator == 0: # read the current joint trajectory and use it as the initialization
                
                self.generate_Peisekah_trajectory()

            if self.pub_iterator > (self.n_int_approach_traj - 1): # finished playing the approach trajectory, set the topic message and shutdown node
                
                rospy.set_param("horizon/xbot_command_pub/approaching_traj/is_initial_pose_reached", True)
                
                rospy.sleep(self.traj_exec_standby_time) # wait before sending the trajectory
                
                rospy.signal_shutdown("Finished publishing approaching trajectory")

            self.pub_iterator = self.pub_iterator + 1 # incrementing publishing counter

            self.rate = rospy.Rate(1/self.dt) # potentially, a trajectory with variable dt can be provided

            self.pack_xbot2_message() # copy the optimized trajectory to xbot command object 

            self.xbot_cmd_pub.publish(self.joint_command) # publish the commands

            self.rate.sleep() # wait
    
    def current_q_p_assigner(self, joints_state): # callback function which reads the joint_states and updates current_q_p

        self.current_q_p = np.array(joints_state.motor_position) # assigning the state to the global variable
    

if __name__ == '__main__':

    horizon_xbot_init_pos_pub = HorizonXbotInitPosPub()

    rospy.init_node('horizon_xbot_init_pos_pub', anonymous = False) # initialize a publisher node (not anonymous, so another node of the same type cannot run concurrently)
    
    try: # start publishing the commands

        horizon_xbot_init_pos_pub.xbot_cmd_publisher()

    except rospy.ROSInterruptException:
        
        pass

