#!/usr/bin/env python3

# No dependence upon the chosen task type ---> standardized horizon solution format

# No explicit dependence upon the urdf 

####################### IMPORTS #######################

import rospy

from xbot_msgs.msg import JointCommand

from horizon.utils import mat_storer

import numpy as np

import pinocchio as pin
#########################################################

class HorizonXbotCmdPub:

    def __init__(self):

        self.xbot_cmd_pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size = 10) # publish on xbotcore/command

        self.urdf_path = rospy.get_param("horizon_xbot_cmd_pub/urdf_path")
        self.urdf = open(self.urdf_path, "r").read() # read the URDF
        self.pin_model = pin.buildModelFromXML(self.urdf)
        self.pin_model_data = self.pin_model.createData()
        self.n_jnts = self.pin_model.nq
        self.joint_names = []
        [self.joint_names.append((self.pin_model.names[i])) for i in range(1, self.n_jnts + 1)]

        self.wait_until_initial_pose = rospy.get_param("/horizon_xbot_cmd_pub/approaching_traj/wait_until_initial_pose") # used 
        self.opt_res_path = rospy.get_param("/horizon/opt_results_path")  # optimal results relative path (wrt to the package)
        self.task_type = rospy.get_param("/horizon/task_type")  # task type

        ## Based on the task_type, load the right solution from the auxiliary folder
        if self.task_type == "jump":

            self.is_adaptive_dt = rospy.get_param("horizon/horizon_solver/is_adaptive_dt")  # if true, use an adaptive dt
            self.is_single_dt = rospy.get_param("horizon/horizon_solver/is_single_dt")  # if true (and if addaptive dt is enable), use only one dt over the entire opt. horizon 

            if self.is_adaptive_dt:

                if self.is_single_dt:

                    self.n_nodes = rospy.get_param("horizon/horizon_solver/variable_dt/single_dt/problem_settings/n_nodes") # number of optimization nodes (remember to run the optimized node before, otherwise the parameter server will not be populated)
                else:

                    self.n_nodes = rospy.get_param("horizon/horizon_solver/variable_dt/multiple_dt/problem_settings/n_nodes") # number of optimization nodes (remember to run the optimized node before, otherwise the parameter server will not be populated)
            
            else:

                self.n_nodes = rospy.get_param("horizon/horizon_solver/constant_dt/problem_settings/n_nodes") # number of optimization nodes (remember to run the optimized node before, otherwise the parameter server will not be populated)

            if self.is_adaptive_dt:

                if self.is_single_dt:

                    self.ms = mat_storer.matStorer(self.opt_res_path+"/single_dt/horizon_offline_solver.mat")
                    
                else:

                    self.ms= mat_storer.matStorer(self.opt_res_path+"/multiple_dt/horizon_offline_solver.mat")
            else:

                self.ms = mat_storer.matStorer(self.opt_res_path+"/fixed_dt/horizon_offline_solver.mat")

        elif self.task_type=="trot":

            self.n_nodes = rospy.get_param("horizon/horizon_solver/problem_settings/n_nodes") # number of optimization nodes (remember to run the optimized node before, otherwise the parameter server will not be populated)
            self.ms = mat_storer.matStorer(self.opt_res_path+"/horizon_offline_solver.mat")

        ## Loading the solution dictionary, based on the selected task_type 

        self.solution = self.ms.load() 
        self.q_p = self.solution["q_p"]
        self.q_p_dot = self.solution["q_p_dot"]
        self.q_p_ddot = self.solution["q_p_ddot"]
        self.tau = self.solution["tau"]
        self.f_contact = self.solution["f_contact"]
        self.solution_time = self.solution["sol_time"]
        self.dt = self.solution["dt_opt"].flatten() 

        self.n_samples = len(self.tau[0, :])
        self.n_jnts = len(self.q_p[:, 0])

        self.time_vector = np.zeros(self.dt.size+1) # time vector used by the trajectory publisher
        for i in range(self.dt.size):
            self.time_vector[i+1] = self.time_vector[i] + self.dt[i]

        self.joint_command = JointCommand() # initializing object for holding the joint command
        self.joint_command.ctrl_mode = rospy.get_param("/horizon_xbot_cmd_pub/horizon_trajectory/ctrl_mode")  
        self.joint_command.name = self.joint_names
        self.joint_command.stiffness = rospy.get_param("/horizon_xbot_cmd_pub/horizon_trajectory/stiffness")  
        self.joint_command.damping = rospy.get_param("/horizon_xbot_cmd_pub/horizon_trajectory/damping")  

        self.pub_iterator = 0 # used to slide through the solution
    
    def pack_xbot2_message(self):
            
        if self.pub_iterator < self.n_samples: # continue sliding and publishing until the end of the solution is reached
            
            position_command = np.zeros((self.n_jnts, 1)).flatten()
            velocity_command = np.zeros((self.n_jnts, 1)).flatten()
            effort_command = np.zeros((self.n_jnts, 1)).flatten()

            for i in range(self.n_jnts):

                position_command[i] = self.q_p[i, self.pub_iterator]
                velocity_command[i] = self.q_p_dot[i, self.pub_iterator]
                effort_command[i] = self.tau[i, self.pub_iterator - 1]

            self.joint_command.position = np.ndarray.tolist(position_command) # tolist necessary, since the joint command field requires a list
            self.joint_command.velocity = np.ndarray.tolist(velocity_command)
            self.joint_command.effort = np.ndarray.tolist(effort_command)  
            
            print("Publishing command sample n.:\t" + str(self.pub_iterator + 1) + "," + "\n") # print a simple debug message
            print("with a rate of :\t" + str(1/self.dt[self.pub_iterator]) + "Hz \n") # print a simple debug message

    def xbot_cmd_publisher(self):

        while not rospy.is_shutdown(): 

            if self.wait_until_initial_pose:

                self.is_initial_pose_reached = rospy.get_param("/horizon/xbot_command_pub/approaching_traj/is_initial_pose_reached") 
        
                if self.is_initial_pose_reached:

                    if self.pub_iterator > (self.n_nodes-1):

                        self.pub_iterator = 0 # replaying trajectory after end
                        # rospy.sleep(2) # sleep between replayed trajectories

                    self.pub_iterator = self.pub_iterator+1 # incrementing publishing counter
                    self.rate = rospy.Rate(1/self.dt[self.pub_iterator-1]) # potentially, a trajectory with variable dt can be provided
                    self.pack_xbot2_message() # copy the optimized trajectory to xbot command object 
                    self.xbot_cmd_pub.publish(self.joint_command) # publish the commands
                    self.rate.sleep() # wait
            else:

                if self.pub_iterator > (self.n_nodes-1):

                        self.pub_iterator = 0 # replaying trajectory after end
                        # rospy.sleep(2) # sleep between replayed trajectories

                self.pub_iterator = self.pub_iterator + 1 # incrementing publishing counter
                self.rate = rospy.Rate(1/self.dt[self.pub_iterator-1]) # potentially, a trajectory with variable dt can be provided
                self.pack_xbot2_message() # copy the optimized trajectory to xbot command object 
                self.xbot_cmd_pub.publish(self.joint_command) # publish the commands
                self.rate.sleep() # wait

if __name__ == '__main__':

    xbot_pub = HorizonXbotCmdPub()

    rospy.init_node('horizon_xbot_pub', anonymous = False) # initialize a publisher node (not anonymous, so another node of the same type cannot run concurrently)
    
    try: # start publishing the commands

        xbot_pub.xbot_cmd_publisher()

    except rospy.ROSInterruptException:
        
        pass

