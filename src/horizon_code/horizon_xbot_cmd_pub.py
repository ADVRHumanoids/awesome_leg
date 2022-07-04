#!/usr/bin/env python3

# No dependence upon the chosen task type ---> standardized horizon solution format

# No explicit dependence upon the urdf 

####################### IMPORTS #######################

import rospy

from xbot_msgs.msg import JointCommand

from horizon.utils import mat_storer

import numpy as np

import pinocchio as pin

from scipy import interpolate

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

        self.wait_until_initial_pose = rospy.get_param("/horizon_xbot_cmd_pub/approaching_traj/wait_until_initial_pose")
        self.sample_dt =  abs(rospy.get_param("/horizon_xbot_cmd_pub/horizon_trajectory/sample_dt"))
        self.traj_pause_time =  abs(rospy.get_param("/horizon_xbot_cmd_pub/horizon_trajectory/traj_pause_time"))

        self.opt_res_path = rospy.get_param("/horizon/opt_results_path")  # optimal results relative path (wrt to the package)
        self.task_type = rospy.get_param("/horizon/task_type")  # task type

        ## Based on the task_type, load the right solution from the auxiliary folder
        if self.task_type == "jump":
            
            self.res_sol_mat_name = rospy.get_param("horizon/horizon_solver/res_sol_mat_name")

            self.ms = mat_storer.matStorer(self.opt_res_path + "/jump_test/" + self.res_sol_mat_name + ".mat")

            # In the case of jump, the position, vel and acc solution also holds the test rig d.o.f. --> this has to be removed
            self.solution = self.ms.load() 
            self.q_p = self.solution["q_p"][1:3,:]
            self.q_p_dot = self.solution["q_p_dot"][1:3,:]
            self.tau = self.solution["tau"][1:3,:]
            self.opt_dt = self.solution["dt_opt"].flatten() 
                            
        elif self.task_type=="trot":

            self.ms = mat_storer.matStorer(self.opt_res_path+"/horizon_offline_solver.mat")

            self.solution = self.ms.load() 
            self.q_p = self.solution["q_p"]
            self.q_p_dot = self.solution["q_p_dot"]
            self.tau = self.solution["tau"]
            self.opt_dt = self.solution["dt_opt"].flatten() 

        ## Loading the solution dictionary, based on the selected task_type 

        self.n_samples = len(self.tau[0, :]) # length of the input vector and not the state vector (these way samples are aligned)
        self.n_jnts = len(self.q_p[:, 0])

        self.joint_command = JointCommand() # initializing object for holding the joint command
        self.joint_command.ctrl_mode = rospy.get_param("/horizon_xbot_cmd_pub/horizon_trajectory/ctrl_mode")  
        self.joint_command.name = self.joint_names
        self.joint_command.stiffness = rospy.get_param("/horizon_xbot_cmd_pub/horizon_trajectory/stiffness")  
        self.joint_command.damping = rospy.get_param("/horizon_xbot_cmd_pub/horizon_trajectory/damping")  

        self.pub_iterator = 0 # used to slide through the solution

        self.adapt_traj2des_rate() 
    
    def adapt_traj2des_rate(self):

        if self.opt_dt[0] > self.sample_dt + 0.0000001:

            raise Exception("Sorry, you have set a publish dt ( " + str(self.sample_dt) + " ) lower than the one of the loaded trajectory ( " + str(self.opt_dt[0]) + ").")             
        
        elif self.opt_dt[0] < self.sample_dt - 0.0000001:

            t_exec = self.opt_dt[0] * (self.n_samples - 1)

            time_vector = np.zeros(self.n_samples)
            for i in range(self.n_samples - 1):
                time_vector[i + 1] = time_vector[i] + self.opt_dt[0]

            self.n_samples_res = round(t_exec / self.sample_dt)
            time_vector_res = np.zeros(self.n_samples_res)
            for i in range(self.n_samples_res - 1):
                time_vector_res[i + 1] = time_vector_res[i] + self.sample_dt

            f_q_p = interpolate.interp1d(time_vector, self.q_p[:, 1:])
            f_q_p_dot = interpolate.interp1d(time_vector, self.q_p_dot[:, 1:])
            f_tau = interpolate.interp1d(time_vector, self.tau)

            self.q_p = f_q_p(time_vector_res)
            self.q_p_dot = f_q_p_dot(time_vector_res)
            self.tau = f_tau(time_vector_res)

            self.n_samples = self.n_samples_res

        else:

            self.sample_dt = self.opt_dt[0]

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
            
            rospy.loginfo("Publishing command sample n.:\t" + str(self.pub_iterator + 1) + "," + "\n") # print a simple debug message
            rospy.loginfo("with a rate of :\t" + str(1/self.sample_dt) + "Hz \n") # print a simple debug message

    def xbot_cmd_publisher(self):

        self.rate = rospy.Rate(1/self.sample_dt) # here, assuming a constant dt

        while not rospy.is_shutdown(): 

            if self.wait_until_initial_pose:

                self.is_initial_pose_reached = rospy.get_param("horizon_xbot_cmd_pub/approaching_traj/is_initial_pose_reached") 
        
                if self.is_initial_pose_reached:

                    if self.pub_iterator > (self.n_samples - 1):

                        self.pub_iterator = 0 # replaying trajectory after end
                        rospy.sleep(self.traj_pause_time) # sleep between replayed trajectories

                    self.pub_iterator = self.pub_iterator + 1 # incrementing publishing counter
                    self.pack_xbot2_message() # copy the optimized trajectory to xbot command object 
                    self.xbot_cmd_pub.publish(self.joint_command) # publish the commands
                    self.rate.sleep() # wait
            else:

                if self.pub_iterator > (self.n_samples - 1):

                        self.pub_iterator = 0 # replaying trajectory after end
                        rospy.sleep(self.traj_pause_time) # sleep between replayed trajectories

                self.pub_iterator = self.pub_iterator + 1 # incrementing publishing counter
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

