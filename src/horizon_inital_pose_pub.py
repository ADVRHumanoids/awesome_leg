#!/usr/bin/env python3

###############################################
import rospy
import rospkg

from xbot_msgs.msg import JointCommand
# from xbot_interface import xbot_interface as xbot

from horizon.utils import mat_storer

import numpy as np

# import warnings

###############################################
rospackage=rospkg.RosPack()

opt_res_rel_path = rospy.get_param("/horizon/opt_results_rel_path")  # optimal results relative path (wrt to the package)

task_type = rospy.get_param("/horizon/task_type")  # task type

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

    n_nodes = rospy.get_param("horizon/horizon_solver/variable_dt/problem_settings/n_nodes") # number of optimization nodes (remember to run the optimized node before, otherwise the parameter server will not be populated)
    ms = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/"+opt_res_rel_path+"/horizon_offline_solver.mat")


solution=ms.load() # loading the solution dictionary

q_p=solution["q_p"]
q_p_dot=solution["q_p_dot"]
q_p_ddot=solution["q_p_ddot"]
tau=solution["tau"]
f_contact=solution["f_contact"]
solution_time=solution["sol_time"]
dt=solution["dt_opt"].flatten() 

time_vector = np.zeros(dt.size+1)
for i in range(dt.size):
    time_vector[i+1] = time_vector[i] + dt[i]

hip_cntrl_mode = rospy.get_param("/horizon/joint_cntrl/hip_joint/control_mode") # hip control mode (bitmask, p-v-e-k-d->[1,2,4,8,16], where p is the LSB)
knee_cntrl_mode = rospy.get_param("/horizon/joint_cntrl/knee_joint/control_mode") # knee control mode (bitmask, p-v-e-k-d->[1,2,4,8,16], where p is the LSB)
hip_joint_stffnss = rospy.get_param("/horizon/joint_cntrl/hip_joint/stiffness") # hip joint stiffness setting (to be used by Xbot, if impedance cntrl is enabled)
knee_joint_stffnss = rospy.get_param("/horizon/joint_cntrl/knee_joint/stiffness") # knee joint stiffness setting (to be used by Xbot, if impedance cntrl is enabled)
hip_joint_damp = rospy.get_param("/horizon/joint_cntrl/hip_joint/damping") # hip joint damping setting (to be used by Xbot, if impedance cntrl is enabled)
knee_joint_damp = rospy.get_param("/horizon/joint_cntrl/knee_joint/damping") # knee joint damping setting (to be used by Xbot, if impedance cntrl is enabled)

joint_command=JointCommand() # initializing object for holding the joint command
joint_command.stiffness=[hip_joint_stffnss,knee_joint_stffnss] # if impedance cntrl is not enabled, xbot simply ignores this setting
joint_command.damping=[hip_joint_damp,knee_joint_damp] # if impedance cntrl is not enabled, xbot simply ignores this setting

ctrl_mode=[hip_cntrl_mode, knee_cntrl_mode] # cntrl mode vector

joint_command.ctrl_mode=ctrl_mode
joint_command.name=["hip_pitch_1","knee_pitch_1"]

pub_iterator=0 # used to slide through the solution
pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size=10) # publish on xbotcore/command
rospy.init_node('horizon_publisher', anonymous=True) # initialize a publisher node (anonymous, so potentially another node of the same type can run concurrently)

###############################################

def pack_xbot2_message():
    if pub_iterator<=(n_nodes-1): # continue sliding and publishing until the end of the solution is reached
        #
        joint_command.position=[q_p[0,pub_iterator],q_p[1,pub_iterator]]
        joint_command.velocity=[q_p_dot[0,pub_iterator],q_p_dot[1,pub_iterator]]
        joint_command.effort=[tau[0,pub_iterator-1],tau[1,pub_iterator-1]]

        print("Publishing trajectory sample n.:\t"+str(pub_iterator)+","+"\n") # print a simple debug message
        print("with publish rate:\t"+str(1/dt[pub_iterator-1])+"\n") # print a simple debug message

def horizon_initial_pose_pub(): # before sending the trajectory, first move smoothly the joints to the first trajectory position reference
    # not possible to do this with velocity or torque references for obvious reasons

    while not rospy.is_shutdown(): 
        if pub_iterator>(n_nodes-1):
            pub_iterator=0 # replaying trajectory after end
            # rospy.sleep(2) # sleep between replayed trajectories
        pub_iterator=pub_iterator+1 # incrementing publishing counter

        rate = rospy.Rate(1/dt[pub_iterator-1]) # potentially, a trajectory with variable dt can be provided

        pack_xbot2_message() # copy trajectory to xbot command object 

        pub.publish(joint_command) # publish the commands

        rate.sleep() # wait

def horizon_pub():

    while not rospy.is_shutdown(): 
        global pub_iterator # using the variable initialized in the global scope
        if pub_iterator>(n_nodes-1):
            pub_iterator=0 # replaying trajectory after end
            # rospy.sleep(2) # sleep between replayed trajectories
        pub_iterator=pub_iterator+1 # incrementing publishing counter

        rate = rospy.Rate(1/dt[pub_iterator-1]) # potentially, a trajectory with variable dt can be provided

        pack_xbot2_message() # copy trajectory to xbot command object 

        pub.publish(joint_command) # publish the commands

        rate.sleep() # wait

###############################################

if __name__ == '__main__':
    try:
        horizon_pub()
    except rospy.ROSInterruptException:
        pass
