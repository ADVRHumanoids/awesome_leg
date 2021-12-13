#!/usr/bin/env python3

###############################################

import rospy
import rospkg

from xbot_msgs.msg import JointCommand

from horizon.utils import mat_storer

###############################################

rospackage=rospkg.RosPack()
ms = mat_storer.matStorer(rospackage.get_path("awesome_leg_pholus")+"/sim_results/horizon_offline_solver.mat")
solution=ms.load() # loading the solution dictionary

q_p=solution["q_p"]
q_p_dot=solution["q_p_dot"]
q_p_ddot=solution["q_p_ddot"]
tau=solution["tau"]
f_contact=solution["f_contact"]
solution_time=solution["sol_time"]

dt = rospy.get_param("/horizon/problem/dt") # optimization dt [s]
n_nodes = rospy.get_param("/horizon/problem/n_nodes") # number of optimization nodes (remember to run the optimized node before, otherwise the parameter server will not be populated)
hip_cntrl_mode = rospy.get_param("/horizon/joints/hip_joint/control_mode") # control mode (position, velocity, effort)
knee_cntrl_mode = rospy.get_param("/horizon/joints/knee_joint/control_mode") # control mode (position, velocity, effort)
hip_joint_stffnss = rospy.get_param("/horizon/joints/hip_joint/stiffness") # hip joint stiffness setting (to be used by Xbot)
knee_joint_stffnss = rospy.get_param("/horizon/joints/knee_joint/stiffness") # knee joint stiffness setting (to be used by Xbot)
hip_joint_damp = rospy.get_param("/horizon/joints/hip_joint/damping") # hip joint damping setting (to be used by Xbot)
knee_joint_damp = rospy.get_param("/horizon/joints/knee_joint/damping") # knee joint damping setting (to be used by Xbot)

joint_command=JointCommand() # initializing object for holding the joint command
joint_command.stiffness=[hip_joint_stffnss,knee_joint_stffnss]
joint_command.damping=[hip_joint_damp,knee_joint_damp]

ctrl_mode=[1,1] # default is position control

if hip_cntrl_mode=="position":
    ctrl_mode[0]=1
elif hip_cntrl_mode=="velocity":
    ctrl_mode[0]=2
elif hip_cntrl_mode=="effort":
    ctrl_mode[0]=4
else: raise ValueError("\nInvalid hip control mode. Please check your configuration file.\n Allowed control modes: \"position\",\t\"velocity\",\t\"effort\"\n")

if knee_cntrl_mode=="position":
    ctrl_mode[1]=1
elif knee_cntrl_mode=="velocity":
    ctrl_mode[1]=2
elif knee_cntrl_mode=="effort":
    ctrl_mode=4
else: raise ValueError("\nInvalid knee control mode. Please check your configuration file.\n Allowed control modes: \"position\",\t\"velocity\",\t\"effort\"\n")

joint_command.ctrl_mode=ctrl_mode

pub_iterator=0 # used to slide thorugh the solution


###############################################
def pack_xbot2_message():
    if pub_iterator<=(n_nodes-1): # continue sliding and publishing until the end of the solution is reached
        joint_command.name=["hip_pitch_1","knee_pitch_1"]
        joint_command.position=[q_p[0,pub_iterator],q_p[1,pub_iterator]]
        joint_command.velocity=[q_p_dot[0,pub_iterator],q_p_dot[1,pub_iterator]]
        joint_command.effort=[tau[0,pub_iterator-1],tau[1,pub_iterator-1]]
        print(pub_iterator)

def horizon_pub():
    global pub_iterator
    pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size=10)
    rospy.init_node('horizon_publisher', anonymous=True)

    rate = rospy.Rate(1/dt) 

    while not rospy.is_shutdown():
        if pub_iterator>(n_nodes-1): pub_iterator=0 # replaying trajectory after end
        pub_iterator=pub_iterator+1 # incrementing publishing counter
        pack_xbot2_message()
        pub.publish(joint_command)  
        rate.sleep()

if __name__ == '__main__':
    try:
        horizon_pub()
    except rospy.ROSInterruptException:
        pass
