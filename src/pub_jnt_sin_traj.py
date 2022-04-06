#!/usr/bin/python3 

### A minimal node for exploiting the my_ell_traj_srvc service ###

import rospy

from awesome_leg_pholus.srv import SinJointTraj, SinJointTrajRequest

rospy.init_node("jnt_sin_traj_setter")

set_ell_traj = rospy.ServiceProxy("/bypass_dsp_rt/my_sin_jnt_traj_srvc", SinJointTraj)

set_ell_traj.wait_for_service()

req = SinJointTrajRequest()

req.t_exec = [4.0, 4.0]
req.center = [0.0, 0.0]
req.phase_off = [0.0, 0]
req.overshoot = [1, 1]

res = set_ell_traj(req)

print(res)
