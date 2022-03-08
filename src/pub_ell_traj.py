#!/usr/bin/python3 

### A minimal node for exploiting the set_impedance service ###

import rospy

from awesome_leg_pholus.srv import EllTrajRt, EllTrajRtRequest

rospy.init_node("ell_traj_setter")

set_ell_traj = rospy.ServiceProxy("/ell_traj_rt/my_ell_traj_srvc", EllTrajRt)

set_ell_traj.wait_for_service()

req = EllTrajRtRequest()

req.t_exec = 0.5
req.x_c = - 0.15
req.z_c = - 0.6
req.a_ellps = 0.2
req.b_ellps = 0.05
req.alpha = 0.0 
req.use_vel_ff = True
req.use_acc_ff = True
req.is_forward = True
res = set_ell_traj(req)

print(res)
