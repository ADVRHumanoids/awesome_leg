#!/usr/bin/python3 

### A minimal node for exploiting the my_ell_traj_srvc service ###

import rospy

from awesome_leg.srv import JumpNow, JumpNowRequest

rospy.init_node("jump_now_srv_node")

jump = rospy.ServiceProxy("/mat_replayer_rt/my_jump_now", JumpNow)

jump.wait_for_service()

req = JumpNowRequest()

req.jump_now = True

res = jump(req)

print(res)