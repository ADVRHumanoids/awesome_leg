#!/usr/bin/python3 

import rospy

from awesome_leg_pholus.srv import BypassDspRt, BypassDspRtRequest

rospy.init_node("bypass_dsp_impedance_setter")

set_jnt_impedance = rospy.ServiceProxy("/bypass_dsp_rt/bypass_dsp_srvc", BypassDspRt)

set_jnt_impedance.wait_for_service()

req = BypassDspRtRequest()

req.jnt_stiffness_setpoint = [100.0, 100.0]
req.jnt_damping_setpoint = [10, 10]

res = set_jnt_impedance(req)

print(res)
