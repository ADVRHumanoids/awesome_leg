#!/usr/bin/python3 

import rospy
from cartesian_interface.srv import SetImpedance, SetImpedanceRequest

rospy.init_node("impedance_setter")

set_imp = rospy.ServiceProxy("/cartesian/tip/set_impedance", SetImpedance)
print('waiting for srv..')
set_imp.wait_for_service()

req = SetImpedanceRequest()

req.impedance.linear.stiffness.x = 200
req.impedance.linear.stiffness.z = 200
req.impedance.linear.damping_ratio.x = 10
req.impedance.linear.damping_ratio.z = 10 

res = set_imp(req)

print(res)