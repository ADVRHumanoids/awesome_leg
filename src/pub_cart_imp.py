#!/usr/bin/python3 

### A minimal node for exploiting the set_impedance service ###

import rospy
from cartesian_interface.srv import SetImpedance, SetImpedanceRequest

rospy.init_node("impedance_setter")

# ns = "/cartesian"
ns = "/cartesio_ell_rt"

set_imp = rospy.ServiceProxy(ns + "/tip/set_impedance", SetImpedance)

set_imp.wait_for_service()

req = SetImpedanceRequest()

req.impedance.linear.stiffness.x = 1000
req.impedance.linear.stiffness.z = 1000
req.impedance.linear.damping_ratio.x = 50
req.impedance.linear.damping_ratio.z = 50

res = set_imp(req)

print(res)