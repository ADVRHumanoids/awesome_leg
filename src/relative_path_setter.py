#!/usr/bin/env python3


import subprocess

import rospy


while (rospy.get_param("/horizon/urdf_path",default=False)==False) or \
      (rospy.get_param("/horizon/media_path",default=False)==False) or \
      (rospy.get_param("/horizon/opt_results_path",default=False)==False):
    # simply wait until all the parameters are set
    continue

# This point is reached only if all the parameters were loaded
urdf_path_raw=rospy.get_param("/horizon/urdf_path")
media_path_raw=rospy.get_param("/horizon/media_path")
opt_results_path_raw=rospy.get_param("/horizon/opt_results_path")

urdf_path=(subprocess.check_output(['set -eu; echo '+urdf_path_raw], shell=True)).decode().strip()
media_path=(subprocess.check_output(['set -eu; echo '+media_path_raw], shell=True)).decode().strip()
opt_results_path=(subprocess.check_output(['set -eu; echo '+opt_results_path_raw], shell=True)).decode().strip()

# Re-set the parameters
rospy.set_param("/horizon/urdf_path",urdf_path)
rospy.set_param("/horizon/media_path",media_path)
rospy.set_param("/horizon/opt_results_path",opt_results_path)

rospy.set_param("/horizon/are_relative_path_set",True)

print(urdf_path)