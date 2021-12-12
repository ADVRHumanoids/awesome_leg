#!/usr/bin/env python3
####################### IMPORTS ########################
import rospy
import xbot_interface.config_options as xbot_opt
from xbot_interface import xbot_interface as xbot
from horizon.utils import mat_storer

def talker():
    pub = rospy.Publisher('/xbotcore/command', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
