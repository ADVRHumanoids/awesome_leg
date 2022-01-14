#!/bin/bash

sleep 4 # waiting 4s to give time to gazebo to load (this may differe depending on the machine on which gazebo in run). A smarter alternative should actually be used.

rosservice call /xbotcore/ros_control/switch true # enabling ros_ctrl (to be able to send commands through ROS); this can also be done through the XBot GUI.
