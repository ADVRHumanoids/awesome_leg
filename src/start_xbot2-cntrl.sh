#!/bin/bash

sleep 4 # waiting 4s to give time to gazebo to load

rosservice call /xbotcore/ros_ctrl/switch true
