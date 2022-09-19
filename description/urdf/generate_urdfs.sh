#!/bin/sh

xacro awesome_leg_standalone.urdf.xacro -o generated/awesome_leg_standalone.urdf 
xacro awesome_leg_xbot.urdf.xacro -o generated/awesome_leg_xbot.urdf 
xacro awesome_leg_test_rig.urdf.xacro -o generated/awesome_leg.urdf

 