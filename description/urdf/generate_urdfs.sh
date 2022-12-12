#!/bin/sh

xacro awesome_leg_standalone.urdf.xacro -o generated/awesome_leg_standalone.urdf 
xacro awesome_leg_xbot.urdf.xacro -o generated/awesome_leg_xbot.urdf 
xacro awesome_leg_test_rig.urdf.xacro -o generated/awesome_leg.urdf
xacro awesome_leg_test_rig.urdf.xacro sliding_guide:=true -o generated/awesome_leg_no_sliding.urdf
xacro awesome_leg_floating.urdf.xacro -o generated/awesome_leg_floating.urdf

