#!/bin/sh

xacro awesome_leg_cartesio.urdf.xacro -o generated/awesome_leg_cartesio.urdf 
xacro awesome_leg_reduced_calibrated.urdf.xacro -o generated/awesome_leg_reduced_calibrated.urdf 
xacro awesome_leg_reduced.urdf.xacro -o generated/awesome_leg_reduced.urdf 
xacro awesome_leg_test_rig.urdf.xacro -o generated/awesome_leg_test_rig.urdf
xacro awesome_leg.urdf.xacro -o generated/awesome_leg.urdf

 