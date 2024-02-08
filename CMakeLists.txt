cmake_minimum_required(VERSION 3.1.3)
set (CMAKE_CXX_STANDARD 17)

project(awesome_leg)

find_package(catkin REQUIRED)
find_package(xacro REQUIRED)

catkin_package()

# Xacro files

set(awesome_leg_standalone  "${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/generated/awesome_leg_standalone.urdf")
set(awesome_leg_xbot  "${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/generated/awesome_leg_xbot.urdf")
set(awesome_leg  "${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/generated/awesome_leg.urdf")
set(awesome_leg_no_sliding  "${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/generated/awesome_leg_no_sliding.urdf")
set(awesome_leg_floating  "${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/generated/awesome_leg_floating.urdf")

xacro_add_xacro_file(${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/awesome_leg_standalone.urdf.xacro
                    ${awesome_leg_standalone})
xacro_add_xacro_file(${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/awesome_leg_xbot.urdf.xacro
                    ${awesome_leg_xbot})
xacro_add_xacro_file(${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/awesome_leg_test_rig.urdf.xacro
                    ${awesome_leg})
xacro_add_xacro_file(${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/awesome_leg_no_sliding.urdf.xacro
                    ${awesome_leg_no_sliding})
xacro_add_xacro_file(${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/awesome_leg_floating.urdf.xacro
                    ${awesome_leg_floating})

list(APPEND URDF_files ${awesome_leg_standalone})
list(APPEND URDF_files ${awesome_leg_xbot})
list(APPEND URDF_files ${awesome_leg})
list(APPEND URDF_files ${awesome_leg_no_sliding})
list(APPEND URDF_files ${awesome_leg_floating})

# add an abstract target to actually trigger the builds
add_custom_target(urdf_files ALL DEPENDS ${URDF_files})

# install stuff
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
install(DIRECTORY description/sdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description)
install(DIRECTORY description/urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description)
install(DIRECTORY description/srdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description)
install(DIRECTORY description/meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description)
install(DIRECTORY description/new_meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description)

