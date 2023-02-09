cmake_minimum_required(VERSION 3.1.3)
set (CMAKE_CXX_STANDARD 17)

project(awesome_leg)

find_package(catkin REQUIRED roscpp rospy urdf sensor_msgs std_msgs message_generation)
find_package(XBotInterface REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(xbot2 QUIET)
find_package(xacro REQUIRED)

option(XBOT2_ENABLE_XENO OFF "Compile against xenomai")

#add_service_files(
#  FILES
#  EllTrajRt.srv
#  SinJointTraj.srv
#  BypassDspRt.srv
#)

add_service_files(
  FILES
  JumpNow.srv
  IqModelCalibRt.srv
  SetIdleState.srv
  SimpleTrigger.srv
  SetSafetyStop.srv
  SetRegEnergyMonitoringStatus.srv
)

add_message_files(
  FILES
  MatReplayerStatus.msg
  IqEstStatus.msg
  IqCalStatus.msg
  ContactEstStatus.msg
  RegPowStatus.msg
  BaseEstStatus.msg
  BtRootStatus.msg
  PluginsManStatus.msg
  IdleState.msg
  IqMeasStatus.msg
  TempOkStatus.msg
  TimerStatus.msg
  SafetyStopState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

# add_subdirectory(src/xbot2_plugins/xbot2_grav_comp)
# add_subdirectory(src/xbot2_plugins/xbot2_imp_cntrl)
#add_subdirectory(src/xbot2_plugins/xbot2_jump)
#add_subdirectory(src/xbot2_plugins/xbot2_iq_model_calib)
#add_subdirectory(src/xbot2_plugins/xbot2_contact_est)
add_subdirectory(src/xbot2_plugins/xbot2_energy_rec)

add_executable(robot_state_logger src/awesome_leg_utils/robot_state_logger.cpp)
target_link_libraries(robot_state_logger ${catkin_LIBRARIES} ${XBotInterface_LIBRARIES} matlogger2::matlogger2)
target_include_directories(robot_state_logger PRIVATE ${catkin_INCLUDE_DIRS} ${XBotInterface_INCLUDE_DIRS} ${xbot_msgs_INCLUDE_DIRS})
set_target_properties(robot_state_logger PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

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

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
install(DIRECTORY description/sdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description)
install(DIRECTORY description/urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description)
install(DIRECTORY description/srdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description)
install(DIRECTORY description/meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description)
install(DIRECTORY description/new_meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description)

install(TARGETS robot_state_logger DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS 
        src/horizon_code/plot_horizon_jump.py src/horizon_code/jump_generation.py 
        src/awesome_leg_utils/robot_state_log_plotter.py
        src/awesome_leg_utils/logger_utilities.py
        src/awesome_leg_utils/param_identification_utilities.py
        src/xbot_cyclic_pos_pub.py
        src/xbot_initial_pos_pub.py
        src/gravity_compensation.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY src/horizon_code/jump_utils  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

