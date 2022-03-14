cmake_minimum_required(VERSION 3.1.3)
project(awesome_leg_pholus)

find_package(catkin REQUIRED COMPONENTS roscpp rospy urdf sensor_msgs xbot_msgs std_msgs message_generation)
find_package(XBotInterface REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(matlogger2 QUIET)
find_package(cartesian_interface REQUIRED)
find_package(xbot2 QUIET)

option(XBOT2_ENABLE_XENO OFF "Compile against xenomai")

add_service_files(
  FILES
  EllTrajRt.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} ${XBotInterface_INCLUDE_DIRS} ${xbot_msgs_INCLUDE_DIRS} ${cartesian_interface_INCLUDE_DIRS})

add_executable(cart2gazebo_bridge src/cartesio2gazebo_bridge.cpp)
add_executable(robot_state_logger src/robot_state_logger.cpp)

add_subdirectory(src/xbot2_grav_comp)
add_subdirectory(src/xbot2_imp_cntrl)

target_link_libraries(cart2gazebo_bridge ${catkin_LIBRARIES})
target_link_libraries(robot_state_logger ${catkin_LIBRARIES} ${XBotInterface_LIBRARIES} matlogger2::matlogger2)
set_target_properties(robot_state_logger PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY description/sdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description)
install(DIRECTORY description/urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description)
install(DIRECTORY description/srdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description)
install(DIRECTORY description/meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description)

