cmake_minimum_required(VERSION 3.1.3)
project(awesome_leg_pholus)

find_package(catkin REQUIRED COMPONENTS roscpp rospy urdf sensor_msgs xbot_msgs)

# find_package(XBotInterface REQUIRED)
# find_package(xbot_msgs REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cart2gazebo_bridge src/cartesio2gazebo_bridge.cpp)
# add_executable(robot_state_logger src/robot_state_logger.cpp)

target_link_libraries(cart2gazebo_bridge ${catkin_LIBRARIES})
# target_link_libraries(robot_state_logger ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY media DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY description/sdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY description/urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY description/srdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY description/meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


