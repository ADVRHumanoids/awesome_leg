cmake_minimum_required(VERSION 3.16)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cartesian_interface REQUIRED)

add_xbot2_plugin(i_q_model_calib_rt
    src/mat_replayer_rt.cpp)
add_executable(traj_replay_triggerer src/traj_replay_triggerer.cpp)
add_library(calib_utils SHARED src/calib_utils.cpp)

add_dependencies(i_q_model_calib_rt ${PROJECT_NAME}_generate_messages calib_utils)
add_dependencies(traj_replay_triggerer ${PROJECT_NAME}_generate_messages)

target_include_directories(i_q_model_calib_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
target_include_directories(calib_utils PRIVATE include ${EIGEN3_INCLUDE_DIR} ${matlogger2_INCLUDE_DIRS})

target_link_libraries(calib_utils PRIVATE Eigen3::Eigen matlogger2::matlogger2)

target_link_libraries(i_q_model_calib_rt PRIVATE ${cartesian_interface_LIBRARIES} calib_utils xbot2::xbot2_ros_support)
target_link_libraries(traj_replay_triggerer PRIVATE ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES} xbot2::xbot2_ros_support)

install(
    TARGETS i_q_model_calib_rt calib_utils DESTINATION lib
)

install(TARGETS i_q_model_calib_rt DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
