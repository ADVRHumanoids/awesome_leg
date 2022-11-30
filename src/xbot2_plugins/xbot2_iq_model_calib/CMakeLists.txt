cmake_minimum_required(VERSION 3.16)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(xbot2 REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(awesome_utils REQUIRED)

add_xbot2_plugin(iq_model_calib_rt
    src/iq_model_calib_rt.cpp)


add_dependencies(iq_model_calib_rt ${PROJECT_NAME}_generate_messages)

target_include_directories(iq_model_calib_rt PRIVATE include src ${EIGEN3_INCLUDE_DIR}
                                                                ${matlogger2_INCLUDE_DIRS}
                                                                ${awesome_utils_INCLUDE_DIRS}
                                                                ${xbot2_INCLUDE_DIRS}
                                                                ${xbot_msgs_INCLUDE_DIRS})

target_link_libraries(iq_model_calib_rt PRIVATE ${cartesian_interface_LIBRARIES}
                                                xbot2::xbot2_ros_support
                                                awesome_utils::awesome_utils)

install(
    TARGETS iq_model_calib_rt DESTINATION lib
)

install(TARGETS iq_model_calib_rt DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
