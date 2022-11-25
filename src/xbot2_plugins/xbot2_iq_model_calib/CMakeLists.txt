cmake_minimum_required(VERSION 3.16)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cartesian_interface REQUIRED)
#find_package(OsqpEigen REQUIRED)
#find_package(osqp REQUIRED)

add_xbot2_plugin(iq_model_calib_rt
    src/iq_model_calib_rt.cpp)

add_library(calib_utils SHARED src/calib_utils.cpp)

add_dependencies(iq_model_calib_rt ${PROJECT_NAME}_generate_messages calib_utils)

target_include_directories(iq_model_calib_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
target_include_directories(iq_model_calib_rt PUBLIC include src ${EIGEN3_INCLUDE_DIR} ${matlogger2_INCLUDE_DIRS})

#target_link_libraries(calib_utils PRIVATE ${cartesian_interface_LIBRARIES} Eigen3::Eigen
#                                                                           matlogger2::matlogger2
#                                                                           xbot2::xbot2_ros_support
#                                                                           OsqpEigen::OsqpEigen osqp::osqp)

target_link_libraries(calib_utils PRIVATE ${cartesian_interface_LIBRARIES} Eigen3::Eigen
                                          matlogger2::matlogger2
                                          xbot2::xbot2_ros_support)

target_link_libraries(iq_model_calib_rt PRIVATE ${cartesian_interface_LIBRARIES} calib_utils xbot2::xbot2_ros_support)

install(
    TARGETS iq_model_calib_rt calib_utils DESTINATION lib
)

install(TARGETS iq_model_calib_rt DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
