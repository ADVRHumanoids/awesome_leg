cmake_minimum_required(VERSION 3.16)

find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)

add_xbot2_plugin(mat_replayer_rt
    src/mat_replayer_rt.cpp)

add_library(plugin_utils src/plugin_utils.cpp)

add_dependencies(mat_replayer_rt ${PROJECT_NAME}_generate_messages plugin_utils)

target_include_directories(mat_replayer_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS})
target_include_directories(plugin_utils PRIVATE include ${EIGEN3_INCLUDE_DIR} ${matlogger2_INCLUDE_DIRS})

target_link_libraries(plugin_utils PRIVATE ${eig} Eigen3::Eigen matlogger2::matlogger2)
target_link_libraries(mat_replayer_rt PRIVATE ${cartesian_interface_LIBRARIES} plugin_utils xbot2::xbot2_ros_support)

install(
    TARGETS mat_replayer_rt DESTINATION lib)
