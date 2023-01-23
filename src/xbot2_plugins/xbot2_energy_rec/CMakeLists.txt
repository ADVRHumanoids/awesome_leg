cmake_minimum_required(VERSION 3.16)
set (CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(awesome_utils REQUIRED)
find_package(base_estimation REQUIRED)
find_package(behaviortree_cpp_v3 QUIET)

add_executable(start_jump_replayer src/start_jump_replayer.cpp)
add_dependencies(start_jump_replayer ${PROJECT_NAME}_generate_messages)
target_link_libraries(start_jump_replayer PRIVATE ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES} xbot2::xbot2_ros_support)

add_xbot2_plugin(jmp_replayer_rt
    src/jump_replayer_rt.cpp)
target_include_directories(jmp_replayer_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
add_dependencies(jmp_replayer_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(jmp_replayer_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support)

add_xbot2_plugin(base_est_rt
    src/base_est_rt.cpp)
target_include_directories(base_est_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} ${base_estimation_INCLUDE_DIRS})
add_dependencies(base_est_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(base_est_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support ${base_estimation_LIBRARIES})

add_xbot2_plugin(iq_model_calib_rt
    src/iq_model_calib_rt.cpp)
target_include_directories(iq_model_calib_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
add_dependencies(iq_model_calib_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(iq_model_calib_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support)

add_xbot2_plugin(bus_power_rt
    src/bus_power_rt.cpp)
target_include_directories(bus_power_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
add_dependencies(bus_power_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(bus_power_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support)

if(behaviortree_cpp_v3_FOUND)
    message("Will compile behaviour tree rt plugin")
    add_xbot2_plugin(bt_rt
        src/bt_rt.cpp)
    target_include_directories(bt_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
    add_dependencies(bt_rt ${PROJECT_NAME}_generate_messages)
    target_link_libraries(bt_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support ${behaviortree_cpp_v3_LIBRARIES})

    install(
        TARGETS bt_rt DESTINATION lib
    )

endif()

install(
    TARGETS jmp_replayer_rt base_est_rt iq_model_calib_rt bus_power_rt DESTINATION lib
)

install(TARGETS start_jump_replayer DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
