cmake_minimum_required(VERSION 3.16)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(awesome_utils REQUIRED)
find_package(base_estimation REQUIRED)

add_executable(start_jump_replayer src/start_jump_replayer.cpp)
add_dependencies(start_jump_replayer ${PROJECT_NAME}_generate_messages)
target_link_libraries(start_jump_replayer PRIVATE ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES} xbot2::xbot2_ros_support)

add_xbot2_plugin(jmp_replayer_rt
    src/jump_replayer_rt.cpp)
target_include_directories(jmp_replayer_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
target_link_libraries(jmp_replayer_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support)

add_xbot2_plugin(contact_est_rt
    src/contact_est_rt.cpp)
target_include_directories(contact_est_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} ${base_estimation_INCLUDE_DIRS})
target_link_libraries(contact_est_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support ${base_estimation_LIBRARIES})

add_xbot2_plugin(iq_model_calib_rt
    src/iq_model_calib_rt.cpp)
target_include_directories(iq_model_calib_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
target_link_libraries(iq_model_calib_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support)

install(
    TARGETS jmp_replayer_rt contact_est_rt DESTINATION lib
)

install(TARGETS start_jump_replayer DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
