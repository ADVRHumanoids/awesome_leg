cmake_minimum_required(VERSION 3.16)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(awesome_utils REQUIRED)

add_xbot2_plugin(mat_replayer_rt
    src/mat_replayer_rt.cpp)
add_executable(start_cyclic_jump_client src/cyclic_jump_triggerer.cpp)
#add_library(plugin_utils SHARED src/plugin_utils.cpp)
add_library(contact_est_utils SHARED src/contact_est_utils.cpp)

add_dependencies(mat_replayer_rt ${PROJECT_NAME}_generate_messages contact_est_utils)
add_dependencies(start_cyclic_jump_client ${PROJECT_NAME}_generate_messages)

target_include_directories(mat_replayer_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
#target_include_directories(plugin_utils PRIVATE include ${EIGEN3_INCLUDE_DIR} ${matlogger2_INCLUDE_DIRS})
target_include_directories(contact_est_utils PUBLIC include ${EIGEN3_INCLUDE_DIR} ${matlogger2_INCLUDE_DIRS})

#target_link_libraries(plugin_utils PRIVATE Eigen3::Eigen matlogger2::matlogger2)
target_link_libraries(contact_est_utils PRIVATE Eigen3::Eigen matlogger2::matlogger2
                                                 ${cartesian_interface_LIBRARIES})

target_link_libraries(mat_replayer_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support contact_est_utils)
target_link_libraries(start_cyclic_jump_client PRIVATE ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES} xbot2::xbot2_ros_support)

install(
    TARGETS mat_replayer_rt contact_est_utils DESTINATION lib
)

install(TARGETS start_cyclic_jump_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
