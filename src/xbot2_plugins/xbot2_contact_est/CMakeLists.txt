cmake_minimum_required(VERSION 3.16)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(awesome_utils REQUIRED)

add_xbot2_plugin(contact_est_rt
    src/contact_est_rt.cpp)

add_dependencies(contact_est_rt ${PROJECT_NAME}_generate_messages)

target_include_directories(contact_est_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )

target_link_libraries(contact_est_rt PRIVATE awesome_utils::awesome_utils
                                             xbot2::xbot2_ros_support
                                             matlogger2::matlogger2)

install(
    TARGETS contact_est_rt DESTINATION lib
)
