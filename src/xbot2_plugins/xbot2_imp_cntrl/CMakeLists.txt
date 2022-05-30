cmake_minimum_required(VERSION 3.16)

find_package(xbot2 REQUIRED)
find_package(matlogger2 1.3.1 REQUIRED)
message("")
message("MatLogger2 include path is set to: " ${matlogger2_INCLUDE_DIRS})
message("")

add_xbot2_plugin(cartesio_imp_cntrl_ros_rt
    src/cartesio_imp_cntrl_ros_rt.cpp)

add_xbot2_plugin(cartesio_ell_rt
    src/cartesio_ell_rt.cpp)

add_xbot2_plugin(bypass_dsp_rt
    src/bypass_dsp.cpp)

add_dependencies(cartesio_imp_cntrl_ros_rt ${PROJECT_NAME}_generate_messages)
add_dependencies(cartesio_ell_rt ${PROJECT_NAME}_generate_messages)
add_dependencies(bypass_dsp_rt ${PROJECT_NAME}_generate_messages)

include_directories(include ${matlogger2_INCLUDE_DIRS})

target_link_libraries(cartesio_imp_cntrl_ros_rt PRIVATE ${cartesian_interface_LIBRARIES})

target_link_libraries(cartesio_ell_rt PRIVATE ${cartesian_interface_LIBRARIES} xbot2::xbot2_ros_support)

target_link_libraries(bypass_dsp_rt PRIVATE ${cartesian_interface_LIBRARIES} xbot2::xbot2_ros_support)

install(
    TARGETS cartesio_imp_cntrl_ros_rt cartesio_ell_rt bypass_dsp_rt
    DESTINATION lib)
