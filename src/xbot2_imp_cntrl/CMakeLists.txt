cmake_minimum_required(VERSION 3.16)

find_package(xbot2 REQUIRED)

add_xbot2_plugin(cartesio_imp_cntrl_ros_rt
    src/cartesio_imp_cntrl_ros_rt.cpp)

add_xbot2_plugin(cartesio_ell_rt
    src/cartesio_ell_rt.cpp)
    
add_dependencies(cartesio_imp_cntrl_ros_rt awesome_leg_pholus_generate_messages)
add_dependencies(cartesio_ell_rt awesome_leg_pholus_generate_messages)

include_directories(include)

target_link_libraries(cartesio_imp_cntrl_ros_rt PRIVATE ${cartesian_interface_LIBRARIES})

target_link_libraries(cartesio_ell_rt PRIVATE ${cartesian_interface_LIBRARIES} xbot2::xbot2_ros_support)

install(
    TARGETS cartesio_imp_cntrl_ros_rt cartesio_ell_rt
    DESTINATION lib)
