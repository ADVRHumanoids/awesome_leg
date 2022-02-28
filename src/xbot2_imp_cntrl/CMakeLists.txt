cmake_minimum_required(VERSION 3.16)

find_package(xbot2 REQUIRED)

add_xbot2_plugin(cartesio_imp_cntrl_rt
    src/cartesio_imp_cntrl_rt.cpp)

add_xbot2_plugin(cartesio_imp_cntrl_ros_rt
    src/cartesio_imp_cntrl_ros_rt.cpp)

add_xbot2_plugin(cartesio_elliptical_rt
    src/cartesio_elliptical_rt.cpp)

add_xbot2_plugin(cartesio_ell_config_rt
    src/cartesio_ell_config_rt.cpp)

include_directories(include)

target_link_libraries(cartesio_imp_cntrl_rt PRIVATE ${cartesian_interface_LIBRARIES})

target_link_libraries(cartesio_imp_cntrl_ros_rt PRIVATE ${cartesian_interface_LIBRARIES})

target_link_libraries(cartesio_elliptical_rt PRIVATE ${cartesian_interface_LIBRARIES})

target_link_libraries(cartesio_ell_config_rt PRIVATE ${cartesian_interface_LIBRARIES})

install(
    TARGETS cartesio_imp_cntrl_rt cartesio_imp_cntrl_ros_rt cartesio_elliptical_rt cartesio_ell_config_rt
    DESTINATION lib)