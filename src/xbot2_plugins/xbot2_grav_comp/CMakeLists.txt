cmake_minimum_required(VERSION 3.16)

find_package(xbot2 REQUIRED)

add_xbot2_plugin(grav_comp_rt
    src/grav_comp_rt.cpp)

add_xbot2_plugin(grav_comp_cart_rt
src/grav_comp_cart_rt.cpp)
include_directories(include)

target_link_libraries(grav_comp_rt PRIVATE ${cartesian_interface_LIBRARIES})
target_link_libraries(grav_comp_cart_rt PRIVATE ${cartesian_interface_LIBRARIES})

install(
    TARGETS grav_comp_rt grav_comp_cart_rt
    DESTINATION lib)