cmake_minimum_required(VERSION 3.16)

find_package(xbot2 REQUIRED)

add_xbot2_plugin(cartesio_cntrl_rt
    src/cartesio_cntrl_rt.cpp)

include_directories(include)

target_link_libraries(cartesio_cntrl_rt PRIVATE ${cartesian_interface_LIBRARIES})

install(
    TARGETS cartesio_cntrl_rt
    DESTINATION lib
    )