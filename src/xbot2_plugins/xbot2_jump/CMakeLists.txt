cmake_minimum_required(VERSION 3.16)

find_package(xbot2 REQUIRED)
find_package(matlogger2 1.3.1 REQUIRED)
message("")
message("MatLogger2 include path is set to: " ${matlogger2_INCLUDE_DIRS})
message("")

add_xbot2_plugin(mat_replayer_rt
    src/mat_replayer_rt.cpp)

add_dependencies(mat_replayer_rt ${PROJECT_NAME}_generate_messages)

include_directories(include ${matlogger2_INCLUDE_DIRS})

target_link_libraries(mat_replayer_rt PRIVATE ${cartesian_interface_LIBRARIES})

install(
    TARGETS mat_replayer_rt DESTINATION lib)
