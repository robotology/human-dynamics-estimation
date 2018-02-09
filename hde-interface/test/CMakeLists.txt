cmake_minimum_required(VERSION 3.0)
project(HDDriverTest)

find_package(YARP REQUIRED)

add_executable(hdedrivertest main.cpp)
target_link_libraries(hdedrivertest ${YARP_LIBRARIES})
