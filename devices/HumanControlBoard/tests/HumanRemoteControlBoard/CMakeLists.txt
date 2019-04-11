cmake_minimum_required(VERSION 3.0)
project(HumanControlBoardDriverTest)

find_package(YARP REQUIRED)

add_executable(humancontrolboarddrivertest main.cpp)
target_link_libraries(humancontrolboarddrivertest ${YARP_LIBRARIES})
