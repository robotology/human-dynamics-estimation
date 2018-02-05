cmake_minimum_required(VERSION 2.6)
project(hde-interface)

find_package(YARP)

set(hde_interface_headers include/hdeinterfacemodule.h include/hdedriver.h)
set(hde_interface_sources src/main.cpp src/hdeinterfacemodule.cpp src/hdedriver.cpp)

include_directories(${YARP_INCLUDE_DIRS} -I include/)

add_executable(hde-interface ${hde_interface_headers} ${hde_interface_sources})

target_link_libraries(hde-interface ${YARP_LIBRARIES})

#install(TARGETS hde- interface RUNTIME DESTINATION bin)
