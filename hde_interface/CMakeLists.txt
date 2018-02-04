cmake_minimum_required(VERSION 2.6)
project(hde_interface)

find_package(YARP)

set(hde_interface_headers include/hdeinterfacemodule.h)
set(hde_interface_sources src/main.cpp src/hdeinterfacemodule.cpp)

include_directories(${YARP_INCLUDE_DIRS} -I include/)

add_executable(hde_interface ${hde_interface_headers} ${hde_interface_sources})

target_link_libraries(hde_interface ${YARP_LIBRARIES})

#install(TARGETS hde_interface RUNTIME DESTINATION bin)
