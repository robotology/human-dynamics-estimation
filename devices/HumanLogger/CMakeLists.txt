# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(YARP COMPONENTS dev)
find_package(robometry)


yarp_prepare_plugin(human_logger
    TYPE hde::devices::HumanLogger
    INCLUDE HumanLogger.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanLogger
    HumanLogger.cpp
    HumanLogger.h)

target_include_directories(HumanLogger PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})

target_link_libraries(HumanLogger PUBLIC
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init
    robometry::robometry
    IHumanState
    IHumanDynamics
    )

if(MSVC)
  target_compile_options(HumanLogger PRIVATE /bigobj)
endif()

yarp_install(
    TARGETS HumanLogger
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
