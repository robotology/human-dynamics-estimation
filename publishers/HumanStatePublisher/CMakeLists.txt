# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(YARP COMPONENTS rosmsg REQUIRED)
find_package(iDynTree REQUIRED)

yarp_prepare_plugin(human_state_publisher
    TYPE hde::publishers::HumanStatePublisher
    INCLUDE HumanStatePublisher.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanStatePublisher
    HumanStatePublisher.cpp
    HumanStatePublisher.h)

target_include_directories(HumanStatePublisher PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(HumanStatePublisher PUBLIC
    IHumanState
    HumanROSMsgs
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init
    YARP::YARP_rosmsg
    iDynTree::idyntree-model
    iDynTree::idyntree-inverse-kinematics
    iDynTree::idyntree-high-level)
    
if(MSVC)
  target_compile_options(HumanStatePublisher PRIVATE /bigobj)
endif()

yarp_install(
    TARGETS HumanStatePublisher
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/conf/human_state_publisher.ini"
    DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
