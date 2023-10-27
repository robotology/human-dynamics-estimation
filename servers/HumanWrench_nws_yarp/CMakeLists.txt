# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(human_wrench_nws_yarp
    TYPE hde::wrappers::HumanWrench_nws_yarp
    INCLUDE HumanWrench_nws_yarp.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanWrench_nws_yarp
    HumanWrench_nws_yarp.cpp
    HumanWrench_nws_yarp.h)

target_include_directories(HumanWrench_nws_yarp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(HumanWrench_nws_yarp PUBLIC
    IHumanWrench
    HumanWrenchMsg
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init)

yarp_install(
    TARGETS HumanWrench_nws_yarp
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
