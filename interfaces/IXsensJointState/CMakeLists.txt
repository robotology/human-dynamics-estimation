# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_library(IXsensJointState INTERFACE)

target_sources(IXsensJointState INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/IXsensJointState.h>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/IXsensJointState.h>)

target_include_directories(IXsensJointState INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde>)

install(
    FILES IXsensJointState.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hde)

