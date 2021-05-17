# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_library(IHumanDynamics INTERFACE)

target_sources(IHumanDynamics INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hde/interfaces/IHumanDynamics.h>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces/IHumanDynamics.h>)

target_include_directories(IHumanDynamics INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces>)

install(
    FILES hde/interfaces/IHumanDynamics.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces)

