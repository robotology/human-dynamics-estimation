# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

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

