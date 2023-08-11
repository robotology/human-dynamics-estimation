# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

add_library(IHumanState INTERFACE)

target_sources(IHumanState INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hde/interfaces/IHumanState.h>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces/IHumanState.h>)

target_include_directories(IHumanState INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces>)

install(
    FILES hde/interfaces/IHumanState.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces)

