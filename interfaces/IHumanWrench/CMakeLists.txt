# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

add_library(IHumanWrench INTERFACE)

target_sources(IHumanWrench INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hde/interfaces/IHumanWrench.h>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces/IHumanWrench.h>)

target_include_directories(IHumanWrench INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces>)

install(
    FILES hde/interfaces/IHumanWrench.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces)

