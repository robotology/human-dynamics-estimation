# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


cmake_minimum_required(VERSION 3.16)
project(PaexoTestApplication)

find_package(IWear REQUIRED)
find_package(WearableActuators REQUIRED)
find_package(YARP COMPONENTS os REQUIRED)

add_executable(${PROJECT_NAME} paexo-test-application.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC
    IWear::IWear
    WearableActuators::WearableActuators
    YARP::YARP_os
    YARP::YARP_init)


