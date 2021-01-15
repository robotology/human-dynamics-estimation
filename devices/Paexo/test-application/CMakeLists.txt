# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)
project(PaexoTestApplication)

find_package(IWear REQUIRED)
find_package(WearableActuators REQUIRED)
find_package(YARP COMPONENTS os REQUIRED)

message(STATUS ${WEARABLEACTUATORS_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} paexo-test-application.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC
    IWear::IWear
    WearableActuators::WearableActuators
    YARP::YARP_OS
    YARP::YARP_init)


