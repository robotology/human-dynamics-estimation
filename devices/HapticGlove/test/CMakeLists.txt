# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)
project(SenseGloveTestApplication)

find_package(IWear REQUIRED)
find_package(WearableActuators REQUIRED)
find_package(YARP COMPONENTS os REQUIRED)
find_package(iDynTree REQUIRED)



add_executable(${PROJECT_NAME} VisualizeIMUData.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC
    IWear::IWear
    YARP::YARP_OS
    YARP::YARP_init
    iDynTree::idyntree-visualization
    )


