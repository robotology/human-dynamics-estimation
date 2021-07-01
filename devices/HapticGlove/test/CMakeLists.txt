# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)
project(SenseGloveTestApplication)

find_package(IWear REQUIRED)
find_package(YARP REQUIRED)
find_package(iDynTree REQUIRED)



add_executable(${PROJECT_NAME} VisualizeFrames.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC
    IWear::IWear
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_dev
    YARP::YARP_math
    iDynTree::idyntree-visualization
    )

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

install(FILES config.ini DESTINATION bin )



