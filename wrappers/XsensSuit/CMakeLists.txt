# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

yarp_prepare_plugin(XsensSuitWrapper
    TYPE iwear::wrappers::XsensSuitWrapper
    INCLUDE XsensSuitWrapper.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(XsensSuitWrapper
    src/XsensSuitWrapper.cpp
    include/XsensSuitWrapper.h)

#add_library(XsensSuitWrapper
#    src/XsensSuitWrapper.cpp
#    include/XsensSuitWrapper.h)

target_include_directories(XsensSuitWrapper PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(XsensSuitWrapper
    IWear YARP::YARP_dev)
