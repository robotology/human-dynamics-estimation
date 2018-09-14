# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_library(SensorsImpl
    SensorsImpl.cpp
    include/Wearable/IWear/Sensors/impl/SensorsImpl.h)
add_library(Wearable::SensorsImpl ALIAS SensorsImpl)

target_include_directories(SensorsImpl PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(SensorsImpl PUBLIC Wearable::IWear)
