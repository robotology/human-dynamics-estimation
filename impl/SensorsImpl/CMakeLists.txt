# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


add_library(SensorsImpl
    SensorsImpl.cpp
    include/Wearable/IWear/Sensors/impl/SensorsImpl.h)
add_library(Wearable::SensorsImpl ALIAS SensorsImpl)

target_include_directories(SensorsImpl PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(SensorsImpl PUBLIC IWear::IWear)

install(
    TARGETS SensorsImpl
    EXPORT SensorsImpl
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
