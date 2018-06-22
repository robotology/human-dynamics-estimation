# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(XSENS_MVN_DRIVER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/XSensMVNCalibrator.cpp)

set(XSENS_MVN_DRIVER_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include/XSensMVNCalibrator.h
                              ${CMAKE_CURRENT_SOURCE_DIR}/include/XSensLogger.h
                              ${CMAKE_CURRENT_SOURCE_DIR}/include/XSensCalibrationQualities.h)

add_library(XSensMVNDriver SHARED ${XSENS_MVN_DRIVER_SOURCES}
                                  ${XSENS_MVN_DRIVER_INCLUDES})

target_include_directories(XSensMVNDriver
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/Wearable>)

target_link_libraries(XSensMVNDriver ${XsensXME_LIBRARIES})

install(TARGETS XSensMVNDriver
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

install(
    FILES ${XSENS_MVN_DRIVER_INCLUDES}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Wearable)
