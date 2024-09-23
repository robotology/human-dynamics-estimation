# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


set(XSENS_MVN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/XSensMVNCalibrator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/XSensMVNDriverImpl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/XSensMVNDriver.cpp)

set(XSENS_MVN_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/XSensLogger.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/XSensCalibrationQualities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/XSensMVNCalibrator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/XSensMVNDriverImpl.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/XSensMVNDriver.h)

add_library(XSensMVN
    ${XSENS_MVN_SOURCES}
    ${XSENS_MVN_INCLUDES})

target_include_directories(XSensMVN
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/XSensMVN>)

target_link_libraries(XSensMVN ${XsensXME_LIBRARIES} IXsensMVNControl)

install(TARGETS XSensMVN
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(
    FILES ${XSENS_MVN_INCLUDES}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/XSensMVN)

add_subdirectory(test)
