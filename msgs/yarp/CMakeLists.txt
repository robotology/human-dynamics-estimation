# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# ===========
# HUMAN STATE
# ===========

yarp_add_idl(HUMANSTATE_FILES thrift/HumanState.thrift)

add_library(HumanStateMsg ${HUMANSTATE_FILES} thrift/HumanState.thrift)
target_link_libraries(HumanStateMsg YARP::YARP_OS YARP::YARP_sig)

target_include_directories(HumanStateMsg PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)

install(TARGETS HumanStateMsg
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
