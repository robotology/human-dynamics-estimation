# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


yarp_prepare_plugin(xsens_suit
    TYPE wearable::devices::XsensSuit
    INCLUDE include/XsensSuit.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(XsensSuit
    src/XsensSuit.cpp
    include/XsensSuit.h)

target_include_directories(XsensSuit PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(XsensSuit PUBLIC
    Wearable::IWear IXsensMVNControl XSensMVN YARP::YARP_dev YARP::YARP_init)

yarp_install(
    TARGETS XsensSuit
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
