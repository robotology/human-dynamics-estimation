# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


yarp_prepare_plugin(iframetransform_to_iwear
    TYPE wearable::devices::IFrameTransformToIWear
    INCLUDE include/IFrameTransformToIWear.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(IFrameTransformToIWear
    src/IFrameTransformToIWear.cpp
    include/IFrameTransformToIWear.h)

target_include_directories(IFrameTransformToIWear PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(IFrameTransformToIWear PUBLIC
    Wearable::IWear YARP::YARP_dev YARP::YARP_init)

yarp_install(
    TARGETS IFrameTransformToIWear
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

set (WEARABLES_XML_FILES conf/iframetransform_to_iwear.xml)
install(FILES ${WEARABLES_XML_FILES}
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${WEARABLES_PROJECT_NAME})
