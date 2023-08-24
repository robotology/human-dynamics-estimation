# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(SenseGlove REQUIRED)

# Compile the plugin by default
yarp_prepare_plugin(hapticGlove  TYPE wearable::devices::HapticGlove
                    INCLUDE include/HapticGlove.h
                    CATEGORY device
                    ADVANCED
                    DEFAULT ON
                    )

yarp_add_plugin(HapticGlove
    src/HapticGlove.cpp
    src/SenseGloveHelper.cpp
    include/HapticGlove.h
    include/SenseGloveHelper.hpp
    )

target_include_directories(HapticGlove PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(HapticGlove
    PUBLIC
    YARP::YARP_dev
    Wearable::IWear
    PRIVATE
    YARP::YARP_init
    SenseGlove
    Eigen3::Eigen
    )

yarp_install(TARGETS HapticGlove
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
             YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
             )

set (WEARABLES_XML_FILES conf/HapticGlove.xml)
install(FILES ${WEARABLES_XML_FILES}
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${WEARABLES_PROJECT_NAME})
