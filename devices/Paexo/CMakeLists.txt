#/*
# * Copyright (C) 2020 iCub Facility
#* Authors: Yeshasvi Tirupachuri
#* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#*/

# Compile the plugin by default
yarp_prepare_plugin(paexo  TYPE wearable::devices::Paexo
                    INCLUDE Paexo.h
                    CATEGORY device
                    ADVANCED
                    DEFAULT ON
                    )

yarp_add_plugin(Paexo src/Paexo.cpp include/Paexo.h)

target_include_directories(Paexo PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(Paexo
    PUBLIC
    YARP::YARP_dev
    YARP::YARP_init
    Wearable::IWear)

yarp_install(TARGETS Paexo
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
             YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
