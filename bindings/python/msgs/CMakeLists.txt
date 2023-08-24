# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


set(H_PREFIX include/Wearable/bindings/msgs)

add_wearables_python_module(
    NAME MsgsBindings
    SOURCES src/WearableData.cpp src/Module.cpp
    HEADERS ${H_PREFIX}/WearableData.h ${H_PREFIX}/BufferedPort.h ${H_PREFIX}/Module.h
    LINK_LIBRARIES Wearable::WearableData YARP::YARP_os)
