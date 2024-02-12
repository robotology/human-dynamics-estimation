# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


set(H_PREFIX include/hde/bindings/msgs)

add_hde_python_module(
    NAME MsgsBindings
    SOURCES src/HumanState.cpp src/Module.cpp
    HEADERS ${H_PREFIX}/HumanState.h ${H_PREFIX}/BufferedPort.h ${H_PREFIX}/Module.h
    LINK_LIBRARIES HumanDynamicsEstimation::HumanStateMsg YARP::YARP_os)