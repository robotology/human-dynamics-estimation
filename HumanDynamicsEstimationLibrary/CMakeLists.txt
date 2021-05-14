# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(OsqpEigen 0.4.0 REQUIRED)
find_package(YARP 3.2 REQUIRED)
find_package(iDynTree REQUIRED)

# Include AddComponent cmake module
include(AddComponent)


add_component(NAME hde
              SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/InverseVelocityKinematics.cpp
                      ${CMAKE_CURRENT_SOURCE_DIR}/src/Utils.cpp
              PUBLIC_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/InverseVelocityKinematics.hpp
                             ${CMAKE_CURRENT_SOURCE_DIR}/include/Utils.hpp
              PUBLIC_LINK_LIBRARIES YARP::YARP_OS
                                    YARP::YARP_dev
                                    YARP::YARP_init
                                    iDynTree::idyntree-model
                                    iDynTree::idyntree-inverse-kinematics
                                    iDynTree::idyntree-high-level
              PRIVATE_LINK_LIBRARIES OsqpEigen::OsqpEigen
                                     osqp::osqp
             )
