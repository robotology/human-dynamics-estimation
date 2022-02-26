# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(OsqpEigen 0.4.0 REQUIRED)
find_package(YARP 3.2 REQUIRED)
find_package(iDynTree REQUIRED)

add_component(NAME algorithms
              SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/InverseVelocityKinematics.cpp
                      ${CMAKE_CURRENT_SOURCE_DIR}/src/DynamicalInverseKinematics.cpp
              PUBLIC_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/hde/algorithms/InverseVelocityKinematics.hpp
                             ${CMAKE_CURRENT_SOURCE_DIR}/include/hde/algorithms/DynamicalInverseKinematics.hpp
              PUBLIC_LINK_LIBRARIES iDynTree::idyntree-model                           
              PRIVATE_LINK_LIBRARIES YARP::YARP_OS
                                     iDynTree::idyntree-high-level
                                     OsqpEigen::OsqpEigen
                                     osqp::osqp
                                     HumanDynamicsEstimation::utils
             )

if(MSVC)
  target_compile_options(algorithms PRIVATE /bigobj)
endif()
