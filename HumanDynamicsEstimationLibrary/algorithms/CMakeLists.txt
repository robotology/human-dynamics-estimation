# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(OsqpEigen 0.4.0 REQUIRED)
find_package(YARP 3.2 REQUIRED)
find_package(iDynTree REQUIRED)

add_component(NAME algorithms
              SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/InverseVelocityKinematics.cpp
                      ${CMAKE_CURRENT_SOURCE_DIR}/src/DynamicalInverseKinematics.cpp
              PUBLIC_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/hde/algorithms/InverseVelocityKinematics.hpp
                             ${CMAKE_CURRENT_SOURCE_DIR}/include/hde/algorithms/DynamicalInverseKinematics.hpp
              PUBLIC_LINK_LIBRARIES iDynTree::idyntree-model
              PRIVATE_LINK_LIBRARIES YARP::YARP_os
                                     iDynTree::idyntree-high-level
                                     OsqpEigen::OsqpEigen
                                     osqp::osqp
                                     HumanDynamicsEstimation::utils
             )

if(MSVC)
  target_compile_options(algorithms PRIVATE /bigobj)
endif()
