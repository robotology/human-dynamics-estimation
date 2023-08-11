# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(OsqpEigen 0.4.0 REQUIRED)
find_package(YARP 3.2 REQUIRED)
find_package(iDynTree REQUIRED)

add_component(NAME utils
              SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/iDynTreeUtils.cpp
              PUBLIC_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/hde/utils/iDynTreeUtils.hpp
              PUBLIC_LINK_LIBRARIES iDynTree::idyntree-core
              PRIVATE_LINK_LIBRARIES YARP::YARP_OS
                                     Eigen3::Eigen
             )
