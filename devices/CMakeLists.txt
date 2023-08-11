# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

add_subdirectory(HumanStateProvider)
add_subdirectory(HumanWrenchProvider)
add_subdirectory(HumanDynamicsEstimator)
add_subdirectory(HumanControlBoard)
add_subdirectory(RobotPositionController)

if(HUMANSTATEPROVIDER_ENABLE_LOGGER)
    add_subdirectory(HumanLogger)
endif()
