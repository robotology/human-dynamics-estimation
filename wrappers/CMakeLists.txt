# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


add_subdirectory(IWear)
add_subdirectory(IWearActuators)
add_subdirectory(IXsensMVNControl)

if(ENABLE_Logger)
    add_subdirectory(IWearLogger)
endif()


