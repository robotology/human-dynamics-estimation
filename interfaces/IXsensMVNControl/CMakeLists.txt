# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


# Group files with the correct path for the target_sources command
file(GLOB_RECURSE IXSENS_MVN_CONTROL_HEADERS *.h)

add_library(IXsensMVNControl INTERFACE)
target_sources(IXsensMVNControl INTERFACE ${IXSENS_MVN_CONTROL_HEADERS})
target_include_directories(IXsensMVNControl INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    )
