# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# Group files with the correct path for the target_sources command
file(GLOB_RECURSE IXSENS_MVN_CONTROL_HEADERS *.h)

add_library(IXsensMVNControl INTERFACE)
target_sources(IXsensMVNControl INTERFACE ${IXSENS_MVN_CONTROL_HEADERS})
target_include_directories(IXsensMVNControl INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    )
