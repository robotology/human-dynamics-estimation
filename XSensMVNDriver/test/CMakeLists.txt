# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# Build the test unit executable
# ==============================
add_executable(testCalibrator ${CMAKE_CURRENT_SOURCE_DIR}/testCalibrator.cpp)
target_link_libraries(testCalibrator XSensMVNDriver)
