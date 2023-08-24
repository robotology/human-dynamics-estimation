# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


# Build the test unit executables
# ===============================
add_executable(testCalibrator ${CMAKE_CURRENT_SOURCE_DIR}/testCalibrator.cpp)
target_link_libraries(testCalibrator XSensMVN)

add_executable(testDriver ${CMAKE_CURRENT_SOURCE_DIR}/testDriver.cpp)
target_link_libraries(testDriver XSensMVN)
