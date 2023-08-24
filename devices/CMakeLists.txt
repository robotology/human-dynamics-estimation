# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


find_package(iDynTree)

add_subdirectory(IWearRemapper)
add_subdirectory(IAnalogSensorToIWear)
add_subdirectory(IFrameTransformToIWear)

if(ENABLE_Paexo)
    add_subdirectory(Paexo)
endif()

if(ENABLE_XsensSuit)
  add_subdirectory(XsensSuit)
endif()

if(ENABLE_ICub)
  add_subdirectory(ICub)
endif()

if(ENABLE_HapticGlove)
    add_subdirectory(HapticGlove)
endif()
