# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(iDynTree)

add_subdirectory(IWearRemapper)
add_subdirectory(IAnalogSensorToIWear)

if(XSENS_MVN_USE_SDK)
  add_subdirectory(XsensSuit)
endif()

if(iDynTree_FOUND)
  add_subdirectory(ICub)
endif()

