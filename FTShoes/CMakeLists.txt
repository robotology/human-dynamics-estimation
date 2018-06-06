# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_library(FTShoes src/FTShoes.cpp include/FTShoes.h)
target_link_libraries(FTShoes IWear)
