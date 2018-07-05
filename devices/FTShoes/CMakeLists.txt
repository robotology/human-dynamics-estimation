# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_library(FTShoes SHARED
    src/FTShoes.cpp
    include/FTShoes.h)

target_include_directories(FTShoes
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/Wearable>)

target_link_libraries(FTShoes IWear)

install(TARGETS FTShoes
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

install(
    FILES include/FTShoes.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Wearable)
