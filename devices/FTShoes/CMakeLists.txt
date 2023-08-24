# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


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
