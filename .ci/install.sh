#!/bin/sh
set -e
set -u

# This script expects to find the following environment variables:
#
# - DEPS_BRANCH
# - TRAVIS_BUILD_TYPE
# - DEPS_INSTALL_PREFIX
# - CMAKE_BUILD_OPTIONS
# - TRAVIS_CMAKE_GENERATOR

mkdir $HOME/git
export CXXFLAGS="-Wno-unused-command-line-argument"

# Install YCM
cd $HOME/git
git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/ycm.git
cd ycm
mkdir build && cd build
cmake .. \
    -G"$TRAVIS_CMAKE_GENERATOR" \
    -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE \
    -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_PREFIX \
cmake --build . --config $TRAVIS_BUILD_TYPE --target install

# Install Yarp
cd $HOME/git
git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/yarp.git
cd yarp
mkdir build && cd build
cmake .. \
    -G"$TRAVIS_CMAKE_GENERATOR" \
    -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE \
    -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_PREFIX \
    -DCREATE_LIB_MATH=ON
cmake --build . --config $TRAVIS_BUILD_TYPE --target install

# Install icub-main
cd $HOME/git
git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/icub-main.git
cd icub-main
mkdir build && cd build
cmake .. \
    -G"$TRAVIS_CMAKE_GENERATOR" \
    -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE \
    -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_PREFIX
cmake --build . --config $TRAVIS_BUILD_TYPE --target install $CMAKE_BUILD_OPTIONS

# Install icub-firmware-shared
cd $HOME/git
git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/icub-firmware-shared.git
cd icub-firmware-shared
mkdir build && cd build
cmake ..
