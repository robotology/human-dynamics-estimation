#!/bin/sh
set -e
set -u

# This script expects to find the following environment variables:
#
# - TRAVIS_BUILD_DIR
# - TRAVIS_BUILD_TYPE
# - TRAVIS_CMAKE_GENERATOR

cd $TRAVIS_BUILD_DIR
mkdir build && cd build
cmake -G"$TRAVIS_CMAKE_GENERATOR" -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE ..
cmake --build . --config $TRAVIS_BUILD_TYPE
