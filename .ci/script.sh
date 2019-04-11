#!/bin/sh
set -e
set -u

cd $TRAVIS_BUILD_DIR

mkdir -p build && cd build

cmake -G"$TRAVIS_CMAKE_GENERATOR" -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE ..
cmake --build . --config $TRAVIS_BUILD_TYPE
