#!/bin/bash

set -ev

case $TRAVIS_OS_NAME in
    osx)
        # Build and install YCM
        git clone https://github.com/robotology/ycm
        cd ycm
        mkdir build && cd build
        cmake -G"${TRAVIS_CMAKE_GENERATOR}" \
              -DCMAKE_BUILD_TYPE=${TRAVIS_BUILD_TYPE} \
              ..
        cmake --build . --config ${TRAVIS_BUILD_TYPE}
        sudo cmake --build . --config ${TRAVIS_BUILD_TYPE} --target install
        cd ../..
    ;;
    linux) ;;
    *) travis_terminate 1
    ;;
esac

# Build and install idyntree
git clone https://github.com/robotology/idyntree
cd idyntree
mkdir build && cd build
cmake -G"${TRAVIS_CMAKE_GENERATOR}" \
      -DCMAKE_BUILD_TYPE=${TRAVIS_BUILD_TYPE} \
      -DIDYNTREE_USES_KDL=OFF \
      -DIDYNTREE_USES_YARP=ON \
      -DIDYNTREE_USES_ICUB_MAIN=OFF \
      ..
cmake --build . --config ${TRAVIS_BUILD_TYPE}
cmake --build . --config ${TRAVIS_BUILD_TYPE} --target install

cd ../..

# Build and install xsense-mvn
git clone https://github.com/robotology-playground/xsens-mvn
cd xsens-mvn
mkdir build && cd build
cmake -G"${TRAVIS_CMAKE_GENERATOR}" \
      -DCMAKE_BUILD_TYPE=${TRAVIS_BUILD_TYPE} \
      -DENABLE_xsens_mvn=OFF \
      -DENABLE_xsens_mvn_wrapper=OFF \
      -DENABLE_xsens_mvn_remote=OFF \
      ..
cmake --build . --config ${TRAVIS_BUILD_TYPE}
cmake --build . --config ${TRAVIS_BUILD_TYPE} --target install
