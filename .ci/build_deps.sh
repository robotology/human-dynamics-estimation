#!/bin/bash
set -e
set -u

# Folder where to clone repositories
GIT_FOLDER=$HOME/git
mkdir -p $GIT_FOLDER

if [ "${TRAVIS_OS_NAME}" = "osx" ] ; then
     # Build and install ycm
    cd $GIT_FOLDER
    git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/ycm.git
    cd ycm
    mkdir build && cd build
    cmake .. \
        -G"$TRAVIS_CMAKE_GENERATOR" \
        -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_PREFIX
    cmake --build . --target install

    # Build and install yarp
    cd $GIT_FOLDER
    git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/yarp.git
    cd yarp
    mkdir build && cd build
    cmake .. \
        -G"$TRAVIS_CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE \
        -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_PREFIX \
        -DCREATE_LIB_MATH=ON
    cmake --build . --config $TRAVIS_BUILD_TYPE --target install

    # Build and install idyntree
    cd $GIT_FOLDER
    git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/idyntree.git
    cd idyntree
    mkdir build && cd build
    cmake .. \
        -G"$TRAVIS_CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE \
        -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_PREFIX
    cmake --build . --config $TRAVIS_BUILD_TYPE --target install
fi

# Build and install xsense-mvn
cd $GIT_FOLDER
git clone https://github.com/robotology-playground/xsens-mvn
cd xsens-mvn
mkdir build && cd build
cmake -G"${TRAVIS_CMAKE_GENERATOR}" \
      -DCMAKE_BUILD_TYPE=${TRAVIS_BUILD_TYPE} \
      -DENABLE_xsens_mvn=OFF \
      -DENABLE_xsens_mvn_wrapper=OFF \
      -DENABLE_xsens_mvn_remote=OFF \
      ..
cmake --build . --config ${TRAVIS_BUILD_TYPE} --target install

# Build and install wearable
cd $GIT_FOLDER
git clone https://github.com/robotology-playground/wearables.git
cd wearables
mkdir build && cd build
cmake -G"${TRAVIS_CMAKE_GENERATOR}" \
      -DCMAKE_BUILD_TYPE=${TRAVIS_BUILD_TYPE} \
      ..
cmake --build . --config ${TRAVIS_BUILD_TYPE} --target install
