#!/bin/bash
set -e
set -u

# Folder where to clone repositories
GIT_FOLDER=$HOME/git
mkdir -p $GIT_FOLDER

if [ "${TRAVIS_OS_NAME}" = "osx" ] ; then
     # Build and install ycm
    cd $GIT_FOLDER

    rm -rf ycm
    git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/ycm.git
    cd ycm
    mkdir -p build && cd build

    cmake .. \
        -G"$TRAVIS_CMAKE_GENERATOR" \
        -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_PREFIX
    cmake --build . --target install

    # Build and install yarp
    cd $GIT_FOLDER

    rm -rf yarp
    git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/yarp.git
    cd yarp
    mkdir -p build && cd build

    cmake .. \
        -G"$TRAVIS_CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE \
        -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_PREFIX \
        -DCREATE_LIB_MATH=ON
    cmake --build . --config $TRAVIS_BUILD_TYPE --target install

    # Build and install icub-main
    cd $GIT_FOLDER

    rm -rf icub-main
    git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/icub-main.git
    cd icub-main
    mkdir -p build && cd build

    cmake .. \
        -G"$TRAVIS_CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE \
        -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_PREFIX
    cmake --build . --config $TRAVIS_BUILD_TYPE --target install

    # Build and install idyntree
    cd $GIT_FOLDER
    rm -rf idyntree
    git clone --depth 1 -b $DEPS_BRANCH https://github.com/robotology/idyntree.git
    cd idyntree
    mkdir -p build && cd build
    cmake .. \
        -G"$TRAVIS_CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE \
        -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_PREFIX
    cmake --build . --config $TRAVIS_BUILD_TYPE --target install
fi

# Build and install wearables
cd $GIT_FOLDER
rm -rf wearables
git clone https://github.com/robotology/wearables.git
cd wearables
mkdir -p build && cd build
cmake -G"${TRAVIS_CMAKE_GENERATOR}" \
      -DCMAKE_BUILD_TYPE=${TRAVIS_BUILD_TYPE} \
      ..
cmake --build . --config ${TRAVIS_BUILD_TYPE} --target install

# Build and install osqp
cd $GIT_FOLDER
rm -rf osqp
git clone --recursive https://github.com/oxfordcontrol/osqp.git
cd osqp
mkdir -p build && cd build
cmake -G"${TRAVIS_CMAKE_GENERATOR}" \
      -DCMAKE_BUILD_TYPE=${TRAVIS_BUILD_TYPE} \
      -DUNITTESTS:BOOL=OFF \
      ..
cmake --build . --config ${TRAVIS_BUILD_TYPE} --target install

# Build and install osqp-Eigen
cd $GIT_FOLDER
rm -rf OsqpEigen
git clone https://github.com/robotology/osqp-eigen.git OsqpEigen
cd OsqpEigen
mkdir -p build && cd build
cmake -G"${TRAVIS_CMAKE_GENERATOR}" \
      -DCMAKE_BUILD_TYPE=${TRAVIS_BUILD_TYPE} \
      -DBUILD_TESTING:BOOL=OFF\
      ..
cmake --build . --config ${TRAVIS_BUILD_TYPE} --target install


