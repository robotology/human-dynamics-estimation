#!/bin/sh

set -e

#if OS is linux or is not set
if [ "$TRAVIS_OS_NAME" = linux -o -z "$TRAVIS_OS_NAME" ]; then
    sudo sh -c 'echo "deb http://www.icub.org/ubuntu trusty contrib/science" > /etc/apt/sources.list.d/icub.list'
    sudo apt-get update
    sudo apt-get -y --force-yes install -qq libeigen3-dev icub-common yarp  
    if [ "$TRAVIS_BUILD_DOCS" ]; then
        sudo apt-get install doxygen doxygen-doc doxygen-gui graphviz
    fi
elif [ "$TRAVIS_OS_NAME" = osx ]; then
    gem install xcpretty
    brew update &> /dev/null
    brew tap robotology/cask
    brew tap homebrew/x11
    brew tap homebrew/science
    brew install eigen ipopt yarp
fi
