#!/bin/bash

set -ev

case $TRAVIS_OS_NAME in
    linux)
        apt-get update
        apt-get install -y --no-install-recommends \
            libgsl-dev \
            coinor-libipopt1v5 \
            coinor-libipopt-dev \
            libmatio2 \
            libmatio-dev
        rm -rf /var/lib/apt/lists/*
    ;;
    osx)
        brew install ccache
        export PATH="/usr/local/opt/ccache/libexec:$PATH"
        gem install xcpretty
        brew update &> /dev/null
        brew tap robotology/cask
        brew tap homebrew/x11
        brew tap homebrew/science
        brew install eigen ipopt yarp
    ;;
    *) exit 1
    ;;
esac
