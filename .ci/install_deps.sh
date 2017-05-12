#!/bin/bash

case $TRAVIS_OS_NAME in
    linux)
        apt-get update
        apt-get install -y --no-install-recommends \
            libgsl-dev \
            coinor-libipopt1v5 \
            coinor-libipopt-dev \
            libmatio2 \
            libmatio-dev \
            qtbase5-dev \
            libqt5xmlpatterns5-dev
        rm -rf /var/lib/apt/lists/*
    ;;
    osx)
        brew install ccache
        export PATH="/usr/local/opt/ccache/libexec:$PATH"
        brew update &> /dev/null
        brew tap robotology/cask
        brew tap homebrew/science
        brew install eigen ipopt yarp qt
        export PATH="/usr/local/opt/qt/bin/:$PATH"
    ;;
    *) exit 1
    ;;
esac
