#!/bin/sh
set -e

brew tap robotology/formulae
brew install eigen dartsim/dart/ipopt robotology/formulae/yarp qt
export PATH="/usr/local/opt/qt/bin/:$PATH"
