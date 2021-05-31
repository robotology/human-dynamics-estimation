# Wearables

[![C++ CI Action Status](https://github.com/robotology/wearables/workflows/C++%20CI%20Workflow/badge.svg)](https://github.com/robotology/wearables/actions/workflows/ci.yml)

# Setting up
## Dependencies
Here following there is a list of the required dependencies you need for using this repository. Instead, the optional dependencies, specific for each wearable-device, are listed in [wearable-device-sources](#wearable-device-sources).

### Required
* [CMake](https://cmake.org/download/) open-source, cross-platform family of tools designed to build, test and package software.
* [YARP](http://www.yarp.it/) software package to handle the data comunication.

It must be noted that `YARP`, together with a subset of the optional dependencies, can be installed together in one place using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild) enabling its [`Human-Dynamics` profile](https://github.com/robotology/robotology-superbuild#human-dynamics).

## Build the suite
### Linux/macOS

```sh
git clone https://github.com/robotology/wearables.git
cd wearables
mkdir build && cd build
cmake ../
make
[sudo] make install
```
Notice: `sudo` is not necessary if you specify the `CMAKE_INSTALL_PREFIX`. In this case it is necessary to add in the `.bashrc` or `.bash_profile` the following lines:
``` sh
export WEARABLES_INSTALL_DIR=/path/where/you/installed
export YARP_DATA_DIRS=${YARP_DATA_DIRS}:${WEARABLES_INSTALL_DIR}/share/yarp
```
Note that this is not necessary if you install `wearables` via the `robotology-superbuild`.


# Wearable device sources
Wearable devices contained in [`/devices/`](/devices) are [YARP devices](http://www.yarp.it/git-master/note_devices.html) that exposes sensors data using `IWear` interface. List of the dependencies and documentation for running some of the wearable data sources can be found at the links in the table below. The compilation of some of theese devices can be enable/disabled changing the CMAKE option `ENABLE_<device>`.

| Device Name | Description | OS | Dependencies| Documentation |
|---------------|------|---------------|----------------------------------------------------------|------|
| IAnalogSensor | Exposes YARP [`IAnalogSensor` Interface](http://www.yarp.it/git-master/classyarp_1_1dev_1_1IAnalogSensor.html). |  Linux/MacOS/Windows  | - |  - |
| FTShoes | Exposes YARP [`ftShoe`](https://github.com/robotology/forcetorque-yarp-devices/tree/master/ftShoe) data. |  Linux/MacOS/Windows   | [forcetorque-yarp-devices](https://github.com/robotology/forcetorque-yarp-devices) | [:books:](/doc/How-to-run-FTshoes.md) |
| XsensSuit | Exposes [XsensSuit](https://www.xsens.com/motion-capture) data. |  Windows   | xsens MVN SDK 2018.0.3 | [:books:](/doc/How-to-run-XsensSuit.md) |
| iCub | Exposes [iCub](https://icub.iit.it/) robot data. |  Linux/MacOS/Windows   | [idyntree](https://github.com/robotology/idyntree) | [:books:](/doc/How-to-run-iCub-as-wearable-source.md) |
| Paexo | Exposes [Paexo](https://paexo.com/?lang=en) data. |  Linux  | Optional flag `ENABLE_PAEXO_USE_iFEELDriver` to use `iFeelDriver` (Contact the maintainer for more details) | - |
