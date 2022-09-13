# Wearables

[![C++ CI Action Status](https://github.com/robotology/wearables/workflows/C++%20CI%20Workflow/badge.svg)](https://github.com/robotology/wearables/actions/workflows/ci.yml)

# Setting up
## Dependencies
Here following there is a list of the required dependencies you need for using this repository. Instead, the optional dependencies, specific for each wearable-device, are listed in [wearable-device-sources](#wearable-device-sources).

### Required
* [CMake](https://cmake.org/download/) open-source, cross-platform family of tools designed to build, test and package software.
* [YARP](http://www.yarp.it/) software package to handle the data comunication.

It must be noted that `YARP`, together with a subset of the optional dependencies, can be installed together in one place using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild) enabling its [`Human-Dynamics` profile](https://github.com/robotology/robotology-superbuild#human-dynamics).

### Optional
* [robometry](https://github.com/robotology/robometry) device: if present, the `IWearLogger` device is installed by default.
* [iDynTree](https://github.com/robotology/idyntree): if present, the `IWearFrameVisualizer` module and `ICub` device are installed by default.
* [pybind11](https://github.com/pybind/pybind11): if present, the user can enable `WEARABLES_COMPILE_PYTHON_BINDINGS` to enable the compilation of the python bindings.

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

### Optional flags
The compilation of some of the software components in `wearables` can be enabled using the following cmake flags:
- `ENABLE_FrameVisualizer`: enable the compilation of the `IWearFrameVisualizer` module that allows to visualize wearble inertial measurements.
- `ENABLE_Logger`: enable the compilation of the `IWearLogger` device that allows to save wearable data as `.mat` or stream the data on YARP as analog vector data.
- `ENABLE_<device>`: enable the compilation of the optional wearable device (see documentation in [Wearable device sources](#wearable-device-sources)).


# Wearable device sources
Wearable devices contained in [`/devices/`](/devices) are [YARP devices](http://www.yarp.it/git-master/note_devices.html) that exposes sensors data using `IWear` interface. List of the dependencies and documentation for running some of the wearable data sources can be found at the links in the table below. The compilation of some of theese devices can be enable/disabled changing the CMAKE option `ENABLE_<device>`.

| Device Name | Description | OS | Dependencies| Documentation |
|---------------|------|---------------|----------------------------------------------------------|------|
| IAnalogSensor | Exposes YARP [`IAnalogSensor` Interface](http://www.yarp.it/git-master/classyarp_1_1dev_1_1IAnalogSensor.html). |  Linux/MacOS/Windows  | - |  - |
| FTShoes | Exposes YARP [`ftShoe`](https://github.com/robotology/forcetorque-yarp-devices/tree/master/ftShoe) data. |  Linux/MacOS/Windows   | [`forcetorque-yarp-devices`](https://github.com/robotology/forcetorque-yarp-devices) | [:books:](/doc/How-to-run-FTshoes.md) |
| XsensSuit | Exposes [`XsensSuit`](https://www.xsens.com/motion-capture) data. |  Windows   | xsens MVN SDK 2018.0.3 | [:books:](/doc/How-to-run-XsensSuit.md) |
| ICub | Exposes [iCub](https://icub.iit.it/) robot data. |  Linux/MacOS/Windows   | [`iDynTree`](https://github.com/robotology/idyntree) | [:books:](/doc/How-to-run-iCub-as-wearable-source.md) |
| Paexo | Exposes [Paexo](https://paexo.com/?lang=en) data. |  Linux  | `iFeelDriver` (Contact the maintainer for more details) | - |
| HapticGlove | Exposes [SenseGlove](https://www.senseglove.com/product/developers-kit/) data. |  Linux/Windows  | [`SenseGloveSDK`](https://github.com/Adjuvo/SenseGlove-API) | [:books:](./doc/How-to-compile-and-run-HapticGlove.md) |
| IFrameTransform | Exposes YARP [`IFrameTransform` Interface](http://www.yarp.it/git-master/classyarp_1_1dev_1_1IFrameTransform.html). |  Linux/MacOS/Windows  | - |  - |
