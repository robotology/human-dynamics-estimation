# Changelog
This file documents notable changes to this project done before November 2023. For changes after that date, plase refer to the release notes of each release at https://github.com/robotology/wearables/releases.

## [1.8.0] - 2023-11-17

### Changed
- Update iframetransform_to_iwear.xml to use new TF device (https://github.com/robotology/wearables/pull/202)

### Added
- CMake: Permit to explictly specify Python installation directory by setting the `WEARABLES_PYTHON_INSTALL_DIR` CMake variable (https://github.com/robotology/wearables/pull/197).

## [1.7.2] - 2023-09-06

### Fixed
- Fixed EMG normalization value in IWearWrapper and IWearLogger (https://github.com/robotology/wearables/pull/186).
- Fixed IWearLogger termination when `BufferManager` configuration fails (https://github.com/robotology/wearables/pull/195).

### Changed
- The license of the repo changed to [`BSD-3-Clause`](https://spdx.org/licenses/BSD-3-Clause.html) (https://github.com/robotology/wearables/pull/192).

## [1.7.1] - 2023-02-07

### Changed
- Use `std::mutex` instead of `std::recursive_mutex` in IWearRemapper (https://github.com/robotology/wearables/pull/174).

### Fixed
- Fixed management of the `period` parameter in `HapticGlove` device (https://github.com/robotology/wearables/pull/173). 

## [1.7.0] - 2022-09-28

### Changed
- The `IWearRemapper` can be attached also to IWear interfaces (https://github.com/robotology/wearables/pull/170)

### Removed
- Remove unused definition in actuators thrift messages(https://github.com/robotology/wearables/pull/171).

## [1.6.0] - 2022-09-21

### Changed
- The minimum required version of CMake is now 3.16 (https://github.com/robotology/wearables/pull/165).

### Added
- Add installation of WearableData thrift messages. (https://github.com/robotology/wearables/pull/166).

## [1.5.0] - 2022-09-13

### Added
- Add python bindings for the `WearablesData` msg. (https://github.com/robotology/wearables/pull/162)

### Changed
- `IWearLogger` compilation is enabled by the cmake flag `ENABLE_Logger`, that is set to `ON` by default if `robometry` is found. (https://github.com/robotology/wearables/pull/159)
- `ICub` wearable device compilation is enabled by the cmake flag `ENABLE_ICub`, that is set to `ON` by default if `iDynTree` is found. (https://github.com/robotology/wearables/pull/159)
- `ENABLE_FrameVisualizer` cmake falg is set to `ON` by default if `iDynTree` is found. (https://github.com/robotology/wearables/pull/159)
- `ENABLE_XsensSuit` cmake falg is set to `ON` by default if `XsensXME` is found. (https://github.com/robotology/wearables/pull/159)

## [1.4.0] - 2022-05-24

### Changed
- Fix deprecated `YARP` functions in `YarpUtilities` component (https://github.com/robotology/wearables/pull/152)
- Ported `iWearLogger` to the usage of `robometry` (new name of `YARP_telemetry`, https://github.com/robotology/wearables/pull/155)

### Added
- Added the possibility to change the `carrier` for `IWearRemapper` connection. (https://github.com/robotology/wearables/pull/147)

## [1.3.0] - 2022-02-25

### Added
- Added management of `ISkinSensor`. (https://github.com/robotology/wearables/pull/139)

### Changed
- Changed forceStrict to false when writing on ports. (https://github.com/robotology/wearables/pull/137)
- Disabled build of XsensSuit device by default. (https://github.com/robotology/wearables/pull/141)

## [1.2.2] - 2021-10-28

### Removed
- Remove `rpc` port from `IWearWrapper` and `IWearRemapper`. (https://github.com/robotology/wearables/pull/131)

### Added
- Added `IWearFrameVisualizer` module to visualize any wearable device frames associated with the link sensors. (https://github.com/robotology/wearables/pull/121)
- Added `HapticGlove` device to stream sense glove data and provide haptic feedback to the user. (https://github.com/robotology/wearables/pull/121)
- Updated `Paexo` wearable device with yarp ports for motor control. (https://github.com/robotology/wearables/pull/124)
- Updated 'iwear_logger' wrapper device with options to stream data through yarp ports. (https://github.com/robotology/wearables/pull/123)
- Added `iwear_logger` wrapper device to log wearable sensors data using `yarp-telemetry`. (https://github.com/robotology/wearables/pull/113)
- Added `AddInstallRPATHSupport` cmake module for linking shared objects of private dependencies. (https://github.com/robotology/wearables/pull/113)
- Updated `Paexo` wearable device with 6D force torque sensors implementation using `iFeelDriver`. (https://github.com/robotology/wearables/pull/117)
- Updated `Xsens` wearable device with `saveCurrentCalibration` option. (https://github.com/robotology/wearables/pull/120)
- Added `iFrameTransformToIWear` wearable device (https://github.com/robotology/wearables/pull/126)

## [1.2.1] - 2021-04-12

### Fixed
  - Fixed linking-related bug in the CMake build system that ignored the `LDFLAGS`  environment variable, breaking the build on some environments (https://github.com/robotology/wearables/pull/110).

## [1.2.0] - 2021-01-25

### Added
- Added the major feature of actuators related interfaces that is discussed in issue (https://github.com/robotology/wearables/issues/104). The related PR (https://github.com/robotology/wearables/pull/105) adds the following changes:
  - Implement wearable actuators related interfaces, and update `IWear` with actuators interfaces
  - Update all the available devices with implementation of wearable actuators interfaces
  - Add wearable actuator command thrift messages
  - Add `IWearActuatorsWrapper` device which provides yarp port to communicate wearable actuator commands
  - Update `Paexo` device with one wearable actuator motor implementation, and add a test application

## [1.1.1] - 2021-01-18

### Added
- Added documentation for compiling the project in the `README.md` (https://github.com/robotology/wearables/pull/96)
- Added configuratin file for `PRO.01` FT Shoes (https://github.com/robotology/wearables/pull/97)

### Changed
- FTShoes configuration files have been cleaned (https://github.com/robotology/wearables/pull/97)
- Cleanup: (https://github.com/robotology/wearables/pull/99)
    - Updated `yarp_prepare_plugin` header file path
    - Added cmake policy to handle `IWear` interface relative path warning

## [1.1.0] - 2020-11-24

### Added
- Added exoskeleton wearable device (https://github.com/robotology/wearables/pull/81)
- Added xsens `.mvn` save option (https://github.com/robotology/wearables/pull/82)
- Added force-plate wearable device configuration (https://github.com/robotology/wearables/pull/84)
- Added yarpdatadumper application (https://github.com/robotology/wearables/pull/87)
- Install `Wearable-devices-dumper` application (https://github.com/robotology/wearables/pull/92)
- Added project version to CMakeLists.txt

### Fixed
- Fixed segmentation fault on `IWearRemapper` termination (https://github.com/robotology/wearables/issues/55).
- Fix MacOS CI (https://github.com/robotology/wearables/pull/85)

### Changed
- Change YARP required version to `3.2`
- Travis CI has been replaced by Github Actions (https://github.com/robotology/wearables/pull/94)
- Plugin `.ini` files are now automatially generated (https://github.com/robotology/yarp/blob/2522239984ef3e6c26939fe740001ec08189a8cb/doc/release/v3_2_0.md#new-features)

## [1.0.0] - 2020-01-28

First release of `wearables`, compatible with YARP 3.3.
