# Changelog

This file documents notable changes to this project done before November 2023. For changes after that date, please refer to the release notes of each release at https://github.com/robotology/human-dynamics-estimation/releases.

## [3.0.0] - 2023-11-09

### Changed
- Stream and visualize human effort data in HumanStateVisualizer (https://github.com/robotology/human-dynamics-estimation/pull/385).
- Naming convention from `wrapper`\ `server` to `nws`\ `nwc` (https://github.com/robotology/human-dynamics-estimation/pull/367).

| Previous devices name | New devices name |
|:------------:|:--------------:|
| HumanDynamicsWrapper | HumanDynamics_nws_yarp |
| HumanDynamicsRemapper | HumanDynamics_nwc_yarp |
| HumanStateWrapper | HumanState_nws_yarp |
| HumanStateRemapper | HumanState_nwc_yarp |
| HumanWrenchWrapper | HumanWrench_nws_yarp |
| HumanWrenchRemapper | HumanWrench_nwc_yarp |
| WearableTargetsWrapper | WearableTargets_nws_yarp |
| WearableTargetsRemapper | WearableTargets_nwc_yarp |

- HumanLogger device to be compatible with the `robot-log-visualizer` (https://github.com/robotology/human-dynamics-estimation/pull/372).
- `controlboardwrapper2` is replaced by `controlBoard_nws_yarp` (https://github.com/robotology/human-dynamics-estimation/pull/378).
- Updated applications and modules that use the transform server (https://github.com/robotology/human-dynamics-estimation/pull/377)

### Added
- Added files for ergoCub teleoperation without using iFeel (https://github.com/robotology/human-dynamics-estimation/pull/374)

## [2.9.0] - 2023-10-17

### Fixed
- [`HumanControlBoard`] Fix attach method (https://github.com/robotology/human-dynamics-estimation/pull/365).

### Added
- Added `HumanWrenchWrapper` and `HumanWrenchRemapper` (https://github.com/robotology/human-dynamics-estimation/pull/362/files).
- Added `IHumanWrench` to `HumanLogger` (https://github.com/robotology/human-dynamics-estimation/pull/363).

## [2.8.0] - 2023-09-06

### Added
- [`HumanStateVisualizer`] Added custom options for window dimensions (https://github.com/robotology/human-dynamics-estimation/pull/341).
- [`HumanStateProvider`] Added a parameter that allows to set custom angles for calibration purposes (https://github.com/robotology/human-dynamics-estimation/pull/341).
- Added configuration files for ergoCub robot. (https://github.com/robotology/human-dynamics-estimation/pull/340)

### Changed
- The license of the repo changed to [`BSD-3-Clause`](https://spdx.org/licenses/BSD-3-Clause.html) (https://github.com/robotology/human-dynamics-estimation/pull/350).
- Devices wait for data available in IHumanState and IHumanDynamics interfaces instead of returning error (https://github.com/robotology/human-dynamics-estimation/pull/357).

### Fixed
- [`HumanLogger`] Removed IWrapper from implemented interface to fix checks on log flags and attached interfaces (https://github.com/robotology/human-dynamics-estimation/pull/344)
- [`HumanLogger`] Fix joint names (https://github.com/robotology/human-dynamics-estimation/pull/352)
- Removed double IWrapper and IMultiWrapper implementation from all devices (https://github.com/robotology/human-dynamics-estimation/pull/347) 


## [2.7.1] - 2023-02-16

### Removed
- Remove deprecated models. (https://github.com/robotology/human-dynamics-estimation/pull/322)

### Changed
- Dependency on robometry is changed to optional. (https://github.com/robotology/human-dynamics-estimation/pull/323)

### Fixed
- Fixed a typo in `DynamicalInverseKinematics` that caused a wrong calculation of the rotation mean error. (https://github.com/robotology/human-dynamics-estimation/pull/330)
- Fixed `HumanLogger` when logging just one kind of data (https://github.com/robotology/human-dynamics-estimation/pull/338)

## [2.7.0] - 2020-10-20

First release with CHANGELOG.

### Added
- Added human logger device for storing human data as .mat (https://github.com/robotology/human-dynamics-estimation/pull/298)
