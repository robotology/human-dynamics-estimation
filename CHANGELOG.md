# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed
- Naming convention from `wrapper`\ `server` to `nws`\ `nwc` (https://github.com/robotology/human-dynamics-estimation/pull/367).
- HumanLogger device to be compatible with the `robot-log-visualizer` (https://github.com/robotology/human-dynamics-estimation/pull/372).

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
