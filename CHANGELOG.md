# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- [`HumanStateVisualizer`] Added custom options for window dimensions (https://github.com/robotology/human-dynamics-estimation/pull/341).
- [`HumanStateProvider`] Added a parameter that allows to set custom angles for calibration purposes (https://github.com/robotology/human-dynamics-estimation/pull/341).
- Added configuration files for ergoCub robot. (https://github.com/robotology/human-dynamics-estimation/pull/340)

### Changed
- The license of the repo changed to [`BSD-3-Clause`](https://spdx.org/licenses/BSD-3-Clause.html) (https://github.com/robotology/human-dynamics-estimation/pull/350).

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
