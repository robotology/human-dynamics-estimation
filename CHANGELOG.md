# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
