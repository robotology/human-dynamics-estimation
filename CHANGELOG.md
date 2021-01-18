# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
