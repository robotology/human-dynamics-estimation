# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Added exoskeleton wearable device (https://github.com/robotology/wearables/pull/81)
- Added xsens `.mvn` save option (https://github.com/robotology/wearables/pull/82)
- Added force-plate wearable device configuratio (https://github.com/robotology/wearables/pull/84)
- Added yarpdatadumper application (https://github.com/robotology/wearables/pull/87)

### Fixed 
- Fixed segmentation failt on `IWearRemapper` termination (https://github.com/robotology/wearables/issues/55).
- Plugin `.ini` files are now automatially generated (https://github.com/robotology/yarp/blob/2522239984ef3e6c26939fe740001ec08189a8cb/doc/release/v3_2_0.md#new-features)
- Change YARP required version to `3.2`
- Fix MacOS CI (https://github.com/robotology/wearables/pull/85)

## [1.0.0] - 2020-01-28

First release of `wearables`, compatible with YARP 3.3.
