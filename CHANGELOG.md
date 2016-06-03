# Ardusat SDK Changelog
This logs all changes to released SDK versions

## [0.1.0] - 2016-06-24
### Rewrote interface to be object oriented
- Created sensor classes for each type of sensor
- Added support for ISL29125, TCS34725, and the ML8511 on the spaceboard
- Exposed reasonable advanced configuration parameters for all sensors
- Added autodetection logic to check if SpaceBoard or Space Kit is being used
- Added doxygen comments

## [0.0.6] - 2016-03-14
### Fixed compatibility with newer (e.g. 1.6.8) versions of the Arduino IDE
- `SoftwareSerial` library shipped with newer IDE has changed, so fixed our local reference copy of
the library to include the older version for compatibility.

## [0.0.5] - 2015-08-11
### Added Codebender compatibility and bug fixes
- Fixed reference to `lroundf` not found in Codebender's old stdlib
- Fixed L3GD20 `sensitivity` type bug

## [0.0.4] - 2015-05-29
### Added checksums & updated examples
- Addded checksum values to JSON & CSV output formats
- Updated config options for IMU sensitivity/range options (default to gyro 2500
  DPS & 16G acceleration)
- Added magnetometer example sketch

## [0.0.3] - 2015-04-23
### Fixed dashes in filenames for new LSM303 drivers

## [0.0.2] - 2015-04-15
### Added Lemsens compatibility to SDK
- Changed LSM303 driver to support LSM303DTR and DHLC
- Optional I2C address changes to support Lemsens board 

## [0.0.1] - 2015-02-27
### Added Changelog
