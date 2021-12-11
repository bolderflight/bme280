# Changelog

## v5.0.0
- Merging Bme280 and Bme280-arduino
- Pulling in CMake tooling from the mcu-support repo
- Pulling in BST provided source files and wrapping C++ around them

## v2.0.1
- Put guards around digitalWriteFast to support microcontrollers that might not have that available
- Switched std::size_t to int to support microcontrollers without STL access

## v2.0.0
- Updated to match our [BME-280](https://github.com/bolderflight/bme280) library for flight software
- Updated license to MIT

## v1.0.2
- Updated license to GPLV3.

## v1.0.1
- Updating library.properties name.

## v1.0.0
- Modified to work with Arduino 1.5 format and creating a baseline release now.
