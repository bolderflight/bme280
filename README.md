[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# Bme280
This library communicates with the [BME280](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/) environmental sensor and is compatible with Arduino and CMake build systems.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Description
The Bosch Sensortec [BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) is an integrated environmental sensor, which combines high linearity, high accuracy sensors for pressure, temperature, and humidity in a compact LGA package. The pressure sensor is an absolute barometric pressure sensor with features exceptionally high accuracy and resolution at very low noise. The integrated temperature sensor has been optimized for very low noise and high resolution. Pressure, temperature, and humidity measurements can be useful for applications involving unmanned vehicles (indicated and true airspeed, altitude, and density altitude), indoor navigation (floor detection), outdoor navigation (altitudes and airspeeds, dead-reckoning, GPS time to first fix improvements) as well as weather monitoring and home automation.

The [BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) samples pressure and temperature to 20 bit resolution and humidity to 16 bit resolution. The [BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) features programmable oversampling, filtering, and standby time between samples. The BME280 supports both I2C and SPI communication.

# Usage
This library supports both I2C and SPI commmunication with the BME280 and supports collecting pressure, temperature, and humidity data. The BME280 temperature data should be treated as die temperature, and is labled as such within this library.

# Installation

## Arduino
Simply clone or download this library into your Arduino/libraries folder. The library is added as:

```C++
#include "bme280.h"
```

Example Arduino executables are located in: *examples/arduino/*, see the Examples list for a complete listing and description. Teensy 3.x, 4.x, and LC devices are used for testing under Arduino and this library should be compatible with other Arduino devices.

## CMake
CMake is used to build this library, which is exported as a library target called *bme280*. The header is added as:

```C++
#include "bme280.h"
```

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and example executables called *i2c_example* and *spi_example*. The example executable source files are located at *examples/cmake/i2c.cc* and *examples/cmake/spi.cc*. Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64
   * IMXRT1062_T40
   * IMXRT1062_T41
   * IMXRT1062_MMOD

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The example targets create executables for communicating with the sensor using I2C or SPI communication. Each target also has a *_hex*, for creating the hex file to upload to the microcontroller, and an *_upload* for using the [Teensy CLI Uploader](https://www.pjrc.com/teensy/loader_cli.html) to flash the Teensy. Please note that the CMake build tooling is expected to be run under Linux or WSL, instructions for setting up your build environment can be found in our [build-tools repo](https://github.com/bolderflight/build-tools).

# Namespace
This library is within the namespace *bfs*.

# Bme280

## Methods

**Bme280()** Default constructor, requires calling the Config method to setup the I2C or SPI bus and I2C address or SPI chip select pin.

**Bme280(TwoWire &ast;i2c, const I2cAddr addr)** Creates a Bme280 object. This constructor is used for the I2C communication interface. A pointer to the I2C bus object is passed along with an enum of the I2C address of the sensor. Use I2C_ADDR_PRIM if the SDO pin is grounded and I2C_ADDR_SEC if the SDO pin is pulled high.

```C++
bfs::Bme280 bme280(&Wire, bfs::Bme280::I2C_ADDR_PRIM);
```

**Bme280(SPIClass &ast;spi, const uint8_t cs)** Creates a Bme280 object. This constructor is used for the SPI communication interface. A pointer to the SPI bus object is passed along with the chip select pin of the sensor. Any pin capable of digital I/O can be used as a chip select pin.

```C++
bfs::Bme280 bme280(&SPI, 2);
```

**void Config(TwoWire &ast;bus, const I2cAddr addr)** This is required when using the default constructor and sets up the I2C bus and I2C address.

**void Config(SPIClass &ast;spi, const uint8_t cs)** This is required when using the default constructor and sets up the SPI bus and chip select pin.

**bool Begin()** Initializes communication with the sensor and configures the default sampling rates, oversampling and low pass filter settings. True is returned if communication is able to be established with the sensor and configuration completes successfully, otherwise, false is returned. The communication bus is not initialized within this library and must be initialized seperately; this enhances compatibility with other sensors that may on the same bus.

```C++
Wire.begin();
Wire.setClock(400000);
bool status = bme280.Begin();
if (!status) {
  // ERROR
}
```

The BME280 features programmable oversampling, filtering, and standby time between samples. By default, these are set to the following values:

| Settings        |                                |
| ---             | ---                            |
| Oversampling    | pressure x 16, temperature x 2, humidity x 1 |
| IIR Filter Coefficient | 16                             |
| Standby Time    | 0.5 ms                         |

| Performance      |         |
| ---              | ---     |
| Data Output Rate | 25 Hz   |
| Filter Bandwidth | 0.53 Hz |
| Response Time    | 0.88 s  |

Optionally, the *ConfigPresOversampling*, *ConfigTempOversampling*, *ConfigHumidityOversampling*, *ConfigFilterCoef* and *ConfigStandbyTime* functions can be used, following *Begin*, to change these settings from their default values. For much more information on the settings and performance implications, please refer to the [BME280 datasheet](docs/BME280-Datasheet.pdf).

**bool ConfigTempOversampling(const Oversampling oversampling)** Sets the temperature oversampling value. The following enumerated oversampling settings are supported:

| Oversampling Name | Oversampling Value |
| ---               | ---                |
| OVERSAMPLING_1X    | 1                  |
| OVERSAMPLING_2X    | 2                  |
| OVERSAMPLING_4X    | 4                  |
| OVERSAMPLING_8X    | 8                  |
| OVERSAMPLING_16X   | 16                 |

True is returned on successfully updating the BME280 configuration, otherwise false is returned.

```C++
bool status = bme280.ConfigTempOversampling(bfs::Bme280::OVERSAMPLING_16X);
if (!status) {
  // ERROR
}
```

**Oversampling temp_oversampling()** Returns the current temperature oversampling value.

**bool ConfigPresOversampling(const Oversampling oversampling)** Sets the pressure oversampling value. The use of this function is identical to *ConfigTempOversampling*.

**Oversampling pres_oversampling()** Returns the current pressure oversampling value.

**bool ConfigHumidityOversampling(const Oversampling oversampling)** Sets the humidity oversampling value. The use of this function is identical to *ConfigTempOversampling*.

**Oversampling humidity_oversampling()** Returns the current humidity oversampling value.

**bool ConfigFilterCoef(const FilterCoef val)** Sets the digital low pass filter IIR coefficient. This filter is applied to all measurements. The filter is given by the following equation:

*data_filtered = (data_filtered_old &ast; (filter_coefficient - 1) + data) / filter_coefficient*

The following enumerated filter coefficients are supported:

| Filter Coefficient Name     | Filter Coefficient Value     | Samples to reach 75% of step response |
| ---                         | ---                          | ---                                   |
| FILTER_COEF_OFF             | 1                            | 1                                     |
| FILTER_COEF_2               | 2                            | 2                                     |
| FILTER_COEF_4               | 4                            | 5                                     |
| FILTER_COEF_8               | 8                            | 11                                    |
| FILTER_COEF_16              | 16                           | 22                                    |

True is returned on successfully updating the BME280 configuration, otherwise false is returned.

```C++
bool status = bme280.ConfigFilterCoef(bfs::Bme280::FILTER_COEF_16);
if (!status) {
  // ERROR
}
```

**FilterCoef filter_coef()** Returns the current filter coefficient value.

**bool ConfigStandbyTime(const StandbyTime standby)** Sets the standby time, which is applied to all measurements. This is the time that the sensor spends idle between taking measurements. 

The following enumerated standby times are supported:

| Standby Time Name | Standby Time  |
| ---               | ---           |
| STANDBY_TIME_0_5_MS    | 0.5 ms        |
| STANDBY_TIME_10_MS     | 10 ms         |
| STANDBY_TIME_20_MS     | 20 ms         |
| STANDBY_TIME_62_5_MS   | 62.5 ms       |
| STANDBY_TIME_125_MS    | 125 ms        |
| STANDBY_TIME_250_MS    | 250 ms        |
| STANDBY_TIME_500_MS    | 500 ms        |
| STANDBY_TIME_1000_MS   | 1000 ms       |

True is returned on successfully updating the BME280 configuration, otherwise false is returned.

```C++
bool status = bme280.ConfigStandbyTime(bfs::Bme280::STANDBY_TIME_0_5_MS);
if (!status) {
  // ERROR
}
```

**StandbyTime standby_time()** Returns the current standby time value.

**bool Reset()** Performs a soft reset of the BME280. True is returned on success.

**bool Read()** Reads data from the BME280 and stores the data in the Bme280 object. Returns true if data is successfully read, otherwise, returns false.

```C++
/* Read the sensor data */
if (bme280.Read()) {
}
```

**float pres_pa()** Returns the pressure data from the Bme280 object in units of Pa.

```C++
float pressure = bme280.pres_pa();
```

**float die_temp_c** Returns the die temperature of the sensor from the Bme280 object in units of degrees C.

```C++
float temperature = bme280.die_temp_c();
```

**float humidity_rh** Returns the humidity from the Bme280 object in units of %RH.

```C++
float temperature = bme280.humidity_rh();
```

## Example List
* **i2c**: demonstrates declaring a *Bme280* object, initializing the sensor, and collecting data. I2C is used to communicate with the BME-280 sensor.
* **spi**: demonstrates declaring a *Bme280* object, initializing the sensor, and collecting data. SPI is used to communicate with the BME-280 sensor.

# Wiring and Pullups 
Please refer to the [BME280 datasheet](docs/BME280-Datasheet.pdf) and your microcontroller's pinout diagram. This library was developed using the [Adafruit Breakout Board](https://www.adafruit.com/products/2652). This library should work well for other breakout boards or embedded sensors, please refer to your vendor's pinout diagram.

## I2C

The BME280 pins should be connected as:
   * VDD: this should be a 1.7V to 3.6V power source. The Adafruit breakout includes voltage regulation enabling a 3-5V range.
   * GND: ground.
   * VDDIO: digital I/O supply voltage. This should be between 1.2V and 3.6V. The Adafruit breakout board connects VDDIO so this is not broken out to a pin.
   * SDI: connect to SDA.
   * SCK: connect to SCL.
   * SDO: ground to select I2C address 0x76. Pull high to VDD to select I2C address 0x77.
   * CSB: connect to VDD.

2.2 kOhm resistors should be used as pullups on SDA and SCL, these resistors should pullup with a 3.3V source.

## SPI

The BME280 pins should be connected as:
   * VDD: this should be a 1.7V to 3.6V power source. The Adafruit breakout includes voltage regulation enabling a 3-5V range.
   * GND: ground.
   * VDDIO: digital I/O supply voltage. This should be between 1.2V and 3.6V. The Adafruit breakout board connects VDDIO so this is not broken out to a pin.
   * SDI: connect to MOSI.
   * SCK: connect to SCK.
   * SDO: connect to MISO.
   * CSB: connect to chip select pin. Pin 10 was used in the code snippets in this document and the included examples, but any digital I/O pin can be used.
