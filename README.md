# bme280-arduino
This library communicates with the [BME280](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/) environmental sensor and is built for use with the Arduino IDE.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)

# Description
The Bosch Sensortec [BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) is an integrated environmental sensor, which combines high linearity, high accuracy sensors for pressure, temperature, and humidity in a compact LGA package. The pressure sensor is an absolute barometric pressure sensor with features exceptionally high accuracy and resolution at very low noise. The integrated temperature sensor has been optimized for very low noise and high resolution. Pressure, temperature, and humidity measurements can be useful for applications involving unmanned vehicles (indicated and true airspeed, altitude, and density altitude), indoor navigation (floor detection), outdoor navigation (altitudes and airspeeds, dead-reckoning, GPS time to first fix improvements) as well as weather monitoring and home automation.

The [BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) samples pressure and temperature to 20 bit resolution and humidity to 16 bit resolution. The [BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) features programmable oversampling, IIR filtering, and standby time between samples. The BME280 supports both I2C and SPI communication.

# Usage
This library supports both I2C and SPI commmunication with the BME280 and supports collecting pressure and temperature data. Although humidity data is available on the sensor, it is not supported by this library - humidity data is returned as relative humidity and is of relatively limited use for aerospace applications. Also limiting output to pressure and temperature improves interoperability with other pressure sensors with similar output measurements (i.e. the BMP280, BMP380, AMS5812, AMS5915, AMS6915, ...).

The BME280 temperature data should be treated as die temperature, and is labled as such within this library.

## Installation
Simply clone or download this library into your Arduino/libraries folder. The library is added as:

```C++
#include "bme280.h"
```

## Methods

**Bme280(i2c_t3 &ast;bus, uint8_t addr)** Creates a Bme280 object. This constructor is used for the I2C communication interface. A pointer to the I2C bus object is passed along with the I2C address of the sensor. The address will be 0x76 if the SDO pin is grounded and 0x77 if the SDO pin is pulled high.

```C++
Bme280 bme280(&Wire, 0x68);
```

**Bme280(SPIClass &ast;bus, uint8_t cs)** Creates a Bme280 object. This constructor is used for the SPI communication interface. A pointer to the SPI bus object is passed along with the chip select pin of the sensor. Any pin capable of digital I/O can be used as a chip select pin.

```C++
Bme280 bme280(&SPI, 2);
```

**bool Begin()** Initializes communication with the sensor and configures the default sampling rates, oversampling and low pass filter settings. True is returned if communication is able to be established with the sensor and configuration completes successfully, otherwise, false is returned.

```C++
bool status = bme280.Begin();
if (!status) {
  // ERROR
}
```

The BME280 features programmable oversampling, IIR filtering, and standby time between samples. By default, these are set to the following values:

| Settings        |                                |
| ---             | ---                            |
| Oversampling    | pressure x 16, temperature x 2 |
| IIR Coefficient | 16                             |
| Standby Time    | 0.5 ms                         |

| Performance      |         |
| ---              | ---     |
| Data Output Rate | 25 Hz   |
| Filter Bandwidth | 0.53 Hz |
| Response Time    | 0.88 s  |

Optionally, the *ConfigPresOversampling*, *ConfigTempOversampling*, *ConfigIir* and *ConfigStandbyTime* functions can be used, following *Begin*, to change these settings from their default values. For much more information on the settings and performance implications, please refer to the [BME280 datasheet](https://github.com/bolderflight/bme280-arduino/blob/main/docs/BME280-Datasheet.pdf).

**bool ConfigTempOversampling(const Oversampling oversampling)** Sets the temperature oversampling value. The following enumerated oversampling settings are supported:

| Oversampling Name | Oversampling Value |
| ---               | ---                |
| OVERSAMPLING_1    | 1                  |
| OVERSAMPLING_2    | 2                  |
| OVERSAMPLING_4    | 4                  |
| OVERSAMPLING_8    | 8                  |
| OVERSAMPLING_16   | 16                 |

True is returned on successfully updating the BME280 configuration, otherwise false is returned.

```C++
bool status = bme280.ConfigTempOversampling(Bme280::OVERSAMPLING_16);
if (!status) {
  // ERROR
}
```

**Oversampling temp_oversampling()** Returns the current temperature oversampling value.

**bool ConfigPresOversampling(const Oversampling oversampling)** Sets the pressure oversampling value. The use of this function is identical to *ConfigTempOversampling*.

**Oversampling pres_oversampling()** Returns the current pressure oversampling value.

**bool ConfigIir(const IirCoefficient iir)** Sets the digital low pass filter IIR coefficient. This filter is applied to all measurements. The filter is given by the following equation:

*data_filtered = (data_filtered_old &ast; (filter_coefficient - 1) + data) / filter_coefficient*

The following enumerated filter coefficients are supported:

| IIR Filter Coefficient Name | IIR Filter Coefficient Value | Samples to reach 75% of step response |
| ---                         | ---                          | ---                                   |
| IIRC_OFF                    | 1                            | 1                                     |
| IIRC_2                      | 2                            | 2                                     |
| IIRC_4                      | 4                            | 5                                     |
| IIRC_8                      | 8                            | 11                                    |
| IIRC_16                     | 16                           | 22                                    |

True is returned on successfully updating the BME280 configuration, otherwise false is returned.

```C++
bool status = bme280.ConfigIir(Bme280::IIRC_16);
if (!status) {
  // ERROR
}
```

**IirCoefficient iir()** Returns the current IIR coefficient value.

**bool ConfigStandbyTime(const StandbyTime standby)** Sets the standby time, which is applied to all measurements. This is the time that the sensor spends idle between taking measurements. 

The following enumerated standby times are supported:

| Standby Time Name | Standby Time  |
| ---               | ---           |
| STANDBY_0_5_MS    | 0.5 ms        |
| STANDBY_10_MS     | 10 ms         |
| STANDBY_20_MS     | 20 ms         |
| STANDBY_62_5_MS   | 62.5 ms       |
| STANDBY_125_MS    | 125 ms        |
| STANDBY_250_MS    | 250 ms        |
| STANDBY_500_MS    | 500 ms        |
| STANDBY_1000_MS   | 1000 ms       |

True is returned on successfully updating the BME280 configuration, otherwise false is returned.

```C++
bool status = bme280.ConfigStandbyTime(Bme280::STANDBY_0_5_MS);
if (!status) {
  // ERROR
}
```

**StandbyTime standby_time()** Returns the current standby time value.

**bool Read()** Reads data from the BME280 and stores the data in the Bme280 object. Returns true if data is successfully read, otherwise, returns false.

```C++
/* Read the sensor data */
if (bme280.Read()) {
}
```

**float pressure_pa()** Returns the pressure data from the Bme280 object in units of Pa.

```C++
float pressure = bme280.pressure_pa();
```

**float die_temperature_c** Returns the die temperature of the sensor from the Bme280 object in units of degrees C.

```C++
float temperature = bme280.die_temperature_c();
```

## Example List
* **Basic_I2C**: demonstrates declaring a *Bme280* object, initializing the sensor, and collecting data. I2C is used to communicate with the BME-280 sensor.
* **Basic_SPI**: demonstrates declaring a *Bme280* object, initializing the sensor, and collecting data. SPI is used to communicate with the BME-280 sensor.

# Wiring and Pullups 
Please refer to the [BME280 datasheet](https://github.com/bolderflight/bme280-arduino/blob/main/docs/BME280-Datasheet.pdf) and your microcontroller's pinout diagram. This library was developed using the [Adafruit Breakout Board](https://www.adafruit.com/products/2652). This library should work well for other breakout boards or embedded sensors, please refer to your vendor's pinout diagram.

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
