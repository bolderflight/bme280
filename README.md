# BME280
Library for communicating with the [BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) environmental sensor using Teensy 3.x and Teensy LC devices.

# Description
The Bosch Sensortec [BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) is an integrated environmental sensor, which combines high linearity, high accuracy sensors for pressure, temperature, and humidity in a compact LGA package. The humidity sensor features an extremely fast response time and high accuracy over a wide temperature range. The pressure sensor is an absolute barometric pressure sensor with features exceptionally high accuracy and resolution at very low noise. The integrated temperature sensor has been optimized for very low noise and high resolution. Pressure, temperature, and humidity measurements can be useful for applications involving unmanned vehicles (indicated and true airspeed, altitude, and density altitude), indoor navigation (floor detection), outdoor navigation (altitudes and airspeeds, dead-reckoning, GPS time to first fix improvements) as well as weather monitoring and home automation.

The [BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) samples pressure and temperature to 20 bit resolution and humidity to 16 bit resolution. The [BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) features programmable oversampling, IIR filtering, and standby time between samples. The BME280 supports both I2C and SPI communication.

# Usage
This library supports both I2C and SPI commmunication with the BME280. The [i2c_t3 enhanced I2C library](https://github.com/nox771/i2c_t3) for Teensy 3.x/LC devices is used for I2C communication.

## Installation
Simply clone or download this library into your Arduino/libraries folder. The [i2c_t3 enhanced I2C library](https://github.com/nox771/i2c_t3) is bundled with the [Teensyduino software](http://pjrc.com/teensy/td_download.html) and is not required to download separately. [Teensyduino](http://pjrc.com/teensy/td_download.html) version 1.37 or newer is required.

## Function Description
This library supports both I2C and SPI communication with the BME280. The *BME280* object declaration is overloaded with different declarations for I2C and SPI communication. All other functions remain the same. 

### I2C Object Declaration

**BME280(uint8_t address, uint8_t bus)**
An BME280 object should be declared, specifying the BME280 I2C address and the Teensy I2C bus number. The BME280 I2C address will be 0x76 if the SDO pin is grounded or 0x77 if the SDO pin is pulled high. For example, the following code declares a BME280 object called *bme* with a BME280 sensor located on Teensy I2C bus 0 (pins 18 and 19) with a sensor address of 0x76 (SDO grounded).

```C++
BME280 bme(0x76, 0);
```

**BME280(uint8_t address, uint8_t bus, i2c_pins pins)**
Optionally, the I2C pins can be specified, which is useful for accessing the [alternate I2C pins](https://github.com/nox771/i2c_t3/#pins) for a given bus. If these aren't specified, this library uses the default pins for a given I2C bus. In this case, a BME280 object should be declared, specifying the BME280 I2C address, the Teensy I2C bus number, and the I2C pins. The BME280 I2C address will be 0x76 if the SDO pin is grounded or 0x77 if the SDO pin is pulled high. For example, the following code declares a BME280 object called *bme* with a BME280 sensor located on Teensy I2C bus 0, alternate I2C bus 0 pins 16 and 17, with a sensor address of 0x76 (SDO grounded).

```C++
BME280 bme(0x76, 0, I2C_PINS_16_17);
```

**BME280(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups)**
Optionally, the I2C pins and pullups can be specified. This is useful for accessing the [alternate I2C pins](https://github.com/nox771/i2c_t3/#pins) for a given bus and using the Teensy's [internal pullups](https://github.com/nox771/i2c_t3/#pullups). If these aren't specified, this library uses the default pins for a given I2C bus and external pullups. In this case, a BME280 object should be declared, specifying the BME280 I2C address, the Teensy I2C bus number, the I2C pins, and the I2C pullups. The BME280 I2C address will be 0x76 if the SDO pin is grounded or 0x77 if the SDO pin is pulled high. For example, the following code declares a BME280 object called *bme* with a BME280 sensor located on Teensy I2C bus 0, alternate I2C bus 0 pins 16 and 17, with internal pullups and a sensor address of 0x76 (SDO grounded).

```C++
BME280 bme(0x76, 0, I2C_PINS_16_17, I2C_PULLUP_INT);
```

### SPI Object Declaratioon

**BME280(uint8_t csPin)**
A BME280 object should be declared, specifying the Teensy chip select pin used. Multiple BME280 or other SPI objects could be used on the same SPI bus, each with their own chip select pin. SPI Bus 0 is used with the default MOSI, MISO, and SCK pins. The chip select pin can be any available digital pin. For example, the following code declares a BME280 object called *bme* with a BME280 sensor located on chip select pin 10.

```C++
BME280 bme(10);
```

**BME280(uint8_t csPin, SPICLass *Spi)**
Optionally, the SPI bus can be specified. This allows selecting SPI buses other than SPI Bus 0. For example, the following code declares a BME280 object called *bme* with a BME280 sensor located on chip select pin 10 and SPI bus 2.

```C++
BME280 bme(10, &SPI2);
```

**setMOSI, setMISO, and setSCK**
*setMOSI*, *setMISO*, and *setSCK* can be used after the SPI object declaration and before calling begin to set alternate MOSI, MISO, and SCK pins. For example, the code below uses SPI Bus 0, but pin 14 for SCK instead of pin 13.

```C++
BME280 bme(10);
void setup() {
   SPI.setSCK(14);
   bme.begin();
}
```

### Common Setup Functions
The following functions are used to setup the BME280 sensor. These should be called once before data collection, typically this is done in the Arduino *void setup()* function. The *begin* function should always be used. Optionally, the *setSampling*, *setFilter*, and *setStandby* functions can be used, following *begin*, to setup the oversampling, IIR filtering, and standby times. If these optional functions are not used, oversampling, IIR filtering, and standby times are set to default values, which should be good for a wide range of applications, and are discussed in greater detail below.

**int begin()**
This should be called in your setup function. It initializes communication with the BME280 and sets up the sensor for reading data. This function returns 0 on a successful initialization and returns -1 on an unsuccesful initialization. If unsuccessful, please check your wiring or try resetting power to the sensor. The following is an example of setting up the BME280.

```C++
int beginStatus;
beginStatus = bme.begin();
```

The BME280 features programmable oversampling, IIR filtering, and standby time between samples. By default, these are set to the following values:

| Settings        |                                              |
| ---             | ---                                          |
| Oversampling    | pressure x 16, temperature x 2, humidity x 1 |
| IIR Coefficient | 16                                           |
| Standby Time    | 0.5 ms                                       |

| Performance      |         |
| ---              | ---     |
| Data Output Rate | 25 Hz   |
| Filter Bandwidth | 0.53 Hz |
| Response Time    | 0.88 s  |

Optionally, the *setSampling*, *setFilter*, and *setStandby* functions can be used, following *begin*, to change these settings from their default values. For much more information on the settings and performance implications, please refer to the [BME280 datasheet](https://github.com/bolderflight/BME280/blob/master/docs/BME280-Datasheet.pdf).

**(optional) int setSampling(bme280_sampling Psampling, bme280_sampling Tsampling, bme280_sampling Hsampling)**
This is an optional function to set the oversampling to values other than the default. Oversampling can be set separately for pressure, temperature, and humidity. The following enumerated oversampling settings are supported:

| Oversampling Name | Oversampling Value |
| ---               | ---                |
| SAMPLING_X1       | 1                  |
| SAMPLING_X2       | 2                  |
| SAMPLING_X4       | 4                  |
| SAMPLING_X8       | 8                  |
| SAMPLING_X16      | 16                 |

Below is an example of selecting an oversampling value of 4 for pressure, 2 for temperature, and 2 for humidity. This function returns 0 on success and -1 on failure.

```C++
int setSamplingStatus;
setSamplingStatus = bme.setSampling(SAMPLING_X4, SAMPLING_X2, SAMPLING_X2);
```

**(optional) int setFilter(bme280_iirc iirc)**
This is an optional function to set the IIR filter coefficient to a value other than the default. This filter is applied to all measurements. The filter is given by the following equation:

*data_filtered = (data_filtered_old &ast; (filter_coefficient - 1) + data) / filter_coefficient*

The following enumerated filter coefficients are supported:

| IIR Filter Coefficient Name | IIR Filter Coefficient Value | Samples to reach 75% of step response |
| ---                         | ---                          | ---                                   |
| IIRC_OFF                    | 1                            | 1                                     |
| IIRC_X2                     | 2                            | 2                                     |
| IIRC_X4                     | 4                            | 5                                     |
| IIRC_X8                     | 8                            | 11                                    |
| IIRC_X16                    | 16                           | 22                                    |

Below is an example of selecting an IIR filter coefficient of 4. This function returns 0 on success and -1 on failure.

```C++
int setFilterStatus;
setFilterStatus = bme.setFilter(IIRC_X4);
```

**(optional) int setStandby(bme280_standby standby)**
This is an optional function to set the standby time to a value other than the default. This standby time is applied to all measurements. 

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

Below is an example of selecting a standby time of 10 ms. This function returns 0 on success and -1 on failure.

```C++
int setStandbyStatus;
setStandbyStatus = bme.setStandby(STANDBY_10_MS);
```
### Common Data Collection Functions
The functions below are used to collect data from the BME280 sensor. Data is returned scaled to engineering units. Pressure data is returned in units of Pascal (Pa), temperature data in units of degrees Celsius (C), and humidity in units of percent relative humidity (%RH). All of the data returned by the function were collected from the BME280 at the same time, so it is preferable to use the function which returns all of the desired data rather than two separate function calls in order to eliminate potential time skews in your results. For example, it would be preferable to use *getData* to get pressure, temperature, and humidity data rather than call *getPressure* followed by *getTemperature* and *getHumidity*. This preference is because the pressure, temperature, and humidity data returned by *getData* were all sampled simultaneously whereas using *getPressure* followed by *getTemperature* and *getHumidity* could possibly introduce a time skew between the pressure, temperature, and humidity data.

**void getData(float&ast; pressure, float&ast; temperature, float&ast; humidity)**
*getData(float&ast; pressure, float&ast; temperature, float&ast; humidity)* samples the BME280 sensor and returns the pressure (Pa), temperature (C), and humidity (%RH).

```C++
float pressure, temperature, humidity;
bme.getData(&pressure, &temperature, &humidity);
```

**float getPressure()**
*getPressure()* samples the BME280 sensor and returns the pressure data as a float in units of Pascal (Pa).

```C++
float pressure, temperature, humidity;
pressure = bme.getPressure();
```

**float getTemperature()**
*getTemperature()* samples the BME280 sensor and returns the temperature data as a float in units of degrees Celsius (C).

```C++
float pressure, temperature, humidity;
temperature = bme.getTemperature();
```

**float getHumidity()**
*getHumidity()* samples the BME280 sensor and returns the humidity data as a float in units of percent relative humidity (%RH).

```C++
float pressure, temperature, humidity;
humidity = bme.getHumidity();
```

## Example List
* **Basic_I2C**: demonstrates declaring a *BME280* object, initializing the sensor, and collecting data. I2C is used to communicate with the BME280 sensor.
* **Basic_SPI**: demonstrates declaring a *BME280* object, initializing the sensor, and collecting data. SPI is used to communicate with the BME280 sensor.

# Wiring and Pullups 
Please refer to the [BME280 datasheet](https://github.com/bolderflight/BME280/blob/master/docs/BME280-Datasheet.pdf) and the [Teensy pinout diagrams](https://www.pjrc.com/teensy/pinout.html). This library was developed using the [Adafruit Breakout Board](https://www.adafruit.com/products/2652). This library should work well for other breakout boards or embedded sensors, please refer to your vendor's pinout diagram.

## I2C

The BME280 pins should be connected as:
   * VDD: this should be a 1.7V to 3.6V power source. The Adafruit breakout includes voltage regulation enabling a 3-5V range.  This can be supplied by the Teensy 3.3V output.
   * GND: ground.
   * VDDIO: digital I/O supply voltage. This should be between 1.2V and 3.6V. The Adafruit breakout board connects VDDIO so this is not broken out to a pin. This can be supplied by the Teensy 3.3V output.
   * SDI: connect to Teensy SDA.
   * SCK: connect to Teensy SCL.
   * SDO: ground to select I2C address 0x76. Pull high to VDD to select I2C address 0x77.
   * CSB: connect to VDD.

By default, the Teensy pinout is:

   * Teensy 3.0:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
   * Teensy 3.1/3.2:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 29: SCL, Pin 30: SDA
   * Teensy 3.5:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 37: SCL, Pin 38: SDA
      * Bus 2 - Pin 3: SCL, Pin 4: SDA
   * Teensy 3.6:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 37: SCL, Pin 38: SDA
      * Bus 2 - Pin 3: SCL, Pin 4: SDA
      * Bus 3 - Pin 56: SDA, Pin 57: SCL
   * Teensy LC:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 22: SCL, Pin 23: SDA  

Alternatively, if the *BME280* object is declared specifying the I2C pins used, the Teensy pinout is:

   * Teensy 3.0:
      * Bus 0 - Pin 16: SCL, Pin 17: SDA
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
   * Teensy 3.1/3.2:
      * Bus 0 - Pin 16: SCL, Pin 17: SDA
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 26: SCL, Pin 31: SDA
      * Bus 1 - Pin 29: SCL, Pin 30: SDA
   * Teensy 3.5:
      * Bus 0 - Pin 7: SCL, Pin 8: SDA
      * Bus 0 - Pin 16: SCL, Pin 17: SDA
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 0 - Pin 33: SCL, Pin 34: SDA
      * Bus 0 - Pin 47: SCL, Pin 48: SDA
      * Bus 1 - Pin 37: SCL, Pin 38: SDA
      * Bus 2 - Pin 3: SCL, Pin 4: SDA
   * Teensy 3.6:
      * Bus 0 - Pin 7: SCL, Pin 8: SDA
      * Bus 0 - Pin 16: SCL, Pin 17: SDA
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 0 - Pin 33: SCL, Pin 34: SDA
      * Bus 0 - Pin 47: SCL, Pin 48: SDA
      * Bus 1 - Pin 37: SCL, Pin 38: SDA
      * Bus 2 - Pin 3: SCL, Pin 4: SDA
      * Bus 3 - Pin 56: SDA, Pin 57: SCL
   * Teensy LC:
      * Bus 0 - Pin 16: SCL, Pin 17: SDA
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 22: SCL, Pin 23: SDA 

4.7 kOhm resistors should be used as pullups on SDA and SCL, these resistors should pullup with a 3.3V source. In some very limited cases with the Teensy 3.0, 3.1/3.2, and 3.5, internal pullups could be used for a single device on a short bus. In this case, the *BME280* object should be declared specifying the I2C pins used and the I2C pullup. 

## SPI

The BME280 pins should be connected as:
   * VDD: this should be a 1.7V to 3.6V power source. The Adafruit breakout includes voltage regulation enabling a 3-5V range.  This can be supplied by the Teensy 3.3V output.
   * GND: ground.
   * VDDIO: digital I/O supply voltage. This should be between 1.2V and 3.6V. The Adafruit breakout board connects VDDIO so this is not broken out to a pin. This can be supplied by the Teensy 3.3V output.
   * SDI: connect to Teensy MOSI.
   * SCK: connect to Teensy SCK.
   * SDO: connect to Teensy MISO.
   * CSB: connect to Teensy chip select pin. Pin 10 was used in the code snippets in this document and the included examples, but any Teensy digital I/O pin can be used.

By default, the Teensy pinout is:

   * SPI Bus 0 - Pin 11: MOSI, Pin 12: MISO, Pin 13: SCK

Alternatively, by selecting the SPI bus, MOSI, MISO, and SCK pins, the Teensy pinout is:

   * Teensy 3.0, 3.1, and 3.2:
      * Bus 0 - Pin 7: MOSI, Pin 8: MISO, Pin 14: SCK
      * Bus 0 - Pin 11: MOSI, Pin 12: MISO, Pin 13: SCK
   * Teensy 3.5 and 3.6:
      * Bus 0 - Pin 7: MOSI, Pin 8: MISO, Pin 14: SCK
      * Bus 0 - Pin 11: MOSI, Pin 12: MISO, Pin 13: SCK
      * Bus 0 - Pin 28: MOSI, Pin 39: MISO, Pin 27: SCK
      * Bus 1 - Pin 0: MOSI, Pin 1: MISO, Pin 32: SCK
      * Bus 1 - Pin 21: MOSI, Pin 5: MISO, Pin 20: SCK
      * Bus 2 - Pin 44: MOSI, Pin 45: MISO, Pin 46: SCK
      * Bus 2 - Pin 52: MOSI, Pin 51: MISO, Pin 53: SCK
   * Teensy LC:
      * Bus 0 - Pin 7: MOSI, Pin 8: MISO, Pin 14: SCK
      * Bus 0 - Pin 11: MOSI, Pin 12: MISO, Pin 13: SCK
      * Bus 1 - Pin 0: MOSI, Pin 1: MISO, Pin 20: SCK
      * Bus 1 - Pin 21: MOSI, Pin 5: MISO, Pin 20: SCK

# Performance
Timing data was collected for the *getData* function on all supported Teensy devices using I2C and SPI. Timing was considered just to: communicate with the BME280 sensor, collect the data off its registers, and parse and scale the data to engineering units. This test gives some indication of performance for the various communication protocols and Teensy devices.

|             | Teensy 3.0 | Teensy 3.1/3.2 | Teensy 3.5 | Teensy 3.6 | Teensy LC |
| ----------- | ---------- | -------------- | ---------- | ---------- | --------- |
| CPU setting | 96 MHz     | 96 MHz         | 120 MHz    | 180 MHz    | 48 MHz    |
| I2C         | 377 us     | 342 us         | 338 us     | 320 us     | 491 us    |
| SPI         | 38 us      | 27 us          | 19 us      | 14 us      | 125 us    |
