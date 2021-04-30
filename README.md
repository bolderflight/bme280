# bme280
This library communicates with the [BME280](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/) environmental sensor.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Description
The Bosch Sensortec [BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) is an integrated environmental sensor, which combines high linearity, high accuracy sensors for pressure, temperature, and humidity in a compact LGA package. The pressure sensor is an absolute barometric pressure sensor, which features exceptionally high accuracy and resolution at very low noise. The integrated temperature sensor has been optimized for very low noise and high resolution. Pressure, temperature, and humidity measurements can be useful for applications involving unmanned vehicles (indicated and true airspeed, altitude, and density altitude), indoor navigation (floor detection), outdoor navigation (altitudes and airspeeds, dead-reckoning, GPS time to first fix improvements) as well as weather monitoring and home automation.

The [BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) samples pressure and temperature to 20 bit resolution and humidity to 16 bit resolution. The [BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) features programmable oversampling, IIR filtering, and standby time between samples. The BME280 supports both I2C and SPI communication.

# Usage
This library supports both I2C and SPI commmunication with the BME280 and supports collecting pressure and temperature data. Although humidity data is available on the sensor, it is not supported by this library - humidity data is returned as relative humidity and is of relatively limited use for aerospace applications. Also limiting output to pressure and temperature improves interoperability with other pressure sensors with similar output measurements (i.e. the BMP280, BMP380, AMS5812, AMS5915, AMS6915, ...).

## Installation
CMake is used to build this library, which is exported as a library target called *bme280*. The header is added as:

```
#include "bme/bme280.h"
```

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and example executables called *i2c_example* and *spi_example*. The example executable source files are located at *examples/i2c.cc* and *examples/spi.cc*. Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64
   * IMXRT1062_T40
   * IMXRT1062_T41

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The *i2c_example* and *spi_example* targets create executables for communicating with the sensor using I2C or SPI communication, respectively. Each target also has a *_hex* for creating the hex file to upload to the microcontroller. 

## Namespace
This library is within the namespace *bfs*.

## Methods
This driver conforms to the [Pressure interface](https://github.com/bolderflight/pres); please refer to those documents for information on the *PresConfig* and *PresData* structs.

**bool Init(const PresConfig &ref)** Initializes communication with the pressure transducer and configures it according to the *PresConfig* struct. Note that the bus is not started within init and should be initialized elsewhere. Returns true on successfully initializing and configuring the pressure transducer.

```C++
/* BME280 pressure transducer */
bfs::Bme280 pres;
/* Config */
bfs::PresConfig config = {
   .bus = &Wire1,
   .dev = 0x76,
   .sampling_period_ms = 20
};
/* Init the bus */
Wire1.begin();
Wire1.setClock(400000);
/* Initialize and configure pressure transducer */
if (!pres.Init(config)) {
   Serial.println("Error initializing communication with pressure transducer");
   while(1) {}
}
```

**bool Read(PresData &ast; const ptr)** Reads data from the pressure transducer and passes the data into the PresData struct. Returns true on successfully reading new data.

```C++
/* Pressure data */
bfs::PresData data;
if (pres.Read(&data)) {

}
```
