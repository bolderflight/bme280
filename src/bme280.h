/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#ifndef SRC_BME280_H_
#define SRC_BME280_H_

#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
#else
#include "core/core.h"
#endif
#include <stddef.h>
#include <stdint.h>
#include "bst/bme280.h"

namespace bfs {

class Bme280 {
 public:
  enum I2cAddr : uint8_t {
    I2C_ADDR_PRIM = BME280_I2C_ADDR_PRIM,
    I2C_ADDR_SEC = BME280_I2C_ADDR_SEC
  };
  enum Oversampling : uint8_t {
    OVERSAMPLING_1X = BME280_OVERSAMPLING_1X,
    OVERSAMPLING_2X = BME280_OVERSAMPLING_2X,
    OVERSAMPLING_4X = BME280_OVERSAMPLING_4X,
    OVERSAMPLING_8X = BME280_OVERSAMPLING_8X,
    OVERSAMPLING_16X = BME280_OVERSAMPLING_16X
  };
  enum FilterCoef : uint8_t {
    FILTER_COEF_OFF = BME280_FILTER_COEFF_OFF,
    FILTER_COEF_2 = BME280_FILTER_COEFF_2,
    FILTER_COEF_4 = BME280_FILTER_COEFF_4,
    FILTER_COEF_8 = BME280_FILTER_COEFF_8,
    FILTER_COEF_16 = BME280_FILTER_COEFF_16
  };
  enum StandbyTime : uint8_t {
    STANDBY_TIME_0_5_MS = BME280_STANDBY_TIME_0_5_MS,
    STANDBY_TIME_62_5_MS = BME280_STANDBY_TIME_62_5_MS,
    STANDBY_TIME_125_MS = BME280_STANDBY_TIME_125_MS,
    STANDBY_TIME_250_MS = BME280_STANDBY_TIME_250_MS,
    STANDBY_TIME_500_MS = BME280_STANDBY_TIME_500_MS,
    STANDBY_TIME_1000_MS = BME280_STANDBY_TIME_1000_MS,
    STANDBY_TIME_10_MS = BME280_STANDBY_TIME_10_MS,
    STANDBY_TIME_20_MS = BME280_STANDBY_TIME_20_MS
  };
  Bme280(TwoWire *i2c, const I2cAddr addr);
  Bme280(SPIClass *spi, const uint8_t cs);
  bool Begin();
  bool ConfigTempOversampling(const Oversampling val);
  bool ConfigPresOversampling(const Oversampling val);
  bool ConfigHumidityOversampling(const Oversampling val);
  inline Oversampling temp_oversampling() const {
    return static_cast<Oversampling>(dev_.settings.osr_t);
  }
  inline Oversampling pres_oversampling() const {
    return static_cast<Oversampling>(dev_.settings.osr_p);
  }
  inline Oversampling humidity_oversampling() const {
    return static_cast<Oversampling>(dev_.settings.osr_h);
  }
  bool ConfigFilterCoef(const FilterCoef val);
  inline FilterCoef filter_coef() const {
    return static_cast<FilterCoef>(dev_.settings.filter);
  }
  bool ConfigStandbyTime(const StandbyTime val);
  inline StandbyTime standby_time() const {
    return static_cast<StandbyTime>(dev_.settings.standby_time);
  }
  bool Read();
  inline bool Reset() {return (bme280_soft_reset(&dev_) == BME280_OK);}
  inline double pres_pa() const {return data_.pressure;}
  inline double die_temp_c() const {return data_.temperature;}
  inline double humidity_rh() const {return data_.humidity;}

 private:
  /* BME-280 device settings */
  bme280_dev dev_;
  /* BME-280 data */
  bme280_data data_;
  /* Description of I2C and SPI interfaces */
  struct I2cIntf {
    uint8_t addr;
    TwoWire *i2c;
  } i2c_intf_;
  struct SpiIntf {
    uint8_t cs;
    SPIClass *spi;
  } spi_intf_;
  /* SPI clock speed, 10 MHz */
  static constexpr int32_t SPI_CLK_ = 10000000;
  /* Function prototype for delaying, ms */
  static void Delay_us(uint32_t period, void *intf_ptr);
  /* Function prototypes for reading and writing I2C and SPI data */
  static int8_t I2cWriteRegisters(uint8_t reg, const uint8_t * data,
                                  uint32_t len, void * intf);
  static int8_t I2cReadRegisters(uint8_t reg, uint8_t * data, uint32_t len,
                                 void * intf);
  static int8_t SpiWriteRegisters(uint8_t reg, const uint8_t * data,
                                  uint32_t len, void * intf);
  static int8_t SpiReadRegisters(uint8_t reg, uint8_t * data, uint32_t len,
                                 void * intf);
};

}  // namespace bfs

#endif  // SRC_BME280_H_
