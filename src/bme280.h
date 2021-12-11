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
  Bme280(TwoWire *i2c, const uint8_t addr);
  Bme280(SPIClass *spi, const uint8_t cs);
  bool Begin();
  bool Read();

 private:
  bme280_dev dev_;
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
