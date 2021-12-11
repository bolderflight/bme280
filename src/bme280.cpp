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

#include "bme280.h"  // NOLINT
#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
#else
#include "core/core.h"
#endif
#include <stddef.h>
#include <stdint.h>

namespace bfs {

Bme280::Bme280(TwoWire *i2c, const I2cAddr addr) {
  i2c_intf_.i2c = i2c;
  i2c_intf_.addr = static_cast<uint8_t>(addr);
  dev_.intf_ptr = &i2c_intf_;
  dev_.intf = BME280_I2C_INTF;
  dev_.read = I2cReadRegisters;
  dev_.write = I2cWriteRegisters;
  dev_.delay_us = Delay_us;
}

Bme280::Bme280(SPIClass *spi, const uint8_t cs) {
  pinMode(cs, OUTPUT);
  spi_intf_.spi = spi;
  spi_intf_.cs = cs;
  dev_.intf_ptr = &spi_intf_;
  dev_.intf = BME280_SPI_INTF;
  dev_.read = SpiReadRegisters;
  dev_.write = SpiWriteRegisters;
  dev_.delay_us = Delay_us;
}

bool Bme280::Begin() {
  /* Initialize communication */
  if (bme280_init(&dev_) != BME280_OK) {
    return false;
  }
  /* Set defaults */
  dev_.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev_.settings.osr_p = BME280_OVERSAMPLING_16X;
  dev_.settings.osr_t = BME280_OVERSAMPLING_2X;
  dev_.settings.filter = BME280_FILTER_COEFF_16;
  dev_.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;
  /* Select the settings to apply */
  uint8_t settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
  if (bme280_set_sensor_settings(settings_sel, &dev_) != BME280_OK) {
    return false;
  }
  /* Set to normal mode */
  if (bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev_) != BME280_OK) {
    return false;
  }
  return true;
}

bool Bme280::ConfigTempOversampling(const Oversampling val) {
  dev_.settings.osr_t = static_cast<uint8_t>(val);
  uint8_t settings_sel = BME280_OSR_TEMP_SEL;
  return (bme280_set_sensor_settings(settings_sel, &dev_) != BME280_OK);
}

bool Bme280::ConfigPresOversampling(const Oversampling val) {
  dev_.settings.osr_p = static_cast<uint8_t>(val);
  uint8_t settings_sel = BME280_OSR_PRESS_SEL;
  return (bme280_set_sensor_settings(settings_sel, &dev_) != BME280_OK);
}

bool Bme280::ConfigHumidityOversampling(const Oversampling val) {
  dev_.settings.osr_h = static_cast<uint8_t>(val);
  uint8_t settings_sel = BME280_OSR_HUM_SEL;
  return (bme280_set_sensor_settings(settings_sel, &dev_) != BME280_OK);
}

bool Bme280::ConfigFilterCoef(const FilterCoef val) {
  dev_.settings.filter = static_cast<uint8_t>(val);
  uint8_t settings_sel = BME280_FILTER_SEL;
  return (bme280_set_sensor_settings(settings_sel, &dev_) != BME280_OK);
}

bool Bme280::ConfigStandbyTime(const StandbyTime val) {
  dev_.settings.standby_time = static_cast<uint8_t>(val);
  uint8_t settings_sel = BME280_STANDBY_SEL;
  return (bme280_set_sensor_settings(settings_sel, &dev_) != BME280_OK);
}

bool Bme280::Read() {
  return (bme280_get_sensor_data(BME280_ALL, &data_, &dev_) == BME280_OK);
  return true;
}

void Bme280::Delay_us(uint32_t period, void *intf_ptr) {
  delayMicroseconds(period);
}

int8_t Bme280::I2cWriteRegisters(uint8_t reg, const uint8_t * data,
                                 uint32_t len, void * intf) {
  /* NULL pointer check */
  if ((!data) || (!intf)) {
    return BME280_E_NULL_PTR;
  }
  /* Check length */
  if (len == 0) {
    return BME280_E_INVALID_LEN;
  }
  I2cIntf *iface = static_cast<I2cIntf *>(intf);
  iface->i2c->beginTransmission(iface->addr);
  if (iface->i2c->write(reg) != 1) {
    return BME280_E_COMM_FAIL;
  }
  if (iface->i2c->write(data, len) != len) {
    return BME280_E_COMM_FAIL;
  }
  iface->i2c->endTransmission();
  return BME280_OK;
}

int8_t Bme280::I2cReadRegisters(uint8_t reg, uint8_t * data, uint32_t len,
                                void * intf) {
  /* NULL pointer check */
  if ((!data) || (!intf)) {
    return BME280_E_NULL_PTR;
  }
  /* Check length */
  if (len == 0) {
    return BME280_E_INVALID_LEN;
  }
  I2cIntf *iface = static_cast<I2cIntf *>(intf);
  iface->i2c->beginTransmission(iface->addr);
  if (iface->i2c->write(reg) != 1) {
    return BME280_E_COMM_FAIL;
  }
  iface->i2c->endTransmission(false);
  if (iface->i2c->requestFrom(iface->addr, len) != len) {
    return BME280_E_COMM_FAIL;
  }
  for (size_t i = 0; i < len; i++) {
    data[i] = iface->i2c->read();
  }
  return BME280_OK;
}

int8_t Bme280::SpiWriteRegisters(uint8_t reg, const uint8_t * data,
                                 uint32_t len, void * intf) {
  /* NULL pointer check */
  if ((!data) || (!intf)) {
    return BME280_E_NULL_PTR;
  }
  /* Check length */
  if (len == 0) {
    return BME280_E_INVALID_LEN;
  }
  SpiIntf *iface = static_cast<SpiIntf *>(intf);
  iface->spi->beginTransaction(SPISettings(SPI_CLK_, MSBFIRST, SPI_MODE3));
  #if defined(TEENSYDUINO)
  digitalWriteFast(iface->cs, LOW);
  #else
  digitalWrite(iface->cs, LOW);
  #endif
  iface->spi->transfer(reg);
  for (size_t i = 0; i < len; i++) {
    iface->spi->transfer(data[i]);
  }
  #if defined(TEENSYDUINO)
  digitalWriteFast(iface->cs, HIGH);
  #else
  digitalWrite(iface->cs, HIGH);
  #endif
  iface->spi->endTransaction();
  return BME280_OK;
}

int8_t Bme280::SpiReadRegisters(uint8_t reg, uint8_t * data, uint32_t len,
                                void * intf) {
  /* NULL pointer check */
  if ((!data) || (!intf)) {
    return BME280_E_NULL_PTR;
  }
  /* Check length */
  if (len == 0) {
    return BME280_E_INVALID_LEN;
  }
  SpiIntf *iface = static_cast<SpiIntf *>(intf);
  iface->spi->beginTransaction(SPISettings(SPI_CLK_, MSBFIRST, SPI_MODE3));
  #if defined(TEENSYDUINO)
  digitalWriteFast(iface->cs, LOW);
  #else
  digitalWrite(iface->cs, LOW);
  #endif
  iface->spi->transfer(reg);
  for (size_t i = 0; i < len; i++) {
    data[i] = iface->spi->transfer(0x00);
  }
  #if defined(TEENSYDUINO)
  digitalWriteFast(iface->cs, HIGH);
  #else
  digitalWrite(iface->cs, HIGH);
  #endif
  iface->spi->endTransaction();
  return BME280_OK;
}

}  // namespace bfs
