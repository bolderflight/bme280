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

#include "bme280/bme280.h"
#include "core/core.h"

namespace bfs {

Bme280::Bme280(TwoWire *bus, uint8_t addr) {
  iface_ = I2C;
  i2c_ = bus;
  conn_ = addr;
}
Bme280::Bme280(SPIClass *bus, uint8_t cs) {
  iface_ = SPI;
  spi_ = bus;
  conn_ = cs;
}
bool Bme280::Begin() {
  if (iface_ == I2C) {
    i2c_->begin();
    i2c_->setClock(I2C_CLOCK_);
  } else {
    pinMode(conn_, OUTPUT);
    /* Toggle CS pin to lock in SPI mode */
    digitalWriteFast(conn_, LOW);
    delay(1);
    digitalWriteFast(conn_, HIGH);
    spi_->begin();
  }
  /* Reset the BME-280 */
  WriteRegister(RESET_REG_, SOFT_RESET_);
  /* Wait for power up */
  delay(10);
  /* Check the WHO AM I */
  uint8_t who_am_i;
  if (!ReadRegisters(WHO_AM_I_REG_, sizeof(who_am_i), &who_am_i)) {
    return false;
  }
  if (who_am_i != WHOAMI_) {
    return false;
  }
  /* Check that BME-280 is not copying trimming parameters */
  uint8_t trimming;
  ReadRegisters(STATUS_REG_, sizeof(trimming), &trimming);
  while (trimming & 0x01) {
    delay(1);
    ReadRegisters(STATUS_REG_, sizeof(trimming), &trimming);
  }
  /* Read the trimming parameters */
  uint8_t trimming_buffer[24];
  if (!ReadRegisters(TRIMMING_REG_, sizeof(trimming_buffer), trimming_buffer)) {
    return false;
  }
  dt1_ = static_cast<uint16_t>(trimming_buffer[1]) << 8 | trimming_buffer[0];
  dt2_ = static_cast<int16_t>(trimming_buffer[3]) << 8 | trimming_buffer[2];
  dt3_ = static_cast<int16_t>(trimming_buffer[5]) << 8 | trimming_buffer[4];
  dp1_ = static_cast<uint16_t>(trimming_buffer[7]) << 8 | trimming_buffer[6];
  dp2_ = static_cast<int16_t>(trimming_buffer[9]) << 8 | trimming_buffer[8];
  dp3_ = static_cast<int16_t>(trimming_buffer[11]) << 8 | trimming_buffer[10];
  dp4_ = static_cast<int16_t>(trimming_buffer[13]) << 8 | trimming_buffer[12];
  dp5_ = static_cast<int16_t>(trimming_buffer[15]) << 8 | trimming_buffer[14];
  dp6_ = static_cast<int16_t>(trimming_buffer[17]) << 8 | trimming_buffer[16];
  dp7_ = static_cast<int16_t>(trimming_buffer[19]) << 8 | trimming_buffer[18];
  dp8_ = static_cast<int16_t>(trimming_buffer[21]) << 8 | trimming_buffer[20];
  dp9_ = static_cast<int16_t>(trimming_buffer[23]) << 8 | trimming_buffer[22];
  /* Configure BME-280 */
  return Configure();
}
bool Bme280::Read() {
  uint8_t buf[6];
  /* Read the data */
  if (!ReadRegisters(DATA_REG_, sizeof(buf), buf)) {
    return false;
  }
  uint32_t pressure_counts = static_cast<uint32_t>(buf[0]) << 12 |
                             static_cast<uint32_t>(buf[1]) << 4 |
                             static_cast<uint32_t>(buf[2]) & 0xF0 >> 4;
  uint32_t temperature_counts = static_cast<uint32_t>(buf[3]) << 12 |
                                static_cast<uint32_t>(buf[4]) << 4 |
                                static_cast<uint32_t>(buf[5]) & 0xF0 >> 4;
  t_ = CompensateTemperature(temperature_counts);
  p_ = CompensatePressure(pressure_counts);
  return true;
}
bool Bme280::ConfigTempOversampling(const Oversampling oversampling) {
  t_samp_ = oversampling;
  return Configure();
}
bool Bme280::ConfigPresOversampling(const Oversampling oversampling) {
  p_samp_ = oversampling;
  return Configure();
}
bool Bme280::ConfigIir(const IirCoefficient iir) {
  iirc_ = iir;
  return Configure();
}
bool Bme280::ConfigStandbyTime(const StandbyTime standby) {
  standby_ = standby;
  return Configure();
}
float Bme280::CompensateTemperature(int32_t counts) {
  int32_t var1 = ((((counts >> 3) - ((int32_t)dt1_ << 1))) *
                 ((int32_t)dt2_)) >> 11;
  int32_t var2 = (((((counts >> 4) - ((int32_t)dt1_)) * ((counts >> 4) -
                 ((int32_t)dt1_))) >> 12) * ((int32_t)dt3_)) >> 14;
  tfine_ = var1 + var2;
  int32_t t = (tfine_ * 5 + 128) >> 8;
  return static_cast<float>(t) / 100.0f;
}
float Bme280::CompensatePressure(int32_t counts) {
  int64_t var1 = ((int64_t)tfine_) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)dp6_;
  var2 = var2 + ((var1 * (int64_t)dp5_) << 17);
  var2 = var2 + (((int64_t)dp4_) << 35);
  var1 = ((var1 * var1 * (int64_t)dp3_) >> 8) + ((var1 * (int64_t)dp2_) << 12);
  var1 = (((((int64_t)1) << 47)+ var1)) * ((int64_t)dp1_) >> 33;
  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  int64_t p = 1048576 - counts;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dp9_) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dp8_) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dp7_) << 4);
  return static_cast<float>(static_cast<uint32_t>(p)) / 256.0f;
}
bool Bme280::Configure() {
  /* Set to sleep mode */
  if (!WriteRegister(CTRL_MEAS_REG_, MODE_SLEEP_)) {
    return false;
  }
  /* Disable humidity sensor */
  if (!WriteRegister(CTRL_HUM_REG_, 0x00)) {
    return false;
  }
  /* Standby time and IIRC config */
  if (!WriteRegister(CONFIG_REG_, standby_ << 5 | iirc_ << 3)) {
    return false;
  }
  /* Oversampling and mode */
  if (!WriteRegister(CTRL_MEAS_REG_, t_samp_ << 5 |
                     p_samp_ << 3 | MODE_NORMAL_)) {
    return false;
  }
  return true;
}
bool Bme280::WriteRegister(uint8_t reg, uint8_t data) {
  uint8_t ret_val;
  if (iface_ == I2C) {
    i2c_->beginTransmission(conn_);
    i2c_->write(reg);
    i2c_->write(data);
    i2c_->endTransmission();
  } else {
    spi_->beginTransaction(SPISettings(SPI_CLOCK_, MSBFIRST, SPI_MODE3));
    digitalWriteFast(conn_, LOW);
    spi_->transfer(reg & ~SPI_READ_);
    spi_->transfer(data);
    digitalWriteFast(conn_, HIGH);
    spi_->endTransaction();
  }
  delay(10);
  ReadRegisters(reg, sizeof(ret_val), &ret_val);
  if (data == ret_val) {
    return true;
  } else {
    return false;
  }
}
bool Bme280::ReadRegisters(uint8_t reg, uint8_t count, uint8_t *data) {
  if (iface_ == I2C) {
    i2c_->beginTransmission(conn_);
    i2c_->write(reg);
    i2c_->endTransmission(false);
    uint8_t bytes_rx = i2c_->requestFrom(conn_, count);
    if (bytes_rx == count) {
      for (std::size_t i = 0; i < bytes_rx; i++) {
        data[i] = i2c_->read();
      }
      return true;
    } else {
      return false;
    }
  } else {
    spi_->beginTransaction(SPISettings(SPI_CLOCK_, MSBFIRST, SPI_MODE3));
    digitalWriteFast(conn_, LOW);
    spi_->transfer(reg | SPI_READ_);
    spi_->transfer(data, count);
    digitalWriteFast(conn_, HIGH);
    spi_->endTransaction();
    return true;
  }
}

}  // namespace bfs
