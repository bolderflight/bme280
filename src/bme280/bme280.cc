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

bool Bme280::Init(const PresConfig &ref) {
  /* Copy the config */
  config_ = ref;
  /* Determine the interface type */
  if (std::holds_alternative<TwoWire *>(config_.bus)) {
    i2c_ = std::get<TwoWire *>(config_.bus);
  } else if (std::holds_alternative<SPIClass *>(config_.bus)) {
    spi_ = std::get<SPIClass *>(config_.bus);
  } else {
    return false;
  }
  /* Health monitoring */
  health_period_ms_ = 5 * ref.sampling_period_ms;
  health_timer_ms_ = 0;
  /* Start communication with sensor */
  return Begin();
}
bool Bme280::Read(PresData * const ptr) {
  if (!ptr) {return false;}
  ptr->new_data = ReadPres();
  ptr->healthy = (health_timer_ms_ < health_period_ms_);
  if (ptr->new_data) {
    health_timer_ms_ = 0;
    ptr->pres_pa = p_;
  }
  return ptr->new_data;
}
bool Bme280::Begin() {
  if (iface_ == SPI) {
    pinMode(config_.dev, OUTPUT);
    /* Toggle CS pin to lock in SPI mode */
    digitalWriteFast(config_.dev, LOW);
    delay(1);
    digitalWriteFast(config_.dev, HIGH);
  }
  /* Reset the BME-280 */
  WriteRegister(RESET_REG_, SOFT_RESET_);
  /* Wait for power up */
  delay(10);
  /* Check the WHO AM I */
  if (!ReadRegisters(WHO_AM_I_REG_, sizeof(who_am_i_), &who_am_i_)) {
    return false;
  }
  if (who_am_i_ != WHOAMI_) {
    return false;
  }
  /* Check that BME-280 is not copying trimming parameters */
  ReadRegisters(STATUS_REG_, sizeof(trimming_), &trimming_);
  while (trimming_ & 0x01) {
    delay(1);
    ReadRegisters(STATUS_REG_, sizeof(trimming_), &trimming_);
  }
  /* Read the trimming parameters */
  uint8_t trim_buf_[24];
  if (!ReadRegisters(TRIMMING_REG_, sizeof(trim_buf_), trim_buf_)) {
    return false;
  }
  dt1_ = static_cast<uint16_t>(trim_buf_[1]) << 8 | trim_buf_[0];
  dt2_ = static_cast<int16_t>(trim_buf_[3]) << 8 | trim_buf_[2];
  dt3_ = static_cast<int16_t>(trim_buf_[5]) << 8 | trim_buf_[4];
  dp1_ = static_cast<uint16_t>(trim_buf_[7]) << 8 | trim_buf_[6];
  dp2_ = static_cast<int16_t>(trim_buf_[9]) << 8 | trim_buf_[8];
  dp3_ = static_cast<int16_t>(trim_buf_[11]) << 8 | trim_buf_[10];
  dp4_ = static_cast<int16_t>(trim_buf_[13]) << 8 | trim_buf_[12];
  dp5_ = static_cast<int16_t>(trim_buf_[15]) << 8 | trim_buf_[14];
  dp6_ = static_cast<int16_t>(trim_buf_[17]) << 8 | trim_buf_[16];
  dp7_ = static_cast<int16_t>(trim_buf_[19]) << 8 | trim_buf_[18];
  dp8_ = static_cast<int16_t>(trim_buf_[21]) << 8 | trim_buf_[20];
  dp9_ = static_cast<int16_t>(trim_buf_[23]) << 8 | trim_buf_[22];
  /* Configure BME-280 */
  return Configure();
}
bool Bme280::ReadPres() {
  /* Read the data */
  if (!ReadRegisters(DATA_REG_, sizeof(buf_), buf_)) {
    return false;
  }
  uint32_t pres_cnts_ = static_cast<uint32_t>(buf_[0]) << 12 |
                        static_cast<uint32_t>(buf_[1]) << 4 |
                        static_cast<uint32_t>(buf_[2]) & 0xF0 >> 4;
  uint32_t temp_cnts_ = static_cast<uint32_t>(buf_[3]) << 12 |
                        static_cast<uint32_t>(buf_[4]) << 4 |
                        static_cast<uint32_t>(buf_[5]) & 0xF0 >> 4;
  t_ = CompensateTemperature(temp_cnts_);
  p_ = CompensatePressure(pres_cnts_);
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
  tvar1_ = ((((counts >> 3) - ((int32_t)dt1_ << 1))) *
           ((int32_t)dt2_)) >> 11;
  tvar2_ = (((((counts >> 4) - ((int32_t)dt1_)) * ((counts >> 4) -
           ((int32_t)dt1_))) >> 12) * ((int32_t)dt3_)) >> 14;
  tfine_ = tvar1_ + tvar2_;
  tvar_ = (tfine_ * 5 + 128) >> 8;
  return static_cast<float>(tvar_) / 100.0f;
}
float Bme280::CompensatePressure(int32_t counts) {
  pvar1_ = ((int64_t)tfine_) - 128000;
  pvar2_ = pvar1_ * pvar1_ * (int64_t)dp6_;
  pvar2_ = pvar2_ + ((pvar1_ * (int64_t)dp5_) << 17);
  pvar2_ = pvar2_ + (((int64_t)dp4_) << 35);
  pvar1_ = ((pvar1_ * pvar1_ * (int64_t)dp3_) >> 8) +
           ((pvar1_ * (int64_t)dp2_) << 12);
  pvar1_ = (((((int64_t)1) << 47)+ pvar1_)) * ((int64_t)dp1_) >> 33;
  if (pvar1_ == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  pvar_ = 1048576 - counts;
  pvar_ = (((pvar_ << 31) - pvar2_) * 3125) / pvar1_;
  pvar1_ = (((int64_t)dp9_) * (pvar_ >> 13) * (pvar_ >> 13)) >> 25;
  pvar2_ = (((int64_t)dp8_) * pvar_) >> 19;
  pvar_ = ((pvar_ + pvar1_ + pvar2_) >> 8) + (((int64_t)dp7_) << 4);
  return static_cast<float>(static_cast<uint32_t>(pvar_)) / 256.0f;
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
  if (iface_ == I2C) {
    i2c_->beginTransmission(config_.dev);
    i2c_->write(reg);
    i2c_->write(data);
    i2c_->endTransmission();
  } else {
    spi_->beginTransaction(SPISettings(SPI_CLOCK_, MSBFIRST, SPI_MODE3));
    digitalWriteFast(config_.dev, LOW);
    spi_->transfer(reg & ~SPI_READ_);
    spi_->transfer(data);
    digitalWriteFast(config_.dev, HIGH);
    spi_->endTransaction();
  }
  delay(10);
  ReadRegisters(reg, sizeof(ret_val_), &ret_val_);
  if (data == ret_val_) {
    return true;
  } else {
    return false;
  }
}
bool Bme280::ReadRegisters(uint8_t reg, uint8_t count, uint8_t *data) {
  if (iface_ == I2C) {
    i2c_->beginTransmission(config_.dev);
    i2c_->write(reg);
    i2c_->endTransmission(false);
    bytes_rx_ = i2c_->requestFrom(config_.dev, count);
    if (bytes_rx_ == count) {
      for (std::size_t i = 0; i < bytes_rx_; i++) {
        data[i] = i2c_->read();
      }
      return true;
    } else {
      return false;
    }
  } else {
    spi_->beginTransaction(SPISettings(SPI_CLOCK_, MSBFIRST, SPI_MODE3));
    digitalWriteFast(config_.dev, LOW);
    spi_->transfer(reg | SPI_READ_);
    spi_->transfer(data, count);
    digitalWriteFast(config_.dev, HIGH);
    spi_->endTransaction();
    return true;
  }
}

}  // namespace bfs
