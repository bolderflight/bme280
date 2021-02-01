/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_BME280_BME280_H_
#define INCLUDE_BME280_BME280_H_

#include "core/core.h"

namespace sensors {

class Bme280 {
 public:
  enum Oversampling : uint8_t {
    OVERSAMPLING_1   = 0x01,
    OVERSAMPLING_2   = 0x02,
    OVERSAMPLING_4   = 0x03,
    OVERSAMPLING_8   = 0x04,
    OVERSAMPLING_16  = 0x05
  };
  enum IirCoefficient : uint8_t {
    IIRC_OFF = 0x00,
    IIRC_2 = 0x01,
    IIRC_4 = 0x02,
    IIRC_8 = 0x03,
    IIRC_16 = 0x04
  };
  enum StandbyTime : uint8_t {
    STANDBY_0_5_MS = 0x00,
    STANDBY_62_5_MS = 0x01,
    STANDBY_125_MS = 0x02,
    STANDBY_250_MS = 0x03,
    STANDBY_500_MS = 0x04,
    STANDBY_1000_MS = 0x05,
    STANDBY_10_MS = 0x06,
    STANDBY_20_MS = 0x07
  };
  #if defined(__IMXRT1062__)
  Bme280(TwoWire *bus, uint8_t addr);
  #else
  Bme280(i2c_t3 *bus, uint8_t addr);
  #endif
  Bme280(SPIClass *bus, uint8_t cs);
  bool Begin();
  bool Read();
  bool temperature_oversampling(Oversampling oversampling);
  Oversampling temperature_oversampling();
  bool pressure_oversampling(Oversampling oversampling);
  Oversampling pressure_oversampling();
  bool iir_coefficient(IirCoefficient iir);
  IirCoefficient iir_coefficient();
  bool standby_time(StandbyTime standby);
  StandbyTime standby_time();
  float pressure_pa();
  float die_temperature_c();

 private:
  enum Interface {
    SPI,
    I2C
  };
  /* Communications interface */
  Interface iface_;
  #if defined(__IMXRT1062__)
  TwoWire *i2c_;
  #else
  i2c_t3 *i2c_;
  #endif
  SPIClass *spi_;
  uint8_t conn_;
  static constexpr uint32_t SPI_CLOCK_ = 10000000;
  static constexpr uint32_t I2C_CLOCK_ = 400000;
  static constexpr uint8_t SPI_READ_ = 0x80;
  /* Configuration */
  Oversampling t_samp_ = OVERSAMPLING_2;
  Oversampling p_samp_ = OVERSAMPLING_16;
  IirCoefficient iirc_ = IIRC_16;
  StandbyTime standby_ = STANDBY_0_5_MS;
  static constexpr uint8_t WHOAMI_ = 0x60;
  /* Data */
  uint16_t dt1_;
  int16_t dt2_, dt3_;
  uint16_t dp1_;
  int16_t dp2_, dp3_, dp4_, dp5_, dp6_, dp7_, dp8_, dp9_;
  int32_t tfine_;
  float p_;
  float t_;
  /* Registers */
  static constexpr uint8_t WHO_AM_I_REG_ = 0xD0;
  static constexpr uint8_t RESET_REG_ = 0xE0;
  static constexpr uint8_t CTRL_HUM_REG_ = 0xF2;
  static constexpr uint8_t STATUS_REG_ = 0xF3;
  static constexpr uint8_t CTRL_MEAS_REG_ = 0xF4;
  static constexpr uint8_t CONFIG_REG_ = 0xF5;
  static constexpr uint8_t DATA_REG_ = 0xF7;
  static constexpr uint8_t TRIMMING_REG_ = 0x88;
  /* Settings */
  static constexpr uint8_t SOFT_RESET_ = 0xB6;
  static constexpr uint8_t MODE_SLEEP_ = 0x00;
  static constexpr uint8_t MODE_NORMAL_ = 0x03;
  bool Configure();
  float CompensateTemperature(int32_t counts);
  float CompensatePressure(int32_t counts);
  bool WriteRegister(uint8_t reg, uint8_t data);
  bool ReadRegisters(uint8_t reg, uint8_t count, uint8_t *data);
};

}  // namespace sensors

#endif  // INCLUDE_BME280_BME280_H_
