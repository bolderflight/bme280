/*
BME280.h
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef BME280_h
#define BME280_h

#include "Arduino.h"
#include "i2c_t3.h"  // I2C library
#include "SPI.h"     // SPI library

enum bme280_sampling
{
  SAMPLING_X1   = 0x01,
  SAMPLING_X2   = 0x02,
  SAMPLING_X4   = 0x03,
  SAMPLING_X8   = 0x04,
  SAMPLING_X16  = 0x05
};

enum bme280_iirc
{
  IIRC_OFF = 0x00,
  IIRC_2 = 0x01,
  IIRC_4 = 0x02,
  IIRC_8 = 0x03,
  IIRC_16 = 0x04
};

enum bme280_standby
{
  STANDBY_0_5_MS = 0x00,
  STANDBY_62_5_MS = 0x01,
  STANDBY_125_MS = 0x02,
  STANDBY_250_MS = 0x03,
  STANDBY_500_MS = 0x04,
  STANDBY_1000_MS = 0x05,
  STANDBY_10_MS = 0x06,
  STANDBY_20_MS = 0x07
};

enum bme280_mode
{
  MODE_SLEEP = 0x00,
  MODE_FORCED = 0x01,
  MODE_NORMAL = 0x03
};

class BME280{
  public:
    BME280();
    BME280(uint8_t address, uint8_t bus);
    BME280(uint8_t address, uint8_t bus, i2c_pins pins);
    BME280(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups);
    BME280(uint8_t csPin);
    BME280(uint8_t csPin, SPIClass *Spi);
    void configure(uint8_t address, uint8_t bus);
    void configure(uint8_t address, uint8_t bus, i2c_pins pins);
    void configure(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups);
    void configure(uint8_t csPin);
    void configure(uint8_t csPin, SPIClass *Spi);
    int begin();
    int setSampling(bme280_sampling Psampling, bme280_sampling Tsampling, bme280_sampling Hsampling);
    int setFilter(bme280_iirc iirc);
    int setStandby(bme280_standby standby); 
    void getData(float* pressure, float* temperature, float* humidity);
    float getTemperature();
    float getPressure();
    float getHumidity();
  private:
    // i2c
    uint8_t _address;
    uint8_t _bus;
    i2c_pins _pins;
    i2c_pullup _pullups;
    const uint32_t _i2cRate = 400000; // 400 kHz
    bool _userDefI2C;

    // spi
    SPIClass *_spi;
    uint8_t _csPin;
    bool _useSPI;
    const uint8_t SPI_READ = 0x80;
    const uint32_t SPI_CLOCK = 10000000; // 10 MHz

    // calibration parameters
    uint16_t _dig_T1;
    int16_t _dig_T2, _dig_T3;
    uint16_t _dig_P1;
    int16_t _dig_P2, _dig_P3, _dig_P4, _dig_P5, _dig_P6, 
      _dig_P7, _dig_P8, _dig_P9;
    uint8_t _dig_H1, _dig_H3;
    int16_t _dig_H2, _dig_H4, _dig_H5;
    int8_t _dig_H6;

    // BME 280 settings
    bme280_sampling _Tsampling = SAMPLING_X2;
    bme280_sampling _Psampling = SAMPLING_X16;
    bme280_sampling _Hsampling = SAMPLING_X1;
    bme280_iirc _iirc = IIRC_16;
    bme280_standby _standby = STANDBY_0_5_MS;
    bme280_mode _mode = MODE_NORMAL;
    const uint8_t _spi3w_en = 0x00;

    // BME280 registers
    const uint8_t WHO_AM_I_REG = 0xD0;

    const uint8_t RESET_REG = 0xE0;
    const uint8_t SOFT_RESET = 0xB6;

    const uint8_t CTRL_HUM_REG = 0xF2;
    const uint8_t STATUS_REG = 0xF3;
    const uint8_t CTRL_MEAS_REG = 0xF4;

    const uint8_t CONFIG_REG = 0xF5;

    const uint8_t DATA_REG = 0xF7;

    const uint8_t DIG_T1_REG = 0x88;
    const uint8_t DIG_T2_REG = 0x8A;
    const uint8_t DIG_T3_REG = 0x8C;

    const uint8_t DIG_P1_REG = 0x8E;
    const uint8_t DIG_P2_REG = 0x90;
    const uint8_t DIG_P3_REG = 0x92;
    const uint8_t DIG_P4_REG = 0x94;
    const uint8_t DIG_P5_REG = 0x96;
    const uint8_t DIG_P6_REG = 0x98;
    const uint8_t DIG_P7_REG = 0x9A;
    const uint8_t DIG_P8_REG = 0x9C;
    const uint8_t DIG_P9_REG = 0x9E;

    const uint8_t DIG_H1_REG = 0xA1;
    const uint8_t DIG_H2_REG = 0xE1;
    const uint8_t DIG_H3_REG = 0xE3;
    const uint8_t DIG_H4_REG = 0xE4;
    const uint8_t DIG_H5_REG = 0xE5;
    const uint8_t DIG_H6_REG = 0xE7;

    void compensateTemperature(int32_t temperatureCounts, int32_t* t_fine, float* temperature);
    void compensatePressure(int32_t pressureCounts, int32_t t_fine, float* pressure);
    void compensateHumidity(int32_t humidityCounts, int32_t t_fine, float* humidity);
    void getDataCounts(int32_t* pressureCounts, int32_t* temperatureCounts, int32_t* humidityCounts);
    int configureBME280();
    void readCalibration();
    bool writeRegister(uint8_t subAddress, uint8_t data);
    void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
};

#endif
