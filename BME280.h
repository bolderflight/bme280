/*
BME280.h
Brian R Taylor
brian.taylor@bolderflight.com
2017-03-27

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

#ifndef SPI_MOSI_PIN
#define SPI_MOSI_PIN
    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)
    enum spi_mosi_pin
    {
      MOSI_PIN_7,
      MOSI_PIN_11
    };
    #endif
    // Teensy 3.5 || Teensy 3.6
    #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    enum spi_mosi_pin
    {
      MOSI_PIN_0,
      MOSI_PIN_7,
      MOSI_PIN_11,
      MOSI_PIN_21,
      MOSI_PIN_28,
      MOSI_PIN_44,
      MOSI_PIN_52
    };
    #endif
    // Teensy LC 
    #if defined(__MKL26Z64__)
    enum spi_mosi_pin
    {
      MOSI_PIN_0,
      MOSI_PIN_7,
      MOSI_PIN_11,
      MOSI_PIN_21
    };
    #endif
#endif

class BME280{
  public:
    BME280(uint8_t address, uint8_t bus);
    BME280(uint8_t address, uint8_t bus, i2c_pins pins);
    BME280(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups);
    BME280(uint8_t csPin);
    BME280(uint8_t csPin, spi_mosi_pin pin);
    int begin();
  private:
    // i2c
    uint8_t _address;
    uint8_t _bus;
    i2c_pins _pins;
    i2c_pullup _pullups;
    const uint32_t _i2cRate = 400000;
    bool _userDefI2C;

    // spi
    uint8_t _csPin;
    spi_mosi_pin _mosiPin;
    bool _useSPI;
    const uint8_t SPI_READ = 0x80;
    const uint32_t SPI_CLOCK = 4000000; // 4 MHz

    // BME280 registers
    const uint8_t WHO_AM_I_REG = 0xD0;

    const uint8_t RESET_REG = 0xE0;
    const uint8_t SOFT_RESET = 0xB6;

    const uint8_t CTRL_HUM_REG = 0xF2;
    const uint8_t STATUS_REG = 0xF3;
    const uint8_t CTRL_MEAS_REG = 0xF4;

    const uint8_t SLEEP_MODE = 0x00;

    const uint8_t CONFIG_REG = 0xF5;

    bool writeRegister(uint8_t subAddress, uint8_t data);
    void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
};

#endif
