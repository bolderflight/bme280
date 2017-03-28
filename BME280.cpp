/*
BME280.cpp
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

// Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC 
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || \
	defined(__MK66FX1M0__) || defined(__MKL26Z64__)

#include "Arduino.h"
#include "BME280.h"
#include "i2c_t3.h"  // I2C library
#include "SPI.h" // SPI Library

/* BME280 object, input the I2C address and I2C bus */
BME280::BME280(uint8_t address, uint8_t bus){
  _address = address; // I2C address
  _bus = bus; // I2C bus
  _userDefI2C = false; // automatic I2C setup
  _useSPI = false; // set to use I2C instead of SPI
}

/* BME280 object, input the I2C address, I2C bus, and I2C pins */
BME280::BME280(uint8_t address, uint8_t bus, i2c_pins pins){
  _address = address; // I2C address
  _bus = bus; // I2C bus
  _pins = pins; // I2C pins
  _pullups = I2C_PULLUP_EXT; // I2C pullups
  _userDefI2C = true; // user defined I2C
  _useSPI = false; // set to use I2C instead of SPI
}

/* BME280 object, input the I2C address, I2C bus, I2C pins, and I2C pullups */
BME280::BME280(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups){
  _address = address; // I2C address
  _bus = bus; // I2C bus
  _pins = pins; // I2C pins
  _pullups = pullups; // I2C pullups
  _userDefI2C = true; // user defined I2C
  _useSPI = false; // set to use I2C instead of SPI
}

BME280::BME280(uint8_t csPin) {
  _csPin = csPin; // SPI CS Pin
  _mosiPin = MOSI_PIN_11; // SPI MOSI Pin, set to default
  _useSPI = true; // set to use SPI instead of I2C             
}

BME280::BME280(uint8_t csPin, spi_mosi_pin pin) {
  _csPin = csPin; // SPI CS Pin
  _mosiPin = pin; // SPI MOSI Pin
  _useSPI = true; // set to use SPI instead of I2C          
}

/* starts communication and sets up the BME280 */
int BME280::begin() {
  uint8_t data[1];

  if( _useSPI ){ // using SPI for communication

    // setting CS pin to output
    pinMode(_csPin,OUTPUT);

    // setting CS pin low to lock BME280 in SPI mode
    digitalWriteFast(_csPin,LOW);

    // delay for pin setting to take effect
    delay(1);

    // setting CS pin high
    digitalWriteFast(_csPin,HIGH);

    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)

      // configure and begin the SPI
      switch( _mosiPin ){

        case MOSI_PIN_7:  // SPI bus 0 alternate 1
          SPI.setMOSI(7);
          SPI.setMISO(8);
          SPI.setSCK(14);
          SPI.begin();
          break;
        case MOSI_PIN_11: // SPI bus 0 default
          SPI.setMOSI(11);
          SPI.setMISO(12);
          SPI.setSCK(13);
          SPI.begin();
          break;
      }
    #endif

    // Teensy 3.5 || Teensy 3.6 
    #if defined(__MK64FX512__) || defined(__MK66FX1M0__)

      // configure and begin the SPI
      switch( _mosiPin ){

        case MOSI_PIN_0:  // SPI bus 1 default
          SPI1.setMOSI(0);
          SPI1.setMISO(1);
          SPI1.setSCK(32);
          SPI1.begin();
          break;
        case MOSI_PIN_7:  // SPI bus 0 alternate 1
          SPI.setMOSI(7);
          SPI.setMISO(8);
          SPI.setSCK(14);
          SPI.begin();
          break;
        case MOSI_PIN_11: // SPI bus 0 default
          SPI.setMOSI(11);
          SPI.setMISO(12);
          SPI.setSCK(13);
          SPI.begin();
          break;
        case MOSI_PIN_21: // SPI bus 1 alternate
          SPI1.setMOSI(21);
          SPI1.setMISO(5);
          SPI1.setSCK(20);
          SPI1.begin();
          break;
        case MOSI_PIN_28: // SPI bus 0 alternate 2
          SPI.setMOSI(28);
          SPI.setMISO(39);
          SPI.setSCK(27);
          SPI.begin();
          break;
        case MOSI_PIN_44: // SPI bus 2 default
          SPI2.setMOSI(44);
          SPI2.setMISO(45);
          SPI2.setSCK(46);
          SPI2.begin();
          break;
        case MOSI_PIN_52: // SPI bus 2 alternate
          SPI2.setMOSI(52);
          SPI2.setMISO(51);
          SPI2.setSCK(53);
          SPI2.begin();
          break;
      }
    #endif

    // Teensy LC 
    #if defined(__MKL26Z64__)

      // configure and begin the SPI
      switch( _mosiPin ){

        case MOSI_PIN_0:  // SPI bus 1 default
          SPI1.setMOSI(0);
          SPI1.setMISO(1);
          SPI1.setSCK(20);
          SPI1.begin();
          break;
        case MOSI_PIN_7:  // SPI bus 0 alternate 1
          SPI.setMOSI(7);
          SPI.setMISO(8);
          SPI.setSCK(14);
          SPI.begin();
          break;
        case MOSI_PIN_11: // SPI bus 0 default
          SPI.setMOSI(11);
          SPI.setMISO(12);
          SPI.setSCK(13);
          SPI.begin();
          break;
        case MOSI_PIN_21: // SPI bus 1 alternate
          SPI1.setMOSI(21);
          SPI1.setMISO(5);
          SPI1.setSCK(20);
          SPI1.begin();
          break;
      }
    #endif
  }
  else{ // using I2C for communication

    if( !_userDefI2C ) { // setup the I2C pins and pullups based on bus number if not defined by user
      /* setting the I2C pins, pullups, and protecting against _bus out of range */
      _pullups = I2C_PULLUP_EXT; // default to external pullups

      #if defined(__MK20DX128__) // Teensy 3.0
        _pins = I2C_PINS_18_19;
        _bus = 0;
      #endif

      #if defined(__MK20DX256__) // Teensy 3.1/3.2
        if(_bus == 1) {
          _pins = I2C_PINS_29_30;
        }
        else{
          _pins = I2C_PINS_18_19;
          _bus = 0;
        }
      #endif

      #if defined(__MK64FX512__) // Teensy 3.5
        if(_bus == 2) {
          _pins = I2C_PINS_3_4;
        }
        else if(_bus == 1) {
          _pins = I2C_PINS_37_38;
        }
        else{
          _pins = I2C_PINS_18_19;
          _bus = 0;
        }
      #endif

      #if defined(__MK66FX1M0__) // Teensy 3.6
        if(_bus == 3) {
          _pins = I2C_PINS_56_57;
        }
        else if(_bus == 2) {
          _pins = I2C_PINS_3_4;
        }
        else if(_bus == 1) {
          _pins = I2C_PINS_37_38;
        }
        else{
          _pins = I2C_PINS_18_19;
          _bus = 0;
        }
      #endif

      #if defined(__MKL26Z64__) // Teensy LC
        if(_bus == 1) {
          _pins = I2C_PINS_22_23;
        }
        else{
          _pins = I2C_PINS_18_19;
          _bus = 0;
        }
      #endif
    }

    // starting the I2C bus
    i2c_t3(_bus).begin(I2C_MASTER, 0, _pins, _pullups, _i2cRate);
  }

  // set to sleep mode
  if( !writeRegister(CTRL_MEAS_REG,SLEEP_MODE) ){
      return -1;
  }

  // reset the BME280
  writeRegister(RESET_REG,SOFT_RESET);

  // wait for power up
  delay(10);

  // check the who am i register
  readRegisters(WHO_AM_I_REG,1,data);
  if(data[0]!=0x60) {
    return -1;
  }

  // successful init, return 0
  return 0;
}

/* writes a byte to BME280 register given a register address and data */
bool BME280::writeRegister(uint8_t subAddress, uint8_t data){
  uint8_t buff[1];

  /* write data to device */
  if( _useSPI ){

    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)

      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
        digitalWriteFast(_csPin,LOW); // select the BME280 chip
        SPI.transfer(subAddress); // write the register address
        SPI.transfer(data); // write the data
        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI.endTransaction(); // end the transaction
      }
    #endif

    // Teensy 3.5 || Teensy 3.6 
    #if defined(__MK64FX512__) || defined(__MK66FX1M0__)

      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
        digitalWriteFast(_csPin,LOW); // select the BME280 chip
        SPI.transfer(subAddress); // write the register address
        SPI.transfer(data); // write the data
        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI.endTransaction(); // end the transaction
      }
      else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
        SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
        digitalWriteFast(_csPin,LOW); // select the BME280 chip
        SPI1.transfer(subAddress); // write the register address
        SPI1.transfer(data); // write the data
        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI1.endTransaction(); // end the transaction
      }
      else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
        SPI2.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
        digitalWriteFast(_csPin,LOW); // select the BME280 chip
        SPI2.transfer(subAddress); // write the register address
        SPI2.transfer(data); // write the data
        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI2.endTransaction(); // end the transaction 
      }
    #endif

    // Teensy LC 
    #if defined(__MKL26Z64__)

      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
        digitalWriteFast(_csPin,LOW); // select the BME280 chip
        SPI.transfer(subAddress); // write the register address
        SPI.transfer(data); // write the data
        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI.endTransaction(); // end the transaction
      }
      else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
        SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
        digitalWriteFast(_csPin,LOW); // select the BME280 chip
        SPI1.transfer(subAddress); // write the register address
        SPI1.transfer(data); // write the data
        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI1.endTransaction(); // end the transaction
      }
    #endif
  }
  else{
    i2c_t3(_bus).beginTransmission(_address); // open the device
    i2c_t3(_bus).write(subAddress); // write the register address
    i2c_t3(_bus).write(data); // write the data
    i2c_t3(_bus).endTransmission();
  }

  delay(1); 

  /* read back the register */
  readRegisters(subAddress,sizeof(buff),&buff[0]);

  /* check the read back register against the written register */
  if(buff[0] == data) {
    return true;
  }
  else{
    return false;
  }
}

/* reads registers from BME280 given a starting register address, number of bytes, and a pointer to store data */
void BME280::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){

  if( _useSPI ){

    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)

      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
        // begin the transaction
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        digitalWriteFast(_csPin,LOW); // select the BME280 chip

        SPI.transfer(subAddress | SPI_READ); // specify the starting register address

        for(uint8_t i = 0; i < count; i++){
          dest[i] = SPI.transfer(0x00); // read the data
        }

        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI.endTransaction(); // end the transaction
      }
    #endif

    // Teensy 3.5 || Teensy 3.6 
    #if defined(__MK64FX512__) || defined(__MK66FX1M0__)

      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
        // begin the transaction
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        digitalWriteFast(_csPin,LOW); // select the BME280 chip

        SPI.transfer(subAddress | SPI_READ); // specify the starting register address

        for(uint8_t i = 0; i < count; i++){
          dest[i] = SPI.transfer(0x00); // read the data
        }

        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI.endTransaction(); // end the transaction
      }
      else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
        // begin the transaction
        SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        digitalWriteFast(_csPin,LOW); // select the BME280 chip

        SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

        for(uint8_t i = 0; i < count; i++){
          dest[i] = SPI1.transfer(0x00); // read the data
        }

        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI1.endTransaction(); // end the transaction
      }
      else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
        // begin the transaction
        SPI2.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        digitalWriteFast(_csPin,LOW); // select the BME280 chip

        SPI2.transfer(subAddress | SPI_READ); // specify the starting register address

        for(uint8_t i = 0; i < count; i++){
          dest[i] = SPI.transfer(0x00); // read the data
        }

        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI2.endTransaction(); // end the transaction
      }
    #endif

    // Teensy LC 
    #if defined(__MKL26Z64__)

      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
        // begin the transaction
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        digitalWriteFast(_csPin,LOW); // select the BME280 chip

        SPI.transfer(subAddress | SPI_READ); // specify the starting register address

        for(uint8_t i = 0; i < count; i++){
          dest[i] = SPI.transfer(0x00); // read the data
        }

        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI.endTransaction(); // end the transaction
      }
      else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
        // begin the transaction
        SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        digitalWriteFast(_csPin,LOW); // select the BME280 chip

        SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

        for(uint8_t i = 0; i < count; i++){
          dest[i] = SPI1.transfer(0x00); // read the data
        }

        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI1.endTransaction(); // end the transaction
      }

    #endif
  }
  else{
    i2c_t3(_bus).beginTransmission(_address); // open the device
    i2c_t3(_bus).write(subAddress); // specify the starting register address
    i2c_t3(_bus).endTransmission(false);

    i2c_t3(_bus).requestFrom(_address, count); // specify the number of bytes to receive

    uint8_t i = 0; // read the data into the buffer
    while( i2c_t3(_bus).available() ){
      dest[i++] = i2c_t3(_bus).readByte();
    }
  }
}

#endif
