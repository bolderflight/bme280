/*
BME280.cpp
Brian R Taylor
brian.taylor@bolderflight.com
2017-03-29

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

  // reset the BME280
  writeRegister(RESET_REG,SOFT_RESET);

  // wait for power up
  delay(10);

  // check the who am i register
  readRegisters(WHO_AM_I_REG,1,data);
  if(data[0]!=0x60) {
    return -1;
  }

  // check that BME280 is not copying calibration data
  readRegisters(STATUS_REG,1,data);
  while((data[0] & 0x01)!=0) {
    delay(1);
    readRegisters(STATUS_REG,1,data);
  }

  // read the calibration data
  readCalibration();

  // setup sensor to default values
  if(configureBME280() != 0) {
    return -1;
  }

  // successful init, return 0
  return 0;
}

/* configure the BME280 sensor */
int BME280::configureBME280() {
  // set to sleep mode
  if( !writeRegister(CTRL_MEAS_REG,MODE_SLEEP) ){
      return -1;
  }
  // humidity sensor configuration
  if( !writeRegister(CTRL_HUM_REG,_Hsampling) ){
      return -1;
  }
  // standby time, iirc, and spi 3 wire configuration
  if( !writeRegister(CONFIG_REG,( (_standby << 5) | (_iirc << 3) | _spi3w_en ) ) ){
      return -1;
  }

  // pressure, temperature, mode configuration
  if( !writeRegister(CTRL_MEAS_REG, ( (_Tsampling << 5) | (_Psampling << 3) | _mode ) ) ){
      return -1;
  }

  // successful configuration, return 0
  return 0;
}

/* read the BME280 calibration data */
void BME280::readCalibration() {
  uint8_t data[2];

  readRegisters(DIG_T1_REG,2,data);
  _dig_T1 = (uint16_t) data[1] << 8 | data[0];
  readRegisters(DIG_T2_REG,2,data);
  _dig_T2 = (int16_t) data[1] << 8 | data[0]; 
  readRegisters(DIG_T3_REG,2,data);
  _dig_T3 = (int16_t) data[1] << 8 | data[0]; 

  readRegisters(DIG_P1_REG,2,data);
  _dig_P1 = (uint16_t) data[1] << 8 | data[0];
  readRegisters(DIG_P2_REG,2,data);
  _dig_P2 = (int16_t) data[1] << 8 | data[0]; 
  readRegisters(DIG_P3_REG,2,data);
  _dig_P3 = (int16_t) data[1] << 8 | data[0]; 
  readRegisters(DIG_P4_REG,2,data);
  _dig_P4 = (int16_t) data[1] << 8 | data[0];
  readRegisters(DIG_P5_REG,2,data);
  _dig_P5 = (int16_t) data[1] << 8 | data[0]; 
  readRegisters(DIG_P6_REG,2,data);
  _dig_P6 = (int16_t) data[1] << 8 | data[0]; 
  readRegisters(DIG_P7_REG,2,data);
  _dig_P7 = (int16_t) data[1] << 8 | data[0];
  readRegisters(DIG_P8_REG,2,data);
  _dig_P8 = (int16_t) data[1] << 8 | data[0]; 
  readRegisters(DIG_P9_REG,2,data);
  _dig_P9 = (int16_t) data[1] << 8 | data[0]; 

  readRegisters(DIG_H1_REG,1,data);
  _dig_H1 = data[0];
  readRegisters(DIG_H2_REG,2,data);
  _dig_H2 = (int16_t) data[1] << 8 | data[0]; 
  readRegisters(DIG_H3_REG,1,data);
  _dig_H3 = data[0];
  readRegisters(DIG_H4_REG,2,data);
  _dig_H4 = (int16_t) data[0] << 4 | (data[1]&(0x0F));
  readRegisters(DIG_H5_REG,2,data);
  _dig_H5 = (int16_t) data[1] << 4 | data[0] >> 4;
  readRegisters(DIG_H6_REG,1,data);
  _dig_H6 = (int8_t) data[0];
}

/* sets the pressure, temperature, and humidity oversampling */
int BME280::setSampling(bme280_sampling Psampling, bme280_sampling Tsampling, bme280_sampling Hsampling) {
  _Psampling = Psampling;
  _Tsampling = Tsampling;
  _Hsampling = Hsampling;

  // setup sensor
  if(configureBME280() != 0) {
    return -1;
  }

  // success, return 0
  return 0;
}

/* sets the BME280 IIR filter coefficient */
int BME280::setFilter(bme280_iirc iirc) {
  _iirc = iirc;

  // setup sensor
  if(configureBME280() != 0) {
    return -1;
  }

  // success, return 0
  return 0;
}

/* sets the BME280 standby time */
int BME280::setStandby(bme280_standby standby) {
  _standby = standby;

  // setup sensor
  if(configureBME280() != 0) {
    return -1;
  }

  // success, return 0
  return 0;
}

/* gets temperature */
float BME280::getTemperature() {
  float pressure, temperature, humidity;
  getData(&pressure, &temperature, &humidity);
  return temperature;
}

/* gets pressure */
float BME280::getPressure() {
  float pressure, temperature, humidity;
  getData(&pressure, &temperature, &humidity);
  return pressure;
}

/* gets humidity */
float BME280::getHumidity() {
  float pressure, temperature, humidity;
  getData(&pressure, &temperature, &humidity);
  return humidity;
}

/* gets temperature, pressure, and humidity data from the BME280 */
void BME280::getData(float* pressure, float* temperature, float* humidity) {
  int32_t t_fine;
  int32_t pressureCounts, temperatureCounts, humidityCounts;
  getDataCounts(&pressureCounts, &temperatureCounts, &humidityCounts);
  compensateTemperature(temperatureCounts,&t_fine,temperature);
  compensatePressure(pressureCounts,t_fine,pressure);
  compensateHumidity(humidityCounts,t_fine,humidity);
}

/* compensates the temperature measurement from the BME280 */
void BME280::compensateTemperature(int32_t temperatureCounts, int32_t* t_fine, float* temperature) {
  int32_t var1, var2, T;
  var1=((((temperatureCounts>>3)-((int32_t)_dig_T1<<1)))*((int32_t)_dig_T2))>>11;
  var2=(((((temperatureCounts>>4)-((int32_t)_dig_T1))*((temperatureCounts>>4)-((int32_t)_dig_T1)))>>12)
    *((int32_t)_dig_T3))>>14;
  *t_fine=var1+var2;
  T=(*t_fine*5+128)>>8;
  *temperature = ((float)T)/100.0f;
}

/* compensates the pressure measurement from the BME280 */
void BME280::compensatePressure(int32_t pressureCounts, int32_t t_fine, float* pressure) {
  int64_t var1, var2, p;
  var1=((int64_t)t_fine)-128000;
  var2=var1*var1*(int64_t)_dig_P6;
  var2=var2+((var1*(int64_t)_dig_P5)<<17);
  var2=var2+(((int64_t)_dig_P4)<<35);
  var1=((var1*var1*(int64_t)_dig_P3)>>8)+((var1*(int64_t)_dig_P2)<<12);
  var1=(((((int64_t)1)<<47)+var1))*((int64_t)_dig_P1)>>33;
  if(var1==0) { 
    *pressure = 0.0f;
  } else {
    p=1048576-pressureCounts;
    p=(((p<<31)-var2)*3125)/var1;
    var1=(((int64_t)_dig_P9)*(p>>13)*(p>>13))>>25;
    var2=(((int64_t)_dig_P8)*p)>>19;
    p=((p+var1+var2)>>8)+(((int64_t)_dig_P7)<<4);
    *pressure=(float)((uint32_t)p)/256.0f;
  }
}

/* compensates the humidity measurement from the BME280 */
void BME280::compensateHumidity(int32_t humidityCounts, int32_t t_fine, float* humidity) {
  int32_t v_x1_u32r;
  v_x1_u32r=(t_fine-((int32_t)76800));
  v_x1_u32r=(((((humidityCounts<<14)-(((int32_t)_dig_H4)<<20)-(((int32_t)_dig_H5)*v_x1_u32r))+
    ((int32_t)16384))>>15)*(((((((v_x1_u32r*((int32_t)_dig_H6))>>10)*
    (((v_x1_u32r*((int32_t)_dig_H3))>>11)+((int32_t)32768)))>>10)+
    ((int32_t)2097152))*((int32_t)_dig_H2)+8192)>>14));
  v_x1_u32r=(v_x1_u32r-(((((v_x1_u32r>>15)*(v_x1_u32r>>15))>>7)*((int32_t)_dig_H1))>>4));
  v_x1_u32r=(v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r=(v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  *humidity=(float)((uint32_t)(v_x1_u32r>>12))/1024.0f;
}

/* returns counts for temperature, pressure, and humidity */
void BME280::getDataCounts(int32_t* pressureCounts, int32_t* temperatureCounts, int32_t* humidityCounts){
  uint8_t data[8];
  readRegisters(DATA_REG,8,data);
  *pressureCounts = (int32_t)((((uint32_t)data[0]) << 12) | (((uint32_t)data[1]) << 4) | (((uint32_t)data[2]&0xF0) >> 4));
  *temperatureCounts = (int32_t)((((uint32_t)data[3]) << 12) | (((uint32_t)data[4]) << 4) | (((uint32_t)data[5]&0xF0) >> 4));
  *humidityCounts =  (int32_t)((((uint32_t)data[6]) << 8) + ((uint32_t)data[7]));
}

/* writes a byte to BME280 register given a register address and data */
bool BME280::writeRegister(uint8_t subAddress, uint8_t data){
  uint8_t buff[1];

  /* write data to device */
  if( _useSPI ){

    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)

      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0)); // begin the transaction
        digitalWriteFast(_csPin,LOW); // select the BME280 chip
        SPI.transfer(subAddress & ~SPI_READ); // write the register address
        SPI.transfer(data); // write the data
        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI.endTransaction(); // end the transaction
      }
    #endif

    // Teensy 3.5 || Teensy 3.6 
    #if defined(__MK64FX512__) || defined(__MK66FX1M0__)

      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0)); // begin the transaction
        digitalWriteFast(_csPin,LOW); // select the BME280 chip
        SPI.transfer(subAddress & ~SPI_READ); // write the register address
        SPI.transfer(data); // write the data
        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI.endTransaction(); // end the transaction
      }
      else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
        SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0)); // begin the transaction
        digitalWriteFast(_csPin,LOW); // select the BME280 chip
        SPI1.transfer(subAddress & ~SPI_READ); // write the register address
        SPI1.transfer(data); // write the data
        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI1.endTransaction(); // end the transaction
      }
      else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
        SPI2.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0)); // begin the transaction
        digitalWriteFast(_csPin,LOW); // select the BME280 chip
        SPI2.transfer(subAddress & ~SPI_READ); // write the register address
        SPI2.transfer(data); // write the data
        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI2.endTransaction(); // end the transaction 
      }
    #endif

    // Teensy LC 
    #if defined(__MKL26Z64__)

      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0)); // begin the transaction
        digitalWriteFast(_csPin,LOW); // select the BME280 chip
        SPI.transfer(subAddress & ~SPI_READ); // write the register address
        SPI.transfer(data); // write the data
        digitalWriteFast(_csPin,HIGH); // deselect the BME280 chip
        SPI.endTransaction(); // end the transaction
      }
      else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
        SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0)); // begin the transaction
        digitalWriteFast(_csPin,LOW); // select the BME280 chip
        SPI1.transfer(subAddress & ~SPI_READ); // write the register address
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

  delay(10); 

  /* read back the register */
  readRegisters(subAddress,1,buff);

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
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
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
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
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
        SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
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
        SPI2.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
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
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
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
        SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
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
