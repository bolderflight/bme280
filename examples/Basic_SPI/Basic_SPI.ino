/*
Basic_SPI.ino
Brian R Taylor
brian.taylor@bolderflight.com
2017-03-30 

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

#include "BME280.h"

/* A BME280 object using SPI chip select pin 10 */
BME280 bme(10);

int beginStatus;

void setup() {
  // serial to display data
  Serial.begin(115200);
  delay(5000);

  // begin communication with
  // BME280 and set to default
  // sampling, iirc, and standby settings
  beginStatus = bme.begin();
}

void loop() {
  float pressure, temperature, humidity;

  if(beginStatus<0){
    delay(1000);
    Serial.println("BME280 initialization unsuccessful");
    Serial.println("Check wiring or try cycling power");
    delay(10000);
  } else {
    /* get the individual data sources */
    /* This approach is only recommended if you only
     *  would like the specified data source (i.e. only
     *  want accel data) since multiple data sources
     *  would have a time skew between them.
     */

    // get the pressure data (Pa)
    pressure = bme.getPressure();

    // get the temperature data (C)
    temperature = bme.getTemperature();

    // get the humidity data (%RH)
    humidity = bme.getHumidity();

    // print the data
    // Serial.print(pressure);
    // Serial.print("\t");
    // Serial.print(temperature);
    // Serial.print("\t");
    // Serial.println(humidity);

    // delay a frame
    delay(50);

    /* get multiple data sources */
    /* In this approach we get data from multiple data
     *  sources (i.e. pressure, temperature, and humidity). 
     *  This is the recommended approach since there is no 
     *  time skew between sources - they are all synced.
     */

    // get the pressure (Pa), temperature (C),
    // and humidity data (%RH) all at once
    bme.getData(&pressure,&temperature,&humidity);

    // print the data
    Serial.print(pressure);
    Serial.print("\t");
    Serial.print(temperature);
    Serial.print("\t");
    Serial.println(humidity);

    // delay a frame
    delay(50);
  }
}
