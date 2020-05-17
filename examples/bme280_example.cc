/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "bme280/bme280.h"

int main() {
  Serial.begin(115200);
  while(!Serial) {}
  sensors::Bme280 bme(&SPI, 5);
  bool status = bme.Begin();
  while (1) {
    status = bme.Read();
    Pressure p = bme.pressure();
    Temperature t = bme.die_temperature();
    Serial.print(p.pa());
    Serial.print("\t");
    Serial.print(t.c());
    Serial.print("\t");
    Serial.print(status);
    Serial.print("\n");
    delay(500);
  }
}