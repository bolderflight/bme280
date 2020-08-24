/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "bme280/bme280.h"

sensors::Bme280 bme(&Wire, 0x76);

int main() {
  Serial.begin(115200);
  while(!Serial) {}
  bool status = bme.Begin();
  while (1) {
    status = bme.Read();
    float p = bme.pressure_pa();
    float t = bme.die_temperature_c();
    Serial.print(p);
    Serial.print("\t");
    Serial.print(t);
    Serial.print("\t");
    Serial.print(status);
    Serial.print("\n");
    delay(500);
  }
}