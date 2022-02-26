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

#include "bme280.h"

/* BME-280 */
bfs::Bme280 bme;

int main() {
  /* Serial monitor for showing status and data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Initialize the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  /* Wire at the primary I2C address */
  bme.Config(&Wire, bfs::Bme280::I2C_ADDR_PRIM);
  /* Initialize the BME-280 */
  if (!bme.Begin()) {
    Serial.println("Error initializing communication with BME-280");
    while (1) {}
  }
  while (1) {
    if (bme.Read()) {
      Serial.print(bme.pres_pa());
      Serial.print("\t");
      Serial.print(bme.die_temp_c());
      Serial.print("\t");
      Serial.println(bme.humidity_rh());
    }
    delay(100);
  }
}
