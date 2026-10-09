/*
  Read_MMC5983

  Prints magnetic field readings (microteslas) from an MMC5983MA over I2C in
  continuous mode. For SPI use mag.begin(SPI, csPin) instead.
*/

#include <AlfredoFusion.h>

AlfredoMMC5983 mag;

void setup() {
  Serial.begin(115200);

  // Alfredo NoU3: the magnetometer is on the IMU bus - use these instead:
  //   Wire1.begin(35, 36, 400000);
  //   mag.begin(Wire1);
  Wire.begin();

  if (!mag.begin(Wire)) {
    while (true) {
      Serial.println("MMC5983MA not found, check wiring");
      delay(1000);
    }
  }
  mag.configure(100, 800, true);  // 100 Hz, 800 Hz bandwidth, auto set/reset (the defaults)

  Serial.println("x\ty\tz (uT)");
}

void loop() {
  float x, y, z;
  if (mag.available() && mag.read(x, y, z)) {
    Serial.printf("%.1f\t%.1f\t%.1f\n", x, y, z);
  }
}
