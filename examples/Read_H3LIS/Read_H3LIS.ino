/*
  Read_H3LIS

  Prints readings from an H3LIS331DL high-g accelerometer over SPI, as fast
  as the sensor produces them. Pin numbers below are the Rotini V4 board;
  change them to match your wiring. For I2C use imu.begin(Wire) instead.
*/

#include <AlfredoFusion.h>

const int PIN_SPI_SCK = 6;
const int PIN_SPI_MISO = 7;
const int PIN_SPI_MOSI = 5;
const int PIN_ACCEL_CS = 47;

AlfredoH3LIS imu;

void setup() {
  Serial.begin(115200);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  if (!imu.begin(SPI, PIN_ACCEL_CS)) {
    while (true) {
      Serial.println("H3LIS331 not found, check wiring");
      delay(1000);
    }
  }
  imu.configure(400, 400);  // 400 Hz, +/-400 g (the defaults)

  Serial.println("x\ty\tz (g)");
}

void loop() {
  float x, y, z;
  if (imu.available() && imu.read(x, y, z)) {
    Serial.printf("%.2f\t%.2f\t%.2f\n", x, y, z);
  }
}
