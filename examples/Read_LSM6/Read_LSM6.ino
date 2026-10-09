/*
  Read_LSM6

  Prints raw accelerometer (g) and gyroscope (deg/s) readings from any
  LSM6-family IMU over I2C, as fast as the sensor produces them.
*/

#include <AlfredoFusion.h>

AlfredoLSM6 imu;

void setup() {
  Serial.begin(115200);

  // Alfredo NoU3: the IMU is on its own bus - use these two lines instead:
  //   Wire1.begin(35, 36, 400000);
  //   imu.begin(Wire1);
  Wire.begin();

  if (!imu.begin(Wire)) {
    while (true) {
      Serial.println("LSM6 not found, check wiring");
      delay(1000);
    }
  }
  // Optional: imu.configure(odrHz, accelRangeG, gyroRangeDps), e.g.
  // imu.configure(208, 8, 1000);

  Serial.print("found ");
  Serial.println(imu.name());
  Serial.println("ax\tay\taz (g)\tgx\tgy\tgz (deg/s)");
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  if (imu.available() && imu.read(ax, ay, az, gx, gy, gz)) {
    Serial.printf("%.3f\t%.3f\t%.3f\t%.2f\t%.2f\t%.2f\n",
                  ax, ay, az, degrees(gx), degrees(gy), degrees(gz));
  }
}
