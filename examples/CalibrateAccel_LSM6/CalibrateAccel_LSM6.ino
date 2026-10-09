/*
  CalibrateAccel_LSM6  (OPTIONAL - orientation works fine without this)

  Six-position accelerometer calibration. Factory accel offset costs roughly
  1-2 degrees of absolute tilt accuracy; this recovers most of it. Yaw is
  unaffected either way, so skip this unless you care about that last degree
  of pitch/roll.

  How to use: rest the board still on each of its six faces (+X up, -X up,
  +Y up, -Y up, +Z up, -Z up) in any order. Faces are detected and measured
  automatically whenever the board is held still - no typing needed. Exact
  90-degree alignment doesn't matter much; a tabletop is plenty flat. When
  all six are done, the values are saved to flash (ESP32) and a
  setAccelCalibration() line is printed to paste into your sketch.
*/

#include <AlfredoFusion.h>

AlfredoLSM6 imu;

const int SAMPLES_PER_FACE = 200;      // ~2 s at the 104 Hz sample rate
const float STILL_GYRO_RAD_S = 0.05f;  // ~3 deg/s
const float FACE_DOMINANCE_G = 0.85f;  // dominant axis must read at least this

// [axis 0..2][0 = axis up, 1 = axis down]; reading of the dominant axis, in g
float faceReading[3][2];
bool faceDone[3][2] = {{false, false}, {false, false}, {false, false}};

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
  Serial.println("Rest the board still on each of its 6 faces, in any order.");
}

void loop() {
  static const char axisName[3] = {'X', 'Y', 'Z'};

  // Accumulate one still measurement: average SAMPLES_PER_FACE samples,
  // starting over if the board moves partway through.
  float sum[3] = {0, 0, 0};
  int n = 0;
  while (n < SAMPLES_PER_FACE) {
    if (!imu.available()) continue;
    float ax, ay, az, gx, gy, gz;
    if (!imu.read(ax, ay, az, gx, gy, gz)) continue;
    if (fabsf(gx) > STILL_GYRO_RAD_S || fabsf(gy) > STILL_GYRO_RAD_S || fabsf(gz) > STILL_GYRO_RAD_S) {
      sum[0] = sum[1] = sum[2] = 0;  // moved: restart this measurement
      n = 0;
      continue;
    }
    sum[0] += ax;
    sum[1] += ay;
    sum[2] += az;
    n++;
  }
  float mean[3] = {sum[0] / n, sum[1] / n, sum[2] / n};

  // Which face is up?
  int axis = 0;
  if (fabsf(mean[1]) > fabsf(mean[axis])) axis = 1;
  if (fabsf(mean[2]) > fabsf(mean[axis])) axis = 2;
  if (fabsf(mean[axis]) < FACE_DOMINANCE_G) return;  // tilted between faces; keep waiting
  int side = mean[axis] > 0 ? 0 : 1;

  if (!faceDone[axis][side]) {
    faceDone[axis][side] = true;
    faceReading[axis][side] = mean[axis];
    Serial.print(mean[axis] > 0 ? '+' : '-');
    Serial.print(axisName[axis]);
    Serial.print(" up recorded (");
    Serial.print(mean[axis], 4);
    Serial.println(" g)");
  }

  for (int a = 0; a < 3; a++)
    for (int s = 0; s < 2; s++)
      if (!faceDone[a][s]) return;  // not finished yet

  // All six faces measured: offset centers each axis, scale normalizes its
  // up-minus-down span to exactly 2 g.
  float cal[6];
  for (int a = 0; a < 3; a++) {
    cal[a] = (faceReading[a][0] + faceReading[a][1]) / 2.0f;      // offsets
    cal[a + 3] = 2.0f / (faceReading[a][0] - faceReading[a][1]);  // scales
  }

#if defined(ARDUINO_ARCH_ESP32)
  // Save where fusion.loadCalibrationFromFlash() finds it at boot.
  AlfredoVQF fusion;
  fusion.saveAccelCalibrationToFlash(cal[0], cal[1], cal[2], cal[3], cal[4], cal[5]);
  Serial.println("\nDone! Saved to flash - loadCalibrationFromFlash() picks it up.");
#else
  Serial.println("\nDone!");
#endif
  Serial.println("Or paste this after creating your AlfredoVQF object:\n");
  Serial.print("  fusion.setAccelCalibration(");
  for (int i = 0; i < 6; i++) {
    Serial.print(cal[i], 5);
    Serial.print(i < 5 ? ", " : ");\n");
  }

  while (true) delay(1000);
}
