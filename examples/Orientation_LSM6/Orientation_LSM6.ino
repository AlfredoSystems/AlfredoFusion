/*
  Orientation_LSM6

  Reads an LSM6-family IMU over I2C and fuses it with AlfredoVQF to get
  orientation (yaw, pitch, roll and a quaternion).

  No calibration steps are needed. Keep the board still for ~2 seconds at
  any point (power-up is easiest) and VQF automatically measures and removes
  the gyro bias; it keeps re-learning bias during rests and while moving, so
  it also tracks temperature drift. Watch the "bias" column settle and yaw
  drift nearly stop after the first rest.

  Yaw is relative to the startup heading (6DOF has no compass), so it starts
  at 0 and refers to wherever the board was pointing at reset.

  Wiring: if the IMU's INT1 pin is connected, set INTERRUPT_PIN and the
  sketch times each sample from the data-ready edge. The measured interval
  corrects the IMU's internal oscillator error (a few percent part-to-part;
  watch "rate" differ from the nominal 104.00). With INTERRUPT_PIN = -1 the
  sketch polls instead, which works fine too.
*/

#include <AlfredoFusion.h>

const int INTERRUPT_PIN = -1;  // LSM6 INT1 pin, or -1 to poll

AlfredoLSM6 imu;
AlfredoVQF fusion(104.0f);  // nominal sample rate; must match imu.configure()

volatile bool imuReady = false;
volatile uint32_t imuReadyAtMicros = 0;

void IRAM_ATTR onImuReady() {
  imuReadyAtMicros = micros();
  imuReady = true;
}

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
  Serial.print("found ");
  Serial.println(imu.name());

#if defined(ARDUINO_ARCH_ESP32)
  // Values saved by the CalibrateAccel / CalibrateGyroScale examples, if run
  fusion.loadCalibrationFromFlash();
#endif

  if (INTERRUPT_PIN >= 0) {
    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), onImuReady, RISING);
    imu.enableInterrupt();
  }

  Serial.println("rate\tyaw\tpitch\troll (deg)\tresting\tbias x/y/z (deg/s)");
}

void loop() {
  static uint32_t lastEdgeMicros = 0;
  float ax, ay, az, gx, gy, gz;

  if (INTERRUPT_PIN >= 0) {
    if (!imuReady) return;
    noInterrupts();
    uint32_t edgeMicros = imuReadyAtMicros;
    imuReady = false;
    interrupts();

    if (imu.read(ax, ay, az, gx, gy, gz)) {
      // Unsigned subtraction stays correct across micros() rollover.
      float dt = lastEdgeMicros ? (edgeMicros - lastEdgeMicros) * 1e-6f : 0;
      lastEdgeMicros = edgeMicros;
      fusion.update(gx, gy, gz, ax, ay, az, dt);
    }
  } else {
    if (!imu.available()) return;
    if (imu.read(ax, ay, az, gx, gy, gz)) fusion.update(gx, gy, gz, ax, ay, az);
  }

  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs >= 100) {
    lastPrintMs = millis();
    float bx, by, bz;
    fusion.getGyroBias(bx, by, bz);
    Serial.printf("%.2f\t%.1f\t%.1f\t%.1f\t%d\t%.3f\t%.3f\t%.3f\n",
                  fusion.getMeasuredSampleRate(),
                  degrees(fusion.getYaw()), degrees(fusion.getPitch()), degrees(fusion.getRoll()),
                  fusion.isResting(), degrees(bx), degrees(by), degrees(bz));
  }
}
