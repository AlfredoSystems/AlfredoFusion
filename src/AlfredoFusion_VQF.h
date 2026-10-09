#pragma once

// AlfredoVQF - 6DOF (gyro + accelerometer) orientation from the VQF sensor
// fusion algorithm (src/vqf/, https://github.com/dlaidig/vqf, Laidig & Seel
// 2023) with automatic online gyroscope bias calibration. No user calibration
// steps are required: gyro bias is measured and removed whenever the sensor is
// briefly at rest, and is also tracked continuously during motion.
//
// Units in: gyroscope rad/s, accelerometer g. Yaw/pitch/roll out in radians,
// standard ZYX convention (yaw about Z, pitch about Y, roll about X). In 6DOF
// there is no absolute heading reference, so yaw is relative to the
// orientation at startup. Feed the magnetometer too (updateMag) and call
// useMagnetometer(true) if you want an absolute 9DOF heading.
//
// On top of the raw filter this adds: unit conversion (VQF wants m/s^2),
// optional accelerometer offset/scale and gyro sensitivity calibration,
// measured-dt correction for the IMU's internal oscillator error, and on the
// ESP32: flash (NVS) persistence of the calibration values and thread safety
// (every method may be called from any task; the raw `vqf` member is the one
// exception).

#include <Arduino.h>
#include "vqf/vqf.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

class AlfredoVQF {
  public:
    // Nominal sample rates. Pass 0 (or nothing) for accelSampleRateHz to use
    // the gyro rate for both (most IMUs sample both at the same rate).
    explicit AlfredoVQF(float gyroSampleRateHz = 104.0f, float accelSampleRateHz = 0.0f);

    // ---- feeding the filter ----

    // Feed each sensor at its own rate. Gyro in rad/s, accel in g.
    void updateGyro(float gx, float gy, float gz);
    void updateAccel(float ax, float ay, float az);

    // Interrupt-driven gyro variant: dt is the measured time (seconds)
    // between this data-ready edge and the previous one. The IMU's internal
    // oscillator is off by a few percent part-to-part, which would become
    // the same percent of angle error on every rotation; the measured dt is
    // outlier-gated and heavily low-pass filtered, then used to correct the
    // integration. Intervals from missed edges (~2x nominal) never pollute
    // the rate estimate.
    void updateGyro(float gx, float gy, float gz, float dt);

    // Convenience for same-rate use: updateGyro + updateAccel.
    void update(float gx, float gy, float gz, float ax, float ay, float az);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    // Magnetometer, any unit (VQF only uses the direction). Only affects the
    // output once useMagnetometer(true) has been called.
    void updateMag(float mx, float my, float mz);
    void useMagnetometer(bool enable);  // 9DOF output: absolute yaw, mag disturbance rejection

    float getMeasuredSampleRate();  // gyro Hz, from the filtered dt (starts at nominal)

    // ---- orientation ----

    // Orientation of the sensor body relative to a Z-up world frame.
    // Quaternion is Hamilton convention, body-to-world.
    void getQuaternion(float& w, float& x, float& y, float& z);
    float getYaw();    // rad, about world Z (up), CCW positive viewed from above
    float getPitch();  // rad, ZYX Euler, about Y
    float getRoll();   // rad, ZYX Euler, about X

    // ---- automatic gyro bias calibration ----

    bool isResting();  // true while VQF detects the sensor is at rest (bias is being measured)

    // Rest-detection diagnostics: each sensor's deviation from its slow
    // low-pass reference, relative to its rest threshold. Both must stay
    // below 1.0 continuously for ~1.5 s for rest to be detected. If one
    // value keeps popping above 1.0, that sensor is what's vetoing rest.
    void getRestDeviations(float& gyroRelative, float& accelRelative);

    void getGyroBias(float& x, float& y, float& z);  // current estimate, rad/s
    float getGyroBiasSigma();                        // its 1-sigma uncertainty, rad/s

    // Seed the bias estimate (e.g. at boot from values a previous run saved)
    // so yaw is accurate before the first rest. sigmaRadPerS expresses how
    // much to trust the seed; the default (~0.3 deg/s) is loose enough for
    // the online estimator to quickly correct what changed since the save.
    void seedGyroBias(float x, float y, float z, float sigmaRadPerS = 0.00524f);

    // ---- optional calibration (see the CalibrateAccel / CalibrateGyroScale examples) ----

    // Accelerometer calibration. Never required; improves absolute tilt
    // accuracy by ~1-2 deg. Applied as: corrected = (raw - offset) * scale, in g.
    void setAccelCalibration(float offsetX, float offsetY, float offsetZ,
                             float scaleX = 1.0f, float scaleY = 1.0f, float scaleZ = 1.0f);

    // Gyro sensitivity calibration. Never required; the chip's factory trim
    // is within ~1%, which costs up to ~3.6 deg per full rotation of yaw.
    // No online estimator can observe scale error in 6DOF, so this is the
    // only fix. Applied as: corrected = raw * scale.
    void setGyroScale(float scaleX, float scaleY, float scaleZ);
    void getGyroScale(float& scaleX, float& scaleY, float& scaleZ);

    void reset();  // forget orientation and bias state

#if defined(ARDUINO_ARCH_ESP32)
    // ---- flash (NVS) persistence, ESP32 only ----
    // Only the calibration values (accel offset/scale, gyro scale) persist;
    // gyro bias is deliberately not saved - it is re-measured from rest at
    // every boot, so no stale estimate can leak between sessions.

    // NVS namespace the values live under (max 15 characters). Set it before
    // loading if several boards/libraries share a chip.
    void setFlashNamespace(const char* ns) { flashNamespace = ns; }

    // Loads what the calibration examples saved; call once at startup.
    void loadCalibrationFromFlash();

    // Used by the calibration examples: applies the values and saves them
    // where loadCalibrationFromFlash() finds them.
    void saveAccelCalibrationToFlash(float offsetX, float offsetY, float offsetZ,
                                     float scaleX, float scaleY, float scaleZ);
    void saveGyroScaleToFlash(float scaleX, float scaleY, float scaleZ);
#endif

    // Direct access to the underlying VQF filter for advanced tuning
    // (setTauAcc, setRestDetectionThresholds, getQuat9D, ...). NOT guarded
    // by the wrapper's lock - only touch this from setup().
    VQF vqf;

  private:
    static VQFParams makeParams();
    void lock();
    void unlock();
    void updateGyroRaw(float gx, float gy, float gz, float dt);
    void updateAccelRaw(float ax, float ay, float az);
    void readQuat(float q[4]);

#if defined(ARDUINO_ARCH_ESP32)
    SemaphoreHandle_t mutex = NULL;
    const char* flashNamespace = "alfredofusion";
#endif
    bool magEnabled = false;
    float accelOffset[3] = {0.0f, 0.0f, 0.0f};
    float accelScale[3] = {1.0f, 1.0f, 1.0f};
    float gyroScale[3] = {1.0f, 1.0f, 1.0f};
    float nominalTs;
    float dtFiltered;
};
