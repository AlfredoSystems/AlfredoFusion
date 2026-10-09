#include "AlfredoFusion_VQF.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <Preferences.h>
static const char* KEY_ACCEL_CAL = "accCal";
static const char* KEY_GYRO_SCALE = "gyrScale";
#endif

static const float GRAVITY_M_S2 = 9.80665f;

AlfredoVQF::AlfredoVQF(float gyroSampleRateHz, float accelSampleRateHz)
    : vqf(makeParams(), 1.0f / gyroSampleRateHz,
          accelSampleRateHz > 0 ? 1.0f / accelSampleRateHz : 1.0f / gyroSampleRateHz),
      nominalTs(1.0f / gyroSampleRateHz),
      dtFiltered(1.0f / gyroSampleRateHz)
{
#if defined(ARDUINO_ARCH_ESP32)
    mutex = xSemaphoreCreateMutex();
#endif
}

// VQF tuning deltas from the published defaults.
VQFParams AlfredoVQF::makeParams()
{
    VQFParams params;
    // The default 2 deg/s bias clip assumes a better-trimmed gyro than some
    // parts have: rest detection is vetoed whenever any axis of the
    // low-passed raw gyro exceeds the clip, so a part with ~2 deg/s of bias
    // never rests and never gets calibrated (measured on real LSM6DSOX
    // boards: X-axis bias riding right at the clip). 5 deg/s accepts
    // real-world part-to-part bias while still rejecting a robot that is
    // actually rotating.
    params.biasClip = 5.0f;
    return params;
}

void AlfredoVQF::lock()
{
#if defined(ARDUINO_ARCH_ESP32)
    if (mutex != NULL) xSemaphoreTake(mutex, portMAX_DELAY);
#endif
}

void AlfredoVQF::unlock()
{
#if defined(ARDUINO_ARCH_ESP32)
    if (mutex != NULL) xSemaphoreGive(mutex);
#endif
}

// ---- feeding ----

void AlfredoVQF::updateGyroRaw(float gx, float gy, float gz, float dt)
{
    float k = 1.0f;
    if (dt > 0) {
        // Accept only intervals within 10% of the current estimate into the
        // rate filter: a missed data-ready edge shows up as ~2x nominal and
        // a fallback poll as an odd in-between value, and neither may
        // pollute the estimate. The gate is relative to the estimate itself,
        // so it still converges even if the oscillator error exceeds 10%.
        // Alpha 0.002 averages over ~500 samples: ISR latency jitter is a
        // few microseconds against a multi-millisecond period, so far under
        // 0.1% survives to the filtered rate.
        if (dt > 0.9f * dtFiltered && dt < 1.1f * dtFiltered) {
            dtFiltered += 0.002f * (dt - dtFiltered);
        }
        // Integrating the gyro over the true period at VQF's fixed nominal
        // period is equivalent to scaling angular rate by (true / nominal).
        // This corrects the strapdown integration - the accuracy-critical
        // part - while leaving filter coefficients untouched. Always use the
        // slow filtered period here, never the raw gap from a missed edge:
        // scaling a sample by ~2x "to preserve rotation" also scales the
        // gyro's DC bias, and VQF's rest detector reads that as a movement
        // transient - frequent misses then keep resetting the rest timer,
        // silently disabling bias calibration. A missed sample only costs
        // (current rate x one period) of angle, which is far cheaper.
        k = dtFiltered / nominalTs;
    }
    vqf_real_t gyr[3] = {gx * gyroScale[0] * k, gy * gyroScale[1] * k, gz * gyroScale[2] * k};
    vqf.updateGyr(gyr);
}

void AlfredoVQF::updateAccelRaw(float ax, float ay, float az)
{
    vqf_real_t acc[3] = {
        (ax - accelOffset[0]) * accelScale[0] * GRAVITY_M_S2,
        (ay - accelOffset[1]) * accelScale[1] * GRAVITY_M_S2,
        (az - accelOffset[2]) * accelScale[2] * GRAVITY_M_S2,
    };
    vqf.updateAcc(acc);
}

void AlfredoVQF::updateGyro(float gx, float gy, float gz)
{
    lock();
    updateGyroRaw(gx, gy, gz, 0);
    unlock();
}

void AlfredoVQF::updateGyro(float gx, float gy, float gz, float dt)
{
    lock();
    updateGyroRaw(gx, gy, gz, dt);
    unlock();
}

void AlfredoVQF::updateAccel(float ax, float ay, float az)
{
    lock();
    updateAccelRaw(ax, ay, az);
    unlock();
}

void AlfredoVQF::update(float gx, float gy, float gz, float ax, float ay, float az)
{
    lock();
    updateGyroRaw(gx, gy, gz, 0);
    updateAccelRaw(ax, ay, az);
    unlock();
}

void AlfredoVQF::update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    lock();
    updateGyroRaw(gx, gy, gz, dt);
    updateAccelRaw(ax, ay, az);
    unlock();
}

void AlfredoVQF::updateMag(float mx, float my, float mz)
{
    vqf_real_t mag[3] = {mx, my, mz};
    lock();
    vqf.updateMag(mag);
    unlock();
}

void AlfredoVQF::useMagnetometer(bool enable)
{
    lock();
    magEnabled = enable;
    unlock();
}

float AlfredoVQF::getMeasuredSampleRate()
{
    lock();
    float rate = 1.0f / dtFiltered;
    unlock();
    return rate;
}

// ---- orientation ----

void AlfredoVQF::readQuat(float q[4])
{
    vqf_real_t out[4];
    lock();
    if (magEnabled) vqf.getQuat9D(out);
    else vqf.getQuat6D(out);
    unlock();
    for (int i = 0; i < 4; i++) q[i] = out[i];
}

void AlfredoVQF::getQuaternion(float& w, float& x, float& y, float& z)
{
    float q[4];
    readQuat(q);
    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];
}

float AlfredoVQF::getYaw()
{
    float q[4];
    readQuat(q);
    return atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                  1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

float AlfredoVQF::getPitch()
{
    float q[4];
    readQuat(q);
    float s = 2.0f * (q[0] * q[2] - q[1] * q[3]);
    s = constrain(s, -1.0f, 1.0f);  // clamp so rounding can't push asinf out of domain
    return asinf(s);
}

float AlfredoVQF::getRoll()
{
    float q[4];
    readQuat(q);
    return atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                  1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
}

// ---- bias ----

bool AlfredoVQF::isResting()
{
    lock();
    bool resting = vqf.getRestDetected();
    unlock();
    return resting;
}

void AlfredoVQF::getRestDeviations(float& gyroRelative, float& accelRelative)
{
    vqf_real_t dev[2];
    lock();
    vqf.getRelativeRestDeviations(dev);
    unlock();
    gyroRelative = dev[0];
    accelRelative = dev[1];
}

void AlfredoVQF::getGyroBias(float& x, float& y, float& z)
{
    vqf_real_t bias[3];
    lock();
    vqf.getBiasEstimate(bias);
    unlock();
    x = bias[0];
    y = bias[1];
    z = bias[2];
}

float AlfredoVQF::getGyroBiasSigma()
{
    lock();
    float sigma = vqf.getBiasEstimate(0);
    unlock();
    return sigma;
}

void AlfredoVQF::seedGyroBias(float x, float y, float z, float sigmaRadPerS)
{
    vqf_real_t bias[3] = {x, y, z};
    lock();
    vqf.setBiasEstimate(bias, sigmaRadPerS);
    unlock();
}

// ---- calibration ----

void AlfredoVQF::setAccelCalibration(float offsetX, float offsetY, float offsetZ,
                                     float scaleX, float scaleY, float scaleZ)
{
    lock();
    accelOffset[0] = offsetX;
    accelOffset[1] = offsetY;
    accelOffset[2] = offsetZ;
    accelScale[0] = scaleX;
    accelScale[1] = scaleY;
    accelScale[2] = scaleZ;
    unlock();
}

void AlfredoVQF::setGyroScale(float scaleX, float scaleY, float scaleZ)
{
    lock();
    gyroScale[0] = scaleX;
    gyroScale[1] = scaleY;
    gyroScale[2] = scaleZ;
    unlock();
}

void AlfredoVQF::getGyroScale(float& scaleX, float& scaleY, float& scaleZ)
{
    lock();
    scaleX = gyroScale[0];
    scaleY = gyroScale[1];
    scaleZ = gyroScale[2];
    unlock();
}

void AlfredoVQF::reset()
{
    lock();
    vqf.resetState();
    unlock();
}

#if defined(ARDUINO_ARCH_ESP32)

void AlfredoVQF::loadCalibrationFromFlash()
{
    Preferences prefs;
    if (!prefs.begin(flashNamespace, true)) {
        return;  // nothing saved yet
    }
    float accCal[6];
    if (prefs.getBytes(KEY_ACCEL_CAL, accCal, sizeof(accCal)) == sizeof(accCal)) {
        setAccelCalibration(accCal[0], accCal[1], accCal[2], accCal[3], accCal[4], accCal[5]);
    }
    float gyrScale[3];
    if (prefs.getBytes(KEY_GYRO_SCALE, gyrScale, sizeof(gyrScale)) == sizeof(gyrScale)) {
        setGyroScale(gyrScale[0], gyrScale[1], gyrScale[2]);
    }
    prefs.end();
}

void AlfredoVQF::saveAccelCalibrationToFlash(float offsetX, float offsetY, float offsetZ,
                                             float scaleX, float scaleY, float scaleZ)
{
    setAccelCalibration(offsetX, offsetY, offsetZ, scaleX, scaleY, scaleZ);
    float cal[6] = {offsetX, offsetY, offsetZ, scaleX, scaleY, scaleZ};
    Preferences prefs;
    if (prefs.begin(flashNamespace)) {
        prefs.putBytes(KEY_ACCEL_CAL, cal, sizeof(cal));
        prefs.end();
    }
}

void AlfredoVQF::saveGyroScaleToFlash(float scaleX, float scaleY, float scaleZ)
{
    setGyroScale(scaleX, scaleY, scaleZ);
    float scale[3] = {scaleX, scaleY, scaleZ};
    Preferences prefs;
    if (prefs.begin(flashNamespace)) {
        prefs.putBytes(KEY_GYRO_SCALE, scale, sizeof(scale));
        prefs.end();
    }
}

#endif
