# AlfredoFusion

IMU drivers and sensor fusion for Arduino. Two things in one library:

1. **Short, simple drivers** for the sensors on Alfredo Systems boards:
   - `AlfredoLSM6` - ST LSM6 family accelerometer/gyroscope (LSM6DS3, DS3TR-C,
     DSL, DSM, DSO, DSOX, DSO32, DSR, DST, ISM330DHCX) over I2C
   - `AlfredoH3LIS` - ST H3LIS331DL high-g accelerometer over SPI or I2C
   - `AlfredoMMC5983` - MEMSIC MMC5983MA magnetometer over SPI or I2C
2. **`AlfredoVQF`** - orientation from the [VQF](https://github.com/dlaidig/vqf)
   filter (Laidig & Seel 2023), the best-published open-source IMU orientation
   filter, with automatic online gyro bias calibration.

[Alfredo-NoU3](https://github.com/AlfredoSystems/Alfredo-NoU3) and
[SimpleMelt](https://github.com/AlfredoSystems/SimpleMelt) are built on it, and
it works on its own with any board.

## Orientation in 10 lines

```cpp
#include <AlfredoFusion.h>

AlfredoLSM6 imu;
AlfredoVQF fusion(104.0f);  // nominal sample rate, matches the driver default

void setup() {
  Wire.begin();
  imu.begin(Wire);
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  if (imu.available() && imu.read(ax, ay, az, gx, gy, gz))  // g, rad/s
    fusion.update(gx, gy, gz, ax, ay, az);
  float yaw = fusion.getYaw();  // radians
}
```

**No calibration steps required.** Any time the sensor sits still for about
2 seconds (power-up included), VQF measures and removes the gyro bias, and it
keeps tracking bias during motion too, so temperature drift is handled. Pitch
and roll are absolute (gravity referenced). In 6DOF yaw is relative to the
startup heading and drifts slowly (a few degrees per minute or better once bias
estimation has converged). Feed `updateMag()` and call `useMagnetometer(true)`
for an absolute 9DOF heading.

The fusion core is sensor-agnostic: feed gyro in **rad/s** and accel in **g**
from any IMU at the rate you passed to the constructor.

## Drivers

All three follow the same pattern: `begin(bus...)` detects and configures the
chip with sensible defaults, `configure(...)` changes rate and range,
`available()` says a new sample is ready, `read(...)` returns it in real units
in the chip's own axes, and `enableInterrupt()` routes data-ready to the
chip's interrupt pin.

```cpp
AlfredoLSM6 imu;        imu.begin(Wire);            imu.configure(104, 4, 500);   // Hz, g, dps
AlfredoH3LIS accel;     accel.begin(SPI, csPin);    accel.configure(400, 400);    // Hz, g
AlfredoMMC5983 mag;     mag.begin(SPI, csPin);      mag.configure(100, 800, true);// Hz, bandwidth, auto set/reset
```

Units out: accel in g, gyro in rad/s, field in microteslas. Driver code is
header-only; include `AlfredoFusion.h` for everything or the individual
`AlfredoFusion_*.h` headers.

## AlfredoVQF API

- `AlfredoVQF(gyroRateHz, accelRateHz = gyroRateHz)` - nominal sample rates.
- `update(gx, gy, gz, ax, ay, az)` / `updateGyro(...)` / `updateAccel(...)` -
  feed samples at their nominal rates.
- `update(..., dt)` / `updateGyro(gx, gy, gz, dt)` - interrupt-driven
  variant: pass the measured seconds between data-ready edges. Filtered
  internally and used to correct for the IMU oscillator's few-percent rate
  error; missed-edge intervals are gated out. See the Orientation example.
- `updateMag(mx, my, mz)` + `useMagnetometer(true)` - 9DOF output.
- `getQuaternion(w, x, y, z)` - Hamilton, body-to-world, Z-up world frame.
- `getYaw()` / `getPitch()` / `getRoll()` - radians, ZYX Euler.
- `isResting()` - true while VQF's rest detection is active.
- `getGyroBias(x, y, z)` / `getGyroBiasSigma()` / `seedGyroBias(...)` - the
  bias estimate, rad/s.
- `setAccelCalibration(ox, oy, oz, sx, sy, sz)` / `setGyroScale(sx, sy, sz)` -
  optional, from the calibration examples.
- ESP32 only: `loadCalibrationFromFlash()`, `saveAccelCalibrationToFlash(...)`,
  `saveGyroScaleToFlash(...)`, `setFlashNamespace(ns)` - NVS persistence of
  the calibration values. On the ESP32 every method is also thread-safe.
- `.vqf` - the underlying `VQF` object, public, for advanced tuning.

## Examples

- **Orientation_LSM6** - the one to start with: yaw/pitch/roll, rest status
  and live gyro bias; polling or interrupt-driven with measured-dt correction.
- **CalibrateAccel_LSM6** - optional six-position accel calibration
  (recovers ~1-2 degrees of absolute tilt).
- **CalibrateGyroScale_LSM6** - optional gyro sensitivity calibration
  (recovers up to ~3.6 degrees per turn of yaw).
- **Read_LSM6**, **Read_H3LIS**, **Read_MMC5983** - print raw readings.

## License

MIT. The vendored `src/vqf/` is Daniel Laidig's VQF, also MIT; the only change
from upstream is `VQF_SINGLE_PRECISION`.
