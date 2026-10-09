# Changelog

## 1.0 - 2026-10-09

First release.

### Added

- `AlfredoLSM6`: I2C driver for the ST LSM6 family (LSM6DS3, DS3TR-C, DSL,
  DSM, DSO, DSOX, DSO32, DSR, DST, ISM330DHCX). Auto-detects the address and
  variant, one-call configuration of rate and ranges, combined accel + gyro
  burst read, INT1 data-ready.
- `AlfredoH3LIS`: SPI and I2C driver for the ST H3LIS331DL high-g
  accelerometer with INT1 data-ready.
- `AlfredoMMC5983`: SPI and I2C driver for the MEMSIC MMC5983MA magnetometer:
  continuous mode, automatic set/reset, measurement-done interrupt, single
  measurements and die temperature.
- `AlfredoVQF`: orientation from the VQF filter with automatic gyro bias
  calibration, measured-dt correction for the IMU oscillator, optional accel
  and gyro-scale calibration, 9DOF output when fed a magnetometer. On the
  ESP32 it is thread-safe and can keep calibration values in flash. Moved
  here from the Alfredo-NoU3 library, where it was `NoU3_VQF`.
- Examples: Orientation_LSM6, CalibrateAccel_LSM6, CalibrateGyroScale_LSM6,
  Read_LSM6, Read_H3LIS, Read_MMC5983.
