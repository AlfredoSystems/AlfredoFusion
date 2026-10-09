#pragma once

// AlfredoFusion - IMU drivers and VQF sensor fusion for Arduino.
//
//   AlfredoLSM6      ST LSM6 family accelerometer/gyroscope (I2C)
//   AlfredoH3LIS     ST H3LIS331DL high-g accelerometer (SPI / I2C)
//   AlfredoMMC5983   MEMSIC MMC5983MA magnetometer (SPI / I2C)
//   AlfredoVQF       6DOF/9DOF orientation with automatic gyro bias calibration
//
// Include this header for everything, or the individual headers to pull in
// only what you use.

#include "AlfredoFusion_LSM6.h"
#include "AlfredoFusion_H3LIS.h"
#include "AlfredoFusion_MMC5983.h"
#include "AlfredoFusion_VQF.h"
