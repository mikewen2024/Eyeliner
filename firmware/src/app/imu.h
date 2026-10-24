/**
 * @file imu.h
 * @brief Both LSM6DSV320X IMUs, polled over I2C at IMU_READ_INTERVAL_MS.
 *
 * Runs on the comms core (imuUpdate() from the comms task); the control core
 * takes a snapshot with imuGet(). The I2C bus (IMUs + PCA9570) is only ever
 * touched from the comms core, so no bus lock is needed.
 *
 * Full scales: low-g accel +-16 g, gyro +-2000 dps, high-g accel +-320 g.
 * Axes are the sensor's own; the two parts are mounted mirrored, so expect
 * opposite signs between them.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define IMU_COUNT 2

struct ImuSample {
  uint8_t  addr;        // 0x6A (U4) or 0x6B (U3)
  bool     present;     // answered WHO_AM_I at init
  float    accel[3];    // low-g accel, m/s^2
  float    gyro[3];     // dps
  float    accelHg[3];  // high-g accel, m/s^2
  float    tempC;
  uint32_t errors;      // I2C errors since boot
};

struct ImuState {
  ImuSample imu[IMU_COUNT];
  uint32_t  seq;        // bumps on every read of both IMUs
  uint32_t  sampleUs;   // micros() when the read started
  uint32_t  readUs;     // how long the last read of both took
};

// Brings up the bus and configures every IMU that answers. Returns the number
// of IMUs found (0 means the control loop has no heading source).
int imuInit();

// Reads both IMUs when the interval has elapsed, else returns at once. Prints
// telemetry when DISPLAY_SERIAL is set.
void imuUpdate();

// Thread-safe copy of the latest published samples (any core).
ImuState imuGet();
