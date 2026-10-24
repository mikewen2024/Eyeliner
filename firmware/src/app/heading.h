/**
 * @file heading.h
 * @brief Spin-rate and heading estimate from centripetal acceleration.
 *
 * Control-core only (headingUpdate() from the control task). The comms core
 * reads a snapshot with headingGet() for telemetry/LEDs.
 *
 * Port of LSM6DSV320XTR_Heading.ino: updateHeading + updateHeadingDotProduct.
 * One IMU, one radial axis (HEADING_IMU_INDEX / HEADING_AXIS in config.h); the
 * two-IMU version that removes the radius from the equation comes later.
 */
#pragma once

#include <stdint.h>

#include "imu.h"

struct HeadingState {
  float    accel;       // low-passed radial acceleration, m/s^2 (signed)
  float    rateDps;     // estimated spin rate, deg/s (>= 0)
  float    headingDeg;  // [0, 360)
  float    k;           // rate constant in use, deg/s per sqrt(m/s^2) (from CH10)
  uint32_t samples;     // IMU samples consumed
};

// Zeroes the filter and heading.
void headingReset();

// Call every control loop. imu: latest snapshot (only new samples are used),
// trimAxis: CH10 in [-1, 1] selecting K in [HEADING_K_MIN, HEADING_K_MAX],
// spinSign: +1 or -1, the robot's spin direction (the IMU only gives |rate|),
// nowUs: micros() for the integration step.
void headingUpdate(const ImuState &imu, float trimAxis, int spinSign, uint32_t nowUs);

// Cosine between the current heading vector and a stick vector (tx, ty) in
// [-1, 1]; 0 when the stick is centred. Same as the old updateHeadingDotProduct.
float headingDot(float tx, float ty);

float headingDegrees();
float headingRateDps();

// Thread-safe copy for the other core.
HeadingState headingGet();
