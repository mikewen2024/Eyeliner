/**
 * @file leds.h
 * @brief SK9822 strip: heading arc indicator plus mode / failsafe colours.
 *
 * Comms-core only (core 0): the LED clock is the I2C SCL line, so this shares
 * the bus with the IMUs and the PCA9570 and must never run concurrently with
 * them. ledsInit() needs Wire already started (imuInit() does that).
 *
 * Port of indicateHeading() from ADXL375_Heading.ino / LSM6DSV320XTR_Heading.ino.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "heading.h"
#include "motor.h"

enum LedMode : uint8_t { LED_MODE_MELTY = 0, LED_MODE_TANK, LED_MODE_FAILSAFE };

struct LedState {
  LedMode  mode;
  uint8_t  r, g, b;      // colour currently on the strip (0,0,0 = off)
  bool     arcOn;        // melty arc indicator lit
  uint32_t frames;       // strip frames sent
  bool     pcaOk;        // last PCA9570 write acknowledged
};

// Sets up the data pin and the PCA9570 (gate closed, IR off, temp mux), clears
// the strip. Returns false if the PCA9570 did not acknowledge.
bool ledsInit();

// Picks the pattern from the current state and resends the strip if it changed.
// Failsafe colour whenever the motors are killed (receiver loss, kill switch)
// or not armed yet; tank colour in tank mode; else the heading arc.
void ledsUpdate(const HeadingState &h, const MotorOutputs &m);

LedState ledsGet();
