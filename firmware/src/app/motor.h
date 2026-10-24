/**
 * @file motor.h
 * @brief Melty stop logic + DShot output for the two ESCs.
 *
 * Control-core only (core 1). The control loop writes the three input
 * registers with motorSetInputs(), then calls motorStopLogic() once per cycle;
 * on failsafe it calls motorKill() instead. Both send one frame per ESC, which
 * AM32 needs continuously. The comms core reads motorGet() for telemetry.
 *
 * Port of Motor_Control.ino: setupMotorsDshot, stopLogic, killMotorsDshot,
 * PWMtodShot. ESC pins: PIN_DSHOT_1 / PIN_DSHOT_2 from board_pins.h.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

enum MotorMode : uint8_t {
  MOTOR_MODE_MELTY = 0,         // +spinup / -spinup with stop-logic modulation
  MOTOR_MODE_TANK,              // right stick y drive, x turn
  MOTOR_MODE_MELTY_FLIPPED,     // melty with both motors negated (and the heading sign)
};

struct MotorInputs {
  MotorMode mode;
  float spinup;       // melty: [0, 1] (signed allowed)
  float headingCos;   // melty: [-1, 1]
  float translation;  // melty: [0, 1]
  float tankX;        // tank: [-1, 1] after dead zone
  float tankY;        // tank: [-1, 1] after dead zone
};

struct MotorOutputs {
  MotorInputs in;      // registers used for this result
  float    D;          // translation after dead zone (squared or not per config)
  float    M;          // -1, 0, +1: which side of the heading the stick points
  float    modulation; // added to motor 1, subtracted from motor 2
  float    throttle[2];// per motor, [-MOTOR_MAX_THROTTLE, +MOTOR_MAX_THROTTLE], sign as sent
  uint16_t dshot[2];   // frame value sent (0 = stop)
  bool     armed[2];   // AM32 arming window passed (MOTORS only)
  bool     killed;     // last frame came from motorKill()
  uint32_t frames;     // frames sent per ESC
};

// Starts DShot on both ESC pins (only with MOTORS). Returns false if an RMT
// channel could not be allocated.
bool motorInit();

// Spin stick pulse width -> spinup: 0 below MOTOR_SPINUP_MIN_US, then linear
// from 0 at MIN_US to 1 at MAX_US.
float motorSpinupFromUs(int us);

// Tank stick axis pulse width -> [-1, 1]: 0 within 1500 +- TANK_DEAD_ZONE_US,
// linear from the dead zone edge to full stick.
float motorTankAxisFromUs(int us);

// SB pulse width -> drive mode (thresholds MOTOR_MODE_LOW_US / HIGH_US).
MotorMode motorModeFromUs(int us);

// +1 for melty, -1 for melty flipped (heading integrates with this sign).
int motorSpinSign(MotorMode mode);

// Write the registers. Call all that apply, then motorStopLogic().
void motorSetMode(MotorMode mode);
void motorSetInputs(float spinup, float headingCos, float translation);
void motorSetTankInputs(float x, float y);

// Compute both throttles from the registers (per mode) and send one frame per ESC.
void motorStopLogic();

// Zero throttle to both ESCs, one frame per call.
void motorKill();

// Thread-safe copy for the other core.
MotorOutputs motorGet();
