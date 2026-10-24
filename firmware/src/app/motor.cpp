#include "motor.h"

#include <Arduino.h>
#include <math.h>

#include "board_pins.h"
#include "config.h"

#ifdef MOTORS
#include <AlfredoDShot.h>
static AlfredoDShot s_esc[2];
#endif

static_assert(MOTOR_MAX_THROTTLE > 0.0f && MOTOR_MAX_THROTTLE <= 1.0f, "MOTOR_MAX_THROTTLE in (0, 1]");
static_assert(MOTOR_AGGRESSION >= 0.0f && MOTOR_AGGRESSION <= 1.0f, "MOTOR_AGGRESSION in [0, 1]");
static_assert(MOTOR_SPINUP_MIN_US < MOTOR_SPINUP_MAX_US, "MOTOR_SPINUP_MIN_US must be below MOTOR_SPINUP_MAX_US");
static_assert(MOTOR_1_DIRECTION == 1 || MOTOR_1_DIRECTION == -1, "MOTOR_1_DIRECTION is +1 or -1");
static_assert(MOTOR_2_DIRECTION == 1 || MOTOR_2_DIRECTION == -1, "MOTOR_2_DIRECTION is +1 or -1");
static_assert(MOTOR_MODE_LOW_US < MOTOR_MODE_HIGH_US, "MOTOR_MODE_LOW_US must be below MOTOR_MODE_HIGH_US");
static_assert(TANK_DEAD_ZONE_US >= 0 && TANK_DEAD_ZONE_US < 500, "TANK_DEAD_ZONE_US in [0, 500)");
static_assert(TANK_MAX_THROTTLE > 0.0f && TANK_MAX_THROTTLE <= 1.0f, "TANK_MAX_THROTTLE in (0, 1]");

static const int kPins[2] = {PIN_DSHOT_1, PIN_DSHOT_2};
static const float kDirection[2] = {(float)MOTOR_1_DIRECTION, (float)MOTOR_2_DIRECTION};

static MotorInputs  s_in;        // the registers
static MotorOutputs s_out;
static MotorOutputs s_shared;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Signed throttle -> AM32 3D frame. Same mapping as test_dshot / PWMtodShot.
static uint16_t toDshot3D(float t) {
  t = clampf(t, -MOTOR_MAX_THROTTLE, MOTOR_MAX_THROTTLE);
  if (fabsf(t) < 1e-4f) return 0;                           // stop
  if (t < 0) return 48 + (uint16_t)(-t * 999.0f + 0.5f);   // 48..1047 reverse
  return 1049 + (uint16_t)(t * 998.0f + 0.5f);             // 1049..2047 forward
}

// Applies the per-motor direction and the hard clamp, then sends
static void sendFrames(const float throttle[2]) {
  for (int i = 0; i < 2; i++) {
    float t = throttle[i] * kDirection[i] + 0.0f;  // + 0.0f turns -0 into +0 for the telemetry
    s_out.throttle[i] = clampf(t, -MOTOR_MAX_THROTTLE, MOTOR_MAX_THROTTLE);
    s_out.dshot[i] = toDshot3D(s_out.throttle[i]);
#ifdef MOTORS
    s_esc[i].send(s_out.dshot[i]);
    s_out.armed[i] = s_esc[i].isArmed();
#else
    s_out.armed[i] = true;  // nothing to arm without ESC output
#endif
  }
  s_out.frames++;
  portENTER_CRITICAL(&s_lock);
  s_shared = s_out;
  portEXIT_CRITICAL(&s_lock);
}

bool motorInit() {
  memset(&s_in, 0, sizeof(s_in));
  memset(&s_out, 0, sizeof(s_out));
  bool ok = true;
#ifdef MOTORS
  for (int i = 0; i < 2; i++) {
    if (!s_esc[i].begin(kPins[i], MOTOR_DSHOT_RATE, MOTOR_DSHOT_BIDIR != 0, MOTOR_POLES)) ok = false;
  }
#endif
  portENTER_CRITICAL(&s_lock);
  s_shared = s_out;
  portEXIT_CRITICAL(&s_lock);
  return ok;
}

float motorSpinupFromUs(int us) {
  if (us < MOTOR_SPINUP_MIN_US) return 0.0f;  // dead zone: stopped, lets the ESCs arm
  float t = (float)(us - MOTOR_SPINUP_MIN_US) / (float)(MOTOR_SPINUP_MAX_US - MOTOR_SPINUP_MIN_US);
  return clampf(t, 0.0f, 1.0f);
}

float motorTankAxisFromUs(int us) {
  int d = us - 1500;
  int mag = d < 0 ? -d : d;
  if (mag <= TANK_DEAD_ZONE_US) return 0.0f;
  float a = (float)(mag - TANK_DEAD_ZONE_US) / (float)(500 - TANK_DEAD_ZONE_US);
  a = clampf(a, 0.0f, 1.0f);
  return d < 0 ? -a : a;
}

MotorMode motorModeFromUs(int us) {
  if (us < MOTOR_MODE_LOW_US) return MOTOR_MODE_POS1;
  if (us > MOTOR_MODE_HIGH_US) return MOTOR_MODE_POS3;
  return MOTOR_MODE_POS2;
}

int motorSpinSign(MotorMode mode) { return mode == MOTOR_MODE_MELTY_FLIPPED ? -1 : 1; }

void motorSetMode(MotorMode mode) { s_in.mode = mode; }

void motorSetInputs(float spinup, float headingCos, float translation) {
  s_in.spinup = clampf(spinup, -1.0f, 1.0f);
  s_in.headingCos = clampf(headingCos, -1.0f, 1.0f);
  s_in.translation = clampf(translation, 0.0f, 1.0f);
}

void motorSetTankInputs(float x, float y) {
  s_in.tankX = clampf(x, -1.0f, 1.0f);
  s_in.tankY = clampf(y, -1.0f, 1.0f);
}

// Tank: y drives both the same way, x turns them against each other
static void tankLogic(const MotorInputs &in) {
  float left = TANK_GAIN_DRIVE * in.tankY + TANK_GAIN_TURN * in.tankX;
  float right = TANK_GAIN_DRIVE * in.tankY - TANK_GAIN_TURN * in.tankX;
  float t[2] = {clampf(left, -1.0f, 1.0f) * TANK_MAX_THROTTLE, clampf(right, -1.0f, 1.0f) * TANK_MAX_THROTTLE};
  s_out.D = s_out.M = s_out.modulation = 0.0f;
  sendFrames(t);
}

void motorStopLogic() {
  const MotorInputs in = s_in;
  s_out.in = in;
  s_out.killed = false;

  if (in.mode == MOTOR_MODE_TANK) {
    tankLogic(in);
    return;
  }

  // Translation demand D, with dead zone
#if MOTOR_TRANSLATION_SQUARED
  float D = in.translation * in.translation;
#else
  float D = in.translation;
#endif
  if (D < MOTOR_TRANSLATION_DEAD_ZONE) D = 0.0f;

  // Modulation sign M: where in the rotation the braking/spike happens
  float M = 0.0f;
  if (in.headingCos > MOTOR_HEADING_THRESHOLD) M = 1.0f;
  else if (-in.headingCos > MOTOR_HEADING_THRESHOLD) M = -1.0f;

  // Amplitude and modulation (still in [-1, 1] units)
#if MOTOR_REVERSE_THROTTLE
  float amplitude = MOTOR_AGGRESSION * (fabsf(in.spinup) + 1.0f);
#else
  float amplitude = -MOTOR_AGGRESSION;
#endif
  float modulation = M * D * in.spinup * amplitude;

  // Both motors spin at the spinup throttle; one dips, the other spikes
  float t[2] = {in.spinup, in.spinup};
  if (fabsf(in.spinup) > MOTOR_SPINUP_DEAD_ZONE) {
    t[0] += modulation;
    t[1] -= modulation;
  }
  // Melty: the motors run against each other (motor 1 = -(spinup + mod),
  // motor 2 = +(spinup - mod), as in the old code); flipped mode negates both,
  // which also reverses the robot's spin
  t[0] = -t[0];
  if (in.mode == MOTOR_MODE_MELTY_FLIPPED) {
    t[0] = -t[0];
    t[1] = -t[1];
  }
  for (int i = 0; i < 2; i++) t[i] = clampf(t[i], -1.0f, 1.0f);

  s_out.D = D;
  s_out.M = M;
  s_out.modulation = modulation;
  sendFrames(t);
}

void motorKill() {
  s_out.killed = true;
  s_out.D = s_out.M = s_out.modulation = 0.0f;
  const float zero[2] = {0.0f, 0.0f};
  sendFrames(zero);
}

MotorOutputs motorGet() {
  MotorOutputs copy;
  portENTER_CRITICAL(&s_lock);
  copy = s_shared;
  portEXIT_CRITICAL(&s_lock);
  return copy;
}
