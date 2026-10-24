#include "heading.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"

static_assert(HEADING_IMU_INDEX >= 0 && HEADING_IMU_INDEX < IMU_COUNT, "HEADING_IMU_INDEX out of range");
static_assert(HEADING_AXIS == 0 || HEADING_AXIS == 1, "HEADING_AXIS is 0 (X) or 1 (Y); Z points up");
static_assert(HEADING_K_MIN > 0.0f && HEADING_K_MAX >= HEADING_K_MIN, "HEADING_K_MIN/MAX: 0 < min <= max");

static const float kRadPerDeg = 0.01745329f;

static HeadingState s_state;
static HeadingState s_shared;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t     s_lastImuSeq = 0;
static uint32_t     s_lastUs = 0;
static bool         s_haveTime = false;

void headingReset() {
  memset(&s_state, 0, sizeof(s_state));
  s_lastImuSeq = 0;
  s_haveTime = false;
  portENTER_CRITICAL(&s_lock);
  s_shared = s_state;
  portEXIT_CRITICAL(&s_lock);
}

void headingUpdate(const ImuState &imu, float trimAxis, int spinSign, uint32_t nowUs) {
  // New IMU sample: run the low-pass once, like the old 20 ms read
  if (imu.seq != s_lastImuSeq) {
    s_lastImuSeq = imu.seq;
    const ImuSample &s = imu.imu[HEADING_IMU_INDEX];
    if (s.present) {
#if HEADING_USE_HIGH_G
      float a = s.accelHg[HEADING_AXIS];
#else
      float a = s.accel[HEADING_AXIS];
#endif
      s_state.accel = (1.0f - HEADING_LOW_PASS_RATIO) * s_state.accel + HEADING_LOW_PASS_RATIO * a;
      s_state.samples++;
    }
  }

  // Dead zone, then rate = K * sqrt(|a|) with K set by CH10 across [K_MIN, K_MAX]
  float a = fabsf(s_state.accel) < HEADING_ACCEL_DEAD_ZONE ? 0.0f : fabsf(s_state.accel);
  s_state.k = HEADING_K_MIN + (HEADING_K_MAX - HEADING_K_MIN) * 0.5f * (trimAxis + 1.0f);
  s_state.rateDps = s_state.k * sqrtf(a);

  // Integrate over the real loop period
  if (s_haveTime) {
    float dt = (float)(nowUs - s_lastUs) * 1e-6f;
    float step = (spinSign < 0 ? -s_state.rateDps : s_state.rateDps) * dt;
    s_state.headingDeg = fmodf(s_state.headingDeg + step, 360.0f);
    if (s_state.headingDeg < 0.0f) s_state.headingDeg += 360.0f;
  }
  s_lastUs = nowUs;
  s_haveTime = true;

  portENTER_CRITICAL(&s_lock);
  s_shared = s_state;
  portEXIT_CRITICAL(&s_lock);
}

float headingDot(float tx, float ty) {
  float mag = sqrtf(tx * tx + ty * ty);
  if (mag == 0.0f) return 0.0f;
  float hx = cosf(kRadPerDeg * s_state.headingDeg);
  float hy = sinf(kRadPerDeg * s_state.headingDeg);
  return (tx * hx + ty * hy) / mag;
}

float headingDegrees() { return s_state.headingDeg; }
float headingRateDps() { return s_state.rateDps; }

HeadingState headingGet() {
  HeadingState copy;
  portENTER_CRITICAL(&s_lock);
  copy = s_shared;
  portEXIT_CRITICAL(&s_lock);
  return copy;
}
