#include "leds.h"

#include <Arduino.h>
#include <Wire.h>

#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#include "board_pins.h"
#include "config.h"

static_assert(LED_COUNT > 0, "LED_COUNT must be positive");
static_assert(LED_BRIGHTNESS >= 0 && LED_BRIGHTNESS <= 31, "LED_BRIGHTNESS is 0..31");
static_assert(LED_ARC_HALF_WIDTH_DEG > 0.0f && LED_ARC_HALF_WIDTH_DEG < 180.0f, "LED_ARC_HALF_WIDTH_DEG in (0, 180)");

static LedState s_state;
static uint8_t  s_pcaShadow = 0x0F;   // PCA9570 power-on default: all outputs high
static uint32_t s_lastSendMs = 0;
static uint8_t  s_curR = 0xFF, s_curG = 0xFF, s_curB = 0xFF;  // force the first send

/* PCA9570 ---------------------------------------------------------------------- */
static bool pcaWrite(uint8_t v) {
  Wire.beginTransmission(PCA9570_I2C_ADDR);
  Wire.write(v);
  bool ok = Wire.endTransmission() == 0;
  if (ok) s_pcaShadow = v;
  s_state.pcaOk = ok;
  return ok;
}

static bool pcaSetBit(int bit, bool high) {
  uint8_t v = high ? (s_pcaShadow | (1u << bit)) : (s_pcaShadow & ~(1u << bit));
  return pcaWrite(v);
}

// 74AHCT2G125 ~1OE on P0: LOW passes SCL to the strip
static bool ledGate(bool open) { return pcaSetBit(PCA9570_BIT_LED_CLK_EN, !open); }

/* SK9822 bit-bang ---------------------------------------------------------------
 * SCL stays an open-drain pad owned by the I2C driver; only the GPIO-matrix
 * output source is switched to plain GPIO for the duration of a frame, so the
 * bus idles high (1k pull-up) and SDA never moves: the I2C devices see clock
 * pulses without a START and ignore them.
 * ---------------------------------------------------------------------------- */
static inline void skDelay() {
#if LED_HALF_PERIOD_US > 0
  delayMicroseconds(LED_HALF_PERIOD_US);
#endif
}

static void skTakeScl() {
  gpio_set_level((gpio_num_t)PIN_LED_CLK, 1);  // released before the handover
  esp_rom_gpio_connect_out_signal(PIN_LED_CLK, SIG_GPIO_OUT_IDX, false, false);
  gpio_set_level((gpio_num_t)PIN_LED_CLK, 0);  // clock idles low for the strip
}

static void skGiveScl() {
  gpio_set_level((gpio_num_t)PIN_LED_CLK, 1);
  esp_rom_gpio_connect_out_signal(PIN_LED_CLK, I2CEXT0_SCL_OUT_IDX, false, false);
}

static void skByte(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    gpio_set_level((gpio_num_t)PIN_LED_DATA, (b >> i) & 1);
    skDelay();
    gpio_set_level((gpio_num_t)PIN_LED_CLK, 1);  // sampled on the rising edge
    skDelay();
    gpio_set_level((gpio_num_t)PIN_LED_CLK, 0);
  }
}

// 32x0 start, per LED {111bbbbb, B, G, R}, 32x0 latch + n/2 extra zero clocks.
// No APA102-style 0xFF end frame: the SK9822 would show it as a white LED.
static void skShow(uint8_t r, uint8_t g, uint8_t b) {
  if (!ledGate(true)) return;  // gate clocks arrive with DATA low: harmless leading zeros
  skTakeScl();
  for (int i = 0; i < 4; i++) skByte(0x00);
  for (int n = 0; n < LED_COUNT; n++) {
    skByte(0xE0 | LED_BRIGHTNESS);
    skByte(b);
    skByte(g);
    skByte(r);
  }
  for (int i = 0; i < 4 + (LED_COUNT + 15) / 16; i++) skByte(0x00);
  gpio_set_level((gpio_num_t)PIN_LED_DATA, 0);
  skGiveScl();
  ledGate(false);
  s_state.frames++;
}

/* API ----------------------------------------------------------------------------- */
bool ledsInit() {
  pinMode(PIN_LED_DATA, OUTPUT);
  digitalWrite(PIN_LED_DATA, LOW);

  uint8_t v = 0;
  v |= 1u << PCA9570_BIT_LED_CLK_EN;  // gate closed
  if (PCA9570_TEMP_MUX_CH & 1) v |= 1u << PCA9570_BIT_TEMP_SEL_A0;
  if (PCA9570_TEMP_MUX_CH & 2) v |= 1u << PCA9570_BIT_TEMP_SEL_A1;
  if (PCA9570_IR_ON) v |= 1u << PCA9570_BIT_IR_EN;
  bool ok = pcaWrite(v);

  skShow(0, 0, 0);
  s_curR = s_curG = s_curB = 0;
  s_lastSendMs = millis();
  return ok;
}

static void setColor(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t now = millis();
  bool changed = r != s_curR || g != s_curG || b != s_curB;
  if (!changed && now - s_lastSendMs < LED_REFRESH_MS) return;
  skShow(r, g, b);
  s_curR = r; s_curG = g; s_curB = b;
  s_state.r = r; s_state.g = g; s_state.b = b;
  s_lastSendMs = now;
}

static inline bool inArc(float headingDeg) {
  float d = fmodf(headingDeg - LED_ARC_CENTER_DEG + 540.0f, 360.0f) - 180.0f;  // [-180, 180)
  return fabsf(d) <= LED_ARC_HALF_WIDTH_DEG;
}

void ledsUpdate(const HeadingState &h, const MotorOutputs &m) {
  s_state.arcOn = false;
  bool armed = m.armed[0] && m.armed[1];
  if (m.killed || !armed) {
    s_state.mode = LED_MODE_FAILSAFE;
    setColor(LED_COLOR_FAILSAFE);
    return;
  }
  if (m.in.mode == MOTOR_MODE_TANK) {
    s_state.mode = LED_MODE_TANK;
    setColor(LED_COLOR_TANK);
    return;
  }
  s_state.mode = LED_MODE_MELTY;
  s_state.arcOn = inArc(h.headingDeg);
  if (s_state.arcOn) setColor(LED_COLOR_ARC); else setColor(LED_OFF);
}

LedState ledsGet() { return s_state; }
