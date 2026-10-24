#include "power.h"

#include <Arduino.h>

#include "board_pins.h"
#include "config.h"

static_assert(VBAT_INTERVAL_MS > 0, "VBAT_INTERVAL_MS must be positive");

static PowerState s_state;
static uint32_t   s_nextMs = 0;

void powerInit() {
  // 11 dB attenuation reads up to ~3.1 V at the pin: 34 V on VCC_HV through the divider
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VBAT_SENSE, ADC_11db);
  memset(&s_state, 0, sizeof(s_state));
  s_nextMs = millis();
  powerUpdate();
}

void powerUpdate() {
  uint32_t now = millis();
  if ((int32_t)(now - s_nextMs) < 0) return;
  s_nextMs = now + VBAT_INTERVAL_MS;

  s_state.senseMv = (uint16_t)analogReadMilliVolts(PIN_VBAT_SENSE);
  s_state.vbatV = s_state.senseMv * 0.001f * (float)(VBAT_RATIO);
  s_state.chipTempC = temperatureRead();
  s_state.sampleMs = now;
  s_state.samples++;
}

PowerState powerGet() { return s_state; }
