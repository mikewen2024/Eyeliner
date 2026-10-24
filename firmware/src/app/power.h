/**
 * @file power.h
 * @brief Battery voltage (Vbat_Sense divider) and ESP32-S3 die temperature,
 *        sampled every VBAT_INTERVAL_MS on the comms core.
 */
#pragma once

#include <stdint.h>

struct PowerState {
  float    vbatV;        // VCC_HV, volts (ADC millivolts * VBAT_RATIO)
  uint16_t senseMv;      // raw calibrated ADC reading at the pin
  float    chipTempC;    // ESP32-S3 internal temperature sensor
  uint32_t samples;
  uint32_t sampleMs;     // millis() of the last sample
};

void powerInit();
void powerUpdate();        // samples when the interval has elapsed, else returns
PowerState powerGet();
