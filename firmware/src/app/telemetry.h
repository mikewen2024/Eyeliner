/**
 * @file telemetry.h
 * @brief ESP-NOW broadcast telemetry: two packed frame types.
 *
 * Comms core only. Every frame starts with TelemHeader; all fields are
 * little-endian, packed, no padding. Python: struct.unpack('<...').
 *   type 1  TelemFast   every ESPNOW_FAST_INTERVAL_MS
 *   type 2  TelemSlow   every ESPNOW_SLOW_INTERVAL_MS
 * Frames are protected by the 802.11 FCS, so a dropped frame shows up only as
 * a gap in seq. The receiver side is src/tests/espnow_dongle.cpp.
 */
#pragma once

#include <stdint.h>

#include "heading.h"
#include "imu.h"
#include "leds.h"
#include "motor.h"
#include "power.h"
#include "receiver.h"

#define TELEM_MAGIC0      'E'
#define TELEM_MAGIC1      'Y'
#define TELEM_VERSION     1
#define TELEM_TYPE_FAST   1
#define TELEM_TYPE_SLOW   2

// Flags in TelemFast.flags
#define TELEM_FLAG_RX_SAFE   (1u << 0)
#define TELEM_FLAG_LINK_UP   (1u << 1)
#define TELEM_FLAG_KILLED    (1u << 2)
#define TELEM_FLAG_ARMED_1   (1u << 3)
#define TELEM_FLAG_ARMED_2   (1u << 4)
#define TELEM_FLAG_LED_ARC   (1u << 5)

struct __attribute__((packed)) TelemHeader {
  uint8_t  magic0, magic1;   // 'E', 'Y'
  uint8_t  type;             // TELEM_TYPE_*
  uint8_t  version;          // TELEM_VERSION
  uint16_t len;              // whole frame length in bytes
  uint16_t seq;              // per-type counter
};

struct __attribute__((packed)) TelemImu {
  float accel[3];            // m/s^2
  float gyro[3];             // dps
  float accelHg[3];          // m/s^2
};

// Group 1
struct __attribute__((packed)) TelemFast {
  TelemHeader hdr;
  uint32_t tMs;              // millis()
  float    headingDeg;
  float    rateDps;
  float    k;                // heading rate constant in use
  float    spinup;           // stop-logic registers
  float    headingCos;
  float    translation;
  float    tankX, tankY;
  float    M;
  float    modulation;
  float    throttle[2];      // signed, as sent (after direction + clamp)
  uint16_t dshot[2];
  uint8_t  mode;             // MotorMode
  uint8_t  flags;            // TELEM_FLAG_*
  uint8_t  lq;               // uplink link quality, %
  uint8_t  rssi;             // uplink RSSI magnitude, dBm
  int16_t  rawCh[4];         // left x, left y, right y, right x (us)
  TelemImu imu[2];           // 0x6A, 0x6B
  uint32_t imuSeq;
  uint32_t rxSeq;
};

// Loop statistics gathered by main.cpp once a second
struct SystemStats {
  uint16_t commsLoopsPerS, controlLoopsPerS;
  uint16_t commsBusyMaxUs, controlBusyMaxUs;
  uint16_t uartBytesPerS;
};

// Group 2
struct __attribute__((packed)) TelemSlow {
  TelemHeader hdr;
  uint32_t tMs;
  float    vbatV;
  uint16_t vbatSenseMv;
  float    chipTempC;
  float    imuTempC[2];
  uint16_t commsLoopsPerS, controlLoopsPerS;
  uint16_t commsBusyMaxUs, controlBusyMaxUs;
  uint16_t uartBytesPerS;
  uint32_t imuErrors[2];
  uint32_t ledFrames;
  uint32_t motorFrames;
  uint32_t espnowSent, espnowFailed;
  uint32_t freeHeap;
  uint8_t  ledMode;
  uint8_t  imuPresentMask;   // bit0 0x6A, bit1 0x6B
  uint8_t  pcaOk;
  uint8_t  pad;
};

static_assert(sizeof(TelemFast) <= 250, "TelemFast exceeds one ESP-NOW frame");
static_assert(sizeof(TelemSlow) <= 250, "TelemSlow exceeds one ESP-NOW frame");

struct TelemetryStats {
  uint32_t sentFast, sentSlow, sendErrors, cbFail;
  bool     enabled;     // SA switch state as last applied
  bool     radioOn;     // WiFi driver running
};

// Starts WiFi (station, no connection) on ESPNOW_WIFI_CHANNEL and ESP-NOW with
// the broadcast peer. Returns false if any step failed.
bool telemetryInit();

// Applies the SA enable switch (starting/stopping the radio) and sends whichever
// groups are due. Call from the comms loop.
void telemetryUpdate(const ReceiverState &rx, const ImuState &imu, const HeadingState &h, const MotorOutputs &m,
                     const LedState &led, const PowerState &pwr, const SystemStats &stats);

TelemetryStats telemetryStats();

// nullptr once init succeeded, else the step or esp_err name that failed.
const char *telemetryInitError();

// Current WiFi channel as reported by the driver (-1 if unavailable).
int telemetryChannel();
