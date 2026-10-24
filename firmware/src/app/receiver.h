/**
 * @file receiver.h
 * @brief CRSF / ELRS receiver input with failsafe, remapped to [-1, 1].
 *
 * Runs on the comms core (receiverUpdate() from the comms task). The control
 * core reads a consistent snapshot with receiverGet(); the copy is guarded by
 * a spinlock, so neither side ever sees a half-written frame.
 *
 * Port of original_code/ELRS_Receiver.ino + FS2A_Receiver.ino (remapInputs).
 * Axis order and defaults match the old rawChannelData[] array:
 *   [0] left x  (CH4) default 1500     [2] right y (CH2) default 1500
 *   [1] left y  (CH3) default 1000     [3] right x (CH1) default 1500
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

enum RxAxis {
  RX_LEFT_X = 0,
  RX_LEFT_Y,
  RX_RIGHT_Y,
  RX_RIGHT_X,
  RX_AXIS_COUNT
};

struct ReceiverState {
  int      raw[RX_AXIS_COUNT];   // stick pulse widths in us (988..2012 from CRSF)
  float    axis[RX_AXIS_COUNT];  // raw remapped to [-1, 1]
  int      aux[6];               // CH5..CH10 (SE, SA, SB, SC, SD, S1) in us
  bool     safe;                 // false: failsafe active, raw/axis hold defaults
  bool     linkUp;               // receiver is delivering channel frames
  uint8_t  lq;                   // uplink link quality, %
  uint8_t  rssi;                 // uplink RSSI, dBm (positive magnitude)
  uint32_t seq;                  // bumps on every publish; detect fresh data
  uint32_t lastFrameUs;          // micros() of the last channel frame seen
  uint32_t uartBytes;            // bytes taken off the UART since boot (~26 per channel frame)
};

// Opens the receiver UART and the CRSF parser. Returns false if the UART failed.
bool receiverInit();

// Parse pending bytes, apply failsafe/remap, publish. Call continuously from
// the comms task; it never blocks. Prints telemetry when DISPLAY_SERIAL is set.
void receiverUpdate();

// Thread-safe copy of the latest published state (any core).
ReceiverState receiverGet();

// Pulse width -> [-1, 1], the same remap the stick axes use.
static inline float rxRemap(int us) {
  float a = (float)(us - 1500) * 0.002f;
  return a > 1.0f ? 1.0f : (a < -1.0f ? -1.0f : a);
}
// CH5..CH10 as an axis: ch is the CRSF channel number (RX_CH_SE .. RX_CH_S1).
static inline float rxAuxAxis(const ReceiverState &st, int ch) { return rxRemap(st.aux[ch - 5]); }

// Pulse width of any channel 1..10 by CRSF channel number.
static inline int rxChannelUs(const ReceiverState &st, int ch) {
  switch (ch) {
    case 1: return st.raw[RX_RIGHT_X];
    case 2: return st.raw[RX_RIGHT_Y];
    case 3: return st.raw[RX_LEFT_Y];
    case 4: return st.raw[RX_LEFT_X];
    default: return (ch >= 5 && ch <= 10) ? st.aux[ch - 5] : 1500;
  }
}
