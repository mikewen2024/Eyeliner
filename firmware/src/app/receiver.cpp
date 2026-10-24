#include "receiver.h"

#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#include "board_pins.h"
#include "config.h"

static HardwareSerial s_uart(RX_UART_NUM);
static AlfredoCRSF    s_crsf;

static const int kDefaultsUs[RX_AXIS_COUNT] = {1500, 1000, 1500, 1500};
static const uint8_t kAxisChannel[RX_AXIS_COUNT] = {RX_CH_LEFT_X, RX_CH_LEFT_Y, RX_CH_RIGHT_Y, RX_CH_RIGHT_X};

// Working copy (comms core only) and the published snapshot (shared)
static ReceiverState   s_work;
static ReceiverState   s_shared;
static portMUX_TYPE    s_lock = portMUX_INITIALIZER_UNLOCKED;

static inline float remap(int us) { return rxRemap(us); }

static void applyDefaults(ReceiverState &st) {
  for (int i = 0; i < RX_AXIS_COUNT; i++) {
    st.raw[i]  = kDefaultsUs[i];
    st.axis[i] = remap(kDefaultsUs[i]);
  }
}

static void publish(const ReceiverState &st) {
  portENTER_CRITICAL(&s_lock);
  s_shared = st;
  portEXIT_CRITICAL(&s_lock);
}

bool receiverInit() {
  // GPIO43 is UART0 TX at boot (bootloader console). The UART driver only turns
  // on the input side of the RX pin and leaves that output driver fighting the
  // receiver, so release both pins to plain inputs first.
  pinMode(PIN_RECEIVER_UART_RX, INPUT);
  pinMode(PIN_RECEIVER_UART_TX, INPUT);
  s_uart.setRxBufferSize(1024);
  s_uart.begin(RX_BAUD, SERIAL_8N1, PIN_RECEIVER_UART_RX, PIN_RECEIVER_UART_TX);
  if (!s_uart) return false;
  s_crsf.begin(s_uart);

  memset(&s_work, 0, sizeof(s_work));
  applyDefaults(s_work);
  s_work.safe = false;  // failsafe until the first channel frame arrives
  s_work.lastFrameUs = micros() - RX_TIMEOUT_US - 1;
  publish(s_work);

#ifdef DISPLAY_SERIAL
  Serial.printf("[rx] UART%d %lu 8N1  RX=GPIO%d  TX=GPIO%d  timeout %lu ms\n", RX_UART_NUM,
                (unsigned long)RX_BAUD, PIN_RECEIVER_UART_RX, PIN_RECEIVER_UART_TX,
                (unsigned long)(RX_TIMEOUT_US / 1000));
#endif
  return true;
}

void receiverUpdate() {
  // Count what the parser is about to drain (bytes landing mid-parse are
  // counted on the next call); this gives a frame-rate figure for free
  s_work.uartBytes += (uint32_t)s_uart.available();
  s_crsf.update();
  uint32_t now = micros();

  // isLinkUp() is true only while channel frames keep arriving (the library
  // drops it 300 ms after the last one), so "last time it was up" tracks the
  // last channel frame to within 300 ms. Link-statistics frames, which ELRS
  // keeps sending after the link is lost, do not count.
  bool linkUp = s_crsf.isLinkUp();
  if (linkUp) s_work.lastFrameUs = now;
  s_work.linkUp = linkUp;

  for (int i = 0; i < RX_AXIS_COUNT; i++) s_work.raw[i] = s_crsf.getChannel(kAxisChannel[i]);
  for (int i = 0; i < 6; i++) s_work.aux[i] = s_crsf.getChannel(RX_CH_SE + i);

  // Failsafe: force the defaults if no channel frame for RX_TIMEOUT_US
  // (also covers startup: lastFrameUs is initialised already expired)
  s_work.safe = (now - s_work.lastFrameUs) <= RX_TIMEOUT_US;
  if (!s_work.safe) applyDefaults(s_work);
  else for (int i = 0; i < RX_AXIS_COUNT; i++) s_work.axis[i] = remap(s_work.raw[i]);

  const crsfLinkStatistics_t *ls = s_crsf.getLinkStatistics();
  s_work.lq   = ls->uplink_Link_quality;
  s_work.rssi = ls->uplink_RSSI_1;
  s_work.seq++;
  publish(s_work);

#ifdef DISPLAY_SERIAL
  static uint32_t lastPrintMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= RX_TELEM_PERIOD_MS) {
    lastPrintMs = nowMs;
    Serial.printf("[rx] %s %s | L %4d %4d  R %4d %4d us | L %+.2f %+.2f  R %+.2f %+.2f | LQ %3u%% RSSI -%u\n",
                  s_work.linkUp ? "link" : "----", s_work.safe ? "ok  " : "FAIL",
                  s_work.raw[RX_LEFT_X], s_work.raw[RX_LEFT_Y], s_work.raw[RX_RIGHT_X], s_work.raw[RX_RIGHT_Y],
                  s_work.axis[RX_LEFT_X], s_work.axis[RX_LEFT_Y], s_work.axis[RX_RIGHT_X], s_work.axis[RX_RIGHT_Y],
                  s_work.lq, s_work.rssi);
  }
#endif
}

ReceiverState receiverGet() {
  ReceiverState copy;
  portENTER_CRITICAL(&s_lock);
  copy = s_shared;
  portEXIT_CRITICAL(&s_lock);
  return copy;
}
