/**
 * @file espnow_dongle.cpp
 * @brief ESP-NOW receiver for the Eyeliner telemetry (src/app/telemetry.h).
 *        Flash onto a second XIAO ESP32-S3 and open its USB serial port.
 *
 * Build/flash: pio run -e espnow_dongle -t upload
 * Monitor:     pio device monitor -e espnow_dongle
 *
 * Output: one text line per frame (default), or raw frames with DONGLE_RAW=1
 * (env espnow_dongle_raw): each frame as received, prefixed by
 * 0xAA 0x55 <len> <rssi as int8>, for tools/telem/telem_logger.py.
 * Channel must match ESPNOW_WIFI_CHANNEL on the robot.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "../app/telemetry.h"

#ifndef DONGLE_CHANNEL
#define DONGLE_CHANNEL 6
#endif
#ifndef DONGLE_RAW
#define DONGLE_RAW 0
#endif

static volatile uint32_t g_frames = 0, g_bad = 0, g_lostFast = 0;
static esp_err_t g_err = ESP_OK;
static uint16_t g_lastFastSeq = 0;
static bool g_haveFast = false;

static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < (int)sizeof(TelemHeader)) { g_bad++; return; }
  const TelemHeader *h = (const TelemHeader *)data;
  if (h->magic0 != TELEM_MAGIC0 || h->magic1 != TELEM_MAGIC1 || h->len != len) { g_bad++; return; }
  g_frames++;

#if DONGLE_RAW
  int8_t rssi = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;
  uint8_t pre[4] = {0xAA, 0x55, (uint8_t)len, (uint8_t)rssi};
  Serial.write(pre, 4);
  Serial.write(data, len);
  return;
#else
  if (h->type == TELEM_TYPE_FAST && len == (int)sizeof(TelemFast)) {
    TelemFast f;
    memcpy(&f, data, sizeof(f));
    if (g_haveFast) {
      uint16_t gap = (uint16_t)(f.hdr.seq - g_lastFastSeq - 1);
      if (gap < 30000) g_lostFast += gap;  // a huge gap is the robot rebooting, not loss
      else g_lostFast = 0;
    }
    g_lastFastSeq = f.hdr.seq;
    g_haveFast = true;
    // Print every 10th fast frame to keep the USB port readable
    if ((f.hdr.seq % 10) != 0) return;
    static const char *modeName[] = {"melty", "tank", "flip"};
    Serial.printf("F %5u t=%lu hdg %6.1f rate %7.1f K %5.1f | spin %+.2f cos %+.2f mag %.2f | M %+.0f mod %+.2f m %+.2f %+.2f | %s%s%s%s | LQ %u RSSI -%u | "
                  "imuA %+6.2f %+6.2f %+6.2f | imuB %+6.2f %+6.2f %+6.2f | lost %lu rssi %d\n",
                  f.hdr.seq, (unsigned long)f.tMs, f.headingDeg, f.rateDps, f.k, f.spinup, f.headingCos, f.translation,
                  f.M, f.modulation, f.throttle[0], f.throttle[1], modeName[f.mode <= 2 ? f.mode : 0],
                  (f.flags & TELEM_FLAG_RX_SAFE) ? "" : " RXFAIL", (f.flags & TELEM_FLAG_KILLED) ? " KILL" : "",
                  (f.flags & TELEM_FLAG_LED_ARC) ? " ARC" : "", f.lq, f.rssi, f.imu[0].accel[0], f.imu[0].accel[1],
                  f.imu[0].accel[2], f.imu[1].accel[0], f.imu[1].accel[1], f.imu[1].accel[2], (unsigned long)g_lostFast,
                  info->rx_ctrl ? info->rx_ctrl->rssi : 0);
  } else if (h->type == TELEM_TYPE_SLOW && len == (int)sizeof(TelemSlow)) {
    TelemSlow s;
    memcpy(&s, data, sizeof(s));
    Serial.printf("S %5u t=%lu vbat %5.2f V (%u mV) chip %4.1f C imu %4.1f %4.1f C | comms %u/s max %u us, control %u/s max %u us, uart %u B/s | "
                  "imu err %lu %lu | led %u frames %lu | espnow sent %lu failed %lu | heap %lu | pca %u present %u\n",
                  s.hdr.seq, (unsigned long)s.tMs, s.vbatV, s.vbatSenseMv, s.chipTempC, s.imuTempC[0], s.imuTempC[1],
                  s.commsLoopsPerS, s.commsBusyMaxUs, s.controlLoopsPerS, s.controlBusyMaxUs, s.uartBytesPerS,
                  (unsigned long)s.imuErrors[0], (unsigned long)s.imuErrors[1], s.ledMode, (unsigned long)s.ledFrames,
                  (unsigned long)s.espnowSent, (unsigned long)s.espnowFailed, (unsigned long)s.freeHeap, s.pcaOk,
                  s.imuPresentMask);
  } else {
    g_bad++;
  }
#endif
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  // WiFi.mode() starts the driver asynchronously: wait for it before setting
  // the channel, or the call fails and the radio stays on channel 1
  WiFi.mode(WIFI_STA);
  uint32_t tw = millis();
  while (!WiFi.STA.started() && millis() - tw < 2000) delay(5);
  g_err = esp_wifi_set_channel(DONGLE_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_ps(WIFI_PS_NONE);
  if (g_err == ESP_OK) g_err = esp_now_init();
  if (g_err == ESP_OK) g_err = esp_now_register_recv_cb(onRecv);
#if !DONGLE_RAW
  uint8_t ch = 0;
  wifi_second_chan_t sc;
  esp_wifi_get_channel(&ch, &sc);
  Serial.printf("=== Eyeliner ESP-NOW dongle === channel %u (want %d), MAC %s, init %s, fast frame %u B, slow frame %u B\n",
                ch, DONGLE_CHANNEL, WiFi.macAddress().c_str(), esp_err_to_name(g_err), (unsigned)sizeof(TelemFast),
                (unsigned)sizeof(TelemSlow));
#endif
}

void loop() {
#if !DONGLE_RAW
  static uint32_t last = 0;
  if (millis() - last >= 2000) {
    last = millis();
    uint8_t ch = 0;
    wifi_second_chan_t sc;
    esp_wifi_get_channel(&ch, &sc);
    Serial.printf("-- ch %u init %s | %lu frames, %lu bad, %lu fast lost\n", ch, esp_err_to_name(g_err),
                  (unsigned long)g_frames, (unsigned long)g_bad, (unsigned long)g_lostFast);
  }
#endif
  delay(1);
}
