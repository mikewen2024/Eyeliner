#include "telemetry.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "config.h"

static_assert(ESPNOW_FAST_INTERVAL_MS > 0 && ESPNOW_SLOW_INTERVAL_MS > 0, "ESP-NOW intervals must be positive");
static_assert(ESPNOW_WIFI_CHANNEL >= 1 && ESPNOW_WIFI_CHANNEL <= 13, "ESPNOW_WIFI_CHANNEL is 1..13");

static const uint8_t kBroadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static TelemetryStats s_stats;
static uint16_t s_seqFast = 0, s_seqSlow = 0;
static uint32_t s_nextFastMs = 0, s_nextSlowMs = 0;
static bool     s_ready = false;
static const char *s_initError = "not initialised";

static void onSent(const esp_now_send_info_t *, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) s_stats.cbFail++;
}

// Brings the WiFi driver up on our channel (idempotent) and ESP-NOW with the
// broadcast peer. Used at init and whenever SA switches telemetry back on.
static bool radioStart() {
  // WiFi.mode() starts the driver through the network event task, so wait for
  // it to be actually up before touching the channel. Do NOT call
  // WiFi.disconnect(true) here: in this core that switches WiFi off.
  if (!WiFi.mode(WIFI_STA)) { s_initError = "WiFi.mode"; return false; }
  uint32_t t0 = millis();
  while (!WiFi.STA.started() && millis() - t0 < 2000) delay(5);
  if (!WiFi.STA.started()) { s_initError = "WiFi start timeout"; return false; }

  esp_err_t err = esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) { s_initError = esp_err_to_name(err); return false; }
  esp_wifi_set_ps(WIFI_PS_NONE);  // no power save: keeps send latency steady
  WiFi.setTxPower(ESPNOW_TX_POWER);

  err = esp_now_init();
  if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) { s_initError = esp_err_to_name(err); return false; }
  esp_now_register_send_cb(onSent);
  if (!esp_now_is_peer_exist(kBroadcast)) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, kBroadcast, 6);
    peer.channel = ESPNOW_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK) { s_initError = esp_err_to_name(err); return false; }
  }
  s_initError = nullptr;
  s_stats.radioOn = true;
  return true;
}

// Tears ESP-NOW and the WiFi driver down: no frames, no receive path, cool chip
static void radioStop() {
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
  s_stats.radioOn = false;
}

bool telemetryInit() {
  memset(&s_stats, 0, sizeof(s_stats));
  s_stats.enabled = true;  // radio starts up; the first update applies the switch
  if (!radioStart()) return false;

  uint32_t now = millis();
  s_nextFastMs = now;
  s_nextSlowMs = now;
  s_ready = true;
  return true;
}

static void fillHeader(TelemHeader &h, uint8_t type, uint16_t len, uint16_t seq) {
  h.magic0 = TELEM_MAGIC0;
  h.magic1 = TELEM_MAGIC1;
  h.type = type;
  h.version = TELEM_VERSION;
  h.len = len;
  h.seq = seq;
}

static void send(const void *frame, size_t len, uint32_t &sentCounter) {
  if (esp_now_send(kBroadcast, (const uint8_t *)frame, len) == ESP_OK) sentCounter++;
  else s_stats.sendErrors++;
}

void telemetryUpdate(const ReceiverState &rx, const ImuState &imu, const HeadingState &h, const MotorOutputs &m,
                     const LedState &led, const PowerState &pwr, const SystemStats &stats) {
  if (!s_ready) return;
  uint32_t now = millis();

  // SA switch: start or stop the radio on a change, then send nothing while off
  bool enable = rxChannelUs(rx, TELEM_ENABLE_CHANNEL) > TELEM_ENABLE_US;
  if (enable != s_stats.enabled) {
    s_stats.enabled = enable;
    if (enable) {
      if (radioStart()) { s_nextFastMs = now; s_nextSlowMs = now; }
    } else {
      radioStop();
    }
  }
  if (!enable || !s_stats.radioOn) return;

  if ((int32_t)(now - s_nextFastMs) >= 0) {
    s_nextFastMs += ESPNOW_FAST_INTERVAL_MS;
    if ((int32_t)(now - s_nextFastMs) > 0) s_nextFastMs = now + ESPNOW_FAST_INTERVAL_MS;

    TelemFast f;
    memset(&f, 0, sizeof(f));
    fillHeader(f.hdr, TELEM_TYPE_FAST, sizeof(f), s_seqFast++);
    f.tMs = now;
    f.headingDeg = h.headingDeg;
    f.rateDps = h.rateDps;
    f.k = h.k;
    f.spinup = m.in.spinup;
    f.headingCos = m.in.headingCos;
    f.translation = m.in.translation;
    f.tankX = m.in.tankX;
    f.tankY = m.in.tankY;
    f.M = m.M;
    f.modulation = m.modulation;
    f.throttle[0] = m.throttle[0];
    f.throttle[1] = m.throttle[1];
    f.dshot[0] = m.dshot[0];
    f.dshot[1] = m.dshot[1];
    f.mode = m.in.mode;
    f.flags = (rx.safe ? TELEM_FLAG_RX_SAFE : 0) | (rx.linkUp ? TELEM_FLAG_LINK_UP : 0) |
              (m.killed ? TELEM_FLAG_KILLED : 0) | (m.armed[0] ? TELEM_FLAG_ARMED_1 : 0) |
              (m.armed[1] ? TELEM_FLAG_ARMED_2 : 0) | (led.arcOn ? TELEM_FLAG_LED_ARC : 0);
    f.lq = rx.lq;
    f.rssi = rx.rssi;
    for (int i = 0; i < 4; i++) f.rawCh[i] = (int16_t)rx.raw[i];
    for (int n = 0; n < 2; n++) {
      for (int i = 0; i < 3; i++) {
        f.imu[n].accel[i] = imu.imu[n].accel[i];
        f.imu[n].gyro[i] = imu.imu[n].gyro[i];
        f.imu[n].accelHg[i] = imu.imu[n].accelHg[i];
      }
    }
    f.imuSeq = imu.seq;
    f.rxSeq = rx.seq;
    send(&f, sizeof(f), s_stats.sentFast);
  }

  if ((int32_t)(now - s_nextSlowMs) >= 0) {
    s_nextSlowMs = now + ESPNOW_SLOW_INTERVAL_MS;

    TelemSlow s;
    memset(&s, 0, sizeof(s));
    fillHeader(s.hdr, TELEM_TYPE_SLOW, sizeof(s), s_seqSlow++);
    s.tMs = now;
    s.vbatV = pwr.vbatV;
    s.vbatSenseMv = pwr.senseMv;
    s.chipTempC = pwr.chipTempC;
    s.imuTempC[0] = imu.imu[0].tempC;
    s.imuTempC[1] = imu.imu[1].tempC;
    s.commsLoopsPerS = stats.commsLoopsPerS;
    s.controlLoopsPerS = stats.controlLoopsPerS;
    s.commsBusyMaxUs = stats.commsBusyMaxUs;
    s.controlBusyMaxUs = stats.controlBusyMaxUs;
    s.uartBytesPerS = stats.uartBytesPerS;
    s.imuErrors[0] = imu.imu[0].errors;
    s.imuErrors[1] = imu.imu[1].errors;
    s.ledFrames = led.frames;
    s.motorFrames = m.frames;
    s.espnowSent = s_stats.sentFast + s_stats.sentSlow;
    s.espnowFailed = s_stats.sendErrors + s_stats.cbFail;
    s.freeHeap = ESP.getFreeHeap();
    s.ledMode = led.mode;
    s.imuPresentMask = (imu.imu[0].present ? 1 : 0) | (imu.imu[1].present ? 2 : 0);
    s.pcaOk = led.pcaOk;
    send(&s, sizeof(s), s_stats.sentSlow);
  }
}

TelemetryStats telemetryStats() { return s_stats; }

const char *telemetryInitError() { return s_initError; }

int telemetryChannel() {
  uint8_t ch = 0;
  wifi_second_chan_t sc;
  return esp_wifi_get_channel(&ch, &sc) == ESP_OK ? ch : -1;
}
