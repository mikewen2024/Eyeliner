/**
 * @file test_dshot.cpp
 * @brief DShot600 bring-up against one AM32 ESC using AlfredoDShot (lib/).
 *        (-D DSHOT_RATE=DSHOT1200 for the test_dshot1200 env; see platformio.ini)
 *
 * Build/flash: pio run -e test_dshot -t upload
 * Monitor:     pio device monitor -e test_dshot
 *
 * What it does:
 *   1. Starts DShot600 on DSHOT_PIN and streams frames at DSHOT_LOOP_HZ.
 *      AlfredoDShot forces zero throttle for the first 1.2 s, which is the
 *      unbroken-zero window AM32 needs before it arms. Nothing else is
 *      required to arm; the status line flips to "ARMED" when that passes.
 *   2. Listens for AM32's serial telemetry (KISS 10-byte packet, 115200 8N1)
 *      on DSHOT_TELEM_PIN. The telemetry-request bit is set on one frame every
 *      DSHOT_TELEM_REQ_MS, so the ESC answers at ~50 Hz.
 *   3. Takes throttle over USB serial. The ESC is assumed to be in 3D mode:
 *      48..1047 is reverse, 1049..2047 forward (1048 is treated as stop).
 *      A signed throttle in -1..+1 is mapped onto that.
 *
 * Serial commands (type and press Enter for numbers):
 *   0.15 / -0.15   signed throttle, clamped to +/-DSHOT_MAX_THROTTLE
 *   +  -           step throttle by DSHOT_STEP
 *   s  (or space)  stop (value 0)
 *   r              reverse the current direction
 *   b              beacon 1 (motor must be stopped)
 *   e              enable Extended DShot Telemetry (bidirectional mode only)
 *   p / n          ramp 0 -> +/-DSHOT_RAMP_PEAK -> 0 over DSHOT_RAMP_S seconds
 *                  (any throttle command or 's' aborts it)
 *   L              loop: forward ramp, reverse ramp, repeat until 's' or any
 *                  throttle command
 *   x              toggle the DShot stream off/on (off = signal line idle, no
 *                  frames at all; on = re-init, which re-runs the arming hold)
 *   i              print link stats
 *   l              probe the telemetry pin level for 50 ms (idle level, edges)
 *   t / T          capture the telemetry pin for 3 ms right after the next
 *                  frame: 't' after a telemetry-request frame, 'T' after a
 *                  plain frame (shows crosstalk only). Prints the edge timeline.
 *   d              toggle raw hex dump of telemetry bytes (gap = new line)
 *
 * Wiring (V3.1 Top Board, Dshot2 connector by default; see DSHOT_PIN):
 *   GPIO8 (D9, net Dshot2)      -> ESC signal, 100R series + ESD on the board
 *   GPIO9 (D10, net ESC_Telem2) <- ESC TELEM pad (ESC TX, 115200 8N1)
 *   GND shared.
 * Bidirectional (one-wire eRPM) DShot needs a 1k pull-up to 3V3 on the ESC side
 * of the series resistor, which the board does not have. It is off by default;
 * build with -D DSHOT_BIDIR=1 if the pull-up is fitted.
 */
#include <Arduino.h>

#include <AlfredoDShot.h>
#include <driver/gpio.h>

/* ---------------------------------------------------------------------------
 * Configuration (override any of these from build_flags)
 * ------------------------------------------------------------------------- */
// Defaults are the V3.1 Top Board's Dshot2 connector (Top_Board_Netlist.net):
//   Dshot1 = D8/GPIO7 with ESC_Telem1 = D7/GPIO44
//   Dshot2 = D9/GPIO8 with ESC_Telem2 = D10/GPIO9
#ifndef DSHOT_PIN
#define DSHOT_PIN 8            // D9, net Dshot2
#endif
#ifndef DSHOT_TELEM_PIN
#define DSHOT_TELEM_PIN 9      // D10, net ESC_Telem2 (ESC -> MCU)
#endif
#ifndef DSHOT_BIDIR
#define DSHOT_BIDIR 0          // 1 = one-wire bidirectional (needs 1k pull-up)
#endif
#ifndef DSHOT_RATE
#define DSHOT_RATE DSHOT600
#endif
#ifndef DSHOT_MOTOR_POLES
#define DSHOT_MOTOR_POLES 14
#endif
#ifndef DSHOT_LOOP_HZ
#define DSHOT_LOOP_HZ 1000     // frame rate
#endif
#ifndef DSHOT_TELEM_REQ_MS
#define DSHOT_TELEM_REQ_MS 0   // serial-telemetry request interval (0 = never)
#endif
#ifndef DSHOT_TELEM_PAUSE_MS
// AM32 drives its telemetry pad open-drain with only the STM32's ~40k internal
// pull-up, so without an external pull-up the wire picks up our own DShot
// frames. Skipping frames for this long after a request gives the ~0.9 ms
// reply a quiet window. 0 = never pause.
#define DSHOT_TELEM_PAUSE_MS 2
#endif
#ifndef DSHOT_MAX_THROTTLE
#define DSHOT_MAX_THROTTLE 0.30f  // bench safety clamp, |throttle| <= this
#endif
#ifndef DSHOT_STEP
#define DSHOT_STEP 0.02f
#endif
#ifndef DSHOT_RAMP_PEAK
#define DSHOT_RAMP_PEAK 0.10f  // signed; negative ramps in reverse
#endif
#ifndef DSHOT_RAMP_S
#define DSHOT_RAMP_S 10.0f     // total up+down time
#endif
#ifndef DSHOT_STATUS_MS
#define DSHOT_STATUS_MS 250
#endif

static_assert(DSHOT_MAX_THROTTLE > 0.0f && DSHOT_MAX_THROTTLE <= 1.0f, "");

/* ---------------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------------- */
static AlfredoDShot esc;
static HardwareSerial telemSerial(1);  // UART1, RX only

static float g_throttle = 0.0f;  // signed, -1..+1
static bool g_wasArmed = false;

// Last decoded serial telemetry packet
struct Telem {
  bool valid = false;
  uint32_t count = 0, crcErr = 0;
  uint32_t lastMs = 0;
  uint8_t tempC = 0;
  float volts = 0, amps = 0;
  uint16_t mAh = 0;
  uint32_t erpm = 0;
};
static Telem g_telem;
static bool g_rawDump = false;
static bool g_ramping = false;
static uint8_t g_capture = 0;  // 1 = after a plain frame, 2 = after a request frame, 3 = after ESC_INFO
static bool g_infoPending = false;
static bool g_txEnabled = true;
static float g_rampPeak = DSHOT_RAMP_PEAK;
static bool g_rampLoop = false;
static uint32_t g_rampStartMs = 0;

/* ---------------------------------------------------------------------------
 * 3D-mode throttle mapping
 * ------------------------------------------------------------------------- */
static uint16_t throttleToDshot3D(float t) {
  if (t > DSHOT_MAX_THROTTLE) t = DSHOT_MAX_THROTTLE;
  if (t < -DSHOT_MAX_THROTTLE) t = -DSHOT_MAX_THROTTLE;
  if (fabsf(t) < 1e-4f) return 0;  // stop
  if (t < 0) return 48 + (uint16_t)(-t * 999.0f + 0.5f);     // 48..1047 reverse
  return 1049 + (uint16_t)(t * 998.0f + 0.5f);               // 1049..2047 forward
}

/* ---------------------------------------------------------------------------
 * AM32 serial telemetry (KISS format)
 *   [0] temp C  [1..2] volts*100  [3..4] amps*100  [5..6] mAh
 *   [7..8] eRPM/100  [9] CRC8 (poly 0x07) over bytes 0..8
 * ------------------------------------------------------------------------- */
static uint8_t crc8(const uint8_t *p, size_t n) {
  uint8_t crc = 0;
  for (size_t i = 0; i < n; ++i) {
    crc ^= p[i];
    for (int b = 0; b < 8; ++b) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

static void pollTelemetry() {
  static uint8_t buf[10];
  static uint8_t n = 0;
  static uint32_t lastByteUs = 0;

  // Bytes of a real packet arrive back to back (87 us apart at 115200). Noise
  // bytes coupled from DShot frames come one per frame (1 ms apart), so any
  // gap over ~3 byte times starts a new packet and isolated bytes never
  // accumulate into one.
  while (telemSerial.available()) {
    uint32_t nowUs = micros();
    uint32_t gap = nowUs - lastByteUs;
    if (gap > 300) {
      n = 0;
      if (g_rawDump) Serial.printf("\nraw +%luus:", (unsigned long)gap);
    }
    lastByteUs = nowUs;
    buf[n++] = (uint8_t)telemSerial.read();
    if (g_rawDump) Serial.printf(" %02X", buf[n - 1]);
    if (n < 10) continue;
    n = 0;
    uint32_t now = millis();
    if (crc8(buf, 9) != buf[9]) {
      g_telem.crcErr++;
      continue;
    }
    // Garbage passes CRC8 1 time in 256, and an idle-low (floating/broken)
    // line yields all-zero bytes whose CRC is also zero. Reject both.
    uint16_t v = (buf[1] << 8) | buf[2], a = (buf[3] << 8) | buf[4];
    bool allZero = true;
    for (int i = 0; i < 9; ++i) allZero &= (buf[i] == 0);
    if (allZero || buf[0] > 150 || v > 6000 || a > 20000) {
      g_telem.crcErr++;
      continue;
    }
    g_telem.valid = true;
    g_telem.count++;
    g_telem.lastMs = now;
    g_telem.tempC = buf[0];
    g_telem.volts = ((buf[1] << 8) | buf[2]) / 100.0f;
    g_telem.amps = ((buf[3] << 8) | buf[4]) / 100.0f;
    g_telem.mAh = (buf[5] << 8) | buf[6];
    g_telem.erpm = (uint32_t)((buf[7] << 8) | buf[8]) * 100;
  }
}

/* ---------------------------------------------------------------------------
 * Serial command input
 * ------------------------------------------------------------------------- */
static void setThrottle(float t) {
  if (t > DSHOT_MAX_THROTTLE) t = DSHOT_MAX_THROTTLE;
  if (t < -DSHOT_MAX_THROTTLE) t = -DSHOT_MAX_THROTTLE;
  g_throttle = t;
  g_ramping = false;
  g_rampLoop = false;
  Serial.printf("> throttle %+.3f -> dshot %u\n", g_throttle, throttleToDshot3D(g_throttle));
}

static void printStats() {
  const AlfredoDShot::Stats &s = esc.stats();
  Serial.printf("stats: sent %lu ok %lu noReply %lu framing %lu badGcr %lu badCrc %lu  "
                "echo %u  serialTelem ok %lu crcErr %lu\n",
                (unsigned long)s.sent, (unsigned long)s.ok, (unsigned long)s.noReply,
                (unsigned long)s.framing, (unsigned long)s.badGcr, (unsigned long)s.badCrc,
                esc.echoPulses(), (unsigned long)g_telem.count, (unsigned long)g_telem.crcErr);
}

static void pollCommands() {
  static char line[16];
  static uint8_t len = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (len) {
        line[len] = 0;
        setThrottle(strtof(line, nullptr));
        len = 0;
      }
      continue;
    }
    if (isdigit((unsigned char)c) || c == '.' || (c == '-' && len == 0)) {
      if (len < sizeof(line) - 1) line[len++] = c;
      continue;
    }
    len = 0;
    switch (c) {
      case '+': setThrottle(g_throttle + DSHOT_STEP); break;
      case '-': setThrottle(g_throttle - DSHOT_STEP); break;
      case 's': case ' ': setThrottle(0.0f); break;
      case 'r': setThrottle(-g_throttle); break;
      case 'b':
        if (g_throttle != 0.0f) { Serial.println("> stop first"); break; }
        esc.command(DSHOT_CMD_BEACON1, 1);
        Serial.println("> beacon 1");
        break;
      case 'e':
#if DSHOT_BIDIR
        esc.command(DSHOT_CMD_EDT_ENABLE);
        Serial.println("> EDT enable queued");
#else
        Serial.println("> EDT needs bidirectional mode (-D DSHOT_BIDIR=1)");
#endif
        break;
      case 'i': printStats(); break;
      case 'x':
        g_txEnabled = !g_txEnabled;
        g_throttle = 0.0f; g_ramping = false;
        if (g_txEnabled) {
          esc.end();
          esc.begin(DSHOT_PIN, DSHOT_RATE, DSHOT_BIDIR != 0, DSHOT_MOTOR_POLES);
          g_wasArmed = false;
        }
        Serial.printf("> dshot stream %s\n", g_txEnabled ? "ON (re-arming)" : "OFF");
        break;
      case 't': g_capture = 2; Serial.println("> capture armed (after next request frame)"); break;
      case 'T': g_capture = 1; Serial.println("> capture armed (after next plain frame)"); break;
      case 'I':
        if (g_throttle != 0.0f) { Serial.println("> stop first"); break; }
        esc.command(DSHOT_CMD_ESC_INFO, 6);
        g_infoPending = true;
        Serial.println("> ESC_INFO x6 queued, capturing 10 ms after the last repeat");
        break;
      case 'l': {
        // Raw pad level, independent of the UART. Tells idle level and how
        // busy the line is. Frames keep going (the loop tick is delayed <60ms).
        uint32_t hi = 0, n = 0, edges = 0;
        int last = gpio_get_level((gpio_num_t)DSHOT_TELEM_PIN);
        uint32_t t0 = micros();
        while (micros() - t0 < 50000) {
          int v = gpio_get_level((gpio_num_t)DSHOT_TELEM_PIN);
          hi += v; n++;
          if (v != last) { edges++; last = v; }
        }
        Serial.printf("> telem pin GPIO%d: high %.1f%% of %lu samples, %lu edges in 50 ms\n",
                      DSHOT_TELEM_PIN, 100.0 * hi / n, (unsigned long)n, (unsigned long)edges);
        // Full pad config from the IDF: pull-up/down, input/output enable,
        // function. Shows whether the ESP itself is driving the pad.
        gpio_dump_io_configuration(stdout, 1ULL << DSHOT_TELEM_PIN);
        gpio_dump_io_configuration(stdout, 1ULL << DSHOT_PIN);
        break;
      }
      case 'p': case 'n': case 'L':
        if (!esc.isArmed()) { Serial.println("> not armed yet"); break; }
        g_rampLoop = (c == 'L');
        g_rampPeak = (c == 'n') ? -DSHOT_RAMP_PEAK : DSHOT_RAMP_PEAK;
        if (g_rampLoop) Serial.println("> looping forward/reverse ramps ('s' stops)");
        g_ramping = true;
        g_rampStartMs = millis();
        Serial.printf("> ramp 0 -> %+.2f -> 0 over %.1f s\n", (double)g_rampPeak, (double)DSHOT_RAMP_S);
        break;
      case 'd': g_rawDump = !g_rawDump; Serial.printf("> raw dump %s\n", g_rawDump ? "on" : "off"); break;
      default: break;
    }
  }
}

/* ---------------------------------------------------------------------------
 * Arduino entry points
 * ------------------------------------------------------------------------- */
void setup() {
#if DSHOT_BIDIR
  // Must run before begin(): with a pull-up fitted, a rebooting ESC otherwise
  // sticks in its bootloader. Harmless without one, but not needed either.
  AlfredoDShot::releaseBootloader(DSHOT_PIN);
#endif

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  Serial.println();
  static const char *rateName[] = {"DShot150", "DShot300", "DShot600", "DShot1200"};
  Serial.printf("=== %s / AM32 arming test (AlfredoDShot) ===\n", rateName[DSHOT_RATE]);
  Serial.printf("signal GPIO%d  telem GPIO%d  bidir %d  loop %d Hz  poles %d  max |thr| %.2f\n",
                DSHOT_PIN, DSHOT_TELEM_PIN, DSHOT_BIDIR, DSHOT_LOOP_HZ, DSHOT_MOTOR_POLES,
                (double)DSHOT_MAX_THROTTLE);
  Serial.println("3D mode: negative = reverse (48..1047), positive = forward (1049..2047)");
  Serial.println("cmds: <num>⏎ throttle | + - step | s stop | r reverse | b beacon | e EDT | i stats");

  telemSerial.begin(115200, SERIAL_8N1, DSHOT_TELEM_PIN, -1);
  // UART idles high. Without a pull-up an unconnected/undriven telem pad floats
  // and picks up every DShot frame from the neighbouring signal pin as garbage.
  gpio_pullup_en((gpio_num_t)DSHOT_TELEM_PIN);

  if (!esc.begin(DSHOT_PIN, DSHOT_RATE, DSHOT_BIDIR != 0, DSHOT_MOTOR_POLES)) {
    Serial.println("!! esc.begin() failed (RMT channel allocation?)");
    while (true) delay(1000);
  }
  Serial.println("streaming zero throttle, waiting ~1.2 s for AM32 to arm ...");
}

void loop() {
  static uint32_t nextUs = micros();
  static uint32_t lastStatusMs = 0;
  static uint32_t lastTelemReqMs = 0;

  pollCommands();
  pollTelemetry();

  if ((int32_t)(micros() - nextUs) < 0) return;
  nextUs += 1000000UL / DSHOT_LOOP_HZ;

  uint32_t now = millis();
  if (g_ramping) {
    float x = (now - g_rampStartMs) / (1000.0f * DSHOT_RAMP_S);  // 0..1
    if (x >= 1.0f) {
      g_throttle = 0.0f;
      if (g_rampLoop) {
        g_rampPeak = -g_rampPeak;
        g_rampStartMs = now;
        Serial.printf("> ramp done, next 0 -> %+.2f -> 0\n", (double)g_rampPeak);
      } else {
        g_ramping = false;
        Serial.println("> ramp done");
      }
    } else {
      g_throttle = g_rampPeak * (x < 0.5f ? 2.0f * x : 2.0f * (1.0f - x));
    }
  }
  if (DSHOT_TELEM_REQ_MS && now - lastTelemReqMs >= DSHOT_TELEM_REQ_MS) {
    lastTelemReqMs = now;
    esc.requestTelemetry();
    nextUs += DSHOT_TELEM_PAUSE_MS * 1000UL;  // quiet window for the reply
  }

  bool requested = (DSHOT_TELEM_REQ_MS && lastTelemReqMs == now);
  if (g_txEnabled) esc.send(throttleToDshot3D(g_throttle));

  // Software logic analyser on the telemetry pin: ~1 MHz polling starting as
  // the frame goes out. Records level transitions.
  if (g_infoPending && !esc.commandPending()) { g_infoPending = false; g_capture = 3; }
  if (g_capture && (g_capture == 3 || (g_capture == 1 ? !requested : requested))) {
    const uint32_t winUs = (g_capture == 3) ? 10000 : 3000;
    const char *label = (g_capture == 3) ? "after ESC_INFO" : requested ? "request frame" : "plain frame";
    g_capture = 0;
    static uint32_t edgeT[400];
    static uint8_t edgeL[400];
    uint32_t t0 = micros(), nEdge = 0;
    int last = gpio_get_level((gpio_num_t)DSHOT_TELEM_PIN);
    edgeT[nEdge] = 0; edgeL[nEdge++] = last;
    while (micros() - t0 < winUs) {
      int v = gpio_get_level((gpio_num_t)DSHOT_TELEM_PIN);
      if (v != last) {
        last = v;
        if (nEdge < 400) { edgeT[nEdge] = micros() - t0; edgeL[nEdge++] = v; }
      }
    }
    Serial.printf("> capture (%s): %lu transitions in %lu ms, start level %d, end level %d\n",
                  label, (unsigned long)(nEdge - 1), (unsigned long)(winUs / 1000), edgeL[0], last);
    for (uint32_t i = 1; i < nEdge; ++i) {
      Serial.printf("  %4luus %s", (unsigned long)edgeT[i], edgeL[i] ? "H" : "L");
      if (i % 8 == 0) Serial.println();
    }
    Serial.println();
    nextUs = micros() + 1000000UL / DSHOT_LOOP_HZ;  // resync after the stall
  }

  if (esc.isArmed() && !g_wasArmed) {
    g_wasArmed = true;
    Serial.println("*** ARMED: throttle commands are now passed through ***");
  }

  if (now - lastStatusMs >= DSHOT_STATUS_MS) {
    lastStatusMs = now;
    Serial.printf("[%s] thr %+.3f dshot %4u frame 0x%04X", !g_txEnabled ? "TX-OFF" : g_wasArmed ? "ARMED " : "arming",
                  g_throttle, throttleToDshot3D(g_throttle), esc.lastFrame());
#if DSHOT_BIDIR
    static const char *st[] = {"OK", "IDLE", "NO-REPLY", "FRAMING", "BAD-GCR", "BAD-CRC"};
    Serial.printf(" | bidir %-8s echo %2u rpm %7.0f", st[esc.status()], esc.echoPulses(),
                  esc.rpm());
#endif
    if (g_telem.valid) {
      Serial.printf(" | telem %3lums ago: %uC %.2fV %.2fA %umAh erpm %lu (rpm %.0f) n=%lu",
                    (unsigned long)(now - g_telem.lastMs), g_telem.tempC, g_telem.volts,
                    g_telem.amps, g_telem.mAh, (unsigned long)g_telem.erpm,
                    g_telem.erpm * 2.0f / DSHOT_MOTOR_POLES, (unsigned long)g_telem.count);
    } else {
      Serial.printf(" | telem: none yet (crcErr %lu)", (unsigned long)g_telem.crcErr);
    }
    Serial.println();
  }
}
