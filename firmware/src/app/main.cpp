/**
 * @file main.cpp
 * @brief Eyeliner firmware entry: starts the control task on core 1 and the
 *        comms task on core 0, then retires Arduino's loop task.
 *
 * Port of original_code/Main_DPD_Alliances.ino onto a two-core layout.
 * See config.h for the core assignment and build switches.
 */
#include <Arduino.h>

#include "config.h"
#include "heading.h"
#include "imu.h"
#include "leds.h"
#include "motor.h"
#include "power.h"
#include "receiver.h"
#include "telemetry.h"

/* ---------------------------------------------------------------------------
 * Core 1: heading + motor control only (1 kHz).
 *   receiver/IMU snapshots -> heading estimate -> stop-logic registers ->
 *   one DShot frame per ESC. On failsafe the ESCs get zero throttle instead.
 * ------------------------------------------------------------------------- */
// Written by the control task, read by the comms task for DISPLAY_SERIAL, so
// the control core never touches the serial port itself.
static volatile int      s_controlCore = -1;
static volatile uint32_t s_controlLoops = 0;
static volatile uint32_t s_controlRxSeq = 0;
static volatile uint32_t s_controlBusyMaxUs = 0;   // longest loop body since last report
static volatile uint32_t s_controlBusySumUs = 0;   // summed loop body time since last report

static volatile bool     s_motorsOk = false;

static void controlTask(void *) {
  s_controlCore = xPortGetCoreID();
  headingReset();
  s_motorsOk = motorInit();
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    uint32_t t0 = micros();
    ReceiverState rx = receiverGet();
    ImuState imu = imuGet();

    // Drive mode from SB; melty flipped spins the other way, so the heading
    // integrates with the opposite sign
    MotorMode mode = motorModeFromUs(rxChannelUs(rx, MOTOR_MODE_CHANNEL));

    // Heading: one radial high-g axis, rate constant K set by S1 (CH10)
    headingUpdate(imu, rxAuxAxis(rx, RX_CH_S1), motorSpinSign(mode), micros());

    // Kill: receiver failsafe, or SC held high
    bool killSwitch = rxChannelUs(rx, MOTOR_KILL_CHANNEL) > MOTOR_KILL_US;
    if (rx.safe && !killSwitch) {
      // Registers: mode; melty spin throttle (own dead zone + scale), heading .
      // right stick direction, |right stick|; tank axes (own dead zone)
      float tx = rx.axis[RX_RIGHT_X], ty = rx.axis[RX_RIGHT_Y];
      float spinup = motorSpinupFromUs(rxChannelUs(rx, MOTOR_SPINUP_CHANNEL));
      motorSetMode(mode);
      motorSetInputs(spinup, headingDot(tx, ty), sqrtf(tx * tx + ty * ty));
      motorSetTankInputs(motorTankAxisFromUs(rx.raw[RX_RIGHT_X]), motorTankAxisFromUs(rx.raw[RX_RIGHT_Y]));
      motorStopLogic();
    } else {
      motorKill();
    }
    s_controlRxSeq = rx.seq;
    s_controlLoops = s_controlLoops + 1;
    uint32_t busy = micros() - t0;
    s_controlBusySumUs = s_controlBusySumUs + busy;
    if (busy > s_controlBusyMaxUs) s_controlBusyMaxUs = busy;
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
}

/* ---------------------------------------------------------------------------
 * Core 0: everything else.
 * ------------------------------------------------------------------------- */
static void commsTask(void *) {
  // Serial is started here, not in setup(), so the USB-serial interrupt is
  // allocated on this core and never lands on the control core
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // never block the comms loop on a host that isn't reading
#ifdef DISPLAY_SERIAL
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  Serial.println("\n=== Eyeliner ===");
#endif

  bool rxOk = receiverInit();
  int imus = imuInit();
  bool ledsOk = ledsInit();  // after imuInit: needs Wire
  powerInit();
#ifdef ESP_NOW
  bool telemOk = telemetryInit();
#else
  bool telemOk = false;
#endif
#ifdef DISPLAY_SERIAL
  Serial.printf("[comms] core %d, receiver %s, %d IMUs, PCA9570 %s, ESP-NOW %s\n", xPortGetCoreID(),
                rxOk ? "up" : "FAILED", imus, ledsOk ? "ok" : "NO ACK",
#ifdef ESP_NOW
                telemOk ? "up" : "FAILED");
#else
                "off");
#endif
#else
  (void)rxOk;
  (void)imus;
  (void)ledsOk;
  (void)telemOk;
#endif
  uint32_t loops = 0, busySum = 0, busyMax = 0;
  static SystemStats stats = {};
  for (;;) {
    uint32_t tLoop = micros();
    receiverUpdate();
    imuUpdate();
    ledsUpdate(headingGet(), motorGet());
    powerUpdate();
#ifdef ESP_NOW
    telemetryUpdate(receiverGet(), imuGet(), headingGet(), motorGet(), ledsGet(), powerGet(), stats);
#endif

    // Once a second: loop statistics (for telemetry and the DISPLAY_SERIAL line)
    static uint32_t lastStatsMs = 0, lastControlLoops = 0, lastUartBytes = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastStatsMs >= 1000) {
      lastStatsMs = nowMs;
      uint32_t cl = s_controlLoops;
      ReceiverState rxs = receiverGet();
      stats.commsLoopsPerS = (uint16_t)loops;
      stats.commsBusyMaxUs = (uint16_t)(busyMax > 65535 ? 65535 : busyMax);
      stats.controlLoopsPerS = (uint16_t)(cl - lastControlLoops);
      stats.controlBusyMaxUs = (uint16_t)(s_controlBusyMaxUs > 65535 ? 65535 : s_controlBusyMaxUs);
      stats.uartBytesPerS = (uint16_t)(rxs.uartBytes - lastUartBytes);
#ifdef DISPLAY_SERIAL
      uint32_t n = cl - lastControlLoops, sum = s_controlBusySumUs;
      Serial.printf("[cores] comms core %d: %lu loops/s, busy avg %lu us max %lu us, uart %lu B/s | "
                    "control core %d: %lu loops/s, busy avg %lu us max %lu us | motors %s\n",
                    xPortGetCoreID(), (unsigned long)loops, (unsigned long)(loops ? busySum / loops : 0),
                    (unsigned long)busyMax, (unsigned long)stats.uartBytesPerS, s_controlCore, (unsigned long)n,
                    (unsigned long)(n ? sum / n : 0), (unsigned long)s_controlBusyMaxUs,
                    s_motorsOk ? "ok" : "INIT FAILED");
      LedState led = ledsGet();
      static const char *ledModeName[] = {"melty", "tank", "failsafe"};
      Serial.printf("[led] %s rgb %3u %3u %3u arc %d | %lu frames | PCA9570 %s\n",
                    ledModeName[led.mode <= 2 ? led.mode : 2], led.r, led.g, led.b, led.arcOn,
                    (unsigned long)led.frames, led.pcaOk ? "ok" : "NO ACK");
      PowerState pw = powerGet();
      TelemetryStats ts = telemetryStats();
      Serial.printf("[pwr] vbat %5.2f V (%u mV at pin) | chip %4.1f C | espnow %s (SA %s) ch %d: fast %lu slow %lu, send errors %lu, cb fail %lu\n",
                    pw.vbatV, pw.senseMv, pw.chipTempC,
#ifdef ESP_NOW
                    telemetryInitError() ? telemetryInitError() : (ts.radioOn ? "radio on" : "radio off"),
                    ts.enabled ? "on" : "off", telemetryChannel(),
#else
                    "off", "-", -1,
#endif
                    (unsigned long)ts.sentFast, (unsigned long)ts.sentSlow, (unsigned long)ts.sendErrors,
                    (unsigned long)ts.cbFail);
#endif
      s_controlBusySumUs = 0;
      s_controlBusyMaxUs = 0;
      lastControlLoops = cl;
      lastUartBytes = rxs.uartBytes;
      loops = busySum = busyMax = 0;
    }

#ifdef DISPLAY_SERIAL
    static uint32_t lastHeadingMs = 0;
    if (nowMs - lastHeadingMs >= HEADING_TELEM_PERIOD_MS) {
      lastHeadingMs = nowMs;
      HeadingState h = headingGet();
      Serial.printf("[hdg] a %+6.2f m/s2 | rate %7.1f dps | heading %6.1f deg | K %5.1f | samples %lu\n", h.accel,
                    h.rateDps, h.headingDeg, h.k, (unsigned long)h.samples);
      MotorOutputs m = motorGet();
      static const char *modeName[] = {"melty", "tank ", "flip "};
      Serial.printf("[mot] %s %s spin %+.2f cos %+.2f mag %.2f tank %+.2f %+.2f | D %.2f M %+.0f mod %+.2f | m1 %+.2f m2 %+.2f | dshot %4u %4u | %s%s | %lu frames\n",
                    m.killed ? "KILL" : "run ", modeName[m.in.mode <= 2 ? m.in.mode : 0], m.in.spinup, m.in.headingCos,
                    m.in.translation, m.in.tankX, m.in.tankY, m.D, m.M, m.modulation, m.throttle[0], m.throttle[1],
                    m.dshot[0], m.dshot[1],
#ifdef MOTORS
                    m.armed[0] ? "armed " : "arming ", m.armed[1] ? "armed" : "arming",
#else
                    "MOTORS off", "",
#endif
                    (unsigned long)m.frames);
    }
#endif
    uint32_t busy = micros() - tLoop;
    busySum += busy;
    if (busy > busyMax) busyMax = busy;
    loops++;
    vTaskDelay(1);
  }
}

void setup() {
  xTaskCreatePinnedToCore(controlTask, "control", CONTROL_TASK_STACK, nullptr, CONTROL_TASK_PRIORITY, nullptr,
                          CONTROL_TASK_CORE);
  xTaskCreatePinnedToCore(commsTask, "comms", COMMS_TASK_STACK, nullptr, COMMS_TASK_PRIORITY, nullptr,
                          COMMS_TASK_CORE);
}

void loop() {
  // Work is in the two pinned tasks; free core 1 of the Arduino loop task.
  vTaskDelete(nullptr);
}
