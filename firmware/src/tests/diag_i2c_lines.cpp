/**
 * @file diag_i2c_lines.cpp
 * @brief I2C bus line diagnostic, independent of the I2C peripheral.
 *
 * Build/flash: pio run -e diag_i2c_lines -t upload
 * Monitor:     pio device monitor -e diag_i2c_lines
 * Runs once at boot. Serial commands:
 *   s          rise-time study on SDA and SCL (see riseStudy())
 *   h          hold SCL low (open-drain) until the next key, for probing
 *   d          hold SDA low (open-drain) until the next key, for probing
 *   f          fast bit-bang at DIAG_FAST_HZ (default 1 MHz): address scan and
 *              WHO_AM_I read from DIAG_FAST_ADDR (default 0x6A, U5)
 *   any other  rerun the full diagnostic
 *
 * Checks:
 *   1. SDA and SCL can each be pulled low by the ESP32, and how long each
 *      takes to float back high (catches a line held high by a stuck or
 *      misfitted device, and weak/missing pull-ups)
 *   2. INT idle level. Read only, never driven. A powered LSM6DSV320X holds
 *      INT1 low until it's configured (datasheet: "output forced to ground")
 *   3. Bit-banged I2C address probe at ~10 kHz, bypassing the I2C peripheral
 *
 * Drive modes: SDA/SCL are only ever released (input) or pulled low
 * (open-drain). INT is input only. The mux select pin is not touched unless
 * DIAG_MUX_SEL_PIN is set.
 */
#include <Arduino.h>

#include "driver/gpio.h"
#include "esp_cpu.h"
#include "soc/gpio_reg.h"

#include "board_pins.h"

#ifndef DIAG_SDA_PIN
#define DIAG_SDA_PIN PIN_I2C_SDA
#endif

#ifndef DIAG_SCL_PIN
#define DIAG_SCL_PIN PIN_I2C_SCL
#endif

#ifndef DIAG_INT_PIN
#define DIAG_INT_PIN PIN_IMU_INT
#endif

// -1: leave the mux select alone (e.g. SEL hard-wired on a bench rig)
#ifndef DIAG_MUX_SEL_PIN
#define DIAG_MUX_SEL_PIN -1
#endif

#ifndef DIAG_MUX_SEL_LEVEL
#define DIAG_MUX_SEL_LEVEL I2C_SEL_LEVEL_IMU
#endif

// Fast bit-bang clock rate and target for the 'f' command
#ifndef DIAG_FAST_HZ
#define DIAG_FAST_HZ 1000000UL
#endif

#ifndef DIAG_FAST_ADDR
#define DIAG_FAST_ADDR IMU_U5_I2C_ADDR
#endif

#define LSM6DSV_WHO_AM_I_REG 0x0F
#define LSM6DSV_WHO_AM_I_VAL 0x73

#define HELP "'s' = rise-time study, 'h'/'d' = hold SCL/SDA low, 'f' = fast bit-bang, any other key = rerun."

// Half an SCL period for the bit-banged probe (~10 kHz)
#define HALF_PERIOD_US 50

static void release(int pin) { pinMode(pin, INPUT); }
static void pullLow(int pin) {
  pinMode(pin, OUTPUT_OPEN_DRAIN);
  digitalWrite(pin, LOW);
}

// Longest wait for a released line to read HIGH again. With R10/R11 (4.7k)
// and a short bus the rise should take a few microseconds
#define RISE_TIMEOUT_US 100000UL
#define RISE_WARN_US    50UL

// Pull a line low, let it go, and time how long it takes to read HIGH again
static void lineTest(const char *name, int pin) {
  release(pin);
  delayMicroseconds(200);
  int idle = digitalRead(pin);
  pullLow(pin);
  delayMicroseconds(200);
  int driven = digitalRead(pin);

  release(pin);
  unsigned long t0 = micros();
  unsigned long riseUs = 0;
  bool rose = false;
  while ((riseUs = micros() - t0) < RISE_TIMEOUT_US) {
    if (digitalRead(pin)) {
      rose = true;
      break;
    }
  }

  const char *verdict = "OK";
  if (idle == 0) verdict = "<-- idles LOW (missing pull-up, or held low)";
  else if (driven == 1) verdict = "<-- can't be pulled LOW (held high)";
  else if (!rose) verdict = "<-- stays LOW after release (held low, or no pull-up)";
  else if (riseUs > RISE_WARN_US) verdict = "<-- slow rise (weak/missing pull-up or heavy load)";

  if (rose) {
    Serial.printf("  %-4s GPIO%-2d idle=%d  driven-low=%d  rise=%lu us  %s\n", name, pin, idle, driven, riseUs,
                  verdict);
  } else {
    Serial.printf("  %-4s GPIO%-2d idle=%d  driven-low=%d  rise=>%lu us  %s\n", name, pin, idle, driven,
                  RISE_TIMEOUT_US, verdict);
  }
}

// Time for a released line to read HIGH, or RISE_TIMEOUT_US if it never does
static unsigned long measureRise(int pin, uint8_t releaseMode) {
  pinMode(pin, releaseMode);
  unsigned long t0 = micros();
  unsigned long us;
  while ((us = micros() - t0) < RISE_TIMEOUT_US) {
    if (digitalRead(pin)) return us;
  }
  return RISE_TIMEOUT_US;
}

static void printRise(unsigned long us) {
  if (us >= RISE_TIMEOUT_US) Serial.print("  >100ms");
  else Serial.printf(" %7lu", us);
}

// Repeated pull-low / release cycles to see whether the rise time drifts.
// A rise time that grows cycle over cycle, and faster with longer holds, means
// the pull-up's supply end is sagging (not tied to a stiff 3V3). If enabling
// the ESP's internal ~45k pull-up speeds the line up, the external pull-up
// isn't effectively connected at the pin
static void riseStudy(const char *name, int pin) {
  static const unsigned long HOLDS_US[] = {200, 2000, 20000};
  const int CYCLES = 10;

  Serial.printf("\n%s GPIO%d rise times in us (%d cycles per row)\n", name, pin, CYCLES);
  for (unsigned long hold : HOLDS_US) {
    Serial.printf("  hold %5lu us:", hold);
    for (int i = 0; i < CYCLES; i++) {
      pullLow(pin);
      delayMicroseconds(hold);
      printRise(measureRise(pin, INPUT));
      delay(5);
    }
    Serial.println();
  }

  delay(1000);
  Serial.print("  after 1 s idle, hold 200 us:");
  pullLow(pin);
  delayMicroseconds(200);
  printRise(measureRise(pin, INPUT));
  Serial.println();

  Serial.print("  ESP internal pull-up on, hold 200 us:");
  for (int i = 0; i < 5; i++) {
    pullLow(pin);
    delayMicroseconds(200);
    printRise(measureRise(pin, INPUT_PULLUP));
    delay(5);
  }
  Serial.println();
  release(pin);
}

static void sclHigh() {
  release(DIAG_SCL_PIN);
  delayMicroseconds(HALF_PERIOD_US);
}
static void sclLow() {
  pullLow(DIAG_SCL_PIN);
  delayMicroseconds(HALF_PERIOD_US);
}
static void sdaHigh() { release(DIAG_SDA_PIN); }
static void sdaLow() { pullLow(DIAG_SDA_PIN); }

// Sends START + address/W and returns true if the device ACKs
static bool probe(uint8_t addr) {
  sdaHigh();
  sclHigh();
  sdaLow(); // START
  delayMicroseconds(HALF_PERIOD_US);
  sclLow();

  uint8_t b = addr << 1;
  for (int i = 7; i >= 0; i--) {
    if (b & (1 << i)) sdaHigh();
    else sdaLow();
    sclHigh();
    sclLow();
  }

  sdaHigh(); // ACK clock
  sclHigh();
  bool ack = digitalRead(DIAG_SDA_PIN) == LOW;
  sclLow();

  sdaLow(); // STOP
  sclHigh();
  sdaHigh();
  delayMicroseconds(HALF_PERIOD_US);
  return ack;
}

static void runDiag() {
  Serial.println("\n=== I2C line diagnostic ===");
#if DIAG_MUX_SEL_PIN >= 0
  Serial.printf("mux SEL GPIO%d = %d\n", DIAG_MUX_SEL_PIN, DIAG_MUX_SEL_LEVEL);
#else
  Serial.println("mux SEL: not driven");
#endif

  Serial.println("lines:");
  lineTest("SDA", DIAG_SDA_PIN);
  lineTest("SCL", DIAG_SCL_PIN);
  int intLevel = digitalRead(DIAG_INT_PIN);
  Serial.printf("  INT  GPIO%-2d level=%d  (read only; powered LSM6DSV holds INT1 LOW until configured)\n",
                DIAG_INT_PIN, intLevel);

  Serial.println("bit-bang probe 0x08-0x77:");
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (probe(a)) {
      Serial.printf("  ACK at 0x%02X%s\n", a,
                    a == IMU_U5_I2C_ADDR ? "  (U5)" : a == IMU_U4_I2C_ADDR ? "  (U4)" : "");
      found++;
    }
  }
  if (!found) Serial.println("  (no ACKs)");

  release(DIAG_SDA_PIN);
  release(DIAG_SCL_PIN);
  Serial.println("done, lines released. " HELP);
}


/* ---------------------------------------------------------------------------
 * Fast bit-bang (register-level GPIO, cycle-counter timing)
 *
 * Pins are set to open-drain output once; writing 1 releases the line to the
 * pull-up and writing 0 pulls it low, so the drive mode stays open-drain.
 * Each half period waits on a CPU cycle-count deadline so call overhead
 * doesn't stretch the clock. After every SCL release the code waits for SCL
 * to actually read HIGH; a release that takes longer than a half period is
 * counted as "late" (pull-up too weak / bus too slow for this rate).
 * ------------------------------------------------------------------------- */

static uint32_t fastHalfCycles;
static uint32_t fastDeadline;
static uint32_t fastSclClocks;
static uint32_t fastSclLate;
static bool fastSclStuck;

static inline void fastWrite(int pin, bool high) {
  if (pin < 32) REG_WRITE(high ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, 1UL << pin);
  else REG_WRITE(high ? GPIO_OUT1_W1TS_REG : GPIO_OUT1_W1TC_REG, 1UL << (pin - 32));
}

static inline int fastRead(int pin) {
  return pin < 32 ? (REG_READ(GPIO_IN_REG) >> pin) & 1 : (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1;
}

static inline void fastHalfWait() {
  fastDeadline += fastHalfCycles;
  while ((int32_t)(esp_cpu_get_cycle_count() - fastDeadline) < 0) {
  }
}

// Release SCL and wait for it to read HIGH (rise time / clock stretching)
static inline void fastSclRelease() {
  fastWrite(DIAG_SCL_PIN, true);
  uint32_t start = esp_cpu_get_cycle_count();
  while (!fastRead(DIAG_SCL_PIN)) {
    if (esp_cpu_get_cycle_count() - start > 1000 * fastHalfCycles) {
      fastSclStuck = true;
      break;
    }
  }
  uint32_t now = esp_cpu_get_cycle_count();
  if (now - start > fastHalfCycles) {
    fastSclLate++;
    fastDeadline = now; // resync so the high phase still gets a full half period
  }
  fastSclClocks++;
}

static void fastStart() {
  fastWrite(DIAG_SDA_PIN, true);
  fastSclRelease();
  fastHalfWait();
  fastWrite(DIAG_SDA_PIN, false);
  fastHalfWait();
  fastWrite(DIAG_SCL_PIN, false);
  fastHalfWait();
}

static void fastRepeatedStart() {
  fastWrite(DIAG_SDA_PIN, true);
  fastHalfWait();
  fastStart();
}

static void fastStop() {
  fastWrite(DIAG_SDA_PIN, false);
  fastHalfWait();
  fastSclRelease();
  fastHalfWait();
  fastWrite(DIAG_SDA_PIN, true);
  fastHalfWait();
}

static void fastWriteBit(bool bit) {
  fastWrite(DIAG_SDA_PIN, bit);
  fastHalfWait();
  fastSclRelease();
  fastHalfWait();
  fastWrite(DIAG_SCL_PIN, false);
}

static bool fastReadBit() {
  fastWrite(DIAG_SDA_PIN, true);
  fastHalfWait();
  fastSclRelease();
  fastHalfWait();
  bool bit = fastRead(DIAG_SDA_PIN);
  fastWrite(DIAG_SCL_PIN, false);
  return bit;
}

// Returns true on ACK
static bool fastWriteByte(uint8_t b) {
  for (int i = 7; i >= 0; i--) fastWriteBit(b & (1 << i));
  return !fastReadBit();
}

static uint8_t fastReadByte(bool ack) {
  uint8_t b = 0;
  for (int i = 0; i < 8; i++) b = (b << 1) | fastReadBit();
  fastWriteBit(!ack);
  return b;
}

static void fastBegin() {
  fastHalfCycles = (uint32_t)getCpuFrequencyMhz() * 1000000UL / (2 * DIAG_FAST_HZ);
  gpio_set_direction((gpio_num_t)DIAG_SDA_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_direction((gpio_num_t)DIAG_SCL_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_pullup_dis((gpio_num_t)DIAG_SDA_PIN);
  gpio_pullup_dis((gpio_num_t)DIAG_SCL_PIN);
  fastWrite(DIAG_SDA_PIN, true);
  fastWrite(DIAG_SCL_PIN, true);
  delayMicroseconds(100);
}

static void fastResetStats() {
  fastSclClocks = 0;
  fastSclLate = 0;
  fastSclStuck = false;
}

static bool fastProbe(uint8_t addr) {
  portDISABLE_INTERRUPTS();
  fastDeadline = esp_cpu_get_cycle_count();
  fastStart();
  bool ack = fastWriteByte(addr << 1);
  fastStop();
  portENABLE_INTERRUPTS();
  return ack;
}

// Register read: START addr+W reg, REPEATED START addr+R, read 1 byte, NACK, STOP
static bool fastReadReg(uint8_t addr, uint8_t reg, uint8_t *val, uint32_t *elapsedCycles) {
  portDISABLE_INTERRUPTS();
  uint32_t t0 = esp_cpu_get_cycle_count();
  fastDeadline = t0;
  fastStart();
  bool ok = fastWriteByte(addr << 1);
  if (ok) ok = fastWriteByte(reg);
  if (ok) {
    fastRepeatedStart();
    ok = fastWriteByte((addr << 1) | 1);
  }
  if (ok) *val = fastReadByte(false);
  fastStop();
  *elapsedCycles = esp_cpu_get_cycle_count() - t0;
  portENABLE_INTERRUPTS();
  return ok;
}

static void runFastBitBang() {
  fastBegin();
  Serial.printf("\n=== bit-bang @ %lu kHz (half period %lu CPU cycles @ %lu MHz) ===\n",
                (unsigned long)(DIAG_FAST_HZ / 1000), (unsigned long)fastHalfCycles,
                (unsigned long)getCpuFrequencyMhz());

  fastResetStats();
  Serial.println("address scan 0x08-0x77:");
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (fastProbe(a)) {
      Serial.printf("  ACK at 0x%02X%s\n", a,
                    a == IMU_U5_I2C_ADDR ? "  (U5)" : a == IMU_U4_I2C_ADDR ? "  (U4)" : "");
      found++;
    }
  }
  if (!found) Serial.println("  (no ACKs)");
  Serial.printf("  SCL clocks %lu, late rises %lu%s\n", (unsigned long)fastSclClocks, (unsigned long)fastSclLate,
                fastSclStuck ? ", SCL STUCK LOW" : "");

  Serial.printf("WHO_AM_I (0x%02X) from 0x%02X, 5 reads:\n", LSM6DSV_WHO_AM_I_REG, DIAG_FAST_ADDR);
  int good = 0;
  for (int i = 0; i < 5; i++) {
    fastResetStats();
    uint8_t val = 0;
    uint32_t cycles = 0;
    bool ok = fastReadReg(DIAG_FAST_ADDR, LSM6DSV_WHO_AM_I_REG, &val, &cycles);
    float us = cycles / (float)getCpuFrequencyMhz();
    if (ok) {
      bool match = val == LSM6DSV_WHO_AM_I_VAL;
      if (match) good++;
      Serial.printf("  read %d: 0x%02X %s  %lu SCL clocks in %.1f us (~%.0f kHz effective), late rises %lu\n", i + 1,
                    val, match ? "OK" : "MISMATCH", (unsigned long)fastSclClocks, us, fastSclClocks * 1000.0f / us,
                    (unsigned long)fastSclLate);
    } else {
      Serial.printf("  read %d: NACK  (late rises %lu%s)\n", i + 1, (unsigned long)fastSclLate,
                    fastSclStuck ? ", SCL STUCK LOW" : "");
    }
    delay(2);
  }
  Serial.printf("WHO_AM_I: %d/5 correct (expect 0x%02X)\n", good, LSM6DSV_WHO_AM_I_VAL);

  release(DIAG_SDA_PIN);
  release(DIAG_SCL_PIN);
  Serial.println("done, lines released. " HELP);
}

static void runRiseStudy() {
  Serial.println("\n=== rise-time study ===");
  riseStudy("SDA", DIAG_SDA_PIN);
  riseStudy("SCL", DIAG_SCL_PIN);
  Serial.println("\ndone, lines released. " HELP);
}

void setup() {
#if DIAG_MUX_SEL_PIN >= 0
  pinMode(DIAG_MUX_SEL_PIN, OUTPUT);
  digitalWrite(DIAG_MUX_SEL_PIN, DIAG_MUX_SEL_LEVEL);
#endif
  pinMode(DIAG_INT_PIN, INPUT);
  release(DIAG_SDA_PIN);
  release(DIAG_SCL_PIN);

  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) delay(10);
  delay(200);

  runDiag();
}

void loop() {
  if (Serial.available()) {
    int c = Serial.read();
    delay(5);
    while (Serial.available()) Serial.read();
    if (c == 's') {
      runRiseStudy();
    } else if (c == 'f') {
      runFastBitBang();
    } else if (c == 'h' || c == 'd') {
      int pin = c == 'h' ? DIAG_SCL_PIN : DIAG_SDA_PIN;
      const char *name = c == 'h' ? "SCL" : "SDA";
      pullLow(pin);
      Serial.printf("\nholding %s (GPIO%d) LOW (open-drain). Press any key to release.\n", name, pin);
      while (!Serial.available()) delay(10);
      while (Serial.available()) Serial.read();
      release(pin);
      Serial.printf("%s released. %s\n", name, HELP);
    } else {
      runDiag();
    }
  }
  delay(10);
}
