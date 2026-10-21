/**
 * @file test_lsm6dsv.cpp
 * @brief Dual LSM6DSV320X test over hardware I2C at 1 MHz (ESP32-S3 I2C peripheral).
 *
 * Build/flash: pio run -e test_lsm6dsv -t upload
 * Monitor:     pio device monitor -e test_lsm6dsv
 *
 * What it checks, for both IMUs (0x6A and 0x6B):
 *   1. I2C bus scan
 *   2. WHO_AM_I == 0x73
 *   3. INT1 open-drain / polarity in IF_CFG, with readback
 *   4. Shared INT line idles at the inactive level once both are configured
 *   5. Accel, gyro and high-g accel stream at the configured ODR from both,
 *      with per-IMU sample rates and the bus time per read
 *
 * Shared INT: both INT1 pins sit on one net (pull-up to 3V3), so both are set
 * open-drain + active-low (wired-OR). The ESP can't tell which IMU pulled the
 * line, so every wakeup reads STATUS + data from both in one burst each.
 *
 * Every LSM_* macro below can be overridden from build_flags in platformio.ini.
 *
 * Datasheet refs (DS14623 rev 2): I2C timing table 8 (400 kHz FM / 1 MHz FM+),
 * address section 5.1.2 (110101[SA0]b), IF_CFG (03h) section 9.3.
 */
#include <Arduino.h>
#include <Wire.h>

#include <atomic>

#include "driver/gpio.h"
#include "esp_cpu.h"
#include "soc/gpio_periph.h"
#include "soc/gpio_reg.h"

#include "board_pins.h"
#include "lsm6dsv320x_reg.h"

/* ---------------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------------- */

// ESP32-S3 has two hardware I2C controllers: 0 -> Wire, 1 -> Wire1
#ifndef LSM_I2C_PORT
#define LSM_I2C_PORT 0
#endif

#ifndef LSM_SDA_PIN
#define LSM_SDA_PIN PIN_I2C_SDA
#endif

#ifndef LSM_SCL_PIN
#define LSM_SCL_PIN PIN_I2C_SCL
#endif

// SCL frequency. LSM6DSV320X supports up to 1 MHz (I2C fast mode plus)
#ifndef LSM_I2C_FREQ_HZ
#define LSM_I2C_FREQ_HZ 1000000
#endif

// 7-bit addresses of the two IMUs (SDO/SA0 low = 0x6A, high = 0x6B).
// Set either to -1 to run with a single IMU
#ifndef LSM_ADDR_A
#define LSM_ADDR_A 0x6A  // U4 on the current netlist
#endif

#ifndef LSM_ADDR_B
#define LSM_ADDR_B 0x6B  // U3 on the current netlist
#endif

// ESP32 GPIO connected to the shared INT1 net
#ifndef LSM_INT_PIN
#define LSM_INT_PIN PIN_IMU_INT
#endif

// 1: open-drain (required on the shared INT net), 0: push-pull
#ifndef LSM_INT_OPEN_DRAIN
#define LSM_INT_OPEN_DRAIN 1
#endif

// 1: active-low, 0: active-high. Open-drain on a pulled-up net should be active-low
#ifndef LSM_INT_ACTIVE_LOW
#define LSM_INT_ACTIVE_LOW 1
#endif

// Output data rate for accel, gyro and high-g. One 28-byte burst per IMU takes
// roughly 0.3 ms at 1 MHz, so two IMUs at 960 Hz use most of the bus
#ifndef LSM_XL_ODR
#define LSM_XL_ODR LSM6DSV320X_ODR_AT_480Hz
#endif

#ifndef LSM_GY_ODR
#define LSM_GY_ODR LSM6DSV320X_ODR_AT_480Hz
#endif

#ifndef LSM_HG_ODR
#define LSM_HG_ODR LSM6DSV320X_HG_XL_ODR_AT_480Hz
#endif

#ifndef LSM_ODR_HZ
#define LSM_ODR_HZ 480.0f
#endif

// If INT stays quiet this long, poll both IMUs anyway (counted separately)
#ifndef LSM_POLL_FALLBACK_MS
#define LSM_POLL_FALLBACK_MS 20
#endif

#ifndef LSM_PRINT_PERIOD_MS
#define LSM_PRINT_PERIOD_MS 250
#endif

/* Sanity checks */
static_assert(LSM_I2C_FREQ_HZ > 0 && LSM_I2C_FREQ_HZ <= 1000000,
              "LSM6DSV320X I2C supports up to 1 MHz (fast mode plus)");
static_assert(LSM_I2C_PORT == 0 || LSM_I2C_PORT == 1, "ESP32-S3 has I2C ports 0 and 1");
static_assert(LSM_ADDR_A == -1 || LSM_ADDR_A == 0x6A || LSM_ADDR_A == 0x6B, "LSM6DSV320X address is 0x6A or 0x6B");
static_assert(LSM_ADDR_B == -1 || LSM_ADDR_B == 0x6A || LSM_ADDR_B == 0x6B, "LSM6DSV320X address is 0x6A or 0x6B");
static_assert(LSM_ADDR_A != LSM_ADDR_B, "the two IMUs need different addresses");

#if !LSM_INT_OPEN_DRAIN && LSM_ADDR_A >= 0 && LSM_ADDR_B >= 0
#warning "Push-pull INT with two IMUs on a shared INT net: the outputs will fight. Use open-drain."
#endif

#if LSM_INT_OPEN_DRAIN && !LSM_INT_ACTIVE_LOW
#warning "Open-drain + active-high: the line idles LOW and is released when asserted. Usually you want active-low."
#endif

#if LSM_I2C_PORT == 0
#define IMU_WIRE Wire
#else
#define IMU_WIRE Wire1
#endif

// One burst from STATUS_REG through the high-g outputs
#define BURST_START    LSM6DSV320X_STATUS_REG          // 0x1E
#define BURST_END      LSM6DSV320X_UI_OUTZ_H_A_OIS_HG  // 0x39
#define BURST_LEN      (BURST_END - BURST_START + 1)   // 28 bytes
#define BURST_OFS(reg) ((reg) - BURST_START)

static const int INT_ACTIVE_LEVEL = LSM_INT_ACTIVE_LOW ? LOW : HIGH;

/* ---------------------------------------------------------------------------
 * Per-IMU state and ST driver glue
 * ------------------------------------------------------------------------- */

struct Imu {
  uint8_t addr;
  bool present;
  stmdev_ctx_t ctx;

  int16_t xl[3], gy[3], hg[3], temp;

  uint32_t xlCount, gyCount, hgCount;  // new samples seen in STATUS
  uint32_t intReads, polledReads;      // burst reads by trigger
  uint32_t errors;
  uint64_t busCycles;                  // total time spent in burst reads
  uint32_t busReads;

  uint32_t lastXl, lastGy, lastHg, lastIntReads, lastPolledReads, lastBusReads;
  uint64_t lastBusCycles;
};

static int32_t platformWrite(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len) {
  Imu *imu = static_cast<Imu *>(handle);
  IMU_WIRE.beginTransmission(imu->addr);
  IMU_WIRE.write(reg);
  IMU_WIRE.write(buf, len);
  return IMU_WIRE.endTransmission() == 0 ? 0 : -1;
}

static int32_t platformRead(void *handle, uint8_t reg, uint8_t *buf, uint16_t len) {
  Imu *imu = static_cast<Imu *>(handle);

  // Register address write, then repeated start into the read
  IMU_WIRE.beginTransmission(imu->addr);
  IMU_WIRE.write(reg);
  if (IMU_WIRE.endTransmission(false) != 0) return -1;

  if (IMU_WIRE.requestFrom((uint16_t)imu->addr, (size_t)len, true) != len) return -1;
  for (uint16_t i = 0; i < len; i++) buf[i] = IMU_WIRE.read();
  return 0;
}

static void platformDelay(uint32_t ms) {
  delay(ms);
}

static Imu imus[2];
static int imuCount = 0;

static void addImu(int addr) {
  if (addr < 0) return;
  Imu &imu = imus[imuCount++];
  memset(&imu, 0, sizeof(imu));
  imu.addr = (uint8_t)addr;
  imu.ctx = {platformWrite, platformRead, platformDelay, &imu, nullptr};
}

/* ---------------------------------------------------------------------------
 * Interrupt
 * ------------------------------------------------------------------------- */

static std::atomic<uint32_t> isrCount{0};
static std::atomic<bool> drdyPending{false};

static void IRAM_ATTR onImuInt() {
  isrCount++;
  drdyPending = true;
}

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

// Runs a driver call and reports it by name if it returns an error
#define CHECK(imu, call)                                                                    \
  do {                                                                                      \
    int32_t ret_ = (call);                                                                  \
    Serial.printf("  [%s] 0x%02X %s\n", ret_ == 0 ? " OK " : "FAIL", (imu).addr, #call); \
  } while (0)

// Reads back the ESP32 pad config: SDA/SCL should be OE=1 OD=1 IE=1, INT should be OE=0 IE=1
static void printPinMode(const char *name, int pin, bool expectOpenDrainIo) {
  uint32_t oe = pin < 32 ? (REG_READ(GPIO_ENABLE_REG) >> pin) & 1 : (REG_READ(GPIO_ENABLE1_REG) >> (pin - 32)) & 1;
  uint32_t od = (REG_READ(GPIO_PIN0_REG + 4 * pin) >> GPIO_PIN0_PAD_DRIVER_S) & 1;
  uint32_t ie = (REG_READ(GPIO_PIN_MUX_REG[pin]) & FUN_IE) ? 1 : 0;
  bool ok = expectOpenDrainIo ? (oe && od && ie) : (!oe && ie);
  Serial.printf("  %-4s GPIO%-2d output-en=%lu open-drain=%lu input-en=%lu  [%s] (expect %s)\n", name, pin,
                (unsigned long)oe, (unsigned long)od, (unsigned long)ie, ok ? " OK " : "FAIL",
                expectOpenDrainIo ? "open-drain I/O" : "input only");
}

static void haltWithError(const char *msg) {
  Serial.printf("[FAIL] %s\n", msg);
  while (true) delay(1000);
}

static void scanBus() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    IMU_WIRE.beginTransmission(addr);
    if (IMU_WIRE.endTransmission() == 0) {
      Serial.printf("  0x%02X\n", addr);
      found++;
    }
  }
  if (!found) Serial.println("  (no devices)");
}

static bool whoAmIOk(Imu &imu) {
  uint8_t id = 0;
  if (lsm6dsv320x_device_id_get(&imu.ctx, &id) != 0) {
    Serial.printf("  0x%02X: no response\n", imu.addr);
    return false;
  }
  Serial.printf("  0x%02X: WHO_AM_I = 0x%02X (expect 0x%02X)\n", imu.addr, id, LSM6DSV320X_ID);
  return id == LSM6DSV320X_ID;
}

// IF_CFG (03h) PP_OD / H_LACTIVE. The ST driver has no setter for these, and
// IF_CFG isn't cleared by SW_RESET, so write it explicitly and read it back
static bool setIntPinMode(Imu &imu, bool openDrain, bool activeLow) {
  lsm6dsv320x_if_cfg_t ifCfg;
  if (lsm6dsv320x_read_reg(&imu.ctx, LSM6DSV320X_IF_CFG, (uint8_t *)&ifCfg, 1) != 0) return false;

  ifCfg.pp_od = openDrain ? 1 : 0;
  ifCfg.h_lactive = activeLow ? 1 : 0;
  // SDA is open-drain in I2C mode (it only goes push-pull under I3C, which needs
  // an I3C controller to assign a dynamic address). Keep its internal pull-up off
  // too; the board has its own
  ifCfg.sda_pu_en = 0;
  if (lsm6dsv320x_write_reg(&imu.ctx, LSM6DSV320X_IF_CFG, (uint8_t *)&ifCfg, 1) != 0) return false;

  lsm6dsv320x_if_cfg_t readback;
  if (lsm6dsv320x_read_reg(&imu.ctx, LSM6DSV320X_IF_CFG, (uint8_t *)&readback, 1) != 0) return false;
  return readback.pp_od == ifCfg.pp_od && readback.h_lactive == ifCfg.h_lactive && readback.sda_pu_en == 0;
}

// Registers that decide whether data is produced and routed to INT1
static void dumpRegisters(Imu &imu) {
  static const struct {
    uint8_t reg;
    const char *name;
  } REGS[] = {
      {LSM6DSV320X_IF_CFG, "IF_CFG"},   {LSM6DSV320X_INT1_CTRL, "INT1_CTRL"},    {LSM6DSV320X_CTRL1, "CTRL1"},
      {LSM6DSV320X_CTRL2, "CTRL2"},     {LSM6DSV320X_CTRL1_XL_HG, "CTRL1_XL_HG"}, {LSM6DSV320X_STATUS_REG, "STATUS"},
  };
  Serial.printf("  0x%02X:", imu.addr);
  for (const auto &r : REGS) {
    uint8_t val = 0;
    if (lsm6dsv320x_read_reg(&imu.ctx, r.reg, &val, 1) == 0) Serial.printf("  %s=0x%02X", r.name, val);
    else Serial.printf("  %s=??", r.name);
  }
  Serial.println();
}

static inline int16_t le16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// One burst: STATUS, temp, gyro, accel, (OIS regs), high-g. Updates whatever
// STATUS says is new
static void readImu(Imu &imu, bool fromInt) {
  uint8_t burst[BURST_LEN];
  uint32_t t0 = esp_cpu_get_cycle_count();
  int32_t ret = lsm6dsv320x_read_reg(&imu.ctx, BURST_START, burst, BURST_LEN);
  imu.busCycles += esp_cpu_get_cycle_count() - t0;
  imu.busReads++;

  if (ret != 0) {
    imu.errors++;
    return;
  }
  if (fromInt) imu.intReads++;
  else imu.polledReads++;

  lsm6dsv320x_status_reg_t status;
  memcpy(&status, &burst[BURST_OFS(LSM6DSV320X_STATUS_REG)], 1);

  if (status.xlda) {
    const uint8_t *p = &burst[BURST_OFS(LSM6DSV320X_OUTX_L_A)];
    for (int i = 0; i < 3; i++) imu.xl[i] = le16(p + 2 * i);
    imu.xlCount++;
  }
  if (status.gda) {
    const uint8_t *p = &burst[BURST_OFS(LSM6DSV320X_OUTX_L_G)];
    for (int i = 0; i < 3; i++) imu.gy[i] = le16(p + 2 * i);
    imu.gyCount++;
  }
  if (status.xlhgda) {
    const uint8_t *p = &burst[BURST_OFS(LSM6DSV320X_UI_OUTX_L_A_OIS_HG)];
    for (int i = 0; i < 3; i++) imu.hg[i] = le16(p + 2 * i);
    imu.hgCount++;
  }
  if (status.tda) imu.temp = le16(&burst[BURST_OFS(LSM6DSV320X_OUT_TEMP_L)]);
}

/* ---------------------------------------------------------------------------
 * Setup / loop
 * ------------------------------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) delay(10);

  addImu(LSM_ADDR_A);
  addImu(LSM_ADDR_B);

  Serial.println("\n=== Dual LSM6DSV320X I2C test ===");
  Serial.printf("I2C%d  SDA=GPIO%d  SCL=GPIO%d  %lu Hz  IMUs:", LSM_I2C_PORT, LSM_SDA_PIN, LSM_SCL_PIN,
                (unsigned long)LSM_I2C_FREQ_HZ);
  for (int i = 0; i < imuCount; i++) Serial.printf(" 0x%02X", imus[i].addr);
  Serial.printf("\nINT1 -> GPIO%d (shared)  %s  active-%s  ODR %.0f Hz\n", LSM_INT_PIN,
                LSM_INT_OPEN_DRAIN ? "open-drain" : "push-pull", LSM_INT_ACTIVE_LOW ? "low" : "high", LSM_ODR_HZ);

  pinMode(LSM_INT_PIN, INPUT);

  // The IDF I2C driver underneath Wire sets SDA/SCL open-drain. Wire also turns on
  // the ESP's internal pull-ups; turn those off, the board has its own
  if (!IMU_WIRE.begin(LSM_SDA_PIN, LSM_SCL_PIN, LSM_I2C_FREQ_HZ)) haltWithError("Wire.begin failed");
  gpio_pullup_dis((gpio_num_t)LSM_SDA_PIN);
  gpio_pullup_dis((gpio_num_t)LSM_SCL_PIN);
  Serial.printf("Actual SCL: %lu Hz\n", (unsigned long)IMU_WIRE.getClock());

  Serial.println("ESP32 pin modes:");
  printPinMode("SDA", LSM_SDA_PIN, true);
  printPinMode("SCL", LSM_SCL_PIN, true);
  printPinMode("INT", LSM_INT_PIN, false);

  // Boot time after power-up
  delay(20);
  scanBus();

  Serial.println("WHO_AM_I:");
  int present = 0;
  for (int i = 0; i < imuCount; i++) {
    imus[i].present = whoAmIOk(imus[i]);
    if (imus[i].present) present++;
  }
  if (!present) haltWithError("no LSM6DSV320X detected");

  // Both power up with INT1 push-pull, driving low. Switch every present IMU to
  // the shared-net config before any of them gets data-ready routed
  Serial.println("Reset + INT1 config:");
  for (int i = 0; i < imuCount; i++) {
    Imu &imu = imus[i];
    if (!imu.present) continue;
    if (lsm6dsv320x_sw_reset(&imu.ctx) != 0) haltWithError("SW reset failed");
    if (!setIntPinMode(imu, LSM_INT_OPEN_DRAIN, LSM_INT_ACTIVE_LOW)) haltWithError("IF_CFG write/readback mismatch");
    Serial.printf("  [ OK ] 0x%02X IF_CFG PP_OD/H_LACTIVE verified\n", imu.addr);
  }

  // Nothing routed yet, so the line should sit at its inactive level
  delay(1);
  int idle = digitalRead(LSM_INT_PIN);
  Serial.printf("[%s] INT idle level = %d (expect %d)\n", idle != INT_ACTIVE_LEVEL ? " OK " : "WARN", idle,
                !INT_ACTIVE_LEVEL);

  Serial.println("Sensor config:");
  for (int i = 0; i < imuCount; i++) {
    Imu &imu = imus[i];
    if (!imu.present) continue;
    CHECK(imu, lsm6dsv320x_block_data_update_set(&imu.ctx, 1));
    CHECK(imu, lsm6dsv320x_xl_full_scale_set(&imu.ctx, LSM6DSV320X_16g));
    CHECK(imu, lsm6dsv320x_gy_full_scale_set(&imu.ctx, LSM6DSV320X_2000dps));
    CHECK(imu, lsm6dsv320x_hg_xl_full_scale_set(&imu.ctx, LSM6DSV320X_320g));

    // Latched: INT stays asserted until the accel output is read, so a missed
    // edge is still caught by the level check in loop()
    CHECK(imu, lsm6dsv320x_data_ready_mode_set(&imu.ctx, LSM6DSV320X_DRDY_LATCHED));

    lsm6dsv320x_pin_int_route_t route = {0};
    route.drdy_xl = 1;
    CHECK(imu, lsm6dsv320x_pin_int1_route_set(&imu.ctx, &route));
  }

  attachInterrupt(digitalPinToInterrupt(LSM_INT_PIN), onImuInt, LSM_INT_ACTIVE_LOW ? FALLING : RISING);

  for (int i = 0; i < imuCount; i++) {
    Imu &imu = imus[i];
    if (!imu.present) continue;
    CHECK(imu, lsm6dsv320x_xl_setup(&imu.ctx, LSM_XL_ODR, LSM6DSV320X_XL_HIGH_PERFORMANCE_MD));
    CHECK(imu, lsm6dsv320x_gy_setup(&imu.ctx, LSM_GY_ODR, LSM6DSV320X_GY_HIGH_PERFORMANCE_MD));
    CHECK(imu, lsm6dsv320x_hg_xl_data_rate_set(&imu.ctx, LSM_HG_ODR, 1));
  }

  delay(50);
  Serial.println("Register readback:");
  for (int i = 0; i < imuCount; i++) {
    if (imus[i].present) dumpRegisters(imus[i]);
  }

  Serial.println("Streaming per IMU: accel [mg] | gyro [dps] | high-g accel [mg] | temp [C]");
}

void loop() {
  static unsigned long lastRead = 0, lastPrint = 0, lastRate = 0;
  static uint32_t lastIsr = 0;

  // Read-gap tracking: a gap longer than one sample period can drop a sample
  static const uint32_t SAMPLE_PERIOD_US = (uint32_t)(1e6f / LSM_ODR_HZ);
  static uint32_t lastReadUs = 0, maxGapUs = 0, longGaps = 0, maxPrintUs = 0;

  // Shared wired-OR INT: on an edge, or while the line is still asserted (one
  // IMU can hold it while the other gets new data), read both. Fall back to a
  // poll if INT stays quiet, so data still flows if INT isn't wired
  bool intActive = drdyPending || digitalRead(LSM_INT_PIN) == INT_ACTIVE_LEVEL;
  bool pollDue = millis() - lastRead >= LSM_POLL_FALLBACK_MS;
  if (intActive || pollDue) {
    drdyPending = false;
    lastRead = millis();

    uint32_t nowUs = micros();
    if (lastReadUs != 0) {
      uint32_t gap = nowUs - lastReadUs;
      if (gap > maxGapUs) maxGapUs = gap;
      if (gap > SAMPLE_PERIOD_US) longGaps++;
    }
    lastReadUs = nowUs;

    for (int i = 0; i < imuCount; i++) {
      if (imus[i].present) readImu(imus[i], intActive);
    }
  }

  unsigned long now = millis();

  if (now - lastPrint >= LSM_PRINT_PERIOD_MS) {
    lastPrint = now;
    uint32_t printStartUs = micros();
    for (int i = 0; i < imuCount; i++) {
      Imu &m = imus[i];
      if (!m.present) continue;
      Serial.printf("0x%02X %8.1f %8.1f %8.1f | %8.2f %8.2f %8.2f | %9.1f %9.1f %9.1f | %5.1f\n", m.addr,
                    lsm6dsv320x_from_fs16_to_mg(m.xl[0]), lsm6dsv320x_from_fs16_to_mg(m.xl[1]),
                    lsm6dsv320x_from_fs16_to_mg(m.xl[2]), lsm6dsv320x_from_fs2000_to_mdps(m.gy[0]) / 1000.0f,
                    lsm6dsv320x_from_fs2000_to_mdps(m.gy[1]) / 1000.0f,
                    lsm6dsv320x_from_fs2000_to_mdps(m.gy[2]) / 1000.0f, lsm6dsv320x_from_fs320_to_mg(m.hg[0]),
                    lsm6dsv320x_from_fs320_to_mg(m.hg[1]), lsm6dsv320x_from_fs320_to_mg(m.hg[2]),
                    lsm6dsv320x_from_lsb_to_celsius(m.temp));
    }
    uint32_t printUs = micros() - printStartUs;
    if (printUs > maxPrintUs) maxPrintUs = printUs;
  }

  if (now - lastRate >= 1000) {
    float dt = (now - lastRate) / 1000.0f;
    uint32_t isrs = isrCount;
    Serial.printf("--- INT edges %.0f/s | max gap between reads %lu us (sample period %lu us), gaps > period %lu, "
                  "slowest data print %lu us\n",
                  (isrs - lastIsr) / dt, (unsigned long)maxGapUs, (unsigned long)SAMPLE_PERIOD_US,
                  (unsigned long)longGaps, (unsigned long)maxPrintUs);
    maxGapUs = 0;
    longGaps = 0;
    maxPrintUs = 0;

    for (int i = 0; i < imuCount; i++) {
      Imu &m = imus[i];
      if (!m.present) continue;
      float xlHz = (m.xlCount - m.lastXl) / dt;
      float gyHz = (m.gyCount - m.lastGy) / dt;
      float hgHz = (m.hgCount - m.lastHg) / dt;
      float intHz = (m.intReads - m.lastIntReads) / dt;
      float pollHz = (m.polledReads - m.lastPolledReads) / dt;
      uint32_t reads = m.busReads - m.lastBusReads;
      float busUs = reads ? (m.busCycles - m.lastBusCycles) / (float)reads / getCpuFrequencyMhz() : 0.0f;

      // Within 5% of the ODR on all three means no samples are being dropped
      bool ok = fabsf(xlHz - LSM_ODR_HZ) < 0.05f * LSM_ODR_HZ && fabsf(gyHz - LSM_ODR_HZ) < 0.05f * LSM_ODR_HZ &&
                fabsf(hgHz - LSM_ODR_HZ) < 0.05f * LSM_ODR_HZ;
      Serial.printf("[%s] 0x%02X new/s: accel %.0f gyro %.0f high-g %.0f (ODR %.0f) | reads/s: INT %.0f, polled "
                    "%.0f | %.0f us/read | I2C errors %lu\n",
                    ok ? "RATE" : "WARN", m.addr, xlHz, gyHz, hgHz, LSM_ODR_HZ, intHz, pollHz, busUs,
                    (unsigned long)m.errors);

      m.lastXl = m.xlCount;
      m.lastGy = m.gyCount;
      m.lastHg = m.hgCount;
      m.lastIntReads = m.intReads;
      m.lastPolledReads = m.polledReads;
      m.lastBusReads = m.busReads;
      m.lastBusCycles = m.busCycles;
    }

    lastRate = now;
    lastIsr = isrs;
  }
}
