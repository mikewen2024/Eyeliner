/**
 * @file test_lsm6dsv_spi.cpp
 * @brief LSM6DSV320X over hardware SPI (ESP32-S3 SPI2/FSPI), as fast as possible.
 *
 * Build/flash: pio run -e test_lsm6dsv_spi -t upload
 * Monitor:     pio device monitor -e test_lsm6dsv_spi
 *
 * Self-contained: every pin and setting is defined in this file (no
 * board_pins.h), and each can be overridden from build_flags.
 *
 * What it does:
 *   1. Hardware SPI via the ESP-IDF spi_master driver at the IMU's max clock
 *      (10 MHz), no DMA, polling transactions, bus held for the whole run
 *   2. WHO_AM_I == 0x73
 *   3. IF_CFG: disables the I2C/I3C block (recommended in SPI mode), sets
 *      INT1 open-drain/push-pull and polarity, reads it back
 *   4. Accel, gyro and high-g accel at their max ODR (7.68 kHz)
 *   5. Reads STATUS + temp + gyro + accel + high-g in ONE 28-byte burst
 *      (0x1E..0x39), benchmarks that transaction, then streams, reporting the
 *      achieved sample rate per sensor against the ODR
 *
 * Hardware notes (DS14623 rev 2):
 *   - CS low selects SPI. CS must NOT be tied to VDDIO (that selects I2C), and
 *     SDO/SA0 becomes the SPI data output, so it can't be strapped to GND/VDDIO
 *   - SPI modes 0 and 3, fc(SPC) max 10 MHz, tv(SO) max 25 ns (table 6)
 */
#include <Arduino.h>

#include <atomic>

#include "driver/spi_master.h"
#include "esp_cpu.h"

#include "lsm6dsv320x_reg.h"

/* ---------------------------------------------------------------------------
 * Configuration (independent of board_pins.h)
 * ------------------------------------------------------------------------- */

// Defaults are the XIAO ESP32-S3's labelled SPI pins (D8/D9/D10, SS on D7).
// Any GPIO works through the GPIO matrix at 10 MHz
#ifndef LSM_SPI_SCK_PIN
#define LSM_SPI_SCK_PIN 7   // D8  -> IMU SCL/SPC (pin 13)
#endif

#ifndef LSM_SPI_MOSI_PIN
#define LSM_SPI_MOSI_PIN 9  // D10 -> IMU SDA/SDI (pin 14)
#endif

#ifndef LSM_SPI_MISO_PIN
#define LSM_SPI_MISO_PIN 8  // D9  <- IMU SDO/SA0 (pin 1)
#endif

#ifndef LSM_SPI_CS_PIN
#define LSM_SPI_CS_PIN 44   // D7  -> IMU CS (pin 12)
#endif

#ifndef LSM_SPI_INT_PIN
#define LSM_SPI_INT_PIN 3   // D2  <- IMU INT1 (pin 4)
#endif

// SPI clock. LSM6DSV320X max is 10 MHz
#ifndef LSM_SPI_CLOCK_HZ
#define LSM_SPI_CLOCK_HZ 10000000
#endif

// SPI mode: the IMU supports 0 and 3
#ifndef LSM_SPI_MODE
#define LSM_SPI_MODE 3
#endif

// IMU SDO valid output time, tv(SO) max. Lets the driver compensate the read
// sample point for the IMU's output delay plus the GPIO matrix
#ifndef LSM_SPI_INPUT_DELAY_NS
#define LSM_SPI_INPUT_DELAY_NS 25
#endif

// 0: poll the STATUS byte (no INT needed, lowest latency)
// 1: read on the INT1 data-ready interrupt
#ifndef LSM_SPI_USE_INT
#define LSM_SPI_USE_INT 0
#endif

// INT1 electrical config: open-drain needs a pull-up on the INT line
#ifndef LSM_SPI_INT_OPEN_DRAIN
#define LSM_SPI_INT_OPEN_DRAIN 1
#endif

#ifndef LSM_SPI_INT_ACTIVE_LOW
#define LSM_SPI_INT_ACTIVE_LOW 1
#endif

// Output data rates (fastest available)
#ifndef LSM_SPI_XL_ODR
#define LSM_SPI_XL_ODR LSM6DSV320X_ODR_AT_7680Hz
#endif

#ifndef LSM_SPI_GY_ODR
#define LSM_SPI_GY_ODR LSM6DSV320X_ODR_AT_7680Hz
#endif

#ifndef LSM_SPI_HG_ODR
#define LSM_SPI_HG_ODR LSM6DSV320X_HG_XL_ODR_AT_7680Hz
#endif

#ifndef LSM_SPI_ODR_HZ
#define LSM_SPI_ODR_HZ 7680.0f
#endif

#ifndef LSM_SPI_PRINT_PERIOD_MS
#define LSM_SPI_PRINT_PERIOD_MS 100
#endif

#ifndef LSM_SPI_BENCH_READS
#define LSM_SPI_BENCH_READS 2000
#endif

static_assert(LSM_SPI_CLOCK_HZ > 0 && LSM_SPI_CLOCK_HZ <= 10000000, "LSM6DSV320X SPI max is 10 MHz");
static_assert(LSM_SPI_MODE == 0 || LSM_SPI_MODE == 3, "LSM6DSV320X supports SPI modes 0 and 3");

#if LSM_SPI_INT_OPEN_DRAIN && !LSM_SPI_INT_ACTIVE_LOW
#warning "Open-drain + active-high: the line idles LOW and is released when asserted. Usually you want active-low."
#endif

#define SPI_HOST_ID     SPI2_HOST
#define SPI_READ_BIT    0x80
#define SPI_MAX_XFER    64

// One burst from STATUS_REG through the high-g outputs
#define BURST_START     LSM6DSV320X_STATUS_REG       // 0x1E
#define BURST_END       LSM6DSV320X_UI_OUTZ_H_A_OIS_HG // 0x39
#define BURST_LEN       (BURST_END - BURST_START + 1)  // 28 bytes
#define BURST_OFS(reg)  ((reg) - BURST_START)

static_assert(BURST_LEN + 1 <= SPI_MAX_XFER, "burst must fit in one non-DMA transaction");

static const int INT_ACTIVE_LEVEL = LSM_SPI_INT_ACTIVE_LOW ? LOW : HIGH;

/* ---------------------------------------------------------------------------
 * SPI platform glue for the ST driver
 * ------------------------------------------------------------------------- */

static spi_device_handle_t spiDev;
static uint8_t spiTx[SPI_MAX_XFER];
static uint8_t spiRx[SPI_MAX_XFER];

static int32_t spiTransfer(size_t len) {
  spi_transaction_t t = {};
  t.length = len * 8;
  t.tx_buffer = spiTx;
  t.rx_buffer = spiRx;
  return spi_device_polling_transmit(spiDev, &t) == ESP_OK ? 0 : -1;
}

static int32_t platformWrite(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len) {
  (void)handle;
  if (len + 1 > SPI_MAX_XFER) return -1;
  spiTx[0] = reg & ~SPI_READ_BIT;
  memcpy(&spiTx[1], buf, len);
  return spiTransfer(len + 1);
}

static int32_t platformRead(void *handle, uint8_t reg, uint8_t *buf, uint16_t len) {
  (void)handle;
  if (len + 1 > SPI_MAX_XFER) return -1;
  spiTx[0] = reg | SPI_READ_BIT;
  memset(&spiTx[1], 0, len);
  if (spiTransfer(len + 1) != 0) return -1;
  memcpy(buf, &spiRx[1], len);
  return 0;
}

static void platformDelay(uint32_t ms) {
  delay(ms);
}

static stmdev_ctx_t imu = {platformWrite, platformRead, platformDelay, nullptr, nullptr};

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

#define CHECK(call)                                                   \
  do {                                                                \
    int32_t ret_ = (call);                                            \
    Serial.printf("  [%s] %s\n", ret_ == 0 ? " OK " : "FAIL", #call); \
  } while (0)

static void haltWithError(const char *msg) {
  Serial.printf("[FAIL] %s\n", msg);
  while (true) delay(1000);
}

static inline int16_t le16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// IF_CFG (03h): disable I2C/I3C (SPI only), INT1 drive mode and polarity.
// IF_CFG survives SW_RESET, so this is written explicitly and read back
static bool setInterfaceConfig(bool openDrain, bool activeLow) {
  lsm6dsv320x_if_cfg_t ifCfg;
  if (lsm6dsv320x_read_reg(&imu, LSM6DSV320X_IF_CFG, (uint8_t *)&ifCfg, 1) != 0) return false;

  ifCfg.i2c_i3c_disable = 1;
  ifCfg.pp_od = openDrain ? 1 : 0;
  ifCfg.h_lactive = activeLow ? 1 : 0;
  ifCfg.sda_pu_en = 0;
  if (lsm6dsv320x_write_reg(&imu, LSM6DSV320X_IF_CFG, (uint8_t *)&ifCfg, 1) != 0) return false;

  lsm6dsv320x_if_cfg_t readback;
  if (lsm6dsv320x_read_reg(&imu, LSM6DSV320X_IF_CFG, (uint8_t *)&readback, 1) != 0) return false;
  Serial.printf("  IF_CFG = 0x%02X (I2C/I3C off=%d, PP_OD=%d, H_LACTIVE=%d)\n", *(uint8_t *)&readback,
                readback.i2c_i3c_disable, readback.pp_od, readback.h_lactive);
  return readback.i2c_i3c_disable == 1 && readback.pp_od == ifCfg.pp_od && readback.h_lactive == ifCfg.h_lactive;
}

static uint8_t burst[BURST_LEN];

static inline bool readBurst() {
  return lsm6dsv320x_read_reg(&imu, BURST_START, burst, BURST_LEN) == 0;
}

// Times back-to-back burst reads to show the per-sample bus cost
static void benchmarkBurst() {
  uint32_t minC = UINT32_MAX, maxC = 0;
  uint64_t total = 0;
  int errors = 0;
  for (int i = 0; i < LSM_SPI_BENCH_READS; i++) {
    uint32_t t0 = esp_cpu_get_cycle_count();
    if (!readBurst()) errors++;
    uint32_t c = esp_cpu_get_cycle_count() - t0;
    total += c;
    if (c < minC) minC = c;
    if (c > maxC) maxC = c;
  }
  float mhz = getCpuFrequencyMhz();
  float avgUs = total / (float)LSM_SPI_BENCH_READS / mhz;
  float wireUs = (BURST_LEN + 1) * 8 * 1e6f / LSM_SPI_CLOCK_HZ;
  Serial.printf("Burst read benchmark (%d x %d bytes):\n", LSM_SPI_BENCH_READS, BURST_LEN + 1);
  Serial.printf("  min %.2f us, avg %.2f us, max %.2f us (on-wire %.2f us at %lu Hz), errors %d\n", minC / mhz,
                avgUs, maxC / mhz, wireUs, (unsigned long)LSM_SPI_CLOCK_HZ, errors);
  Serial.printf("  -> up to ~%.0f full reads/s, %.1fx the %.0f Hz ODR\n", 1e6f / avgUs,
                (1e6f / avgUs) / LSM_SPI_ODR_HZ, LSM_SPI_ODR_HZ);
}

/* ---------------------------------------------------------------------------
 * Setup / loop
 * ------------------------------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) delay(10);

  Serial.println("\n=== LSM6DSV320X SPI test ===");
  Serial.printf("SPI2  SCK=GPIO%d  MOSI=GPIO%d  MISO=GPIO%d  CS=GPIO%d  mode %d  %lu Hz\n", LSM_SPI_SCK_PIN,
                LSM_SPI_MOSI_PIN, LSM_SPI_MISO_PIN, LSM_SPI_CS_PIN, LSM_SPI_MODE, (unsigned long)LSM_SPI_CLOCK_HZ);
  Serial.printf("INT1 -> GPIO%d  %s  active-%s  (%s)\n", LSM_SPI_INT_PIN,
                LSM_SPI_INT_OPEN_DRAIN ? "open-drain" : "push-pull", LSM_SPI_INT_ACTIVE_LOW ? "low" : "high",
                LSM_SPI_USE_INT ? "reads on INT" : "polling STATUS");

  pinMode(LSM_SPI_INT_PIN, INPUT);

  const spi_bus_config_t busCfg = {
      .mosi_io_num = LSM_SPI_MOSI_PIN,
      .miso_io_num = LSM_SPI_MISO_PIN,
      .sclk_io_num = LSM_SPI_SCK_PIN,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = SPI_MAX_XFER,
  };
  if (spi_bus_initialize(SPI_HOST_ID, &busCfg, SPI_DMA_DISABLED) != ESP_OK) haltWithError("spi_bus_initialize");

  spi_device_interface_config_t devCfg = {};
  devCfg.mode = LSM_SPI_MODE;
  devCfg.clock_speed_hz = LSM_SPI_CLOCK_HZ;
  devCfg.input_delay_ns = LSM_SPI_INPUT_DELAY_NS;
  devCfg.spics_io_num = LSM_SPI_CS_PIN;
  devCfg.cs_ena_posttrans = 1; // th(CS) 20 ns > half a clock at 10 MHz
  devCfg.queue_size = 1;
  if (spi_bus_add_device(SPI_HOST_ID, &devCfg, &spiDev) != ESP_OK) haltWithError("spi_bus_add_device");

  int actualKhz = 0;
  spi_device_get_actual_freq(spiDev, &actualKhz);
  Serial.printf("Actual SCK: %d kHz\n", actualKhz);

  // Hold the bus for the whole test: skips per-transaction arbitration
  if (spi_device_acquire_bus(spiDev, portMAX_DELAY) != ESP_OK) haltWithError("spi_device_acquire_bus");

  // Boot time after power-up
  delay(20);

  uint8_t id = 0;
  if (lsm6dsv320x_device_id_get(&imu, &id) != 0) haltWithError("WHO_AM_I read failed");
  Serial.printf("WHO_AM_I = 0x%02X (expect 0x%02X)\n", id, LSM6DSV320X_ID);
  if (id != LSM6DSV320X_ID) {
    haltWithError(id == 0x00 || id == 0xFF ? "no IMU answer (0x00/0xFF: check MISO wiring, CS, and SDO/SA0 strap)"
                                           : "unexpected WHO_AM_I");
  }

  if (lsm6dsv320x_sw_reset(&imu) != 0) haltWithError("SW reset failed");

  Serial.println("Interface config:");
  if (!setInterfaceConfig(LSM_SPI_INT_OPEN_DRAIN, LSM_SPI_INT_ACTIVE_LOW)) haltWithError("IF_CFG readback mismatch");

  Serial.println("Sensor config:");
  CHECK(lsm6dsv320x_block_data_update_set(&imu, 1));
  CHECK(lsm6dsv320x_xl_full_scale_set(&imu, LSM6DSV320X_16g));
  CHECK(lsm6dsv320x_gy_full_scale_set(&imu, LSM6DSV320X_2000dps));
  CHECK(lsm6dsv320x_hg_xl_full_scale_set(&imu, LSM6DSV320X_320g));
  CHECK(lsm6dsv320x_data_ready_mode_set(&imu, LSM6DSV320X_DRDY_LATCHED));

#if LSM_SPI_USE_INT
  lsm6dsv320x_pin_int_route_t route = {0};
  route.drdy_xl = 1;
  CHECK(lsm6dsv320x_pin_int1_route_set(&imu, &route));
  attachInterrupt(digitalPinToInterrupt(LSM_SPI_INT_PIN), onImuInt, LSM_SPI_INT_ACTIVE_LOW ? FALLING : RISING);
#endif

  CHECK(lsm6dsv320x_xl_setup(&imu, LSM_SPI_XL_ODR, LSM6DSV320X_XL_HIGH_PERFORMANCE_MD));
  CHECK(lsm6dsv320x_gy_setup(&imu, LSM_SPI_GY_ODR, LSM6DSV320X_GY_HIGH_PERFORMANCE_MD));
  CHECK(lsm6dsv320x_hg_xl_data_rate_set(&imu, LSM_SPI_HG_ODR, 1));

  delay(50);
  benchmarkBurst();

  Serial.println("Streaming: accel [mg] | gyro [dps] | high-g accel [mg] | temp [C]");
}

void loop() {
  static int16_t xl[3], gy[3], hg[3], temp;
  static uint32_t xlCount = 0, gyCount = 0, hgCount = 0, reads = 0, errors = 0;
  static uint32_t lastXl = 0, lastGy = 0, lastHg = 0, lastReads = 0, lastIsr = 0;
  static unsigned long lastPrint = 0, lastRate = 0;

#if LSM_SPI_USE_INT
  bool due = drdyPending || digitalRead(LSM_SPI_INT_PIN) == INT_ACTIVE_LEVEL;
#else
  bool due = true;
#endif

  if (due) {
    drdyPending = false;
    if (!readBurst()) {
      errors++;
    } else {
      reads++;
      lsm6dsv320x_status_reg_t status;
      memcpy(&status, &burst[BURST_OFS(LSM6DSV320X_STATUS_REG)], 1);

      if (status.xlda) {
        const uint8_t *p = &burst[BURST_OFS(LSM6DSV320X_OUTX_L_A)];
        for (int i = 0; i < 3; i++) xl[i] = le16(p + 2 * i);
        xlCount++;
      }
      if (status.gda) {
        const uint8_t *p = &burst[BURST_OFS(LSM6DSV320X_OUTX_L_G)];
        for (int i = 0; i < 3; i++) gy[i] = le16(p + 2 * i);
        gyCount++;
      }
      if (status.xlhgda) {
        const uint8_t *p = &burst[BURST_OFS(LSM6DSV320X_UI_OUTX_L_A_OIS_HG)];
        for (int i = 0; i < 3; i++) hg[i] = le16(p + 2 * i);
        hgCount++;
      }
      if (status.tda) temp = le16(&burst[BURST_OFS(LSM6DSV320X_OUT_TEMP_L)]);
    }
  }

  unsigned long now = millis();

  if (now - lastPrint >= LSM_SPI_PRINT_PERIOD_MS) {
    lastPrint = now;
    Serial.printf("%8.1f %8.1f %8.1f | %8.2f %8.2f %8.2f | %9.1f %9.1f %9.1f | %5.1f\n",
                  lsm6dsv320x_from_fs16_to_mg(xl[0]), lsm6dsv320x_from_fs16_to_mg(xl[1]),
                  lsm6dsv320x_from_fs16_to_mg(xl[2]), lsm6dsv320x_from_fs2000_to_mdps(gy[0]) / 1000.0f,
                  lsm6dsv320x_from_fs2000_to_mdps(gy[1]) / 1000.0f, lsm6dsv320x_from_fs2000_to_mdps(gy[2]) / 1000.0f,
                  lsm6dsv320x_from_fs320_to_mg(hg[0]), lsm6dsv320x_from_fs320_to_mg(hg[1]),
                  lsm6dsv320x_from_fs320_to_mg(hg[2]), lsm6dsv320x_from_lsb_to_celsius(temp));
  }

  if (now - lastRate >= 1000) {
    float dt = (now - lastRate) / 1000.0f;
    float xlHz = (xlCount - lastXl) / dt;
    float gyHz = (gyCount - lastGy) / dt;
    float hgHz = (hgCount - lastHg) / dt;
    float readHz = (reads - lastReads) / dt;
    uint32_t isrs = isrCount;
    float isrHz = (isrs - lastIsr) / dt;

    // Within 5% of the ODR on all three means no samples are being dropped
    bool ok = fabsf(xlHz - LSM_SPI_ODR_HZ) < 0.05f * LSM_SPI_ODR_HZ &&
              fabsf(gyHz - LSM_SPI_ODR_HZ) < 0.05f * LSM_SPI_ODR_HZ &&
              fabsf(hgHz - LSM_SPI_ODR_HZ) < 0.05f * LSM_SPI_ODR_HZ;
    Serial.printf("[%s] new samples/s: accel %.0f, gyro %.0f, high-g %.0f (ODR %.0f) | burst reads/s %.0f",
                  ok ? "RATE" : "WARN", xlHz, gyHz, hgHz, LSM_SPI_ODR_HZ, readHz);
#if LSM_SPI_USE_INT
    Serial.printf(" | INT %.0f Hz", isrHz);
#else
    (void)isrHz;
#endif
    Serial.printf(" | SPI errors %lu\n", (unsigned long)errors);

    lastRate = now;
    lastXl = xlCount;
    lastGy = gyCount;
    lastHg = hgCount;
    lastReads = reads;
    lastIsr = isrs;
  }
}
