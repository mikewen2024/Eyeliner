/**
 * @file test_lsm6dsv.cpp
 * @brief LSM6DSV320X bring-up test over hardware I2C (ESP32-S3 I2C peripheral).
 *
 * Build/flash: pio run -e test_lsm6dsv -t upload && pio device monitor -e test_lsm6dsv
 *
 * What it checks:
 *   1. I2C bus scan (through the TMUX154E, selected to the IMU side)
 *   2. WHO_AM_I == 0x73 at LSM_I2C_ADDR
 *   3. INT1 electrical config (push-pull / open-drain, polarity) with readback
 *   4. INT line idles at the inactive level once configured
 *   5. Accel data-ready interrupt rate matches the configured ODR
 *   6. Streams low-g accel, gyro, and high-g accel
 *
 * Every LSM_* macro below can be overridden from build_flags in platformio.ini,
 * e.g.  -D LSM_I2C_ADDR=0x6B  -D LSM_INT_OPEN_DRAIN=0
 *
 * Datasheet refs (DS14623 rev 2): I2C timing table 8 (400 kHz FM / 1 MHz FM+),
 * address section 5.1.2 (110101[SA0]b), IF_CFG (03h) section 9.3.
 */
#include <Arduino.h>
#include <Wire.h>

#include <atomic>

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

// SCL frequency in Hz. LSM6DSV320X supports up to 1 MHz (I2C fast mode plus)
#ifndef LSM_I2C_FREQ_HZ
#define LSM_I2C_FREQ_HZ 400000
#endif

// 7-bit address: 0x6A (SDO/SA0 = GND, U5) or 0x6B (SDO/SA0 = VDDIO, U4)
#ifndef LSM_I2C_ADDR
#define LSM_I2C_ADDR IMU_U5_I2C_ADDR
#endif

// The other IMU sharing the INT net. It is set to open-drain/active-low with
// nothing routed, so it releases the line. -1 to skip (single-IMU boards)
#ifndef LSM_PEER_I2C_ADDR
#define LSM_PEER_I2C_ADDR (LSM_I2C_ADDR ^ 0x01)
#endif

// I2C mux select pin and level for the IMU side. -1 if there's no mux
#ifndef LSM_I2C_MUX_SEL_PIN
#define LSM_I2C_MUX_SEL_PIN PIN_I2C_SEL
#endif

#ifndef LSM_I2C_MUX_SEL_LEVEL
#define LSM_I2C_MUX_SEL_LEVEL I2C_SEL_LEVEL_IMU
#endif

// ESP32 GPIO connected to LSM6DSV320X INT1
#ifndef LSM_INT_PIN
#define LSM_INT_PIN PIN_IMU_INT
#endif

// 1: open-drain (needs a pull-up; required on a shared INT net)
// 0: push-pull
#ifndef LSM_INT_OPEN_DRAIN
#define LSM_INT_OPEN_DRAIN 1
#endif

// 1: active-low, 0: active-high. Open-drain on a pulled-up net should be active-low
#ifndef LSM_INT_ACTIVE_LOW
#define LSM_INT_ACTIVE_LOW 1
#endif

// Enable the ESP32 internal pull-up on the INT pin (board already has R9 4.7k)
#ifndef LSM_INT_ESP_PULLUP
#define LSM_INT_ESP_PULLUP 0
#endif

#ifndef LSM_PRINT_PERIOD_MS
#define LSM_PRINT_PERIOD_MS 100
#endif

/* Sanity checks */
static_assert(LSM_I2C_FREQ_HZ > 0 && LSM_I2C_FREQ_HZ <= 1000000,
              "LSM6DSV320X I2C supports up to 1 MHz (fast mode plus)");
static_assert(LSM_I2C_ADDR == 0x6A || LSM_I2C_ADDR == 0x6B,
              "LSM6DSV320X 7-bit address must be 0x6A or 0x6B");
static_assert(LSM_I2C_PORT == 0 || LSM_I2C_PORT == 1, "ESP32-S3 has I2C ports 0 and 1");
static_assert(LSM_PEER_I2C_ADDR != LSM_I2C_ADDR, "Peer IMU address must differ from the test IMU");

#if LSM_INT_OPEN_DRAIN && !LSM_INT_ACTIVE_LOW
#warning "Open-drain + active-high: the pull-up makes the line idle HIGH only while the IMU is asserting. Usually you want active-low."
#endif

#if !LSM_INT_OPEN_DRAIN && LSM_PEER_I2C_ADDR < 0
#warning "Push-pull INT with no peer configured: on the V3 board the other IMU also drives INT push-pull by default."
#endif

#if LSM_I2C_PORT == 0
#define IMU_WIRE Wire
#else
#define IMU_WIRE Wire1
#endif

// Output data rates used for the interrupt-rate check
#define LSM_XL_ODR       LSM6DSV320X_ODR_AT_120Hz
#define LSM_XL_ODR_HZ    120.0f
#define LSM_GY_ODR       LSM6DSV320X_ODR_AT_120Hz
#define LSM_HG_ODR       LSM6DSV320X_HG_XL_ODR_AT_480Hz

static const int INT_ACTIVE_LEVEL = LSM_INT_ACTIVE_LOW ? LOW : HIGH;

/* ---------------------------------------------------------------------------
 * Platform glue for the ST driver
 * ------------------------------------------------------------------------- */

struct ImuBus {
  TwoWire *wire;
  uint8_t addr;
};

static int32_t platformWrite(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len) {
  ImuBus *bus = static_cast<ImuBus *>(handle);

  bus->wire->beginTransmission(bus->addr);
  bus->wire->write(reg);
  bus->wire->write(buf, len);
  return bus->wire->endTransmission() == 0 ? 0 : -1;
}

static int32_t platformRead(void *handle, uint8_t reg, uint8_t *buf, uint16_t len) {
  ImuBus *bus = static_cast<ImuBus *>(handle);

  // Register address write, then repeated start into the read
  bus->wire->beginTransmission(bus->addr);
  bus->wire->write(reg);
  if (bus->wire->endTransmission(false) != 0) return -1;

  if (bus->wire->requestFrom((uint16_t)bus->addr, (size_t)len, true) != len) return -1;
  for (uint16_t i = 0; i < len; i++) {
    buf[i] = bus->wire->read();
  }
  return 0;
}

static void platformDelay(uint32_t ms) {
  delay(ms);
}

static ImuBus imuBus = {&IMU_WIRE, LSM_I2C_ADDR};
static stmdev_ctx_t imu = {platformWrite, platformRead, platformDelay, &imuBus, nullptr};

#if LSM_PEER_I2C_ADDR >= 0
static ImuBus peerBus = {&IMU_WIRE, LSM_PEER_I2C_ADDR};
static stmdev_ctx_t peer = {platformWrite, platformRead, platformDelay, &peerBus, nullptr};
#endif

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

static bool whoAmIOk(const stmdev_ctx_t *ctx, uint8_t addr) {
  uint8_t id = 0;
  if (lsm6dsv320x_device_id_get(ctx, &id) != 0) {
    Serial.printf("  0x%02X: no response\n", addr);
    return false;
  }
  Serial.printf("  0x%02X: WHO_AM_I = 0x%02X (expect 0x%02X)\n", addr, id, LSM6DSV320X_ID);
  return id == LSM6DSV320X_ID;
}

// IF_CFG (03h) PP_OD / H_LACTIVE. The ST driver has no setter for these, and
// IF_CFG isn't cleared by SW_RESET, so write it explicitly and read it back
static bool setIntPinMode(const stmdev_ctx_t *ctx, bool openDrain, bool activeLow) {
  lsm6dsv320x_if_cfg_t ifCfg;
  if (lsm6dsv320x_read_reg(ctx, LSM6DSV320X_IF_CFG, (uint8_t *)&ifCfg, 1) != 0) return false;

  ifCfg.pp_od = openDrain ? 1 : 0;
  ifCfg.h_lactive = activeLow ? 1 : 0;
  if (lsm6dsv320x_write_reg(ctx, LSM6DSV320X_IF_CFG, (uint8_t *)&ifCfg, 1) != 0) return false;

  lsm6dsv320x_if_cfg_t readback;
  if (lsm6dsv320x_read_reg(ctx, LSM6DSV320X_IF_CFG, (uint8_t *)&readback, 1) != 0) return false;
  return readback.pp_od == ifCfg.pp_od && readback.h_lactive == ifCfg.h_lactive;
}

/* ---------------------------------------------------------------------------
 * Setup / loop
 * ------------------------------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) delay(10);

  Serial.println("\n=== LSM6DSV320X I2C test ===");
  Serial.printf("I2C%d  SDA=GPIO%d  SCL=GPIO%d  %lu Hz  addr=0x%02X\n",
                LSM_I2C_PORT, LSM_SDA_PIN, LSM_SCL_PIN, (unsigned long)LSM_I2C_FREQ_HZ, LSM_I2C_ADDR);
  Serial.printf("INT1 -> GPIO%d  %s  active-%s\n", LSM_INT_PIN,
                LSM_INT_OPEN_DRAIN ? "open-drain" : "push-pull", LSM_INT_ACTIVE_LOW ? "low" : "high");

  // Route the mux to the IMUs before touching the bus
#if LSM_I2C_MUX_SEL_PIN >= 0
  pinMode(LSM_I2C_MUX_SEL_PIN, OUTPUT);
  digitalWrite(LSM_I2C_MUX_SEL_PIN, LSM_I2C_MUX_SEL_LEVEL);
  Serial.printf("I2C mux SEL GPIO%d = %d\n", LSM_I2C_MUX_SEL_PIN, LSM_I2C_MUX_SEL_LEVEL);
#endif

  pinMode(LSM_INT_PIN, LSM_INT_ESP_PULLUP ? INPUT_PULLUP : INPUT);

  if (!IMU_WIRE.begin(LSM_SDA_PIN, LSM_SCL_PIN, LSM_I2C_FREQ_HZ)) haltWithError("Wire.begin failed");
  Serial.printf("Actual SCL: %lu Hz\n", (unsigned long)IMU_WIRE.getClock());

  // Boot time after power-up
  delay(20);
  scanBus();

  Serial.println("WHO_AM_I:");
  if (!whoAmIOk(&imu, LSM_I2C_ADDR)) haltWithError("test IMU not detected");

  // Release the shared INT net from the other IMU first, so it can't fight the one under test
#if LSM_PEER_I2C_ADDR >= 0
  if (whoAmIOk(&peer, LSM_PEER_I2C_ADDR)) {
    lsm6dsv320x_pin_int_route_t none = {0};
    lsm6dsv320x_pin_int1_route_set(&peer, &none);
    if (!setIntPinMode(&peer, true, true)) haltWithError("peer IF_CFG write/readback mismatch");
    Serial.println("  peer INT1 released (open-drain, active-low, nothing routed)");
  } else {
    Serial.println("  peer not found, skipping (INT line may be held by it)");
  }
#endif

  if (lsm6dsv320x_sw_reset(&imu) != 0) haltWithError("SW reset failed");

  if (!setIntPinMode(&imu, LSM_INT_OPEN_DRAIN, LSM_INT_ACTIVE_LOW)) {
    haltWithError("IF_CFG write/readback mismatch");
  }
  Serial.println("[ OK ] IF_CFG PP_OD/H_LACTIVE verified");

  // With nothing routed yet, the line should be at its inactive level
  delay(1);
  int idle = digitalRead(LSM_INT_PIN);
  Serial.printf("[%s] INT idle level = %d (expect %d)\n",
                idle != INT_ACTIVE_LEVEL ? " OK " : "WARN", idle, !INT_ACTIVE_LEVEL);

  lsm6dsv320x_block_data_update_set(&imu, 1);
  lsm6dsv320x_xl_full_scale_set(&imu, LSM6DSV320X_16g);
  lsm6dsv320x_gy_full_scale_set(&imu, LSM6DSV320X_2000dps);
  lsm6dsv320x_hg_xl_full_scale_set(&imu, LSM6DSV320X_320g);

  // Latched: INT stays asserted until the accel output is read, so a missed
  // edge is still caught by the level check in loop()
  lsm6dsv320x_data_ready_mode_set(&imu, LSM6DSV320X_DRDY_LATCHED);

  lsm6dsv320x_pin_int_route_t route = {0};
  route.drdy_xl = 1;
  if (lsm6dsv320x_pin_int1_route_set(&imu, &route) != 0) haltWithError("INT1 route failed");

  attachInterrupt(digitalPinToInterrupt(LSM_INT_PIN), onImuInt, LSM_INT_ACTIVE_LOW ? FALLING : RISING);

  lsm6dsv320x_xl_setup(&imu, LSM_XL_ODR, LSM6DSV320X_XL_HIGH_PERFORMANCE_MD);
  lsm6dsv320x_gy_setup(&imu, LSM_GY_ODR, LSM6DSV320X_GY_HIGH_PERFORMANCE_MD);
  lsm6dsv320x_hg_xl_data_rate_set(&imu, LSM_HG_ODR, 1);

  Serial.println("Streaming: accel [mg] | gyro [dps] | high-g accel [mg]");
}

void loop() {
  static int16_t xlRaw[3], gyRaw[3], hgRaw[3];
  static uint32_t samples = 0;
  static uint32_t readErrors = 0;
  static unsigned long lastPrint = 0;
  static unsigned long lastRate = 0;
  static uint32_t lastIsrCount = 0;
  static uint32_t lastSamples = 0;

  if (drdyPending || digitalRead(LSM_INT_PIN) == INT_ACTIVE_LEVEL) {
    drdyPending = false;

    lsm6dsv320x_data_ready_t drdy;
    if (lsm6dsv320x_flag_data_ready_get(&imu, &drdy) != 0) {
      readErrors++;
    } else {
      if (drdy.drdy_xl && lsm6dsv320x_acceleration_raw_get(&imu, xlRaw) == 0) samples++;
      if (drdy.drdy_gy) lsm6dsv320x_angular_rate_raw_get(&imu, gyRaw);
      if (drdy.drdy_hgxl) lsm6dsv320x_hg_acceleration_raw_get(&imu, hgRaw);
    }
  }

  unsigned long now = millis();

  if (now - lastPrint >= LSM_PRINT_PERIOD_MS) {
    lastPrint = now;
    Serial.printf("%8.1f %8.1f %8.1f | %8.2f %8.2f %8.2f | %9.1f %9.1f %9.1f\n",
                  lsm6dsv320x_from_fs16_to_mg(xlRaw[0]),
                  lsm6dsv320x_from_fs16_to_mg(xlRaw[1]),
                  lsm6dsv320x_from_fs16_to_mg(xlRaw[2]),
                  lsm6dsv320x_from_fs2000_to_mdps(gyRaw[0]) / 1000.0f,
                  lsm6dsv320x_from_fs2000_to_mdps(gyRaw[1]) / 1000.0f,
                  lsm6dsv320x_from_fs2000_to_mdps(gyRaw[2]) / 1000.0f,
                  lsm6dsv320x_from_fs320_to_mg(hgRaw[0]),
                  lsm6dsv320x_from_fs320_to_mg(hgRaw[1]),
                  lsm6dsv320x_from_fs320_to_mg(hgRaw[2]));
  }

  if (now - lastRate >= 1000) {
    float dt = (now - lastRate) / 1000.0f;
    uint32_t isrs = isrCount;
    float isrHz = (isrs - lastIsrCount) / dt;
    float sampleHz = (samples - lastSamples) / dt;

    // Allow +-10% for ODR tolerance and timing jitter
    bool rateOk = fabsf(isrHz - LSM_XL_ODR_HZ) < 0.1f * LSM_XL_ODR_HZ;
    Serial.printf("[%s] INT %.1f Hz, samples %.1f Hz (ODR %.0f Hz), I2C errors %lu\n",
                  rateOk ? "RATE" : "WARN", isrHz, sampleHz, LSM_XL_ODR_HZ, (unsigned long)readErrors);

    lastRate = now;
    lastIsrCount = isrs;
    lastSamples = samples;
  }
}
