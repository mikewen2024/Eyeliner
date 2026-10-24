#include "imu.h"

#include <Arduino.h>
#include <Wire.h>

#include "driver/gpio.h"

#include "board_pins.h"
#include "config.h"
#include "lsm6dsv320x_reg.h"

static_assert(IMU_READ_INTERVAL_MS > 0, "IMU_READ_INTERVAL_MS must be positive");
static_assert(IMU_I2C_FREQ_HZ > 0 && IMU_I2C_FREQ_HZ <= 1000000, "LSM6DSV320X I2C max is 1 MHz");

#define IMU_WIRE Wire

// One burst from STATUS_REG through the high-g outputs: 28 bytes, ~390 us at 1 MHz
#define BURST_START    LSM6DSV320X_STATUS_REG
#define BURST_END      LSM6DSV320X_UI_OUTZ_H_A_OIS_HG
#define BURST_LEN      (BURST_END - BURST_START + 1)
#define BURST_OFS(reg) ((reg) - BURST_START)

static const float kG = 9.80665f;

struct ImuDev {
  uint8_t      addr;
  bool         present;
  stmdev_ctx_t ctx;
  int16_t      xl[3], gy[3], hg[3], temp;
  uint32_t     errors;
};

static ImuDev       s_dev[IMU_COUNT];
static ImuState     s_work;
static ImuState     s_shared;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t     s_nextReadMs = 0;

/* ST driver glue ------------------------------------------------------------ */
static int32_t platformWrite(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len) {
  ImuDev *d = static_cast<ImuDev *>(handle);
  IMU_WIRE.beginTransmission(d->addr);
  IMU_WIRE.write(reg);
  IMU_WIRE.write(buf, len);
  return IMU_WIRE.endTransmission() == 0 ? 0 : -1;
}

static int32_t platformRead(void *handle, uint8_t reg, uint8_t *buf, uint16_t len) {
  ImuDev *d = static_cast<ImuDev *>(handle);
  IMU_WIRE.beginTransmission(d->addr);
  IMU_WIRE.write(reg);
  if (IMU_WIRE.endTransmission(false) != 0) return -1;
  if (IMU_WIRE.requestFrom((uint16_t)d->addr, (size_t)len, true) != len) return -1;
  for (uint16_t i = 0; i < len; i++) buf[i] = IMU_WIRE.read();
  return 0;
}

static void platformDelay(uint32_t ms) { delay(ms); }

static inline int16_t le16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

/* Setup ---------------------------------------------------------------------- */

// IF_CFG: INT1 open-drain + active-low (both INT1 pins share one net), and the
// sensor's own SDA pull-up off (the board has 1k). Not touched by SW_RESET.
static bool setIfCfg(ImuDev &d) {
  lsm6dsv320x_if_cfg_t cfg;
  if (lsm6dsv320x_read_reg(&d.ctx, LSM6DSV320X_IF_CFG, (uint8_t *)&cfg, 1) != 0) return false;
  cfg.pp_od = 1;
  cfg.h_lactive = 1;
  cfg.sda_pu_en = 0;
  if (lsm6dsv320x_write_reg(&d.ctx, LSM6DSV320X_IF_CFG, (uint8_t *)&cfg, 1) != 0) return false;
  lsm6dsv320x_if_cfg_t rb;
  if (lsm6dsv320x_read_reg(&d.ctx, LSM6DSV320X_IF_CFG, (uint8_t *)&rb, 1) != 0) return false;
  return rb.pp_od == 1 && rb.h_lactive == 1 && rb.sda_pu_en == 0;
}

static bool configure(ImuDev &d) {
  int32_t rc = 0;
  rc |= lsm6dsv320x_sw_reset(&d.ctx);
  if (!setIfCfg(d)) return false;
  rc |= lsm6dsv320x_block_data_update_set(&d.ctx, 1);
  rc |= lsm6dsv320x_xl_full_scale_set(&d.ctx, LSM6DSV320X_16g);
  rc |= lsm6dsv320x_gy_full_scale_set(&d.ctx, LSM6DSV320X_2000dps);
  rc |= lsm6dsv320x_hg_xl_full_scale_set(&d.ctx, LSM6DSV320X_320g);
  rc |= lsm6dsv320x_xl_setup(&d.ctx, IMU_XL_ODR, LSM6DSV320X_XL_HIGH_PERFORMANCE_MD);
  rc |= lsm6dsv320x_gy_setup(&d.ctx, IMU_GY_ODR, LSM6DSV320X_GY_HIGH_PERFORMANCE_MD);
  rc |= lsm6dsv320x_hg_xl_data_rate_set(&d.ctx, IMU_HG_ODR, 1);
  return rc == 0;
}

static void publish() {
  portENTER_CRITICAL(&s_lock);
  s_shared = s_work;
  portEXIT_CRITICAL(&s_lock);
}

int imuInit() {
  const uint8_t addrs[IMU_COUNT] = {IMU_U4_I2C_ADDR, IMU_U3_I2C_ADDR};
  memset(&s_work, 0, sizeof(s_work));

  // Wire's underlying driver sets SDA/SCL open-drain and turns on the internal
  // pull-ups; the board has 1k pull-ups, so turn the internal ones off again
  if (!IMU_WIRE.begin(PIN_I2C_SDA, PIN_I2C_SCL, IMU_I2C_FREQ_HZ)) {
#ifdef DISPLAY_SERIAL
    Serial.println("[imu] Wire.begin failed");
#endif
    return 0;
  }
  gpio_pullup_dis((gpio_num_t)PIN_I2C_SDA);
  gpio_pullup_dis((gpio_num_t)PIN_I2C_SCL);
  pinMode(PIN_IMU_INT, INPUT);
  delay(20);  // sensor boot time

  int found = 0;
  for (int i = 0; i < IMU_COUNT; i++) {
    ImuDev &d = s_dev[i];
    memset(&d, 0, sizeof(d));
    d.addr = addrs[i];
    d.ctx = {platformWrite, platformRead, platformDelay, &d, nullptr};

    uint8_t id = 0;
    bool ok = lsm6dsv320x_device_id_get(&d.ctx, &id) == 0 && id == LSM6DSV320X_ID;
    if (ok) ok = configure(d);
    d.present = ok;
    if (ok) found++;

    s_work.imu[i].addr = d.addr;
    s_work.imu[i].present = ok;
#ifdef DISPLAY_SERIAL
    Serial.printf("[imu] 0x%02X: WHO_AM_I 0x%02X %s\n", d.addr, id, ok ? "configured" : "NOT FOUND / config failed");
#endif
  }
#ifdef DISPLAY_SERIAL
  Serial.printf("[imu] I2C SDA=GPIO%d SCL=GPIO%d %lu Hz, %d/%d IMUs, read every %d ms\n", PIN_I2C_SDA, PIN_I2C_SCL,
                (unsigned long)IMU_I2C_FREQ_HZ, found, IMU_COUNT, IMU_READ_INTERVAL_MS);
#endif
  publish();
  s_nextReadMs = millis();
  return found;
}

/* Read ----------------------------------------------------------------------- */

static void readOne(ImuDev &d, ImuSample &out) {
  uint8_t burst[BURST_LEN];
  if (lsm6dsv320x_read_reg(&d.ctx, BURST_START, burst, BURST_LEN) != 0) {
    d.errors++;
    out.errors = d.errors;
    return;
  }
  lsm6dsv320x_status_reg_t status;
  memcpy(&status, &burst[BURST_OFS(LSM6DSV320X_STATUS_REG)], 1);

  // Only take what STATUS flags as new; otherwise keep the previous sample
  if (status.xlda) {
    const uint8_t *p = &burst[BURST_OFS(LSM6DSV320X_OUTX_L_A)];
    for (int i = 0; i < 3; i++) d.xl[i] = le16(p + 2 * i);
  }
  if (status.gda) {
    const uint8_t *p = &burst[BURST_OFS(LSM6DSV320X_OUTX_L_G)];
    for (int i = 0; i < 3; i++) d.gy[i] = le16(p + 2 * i);
  }
  if (status.xlhgda) {
    const uint8_t *p = &burst[BURST_OFS(LSM6DSV320X_UI_OUTX_L_A_OIS_HG)];
    for (int i = 0; i < 3; i++) d.hg[i] = le16(p + 2 * i);
  }
  if (status.tda) d.temp = le16(&burst[BURST_OFS(LSM6DSV320X_OUT_TEMP_L)]);

  for (int i = 0; i < 3; i++) {
    out.accel[i]   = lsm6dsv320x_from_fs16_to_mg(d.xl[i]) * 0.001f * kG;
    out.gyro[i]    = lsm6dsv320x_from_fs2000_to_mdps(d.gy[i]) * 0.001f;
    out.accelHg[i] = lsm6dsv320x_from_fs320_to_mg(d.hg[i]) * 0.001f * kG;
  }
  out.tempC  = lsm6dsv320x_from_lsb_to_celsius(d.temp);
  out.errors = d.errors;
}

void imuUpdate() {
  uint32_t nowMs = millis();
  if ((int32_t)(nowMs - s_nextReadMs) < 0) return;
  // Fixed cadence: schedule from the previous deadline, not from now
  s_nextReadMs += IMU_READ_INTERVAL_MS;
  if ((int32_t)(nowMs - s_nextReadMs) > 0) s_nextReadMs = nowMs + IMU_READ_INTERVAL_MS;  // fell behind

  uint32_t t0 = micros();
  s_work.sampleUs = t0;
  for (int i = 0; i < IMU_COUNT; i++) {
    if (s_dev[i].present) readOne(s_dev[i], s_work.imu[i]);
  }
  s_work.readUs = micros() - t0;
  s_work.seq++;
  publish();

#ifdef DISPLAY_SERIAL
  static uint32_t lastPrintMs = 0;
  if (nowMs - lastPrintMs >= IMU_TELEM_PERIOD_MS) {
    lastPrintMs = nowMs;
    for (int i = 0; i < IMU_COUNT; i++) {
      const ImuSample &s = s_work.imu[i];
      if (!s.present) {
        Serial.printf("[imu] 0x%02X absent\n", s.addr);
        continue;
      }
      Serial.printf("[imu] 0x%02X a %+6.2f %+6.2f %+6.2f m/s2 | g %+7.1f %+7.1f %+7.1f dps | hg %+6.1f %+6.1f %+6.1f | %4.1f C | err %lu | %lu us\n",
                    s.addr, s.accel[0], s.accel[1], s.accel[2], s.gyro[0], s.gyro[1], s.gyro[2], s.accelHg[0],
                    s.accelHg[1], s.accelHg[2], s.tempC, (unsigned long)s.errors, (unsigned long)s_work.readUs);
    }
  }
#endif
}

ImuState imuGet() {
  ImuState copy;
  portENTER_CRITICAL(&s_lock);
  copy = s_shared;
  portEXIT_CRITICAL(&s_lock);
  return copy;
}
