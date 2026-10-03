/**
 * @file main.c
 * @brief Eyeliner V3 firmware entry point (ESP-IDF, no Arduino).
 *
 * Brings up every GPIO used by the V3 Top Board netlist, connects to the
 * LSM6DSV320X IMUs over the hardware I2C master driver, configures their
 * shared INT1 net as open-drain / active-low, then starts one task per core:
 *
 *   core 0 (PRO_CPU): comms_task   - receiver / telemetry / logging (Wi-Fi and
 *                                    other IDF system tasks also live here)
 *   core 1 (APP_CPU): control_task - IMU data-ready driven control loop
 *
 * Pin numbers come from include/board_pins.h.
 */
#include <stdatomic.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board_pins.h"
#include "lsm6dsv320x_reg.h"

static const char *TAG = "main";

/* ---------------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------------- */

// IMU used by the control loop; the other one on the bus is kept quiet
#ifndef IMU_PRIMARY_ADDR
#define IMU_PRIMARY_ADDR IMU_U5_I2C_ADDR
#endif
#define IMU_SECONDARY_ADDR (IMU_PRIMARY_ADDR ^ 0x01)

// LSM6DSV320X supports up to 1 MHz (I2C fast mode plus)
#ifndef IMU_I2C_FREQ_HZ
#define IMU_I2C_FREQ_HZ 400000
#endif

#define IMU_I2C_PORT       I2C_NUM_0
#define IMU_I2C_TIMEOUT_MS 10

#define IMU_XL_ODR LSM6DSV320X_ODR_AT_960Hz
#define IMU_GY_ODR LSM6DSV320X_ODR_AT_960Hz
#define IMU_HG_ODR LSM6DSV320X_HG_XL_ODR_AT_960Hz

// Control task wakes on INT1; this bounds the wait if an edge is ever missed
#define CONTROL_INT_TIMEOUT_MS 10

#define CONTROL_TASK_CORE     1
#define CONTROL_TASK_PRIORITY 10
#define CONTROL_TASK_STACK    4096

#define COMMS_TASK_CORE     0
#define COMMS_TASK_PRIORITY 5
#define COMMS_TASK_STACK    4096

_Static_assert(IMU_I2C_FREQ_HZ > 0 && IMU_I2C_FREQ_HZ <= 1000000, "LSM6DSV320X I2C max is 1 MHz");
_Static_assert(IMU_PRIMARY_ADDR == 0x6A || IMU_PRIMARY_ADDR == 0x6B, "LSM6DSV320X address is 0x6A or 0x6B");

/* ---------------------------------------------------------------------------
 * Shared state
 * ------------------------------------------------------------------------- */

typedef struct {
  int16_t accel[3];   // low-g, raw (+-16 g)
  int16_t gyro[3];    // raw (+-2000 dps)
  int16_t accel_hg[3]; // high-g, raw (+-320 g)
} imu_sample_t;

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t imu_dev;
static i2c_master_dev_handle_t imu_secondary_dev;
static stmdev_ctx_t imu;
static stmdev_ctx_t imu_secondary;

static adc_oneshot_unit_handle_t adc1;
static adc_channel_t vbat_adc_channel;
static adc_channel_t temp_adc_channel;

static TaskHandle_t control_task_handle;

static portMUX_TYPE sample_lock = portMUX_INITIALIZER_UNLOCKED;
static imu_sample_t latest_sample;
static atomic_uint_fast32_t sample_count;
static atomic_uint_fast32_t imu_error_count;

/* ---------------------------------------------------------------------------
 * GPIO
 * ------------------------------------------------------------------------- */

static esp_err_t gpio_config_pins(uint64_t mask, gpio_mode_t mode, bool pull_up, gpio_int_type_t intr) {
  const gpio_config_t cfg = {
      .pin_bit_mask = mask,
      .mode = mode,
      .pull_up_en = pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = intr,
  };
  return gpio_config(&cfg);
}

// Puts every netlist GPIO into a known, safe state. Peripheral drivers
// (I2C, RMT for DShot, UART for the receiver) take their pins over later.
// I2C SDA/SCL are configured by the I2C master driver in imu_bus_init().
static esp_err_t board_gpio_init(void) {
  // Outputs, driven low
  //   DShot: idle low until the RMT driver owns the pins, so ESCs see no pulses
  //   LED data: WS2812 idle low
  //   Temp mux selects: channel 0
  const uint64_t outputs_low = BIT64(PIN_DSHOT_1) | BIT64(PIN_DSHOT_2) | BIT64(PIN_LED_DATA) |
                               BIT64(PIN_TEMP_SEL_1) | BIT64(PIN_TEMP_SEL_2);
  ESP_RETURN_ON_ERROR(gpio_config_pins(outputs_low, GPIO_MODE_OUTPUT, false, GPIO_INTR_DISABLE), TAG, "outputs");
  ESP_RETURN_ON_ERROR(gpio_set_level(PIN_DSHOT_1, 0), TAG, "dshot1");
  ESP_RETURN_ON_ERROR(gpio_set_level(PIN_DSHOT_2, 0), TAG, "dshot2");
  ESP_RETURN_ON_ERROR(gpio_set_level(PIN_LED_DATA, 0), TAG, "led");
  ESP_RETURN_ON_ERROR(gpio_set_level(PIN_TEMP_SEL_1, 0), TAG, "temp sel1");
  ESP_RETURN_ON_ERROR(gpio_set_level(PIN_TEMP_SEL_2, 0), TAG, "temp sel2");

  // I2C mux select: route SDA/SCL to the IMUs. Set before the bus is started
  ESP_RETURN_ON_ERROR(gpio_config_pins(BIT64(PIN_I2C_SEL), GPIO_MODE_OUTPUT, false, GPIO_INTR_DISABLE), TAG, "i2c sel");
  ESP_RETURN_ON_ERROR(gpio_set_level(PIN_I2C_SEL, I2C_SEL_LEVEL_IMU), TAG, "i2c sel");

  // UART-style inputs, pulled up so they idle high when nothing is plugged in.
  // Receiver pins stay inputs until the UART driver claims them, so nothing
  // is driven against the receiver before the TX/RX direction is confirmed
  const uint64_t uart_inputs = BIT64(PIN_ESC_TELEM_1) | BIT64(PIN_ESC_TELEM_2) |
                               BIT64(PIN_RECEIVER_TX) | BIT64(PIN_RECEIVER_RX);
  ESP_RETURN_ON_ERROR(gpio_config_pins(uart_inputs, GPIO_MODE_INPUT, true, GPIO_INTR_DISABLE), TAG, "uart inputs");

  // IMU INT1 (shared, open-drain, active-low, 4.7k external pull-up R9).
  // ISR is attached from the control task so it's allocated on core 1
  ESP_RETURN_ON_ERROR(gpio_config_pins(BIT64(PIN_IMU_INT), GPIO_MODE_INPUT, false, GPIO_INTR_NEGEDGE), TAG, "imu int");

  // Analog inputs: ADC1 oneshot (ADC1 is usable alongside Wi-Fi, unlike ADC2)
  const adc_oneshot_unit_init_cfg_t adc_cfg = {.unit_id = ADC_UNIT_1};
  ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adc_cfg, &adc1), TAG, "adc1");

  adc_unit_t unit;
  ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(PIN_VBAT_SENSE, &unit, &vbat_adc_channel), TAG, "vbat io");
  ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(PIN_TEMP_SENSE, &unit, &temp_adc_channel), TAG, "temp io");

  const adc_oneshot_chan_cfg_t chan_cfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
  ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc1, vbat_adc_channel, &chan_cfg), TAG, "vbat ch");
  ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc1, temp_adc_channel, &chan_cfg), TAG, "temp ch");

  return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * LSM6DSV320X over I2C
 * ------------------------------------------------------------------------- */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len) {
  i2c_master_transmit_multi_buffer_info_t parts[2] = {
      {.write_buffer = &reg, .buffer_size = 1},
      {.write_buffer = (uint8_t *)buf, .buffer_size = len},
  };
  return i2c_master_multi_buffer_transmit(handle, parts, 2, IMU_I2C_TIMEOUT_MS) == ESP_OK ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len) {
  return i2c_master_transmit_receive(handle, &reg, 1, buf, len, IMU_I2C_TIMEOUT_MS) == ESP_OK ? 0 : -1;
}

static void platform_delay(uint32_t ms) {
  TickType_t ticks = pdMS_TO_TICKS(ms);
  vTaskDelay(ticks > 0 ? ticks : 1);
}

static esp_err_t imu_bus_init(void) {
  const i2c_master_bus_config_t bus_cfg = {
      .i2c_port = IMU_I2C_PORT,
      .sda_io_num = PIN_I2C_SDA,
      .scl_io_num = PIN_I2C_SCL,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = false, // R10/R11 4.7k on the board
  };
  ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &i2c_bus), TAG, "i2c bus");

  const i2c_device_config_t primary_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = IMU_PRIMARY_ADDR,
      .scl_speed_hz = IMU_I2C_FREQ_HZ,
  };
  ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &primary_cfg, &imu_dev), TAG, "imu dev");

  i2c_device_config_t secondary_cfg = primary_cfg;
  secondary_cfg.device_address = IMU_SECONDARY_ADDR;
  ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &secondary_cfg, &imu_secondary_dev), TAG, "imu2 dev");

  imu = (stmdev_ctx_t){
      .write_reg = platform_write,
      .read_reg = platform_read,
      .mdelay = platform_delay,
      .handle = imu_dev,
  };
  imu_secondary = imu;
  imu_secondary.handle = imu_secondary_dev;

  return ESP_OK;
}

static bool imu_who_am_i_ok(const stmdev_ctx_t *ctx, uint8_t addr) {
  uint8_t id = 0;
  if (lsm6dsv320x_device_id_get(ctx, &id) != 0) {
    ESP_LOGW(TAG, "IMU 0x%02X: no response", addr);
    return false;
  }
  if (id != LSM6DSV320X_ID) {
    ESP_LOGW(TAG, "IMU 0x%02X: WHO_AM_I 0x%02X, expected 0x%02X", addr, id, LSM6DSV320X_ID);
    return false;
  }
  return true;
}

// IF_CFG (03h): PP_OD = 1 (open-drain), H_LACTIVE = 1 (active-low). The ST
// driver has no setter for these. IF_CFG survives SW_RESET, so this can be
// written before or after a reset
static esp_err_t imu_set_int_open_drain(const stmdev_ctx_t *ctx) {
  lsm6dsv320x_if_cfg_t if_cfg;
  if (lsm6dsv320x_read_reg(ctx, LSM6DSV320X_IF_CFG, (uint8_t *)&if_cfg, 1) != 0) return ESP_FAIL;

  if_cfg.pp_od = 1;
  if_cfg.h_lactive = 1;
  // SDA is open-drain in I2C mode (it only goes push-pull under I3C, which needs
  // an I3C controller to assign a dynamic address). Keep its internal pull-up off
  // too; the board has R11
  if_cfg.sda_pu_en = 0;
  if (lsm6dsv320x_write_reg(ctx, LSM6DSV320X_IF_CFG, (uint8_t *)&if_cfg, 1) != 0) return ESP_FAIL;

  lsm6dsv320x_if_cfg_t readback;
  if (lsm6dsv320x_read_reg(ctx, LSM6DSV320X_IF_CFG, (uint8_t *)&readback, 1) != 0) return ESP_FAIL;
  return (readback.pp_od == 1 && readback.h_lactive == 1 && readback.sda_pu_en == 0) ? ESP_OK
                                                                                     : ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t imu_init(void) {
  // Boot time after power-up
  vTaskDelay(pdMS_TO_TICKS(20));

  // Both IMUs share INT1 and power up push-pull, so the secondary has to be
  // released first or it'll fight the primary on the shared net
  if (imu_who_am_i_ok(&imu_secondary, IMU_SECONDARY_ADDR)) {
    const lsm6dsv320x_pin_int_route_t none = {0};
    lsm6dsv320x_pin_int1_route_set(&imu_secondary, &none);
    ESP_RETURN_ON_ERROR(imu_set_int_open_drain(&imu_secondary), TAG, "imu2 IF_CFG");
    ESP_LOGI(TAG, "IMU 0x%02X: INT1 released (open-drain, nothing routed)", IMU_SECONDARY_ADDR);
  } else {
    ESP_LOGW(TAG, "IMU 0x%02X not found; if it's populated it may hold INT low", IMU_SECONDARY_ADDR);
  }

  if (!imu_who_am_i_ok(&imu, IMU_PRIMARY_ADDR)) return ESP_ERR_NOT_FOUND;
  if (lsm6dsv320x_sw_reset(&imu) != 0) return ESP_FAIL;
  ESP_RETURN_ON_ERROR(imu_set_int_open_drain(&imu), TAG, "imu IF_CFG");

  // Nothing routed yet, so the pulled-up line should idle high
  if (gpio_get_level(PIN_IMU_INT) == 0) {
    ESP_LOGW(TAG, "INT line is low with nothing routed; check R9 and the secondary IMU");
  }

  int32_t ret = 0;
  ret += lsm6dsv320x_block_data_update_set(&imu, 1);
  ret += lsm6dsv320x_xl_full_scale_set(&imu, LSM6DSV320X_16g);
  ret += lsm6dsv320x_gy_full_scale_set(&imu, LSM6DSV320X_2000dps);
  ret += lsm6dsv320x_hg_xl_full_scale_set(&imu, LSM6DSV320X_320g);

  // Latched: INT stays asserted until the accel output is read, so a missed
  // edge is recovered by the level check in the control task
  ret += lsm6dsv320x_data_ready_mode_set(&imu, LSM6DSV320X_DRDY_LATCHED);

  const lsm6dsv320x_pin_int_route_t route = {.drdy_xl = 1};
  ret += lsm6dsv320x_pin_int1_route_set(&imu, &route);

  ret += lsm6dsv320x_xl_setup(&imu, IMU_XL_ODR, LSM6DSV320X_XL_HIGH_PERFORMANCE_MD);
  ret += lsm6dsv320x_gy_setup(&imu, IMU_GY_ODR, LSM6DSV320X_GY_HIGH_PERFORMANCE_MD);
  ret += lsm6dsv320x_hg_xl_data_rate_set(&imu, IMU_HG_ODR, 1);

  if (ret != 0) return ESP_FAIL;

  ESP_LOGI(TAG, "IMU 0x%02X: configured, INT1 open-drain active-low", IMU_PRIMARY_ADDR);
  return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Tasks
 * ------------------------------------------------------------------------- */

static void IRAM_ATTR imu_int_isr(void *arg) {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(control_task_handle, &woken);
  portYIELD_FROM_ISR(woken);
}

static void imu_read_sample(void) {
  lsm6dsv320x_data_ready_t drdy;
  if (lsm6dsv320x_flag_data_ready_get(&imu, &drdy) != 0) {
    atomic_fetch_add(&imu_error_count, 1);
    return;
  }

  imu_sample_t sample;
  portENTER_CRITICAL(&sample_lock);
  sample = latest_sample;
  portEXIT_CRITICAL(&sample_lock);

  bool ok = true;
  if (drdy.drdy_xl) ok &= lsm6dsv320x_acceleration_raw_get(&imu, sample.accel) == 0;
  if (drdy.drdy_gy) ok &= lsm6dsv320x_angular_rate_raw_get(&imu, sample.gyro) == 0;
  if (drdy.drdy_hgxl) ok &= lsm6dsv320x_hg_acceleration_raw_get(&imu, sample.accel_hg) == 0;

  if (!ok) {
    atomic_fetch_add(&imu_error_count, 1);
    return;
  }

  portENTER_CRITICAL(&sample_lock);
  latest_sample = sample;
  portEXIT_CRITICAL(&sample_lock);

  if (drdy.drdy_xl) atomic_fetch_add(&sample_count, 1);
}

// Core 1: IMU-driven control loop
static void control_task(void *arg) {
  // Installing the ISR service from this task allocates the GPIO interrupt on core 1
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_IMU_INT, imu_int_isr, NULL));

  while (true) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CONTROL_INT_TIMEOUT_MS));

    // Drain while the latched DRDY is still asserting (active-low)
    while (gpio_get_level(PIN_IMU_INT) == 0) {
      imu_read_sample();
    }

    // TODO: heading estimate + motor control
  }
}

// Core 0: receiver, telemetry, logging
static void comms_task(void *arg) {
  uint32_t last_count = atomic_load(&sample_count);
  TickType_t last_wake = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));

    uint32_t count = atomic_load(&sample_count);
    imu_sample_t sample;
    portENTER_CRITICAL(&sample_lock);
    sample = latest_sample;
    portEXIT_CRITICAL(&sample_lock);

    int vbat_raw = 0;
    adc_oneshot_read(adc1, vbat_adc_channel, &vbat_raw);

    ESP_LOGI(TAG, "IMU %lu Hz, errors %lu | accel %.0f %.0f %.0f mg | gyro %.1f %.1f %.1f dps | vbat raw %d",
             (unsigned long)(count - last_count), (unsigned long)atomic_load(&imu_error_count),
             lsm6dsv320x_from_fs16_to_mg(sample.accel[0]),
             lsm6dsv320x_from_fs16_to_mg(sample.accel[1]),
             lsm6dsv320x_from_fs16_to_mg(sample.accel[2]),
             lsm6dsv320x_from_fs2000_to_mdps(sample.gyro[0]) / 1000.0f,
             lsm6dsv320x_from_fs2000_to_mdps(sample.gyro[1]) / 1000.0f,
             lsm6dsv320x_from_fs2000_to_mdps(sample.gyro[2]) / 1000.0f,
             vbat_raw);
    last_count = count;

    // TODO: CRSF receiver + telemetry
  }
}

/* ---------------------------------------------------------------------------
 * Entry point
 * ------------------------------------------------------------------------- */

void app_main(void) {
  ESP_ERROR_CHECK(board_gpio_init());
  ESP_ERROR_CHECK(imu_bus_init());

  // Expect SDA/SCL: OutputEn 1, OpenDrain 1, InputEn 1. INT: OutputEn 0, InputEn 1
  gpio_dump_io_configuration(stdout, BIT64(PIN_I2C_SDA) | BIT64(PIN_I2C_SCL) | BIT64(PIN_IMU_INT));

  ESP_ERROR_CHECK(imu_init());

  // Control first, so its handle is valid before the ISR can fire
  xTaskCreatePinnedToCore(control_task, "control", CONTROL_TASK_STACK, NULL, CONTROL_TASK_PRIORITY,
                          &control_task_handle, CONTROL_TASK_CORE);
  xTaskCreatePinnedToCore(comms_task, "comms", COMMS_TASK_STACK, NULL, COMMS_TASK_PRIORITY, NULL,
                          COMMS_TASK_CORE);
}
