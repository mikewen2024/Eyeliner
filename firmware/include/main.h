/**
 * @file main.h
 * @brief Eyeliner V3 firmware: configuration, shared types, and board/IMU
 *        bring-up functions used by main.c.
 *
 * Pin numbers live in board_pins.h.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "lsm6dsv320x_reg.h"

#include "board_pins.h"

/* ---------------------------------------------------------------------------
 * IMU / I2C configuration
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

_Static_assert(IMU_I2C_FREQ_HZ > 0 && IMU_I2C_FREQ_HZ <= 1000000, "LSM6DSV320X I2C max is 1 MHz");
_Static_assert(IMU_PRIMARY_ADDR == 0x6A || IMU_PRIMARY_ADDR == 0x6B, "LSM6DSV320X address is 0x6A or 0x6B");

/* ---------------------------------------------------------------------------
 * Task configuration
 * ------------------------------------------------------------------------- */

// Control task wakes on INT1; this bounds the wait if an edge is ever missed
#define CONTROL_INT_TIMEOUT_MS 10

#define CONTROL_TASK_CORE     1
#define CONTROL_TASK_PRIORITY 10
#define CONTROL_TASK_STACK    4096

#define COMMS_TASK_CORE     0
#define COMMS_TASK_PRIORITY 5
#define COMMS_TASK_STACK    4096

/* ---------------------------------------------------------------------------
 * Types
 * ------------------------------------------------------------------------- */

typedef struct {
  int16_t accel[3];    // low-g, raw (+-16 g)
  int16_t gyro[3];     // raw (+-2000 dps)
  int16_t accel_hg[3]; // high-g, raw (+-320 g)
} imu_sample_t;

/* ---------------------------------------------------------------------------
 * Board bring-up
 * ------------------------------------------------------------------------- */

// Puts every netlist GPIO into a known, safe state and sets up the ADC inputs
esp_err_t board_gpio_init(void);

/* ---------------------------------------------------------------------------
 * LSM6DSV320X
 * ------------------------------------------------------------------------- */

// Creates the I2C master bus and both IMU devices, and fills in the driver contexts
esp_err_t imu_bus_init(void);

// Releases the secondary IMU's INT1, then resets and configures the primary
esp_err_t imu_init(void);

// Returns true if the IMU at ctx answers with the expected WHO_AM_I
bool imu_who_am_i_ok(const stmdev_ctx_t *ctx, uint8_t addr);

// IF_CFG (03h): INT1/INT2 open-drain, active-low, verified by readback
esp_err_t imu_set_int_open_drain(const stmdev_ctx_t *ctx);

// Reads whichever outputs are ready into the shared latest sample
void imu_read_sample(void);
