/**
 * @file board_pins.h
 * @brief Eyeliner V3 Top Board: schematic net -> ESP32-S3 GPIO mapping.
 *
 * Source: hardware/Eyeliner_PCB_V3/Top_Board_Netlist.net (module U2,
 * Seeed XIAO-ESP32-S3-SMD). XIAO pad -> GPIO numbers come from the
 * arduino-esp32 XIAO_ESP32S3 variant; the four bottom JTAG pads are fixed
 * ESP32-S3 pins (MTCK=39, MTDO=40, MTDI=41, MTMS=42).
 *
 * Plain #defines so they can be used in #if and overridden via build_flags.
 * Re-check this file whenever the netlist is regenerated.
 */
#pragma once

/* ---------------------------------------------------------------------------
 * I2C bus: D0/D1 straight to both LSM6DSV320X and the PCA9570 (IC1, 0x24).
 * 1k pull-ups (R13 SDA, R10 SCL), 100R series (R16 SDA, R15 SCL).
 * ------------------------------------------------------------------------- */
#define PIN_I2C_SDA             1   // D0      net /Top_Board/SDA
#define PIN_I2C_SCL             2   // D1      net /Top_Board/SCL

// No I2C mux on the current board (TMUX154E removed); MTDI is unconnected.
// Kept only so existing code that drives the old select line still builds.
#define PIN_I2C_SEL             41  // MTDI    (unconnected)

#define I2C_SEL_LEVEL_IMU       0
#define I2C_SEL_LEVEL_EXTRA     1

/* ---------------------------------------------------------------------------
 * IMUs: two LSM6DSV320X on the same bus (via the mux above)
 * INT1 of both parts is wired to one net with a 4.7k pull-up (R9) to 3V3,
 * so INT1 must be open-drain + active-low on both (wired-OR), or the part
 * not in use must release the line. INT2 is unconnected on both.
 * ------------------------------------------------------------------------- */
#define PIN_IMU_INT             3   // D2      net /Top_Board/INT (U4 + U5 INT1)

#define IMU_U4_I2C_ADDR         0x6B  // SDO/SA0 tied to 3V3
#define IMU_U5_I2C_ADDR         0x6A  // SDO/SA0 tied to GND

/* ---------------------------------------------------------------------------
 * ESCs (100R series + RCLAMP3346P ESD on U7)
 * ------------------------------------------------------------------------- */
#define PIN_DSHOT_1             8   // D9      net /Top_Board/Dshot1
#define PIN_DSHOT_2             7   // D8      net /Top_Board/Dshot2
#define PIN_ESC_TELEM_1         40  // MTDO    net /Top_Board/ESC_Telem1
#define PIN_ESC_TELEM_2         42  // MTMS    net /Top_Board/ESC_Telem2

/* ---------------------------------------------------------------------------
 * RC receiver (100R series + ESD on U8). Net names as drawn in the schematic;
 * confirm which side each name is referenced to before assigning UART RX/TX.
 * ------------------------------------------------------------------------- */
#define PIN_RECEIVER_TX         6   // D5      net /Top_Board/Receiver_TX
#define PIN_RECEIVER_RX         7   // D6      net /Top_Board/Receiver_RX

/* ---------------------------------------------------------------------------
 * LEDs: level-shifted through Q1 (BSS138) to the LEDs1 connector
 * ------------------------------------------------------------------------- */
#define PIN_LED_DATA            39  // MTCK    net /Top_Board/LEDs

/* ---------------------------------------------------------------------------
 * Analog sensing
 * ------------------------------------------------------------------------- */
#define PIN_VBAT_SENSE          6   // D5      net /Top_Board/Vbat_Sense (22k/2k divider, ADC1_CH5)
#define PIN_TEMP_SENSE          9   // D10     net /Top_Board/Temp_Sense (10k pull-up, ADC1_CH8)
#define PIN_TEMP_SEL_1          43  // D6      net /Top_Board/Temp_Sel1 -> U6 (TMUX1204) A0
#define PIN_TEMP_SEL_2          44  // D7      net /Top_Board/Temp_Sel2 -> U6 (TMUX1204) A1
