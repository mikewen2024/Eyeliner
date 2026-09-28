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
 * I2C bus (4.7k pull-ups R10/R11 to 3V3 on the ESP side of the mux)
 * SDA/SCL pass through U3 (TMUX154E, 2:1 mux):
 *   SEL = LOW  -> A0/B0 -> SDA_Accel/SCL_Accel (both LSM6DSV320X)
 *   SEL = HIGH -> A1/B1 -> SDA_Extra/SCL_Extra (JST-SH connector I2C1)
 * U3 ~EN is tied to GND (always enabled).
 * ------------------------------------------------------------------------- */
#define PIN_I2C_SDA             1   // D0      net /Top_Board/SDA
#define PIN_I2C_SCL             2   // D1      net /Top_Board/SCL
#define PIN_I2C_SEL             41  // MTDI    net /Top_Board/I2C_Sel -> U3 SEL

#define I2C_SEL_LEVEL_IMU       0   // LOW : route bus to the IMUs
#define I2C_SEL_LEVEL_EXTRA     1   // HIGH: route bus to the I2C1 connector

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
#define PIN_RECEIVER_TX         4   // D3      net /Top_Board/Receiver_TX
#define PIN_RECEIVER_RX         5   // D4      net /Top_Board/Receiver_RX

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
