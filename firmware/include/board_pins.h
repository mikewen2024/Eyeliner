/**
 * @file board_pins.h
 * @brief Eyeliner V3.1 Top Board: schematic net -> ESP32-S3 GPIO mapping.
 *
 * Source: hardware/Eyeliner_PCB_V3/Top_Board_Netlist.net (Eeschema export
 * 2026-10-14, module U2 = Seeed XIAO-ESP32-S3-SMD). XIAO pad -> GPIO numbers
 * come from the arduino-esp32 XIAO_ESP32S3 variant (D0=1 D1=2 D2=3 D3=4 D4=5
 * D5=6 D6=43 D7=44 D8=7 D9=8 D10=9); the bottom JTAG pads are fixed ESP32-S3
 * pins (MTCK=39, MTDO=40, MTDI=41, MTMS=42). Of those only MTDO is used.
 *
 * Plain #defines so they can be used in #if and overridden via build_flags.
 * Re-check this file whenever the netlist is regenerated.
 *
 *   XIAO pad  GPIO  net                 goes to
 *   D0        1     SDA                 U3/U4 IMUs, IC1 PCA9570, I2C1 conn (100R)
 *   D1        2     SCL                 same + IC2 1A (gated SK9822 clock)
 *   D2        3     INT                 U3 INT1 + U4 INT1, 4.7k (R1) to 3V3
 *   D3        4     Sense               U5 TMUX1204 common, 1k (R14) to 3V3
 *   D4        5     Vbat_Sense          VCC_HV / (20k R11 + 2k R12)
 *   D5        6     -> Receiver RX      MCU UART TX (100R R3, ESD U6)
 *   D6        43    <- Receiver TX      MCU UART RX (100R R2, ESD U6)
 *   D7        44    ESC_Telem1          ESCs1 pin 6 (100R R22, ESD U7)
 *   D8        7     Dshot1              ESCs1 pin 5 (100R R23, ESD U7)
 *   D9        8     Dshot2              ESCs1 pin 2 (100R R21, ESD U7)
 *   D10       9     ESC_Telem2          ESCs1 pin 1 (100R R20, ESD U7)
 *   MTDO      40    LEDs                IC2 2A -> Conn_LEDs1 pin 2 (SK9822 data)
 *   MTCK/MTDI/MTMS  39/41/42            unconnected
 */
#pragma once

/* ---------------------------------------------------------------------------
 * I2C bus. 1k pull-ups to VCC_3V3 (R13 SDA, R10 SCL). Devices:
 *   U3  LSM6DSV320X  0x6B  (SDO/SA0 -> 3V3)
 *   U4  LSM6DSV320X  0x6A  (SDO/SA0 -> GND)
 *   IC1 PCA9570      0x24  (4-bit I/O expander, fixed address)
 * The I2C1 connector hangs on the same bus through 100R (R15 SCL, R16 SDA).
 * No mux on this spin (TMUX154E removed).
 * ------------------------------------------------------------------------- */
#define PIN_I2C_SDA             1   // D0      net /Top_Board/SDA
#define PIN_I2C_SCL             2   // D1      net /Top_Board/SCL

/* ---------------------------------------------------------------------------
 * IMUs: two LSM6DSV320X. INT1 of both is one net with a 4.7k pull-up (R1), so
 * INT1 must be open-drain + active-low on both (wired-OR). INT2 unconnected.
 * CS is tied to 3V3 on both (I2C mode only); aux SPI pins grounded.
 * ------------------------------------------------------------------------- */
#define PIN_IMU_INT             3   // D2      net /Top_Board/INT (U3 + U4 INT1)

#define IMU_U3_I2C_ADDR         0x6B  // SDO/SA0 tied to 3V3
#define IMU_U4_I2C_ADDR         0x6A  // SDO/SA0 tied to GND

/* ---------------------------------------------------------------------------
 * PCA9570 I/O expander (IC1, 0x24). Pins are outputs on the expander, not GPIOs.
 * ------------------------------------------------------------------------- */
#define PCA9570_I2C_ADDR        0x24
#define PCA9570_BIT_LED_CLK_EN  0   // P0 -> IC2 ~1OE: LOW = SCL passed to LED clock
#define PCA9570_BIT_TEMP_SEL_A0 1   // P1 -> U5 TMUX1204 A0
#define PCA9570_BIT_TEMP_SEL_A1 2   // P2 -> U5 TMUX1204 A1
#define PCA9570_BIT_IR_EN       3   // P3 -> Q1 (BSS138) gate; HIGH = IR1 on (100R R5)

/* ---------------------------------------------------------------------------
 * ESCs (ESCs1 JST-SH 6-pin, 100R series, RCLAMP3346P ESD on U7)
 * ------------------------------------------------------------------------- */
#define PIN_DSHOT_1             7   // D8      net /Top_Board/Dshot1    -> ESCs1.5
#define PIN_DSHOT_2             8   // D9      net /Top_Board/Dshot2    -> ESCs1.2
#define PIN_ESC_TELEM_1         44  // D7      net /Top_Board/ESC_Telem1 <- ESCs1.6
#define PIN_ESC_TELEM_2         9   // D10     net /Top_Board/ESC_Telem2 <- ESCs1.1

/* ---------------------------------------------------------------------------
 * RC receiver (Receiver1, BetaFPV ELRS Lite footprint; 100R series, ESD on U6).
 * Net names are from the receiver's point of view: Receiver_TX is the
 * receiver's output, so it is the MCU's UART RX.
 * ------------------------------------------------------------------------- */
#define PIN_RECEIVER_UART_RX    43  // D6      net /Top_Board/Receiver_TX (receiver -> MCU)
#define PIN_RECEIVER_UART_TX    6   // D5      net /Top_Board/Receiver_RX (MCU -> receiver)

/* ---------------------------------------------------------------------------
 * LEDs (Conn_LEDs1 JST-SH 4-pin: GND, DATA, CLK, 5V) for an SK9822/APA102 strip.
 * Both lines go through a 74AHCT2G125 (IC2) 3V3 -> 5V level shifter.
 *   DATA: GPIO40 -> IC2 2A/2Y (always enabled, ~2OE grounded)
 *   CLK:  the I2C SCL line -> IC2 1A/1Y, enabled by PCA9570 P0 = LOW
 * So the LED clock is bit-banged on PIN_I2C_SCL with the gate open, and the
 * gate must be closed again before the bus is used for I2C.
 * ------------------------------------------------------------------------- */
#define PIN_LED_DATA            40  // MTDO    net /Top_Board/LEDs
#define PIN_LED_CLK             PIN_I2C_SCL

/* ---------------------------------------------------------------------------
 * Analog sensing (both on ADC1)
 * ------------------------------------------------------------------------- */
#define PIN_VBAT_SENSE          5   // D4      net /Top_Board/Vbat_Sense (ADC1_CH4)
#define VBAT_DIVIDER_RATIO      11.0f // (20k + 2k) / 2k

#define PIN_TEMP_SENSE          4   // D3      net /Top_Board/Sense (ADC1_CH3), 1k R14 to 3V3
#define TEMP_SENSE_PULLUP_OHMS  1000.0f
// TMUX1204 (U5) channel select via PCA9570 P1 (A0) / P2 (A1): S = 1 + A1*2 + A0
#define TEMP_MUX_CH_REF         0   // S1: 4.7k (R19) to GND, calibration reference
#define TEMP_MUX_CH_RTD_ONBOARD 1   // S2: RTD_Onboard1
#define TEMP_MUX_CH_TEMP2       2   // S3: Temps1 pin 2 (100R R17)
#define TEMP_MUX_CH_TEMP1       3   // S4: Temps1 pin 3 (100R R18)

/* ---------------------------------------------------------------------------
 * Legacy names from the V3.0 header, kept only so older sources still build.
 * None of these nets exist on V3.1; they point at unconnected JTAG pads.
 * Remove the users rather than relying on these.
 * ------------------------------------------------------------------------- */
#define PIN_RECEIVER_TX         PIN_RECEIVER_UART_RX   // old name, same net
#define PIN_RECEIVER_RX         PIN_RECEIVER_UART_TX
#define IMU_U5_I2C_ADDR         IMU_U4_I2C_ADDR        // old ref for the 0x6A part
#define PIN_I2C_SEL             41  // MTDI, unconnected (no I2C mux on V3.1)
#define I2C_SEL_LEVEL_IMU       0
#define I2C_SEL_LEVEL_EXTRA     1
#define PIN_TEMP_SEL_1          39  // MTCK, unconnected (select is on the PCA9570 now)
#define PIN_TEMP_SEL_2          42  // MTMS, unconnected
