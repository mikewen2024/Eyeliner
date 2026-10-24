/**
 * @file config.h
 * @brief Eyeliner firmware: build options, task layout and tuning constants.
 *
 * Build-time switches (set with -D in platformio.ini, [env:eyeliner]):
 *   DISPLAY_SERIAL  print telemetry over USB serial
 *   MOTORS          actually drive the ESCs (otherwise the control loop runs dry)
 *   ESP_NOW         bring up the ESP-NOW link on the comms core
 *
 * Core layout (ESP32-S3 has cores 0 and 1):
 *   core 1  control task: heading estimate + motor control, nothing else
 *   core 0  comms task:   receiver, telemetry, ESP-NOW, LEDs, housekeeping
 * Arduino's own loopTask also lives on core 1; main.cpp deletes it after setup.
 */
#pragma once

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Tasks
 * ------------------------------------------------------------------------- */
#define CONTROL_TASK_CORE      1
#define CONTROL_TASK_PRIORITY  10
#define CONTROL_TASK_STACK     8192

#define COMMS_TASK_CORE        0
#define COMMS_TASK_PRIORITY    5
#define COMMS_TASK_STACK       8192

/* ---------------------------------------------------------------------------
 * Receiver (CRSF / ELRS on UART1, pins from board_pins.h)
 * ------------------------------------------------------------------------- */
#define RX_UART_NUM            1
#define RX_BAUD                420000

// Failsafe: no channel frames for this long -> sticks revert to defaults and
// the control loop streams stop frames. Same 2 s as the old receiver code.
#ifndef RX_TIMEOUT_US
#define RX_TIMEOUT_US          2000000UL
#endif

// Stick -> [-1, 1]: (us - 1500) * 0.002, clamped. Same as the old remapInputs().
#define RX_CENTER_US           1500
#define RX_SCALE_PER_US        0.002f

// How often the DISPLAY_SERIAL receiver line is printed
#define RX_TELEM_PERIOD_MS     100

/* ---------------------------------------------------------------------------
 * Receiver channel map (see include/main.h and the transmitter setup)
 *   CH1 right stick x (2000 = full right)   CH5 SE
 *   CH2 right stick y (2000 = full up)      CH6 SA, clock mode
 *   CH3 left stick y  (2000 = full up)      CH7 SB, 3 position
 *   CH4 left stick x  (2000 = full right)   CH8 SC, 3 position
 *                                           CH9 SD, single      CH10 S1, 2 position
 * ------------------------------------------------------------------------- */
#define RX_CH_RIGHT_X          1
#define RX_CH_RIGHT_Y          2
#define RX_CH_LEFT_Y           3
#define RX_CH_LEFT_X           4
#define RX_CH_SE               5
#define RX_CH_SA               6
#define RX_CH_SB               7
#define RX_CH_SC               8
#define RX_CH_SD               9
#define RX_CH_S1               10

/* ---------------------------------------------------------------------------
 * IMUs: two LSM6DSV320X on I2C (pins/addresses from board_pins.h), read by the
 * comms task on core 0 and published for the control core.
 * ------------------------------------------------------------------------- */
// How often both IMUs are read (both in one go). Same 20 ms as the old
// UPDATE_INTERVAL_MICROS in the heading code.
#ifndef IMU_READ_INTERVAL_MS
#define IMU_READ_INTERVAL_MS   20
#endif

// LSM6DSV320X supports fast mode plus; 1 MHz is proven on this board
#ifndef IMU_I2C_FREQ_HZ
#define IMU_I2C_FREQ_HZ        1000000
#endif

// Sensor output rates. They only need to beat the read interval; higher ODR
// means a fresher sample at each read at the cost of more noise bandwidth.
#ifndef IMU_XL_ODR
#define IMU_XL_ODR             LSM6DSV320X_ODR_AT_480Hz
#endif
#ifndef IMU_GY_ODR
#define IMU_GY_ODR             LSM6DSV320X_ODR_AT_480Hz
#endif
#ifndef IMU_HG_ODR
#define IMU_HG_ODR             LSM6DSV320X_HG_XL_ODR_AT_480Hz
#endif

// How often the DISPLAY_SERIAL IMU lines are printed
#define IMU_TELEM_PERIOD_MS    200

/* ---------------------------------------------------------------------------
 * Heading estimate (control task, core 1). Port of LSM6DSV320XTR_Heading.ino.
 *
 * Spin rate from centripetal acceleration on one radial axis of one IMU:
 *   rate [deg/s] = K * sqrt(|a|)        a in m/s^2
 * where K = (180/pi) * sqrt(1000 / r_mm) for an IMU r_mm from the spin axis
 * (r = 50 mm -> K = 256). K is set live by S1 (CH10): full down = HEADING_K_MIN,
 * full up = HEADING_K_MAX, linear in between. Heading integrates rate at the
 * control loop rate. The 320 g high-g channel saturates at ~2400 rpm at 50 mm.
 * ------------------------------------------------------------------------- */
#ifndef HEADING_IMU_INDEX
#define HEADING_IMU_INDEX        0      // 0 = 0x6A (U4), 1 = 0x6B (U3)
#endif
#ifndef HEADING_AXIS
#define HEADING_AXIS             1      // 0 = X, 1 = Y: the radial axis (Z is up)
#endif
#ifndef HEADING_USE_HIGH_G
#define HEADING_USE_HIGH_G       1      // 1 = 320 g channel (needed above ~540 rpm), 0 = 16 g
#endif
#ifndef HEADING_K_MIN
#define HEADING_K_MIN            244.0f // deg/s per sqrt(m/s^2); r = 55 mm
#endif
#ifndef HEADING_K_MAX
#define HEADING_K_MAX            270.0f // r = 45 mm
#endif
#ifndef HEADING_ACCEL_DEAD_ZONE
#define HEADING_ACCEL_DEAD_ZONE  4.0f   // m/s^2 below this counts as not spinning
#endif
#ifndef HEADING_LOW_PASS_RATIO
#define HEADING_LOW_PASS_RATIO   0.05f  // applied once per new IMU sample (every IMU_READ_INTERVAL_MS)
#endif

// How often the DISPLAY_SERIAL heading line is printed (by the comms core)
#define HEADING_TELEM_PERIOD_MS  100

/* ---------------------------------------------------------------------------
 * Motor control (control task, core 1). Port of Motor_Control.ino: stopLogic.
 *
 * stopLogic reads three registers the control loop fills in each cycle:
 *   spinup       left stick x, [-1, 1]      (signed spin throttle)
 *   headingCos   heading . right stick dir  ([-1, 1], 0 when stick centred)
 *   translation  |right stick| from centre  ([0, 1])
 * and streams one DShot frame per ESC per call. Frames go out only with MOTORS
 * defined; the maths runs either way so telemetry shows what would be sent.
 * ------------------------------------------------------------------------- */
// Motor direction for a positive (forward) throttle, per motor: +1 or -1.
// Set empirically; everything below is computed before these are applied.
#ifndef MOTOR_1_DIRECTION
#define MOTOR_1_DIRECTION          1      // ESC on PIN_DSHOT_1
#endif
#ifndef MOTOR_2_DIRECTION
#define MOTOR_2_DIRECTION          1      // ESC on PIN_DSHOT_2
#endif

// Drive mode from the 3-position switch SB (CH7):
//   position 1 (low)  melty: motors opposite, +spinup / -spinup, stop logic on
//   position 2 (mid)  tank:  right stick y drives, x turns
//   position 3 (high) melty flipped: position 1 with both motors negated
#ifndef MOTOR_MODE_CHANNEL
#define MOTOR_MODE_CHANNEL         RX_CH_SB
#endif
#ifndef MOTOR_MODE_LOW_US
#define MOTOR_MODE_LOW_US          1250   // below: position 1
#endif
#ifndef MOTOR_MODE_HIGH_US
#define MOTOR_MODE_HIGH_US         1750   // above: position 3, else position 2
#endif
#ifndef MOTOR_MODE_POS1
#define MOTOR_MODE_POS1            MOTOR_MODE_MELTY
#endif
#ifndef MOTOR_MODE_POS2
#define MOTOR_MODE_POS2            MOTOR_MODE_TANK
#endif
#ifndef MOTOR_MODE_POS3
#define MOTOR_MODE_POS3            MOTOR_MODE_MELTY_FLIPPED
#endif

// Melty spin throttle input: the throttle-style left stick y (rests at ~990
// us). Below MOTOR_SPINUP_MIN_US is the dead zone (spinup = 0, zero frames, so
// the ESCs can arm); [MIN_US, MAX_US] maps linearly to [0, 1] from the threshold.
#ifndef MOTOR_SPINUP_CHANNEL
#define MOTOR_SPINUP_CHANNEL       RX_CH_LEFT_Y
#endif
#ifndef MOTOR_SPINUP_MIN_US
#define MOTOR_SPINUP_MIN_US        1100
#endif
#ifndef MOTOR_SPINUP_MAX_US
#define MOTOR_SPINUP_MAX_US        2000
#endif

// Tank mode (right stick). Each axis is 0 within 1500 +- TANK_DEAD_ZONE_US and
// scales linearly from 0 at the dead zone edge to 1 at full stick; then
//   left  = DRIVE * y + TURN * x,  right = DRIVE * y - TURN * x
// clamped to [-1, 1] and multiplied by TANK_MAX_THROTTLE. Negative gains
// reverse that axis. Motor 1 is the left motor, motor 2 the right.
#ifndef TANK_DEAD_ZONE_US
#define TANK_DEAD_ZONE_US          50
#endif
#ifndef TANK_MAX_THROTTLE
#define TANK_MAX_THROTTLE          0.30f  // full stick in tank mode (also under MOTOR_MAX_THROTTLE)
#endif
#ifndef TANK_GAIN_DRIVE
#define TANK_GAIN_DRIVE            1.0f
#endif
#ifndef TANK_GAIN_TURN
#define TANK_GAIN_TURN             0.5f
#endif

// Kill switch: SC (CH8) above this pulse width stops both motors (stop frames)
// and puts the LEDs in failsafe colour. Set above 2100 to disable.
#ifndef MOTOR_KILL_CHANNEL
#define MOTOR_KILL_CHANNEL         RX_CH_SC
#endif
#ifndef MOTOR_KILL_US
#define MOTOR_KILL_US              1800
#endif

#ifndef MOTOR_HEADING_THRESHOLD
#define MOTOR_HEADING_THRESHOLD    0.96f  // |cos| above this: stick direction is aligned with heading
#endif
#ifndef MOTOR_TRANSLATION_DEAD_ZONE
#define MOTOR_TRANSLATION_DEAD_ZONE 0.1f  // on D (see MOTOR_TRANSLATION_SQUARED)
#endif
#ifndef MOTOR_TRANSLATION_SQUARED
#define MOTOR_TRANSLATION_SQUARED  1      // 1 = D is |stick|^2 like the old code ("not square rooted!"), 0 = |stick|
#endif
#ifndef MOTOR_SPINUP_DEAD_ZONE
#define MOTOR_SPINUP_DEAD_ZONE     0.1f   // no modulation while |spinup| is below this
#endif
#ifndef MOTOR_AGGRESSION
#define MOTOR_AGGRESSION           1.0f   // 0..1, how hard dips/spikes push towards the limits
#endif
#ifndef MOTOR_REVERSE_THROTTLE
#define MOTOR_REVERSE_THROTTLE     1      // 1 = braking side may reverse, 0 = only dips
#endif
#ifndef MOTOR_MAX_THROTTLE
#define MOTOR_MAX_THROTTLE         0.30f  // hard clamp on |throttle| per motor in every mode; 1.0 for driving
#endif

// DShot: AM32 in 3D mode (48..1047 reverse, 1049..2047 forward, 0 stop).
// No pull-ups on the signal lines, so no bidirectional eRPM. AM32 locks the
// frame rate at detection: changing MOTOR_DSHOT_RATE needs an ESC power cycle.
#ifndef MOTOR_DSHOT_RATE
#define MOTOR_DSHOT_RATE           DSHOT1200 // DSHOT300, DSHOT600 or DSHOT1200 (all verified against AM32)
#endif
#ifndef MOTOR_DSHOT_BIDIR
#define MOTOR_DSHOT_BIDIR          0
#endif
#ifndef MOTOR_POLES
#define MOTOR_POLES                14
#endif

// How often the DISPLAY_SERIAL motor line is printed (by the comms core)
#define MOTOR_TELEM_PERIOD_MS      100

/* ---------------------------------------------------------------------------
 * LED strip (SK9822 on Conn_LEDs1), comms task on core 0.
 * DATA = GPIO40; CLK = the I2C SCL line through the 74AHCT2G125, enabled by
 * PCA9570 P0 = LOW. A frame is sent only when the pattern changes (or every
 * LED_REFRESH_MS as a keepalive), since each one costs ~0.5 ms of bus time.
 *
 * Three patterns, each colour a macro (R, G, B; LED_OFF turns one off):
 *   1. melty / drift (SB low or high): LED_COLOR_ARC while the heading is inside
 *      LED_ARC_CENTER_DEG +- LED_ARC_HALF_WIDTH_DEG, off elsewhere
 *   2. tank (SB middle):               LED_COLOR_TANK, constant
 *   3. failsafe / init:                LED_COLOR_FAILSAFE, constant, whenever the
 *      motors are being killed (no receiver signal, kill switch) or not armed yet
 * ------------------------------------------------------------------------- */
#define LED_OFF                    0, 0, 0
#define LED_WHITE                  255, 255, 255
#define LED_RED                    255, 0, 0
#define LED_GREEN                  0, 255, 0
#define LED_BLUE                   0, 0, 255

#ifndef LED_COUNT
#define LED_COUNT                  5      // LEDs on the strip (over-counting is harmless)
#endif
#ifndef LED_BRIGHTNESS
#define LED_BRIGHTNESS             8      // SK9822 global brightness, 0..31
#endif
#ifndef LED_HALF_PERIOD_US
#define LED_HALF_PERIOD_US         1      // bit-bang half clock period
#endif
#ifndef LED_REFRESH_MS
#define LED_REFRESH_MS             250    // keepalive resend of an unchanged pattern
#endif
#ifndef LED_ARC_CENTER_DEG
#define LED_ARC_CENTER_DEG         135.0f // "forward" heading, from the old indicateHeading (125..145)
#endif
#ifndef LED_ARC_HALF_WIDTH_DEG
#define LED_ARC_HALF_WIDTH_DEG     10.0f
#endif
#ifndef LED_COLOR_ARC
#define LED_COLOR_ARC              LED_WHITE
#endif
#ifndef LED_COLOR_TANK
#define LED_COLOR_TANK             LED_BLUE
#endif
#ifndef LED_COLOR_FAILSAFE
#define LED_COLOR_FAILSAFE         LED_RED
#endif

// PCA9570 outputs other than the LED gate, written once at LED init:
#ifndef PCA9570_IR_ON
#define PCA9570_IR_ON              0      // P3 -> Q1 -> IR1 emitter (power-on default is ON)
#endif
#ifndef PCA9570_TEMP_MUX_CH
#define PCA9570_TEMP_MUX_CH        TEMP_MUX_CH_REF
#endif

/* ---------------------------------------------------------------------------
 * Battery / chip monitoring (comms task, core 0)
 * Vbat_Sense is VCC_HV through a 20k/2k divider on GPIO5 (ADC1, so it works
 * with WiFi on). The chip temperature is the ESP32-S3's internal sensor.
 * No current-sense hardware exists for 3V3 or 5V on the XIAO or the board.
 * ------------------------------------------------------------------------- */
#ifndef VBAT_INTERVAL_MS
#define VBAT_INTERVAL_MS           1000   // Vbat_Sense ADC read + divider maths every N ms
#endif
#ifndef VBAT_RATIO
#define VBAT_RATIO                 VBAT_DIVIDER_RATIO  // 11.0 from board_pins.h
#endif

/* ---------------------------------------------------------------------------
 * ESP-NOW telemetry (comms task, core 0; only built with ESP_NOW defined).
 * Broadcast, two packed little-endian frame types, see telemetry.h:
 *   group 1 "fast": heading, stop-logic registers, motor outputs, link stats,
 *                   both IMUs                       every ESPNOW_FAST_INTERVAL_MS
 *   group 2 "slow": battery, chip temperature, loop and bus statistics
 *                                                   every ESPNOW_SLOW_INTERVAL_MS
 * ------------------------------------------------------------------------- */
#ifndef ESPNOW_WIFI_CHANNEL
#define ESPNOW_WIFI_CHANNEL        6      // same as the old ESP_Now.ino
#endif
#ifndef ESPNOW_FAST_INTERVAL_MS
#define ESPNOW_FAST_INTERVAL_MS    10     // group 1, 100 Hz
#endif
#ifndef ESPNOW_SLOW_INTERVAL_MS
#define ESPNOW_SLOW_INTERVAL_MS    1000   // group 2, 1 Hz
#endif
// Telemetry on/off from the transmitter: SA (CH6) above TELEM_ENABLE_US sends,
// below stops the frames and the WiFi driver (the radio warms the module by
// ~10 C). Set TELEM_ENABLE_US to 0 to always send, or above 2100 to never send.
#ifndef TELEM_ENABLE_CHANNEL
#define TELEM_ENABLE_CHANNEL       RX_CH_SA
#endif
#ifndef TELEM_ENABLE_US
#define TELEM_ENABLE_US            1500
#endif
#ifndef ESPNOW_TX_POWER
#define ESPNOW_TX_POWER            WIFI_POWER_19_5dBm  // wifi_power_t (max); no ELRS LQ impact seen on the bench
#endif
