// Motor control (not DShot)
#include <ESP32Servo.h> 

// MPU6050 interface
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ADXL 375 interface
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>

// Attachinterrupts
// #include "PinChangeInterrupt.h"

// Conditional Macros
// #define DISPLAY_SERIAL // Runs serial outputs
#undef DISPLAY_SERIAL
#define ESP_NOW // Runs ESP as transmitter for ESP-Now
// #undef ESP_NOW
#define MOTORS // Enables motors
// #undef MOTORS

// Timing and other basic features
int current_micros;
int last_micros;
int delta_micros;
double ticks_per_second;
#define TIMEOUT_MICROS 500000UL // 500 milliseconds

// Currently not using MPU
// MPU6050 setup
// MPU6050 gyro;

// MPU6050 Offsets:
#define XGYRO_MANUAL_OFFSET 204
#define YGYRO_MANUAL_OFFSET 36
#define ZGYRO_MANUAL_OFFSET -38

// Indicator LEDs
#define INDICATOR1 10 // Red
#define INDICATOR2 9 // Red

// ADXL 375 Setup (I2C, not SPI)
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

// Manual ADXL 375 Offsets (m/s^2)
#define ADXL_X_ACCEL_MANUAL_OFFSET -0.6
#define ADXL_Y_ACCEL_MANUAL_OFFSET -0.421
#define ADXL_Z_ACCEL_MANUAL_OFFSET 2.0
#define ACCELERATION_DEAD_ZONE 3.75 // Bounds absolute value of adjusted acceleration, m/s^2
#define LOW_PASS_RATIO 0.05
double DEGREES_PER_RADIAN = 57.2957d;
double RADIANS_PER_DEGREE = 0.0174533d;
double raw_acceleration;
double radius_of_rotation; // Radius of rotation in millimeters
double last_radius_of_rotation;
double degrees_per_second; // Rotation rate
double previous_degrees_per_second; // Last measured rotation rate for 
double heading_degrees; // Should only be positive for now

// Initializing stoplogic() function variables
double heading_x = 0.0d;
double heading_y = 0.0d;
double throttle_magnitude = 0.0d;
double throttle_x_normalized = 0.0d;
double throttle_y_normalized = 0.0d;
double throttle_multiplier_1 = 0.0d;
double throttle_multiplier_2 = 0.0d;

// Define PWM input pins for each channel
#define CHANNEL_1 1 // Left X, left = 1000, right = 2000
// Start and end times for a pulse
volatile unsigned long ch1Begin = micros();
volatile unsigned long ch1End = micros();
volatile bool ch1Available = false;
void ch1Interrupt(){
  if (digitalRead(CHANNEL_1) == HIGH) {
    ch1Begin = micros();
    ch1Available = false;
  } else {
    ch1End = micros();
    ch1Available = true;
  }
}

#define CHANNEL_2 3 // Left Y, down = 1000, up = 2000
// Start and end times for a pulse
volatile unsigned long ch2Begin = micros();
volatile unsigned long ch2End = micros();
volatile bool ch2Available = false;
void ch2Interrupt(){
  if (digitalRead(CHANNEL_2) == HIGH) {
    ch2Begin = micros();
    ch2Available = false;
  } else {
    ch2End = micros();
    ch2Available = true;
  }
}

#define CHANNEL_3 2  // Right Y, down = 1000, up = 2000
// Start and end times for a pulse
volatile unsigned long ch3Begin = micros();
volatile unsigned long ch3End = micros();
volatile bool ch3Available = false;
void ch3Interrupt(){
  if (digitalRead(CHANNEL_3) == HIGH) {
    ch3Begin = micros();
    ch3Available = false;
  } else {
    ch3End = micros();
    ch3Available = true;
  }
}

#define CHANNEL_4 4 // Right X, left = 1000, right = 2000
// Start and end times for a pulse
volatile unsigned long ch4Begin = micros();
volatile unsigned long ch4End = micros();
volatile bool ch4Available = false;
void ch4Interrupt(){
  if (digitalRead(CHANNEL_4) == HIGH) {
    ch4Begin = micros();
    ch4Available = false;
  } else {
    ch4End = micros();
    ch4Available = true;
  }
}

// Values for outputs to be stored in
int channel1 = 1500;
int channel2 = 1500;
int channel3 = 1500;
int channel4 = 1500;

// Stores last non-default values of channels 3 and 4
int channel1_nonzero = 1500;
int channel3_nonzero = 1500;
int channel4_nonzero = 1500;

// Stores time value for when 1000 (only in failsafe mode for channel 2) was last received on channel 2
int channel2_last_non_neutral = micros();

#ifdef MOTORS
    // ESCs
    // Servo esc1;
    Servo esc2;

    // ESC Pinouts
    // #define ESC_1 43
    #define ESC_2 44
#endif

double TRANSLATIONAL_ANGLE_BOUND = 90.0d; // ± this range for boolean
double MOTOR_BRAKING_THROTTLE_PERCENTAGE = 0.2d; // Using running brake currently
double AGGRESSION = 1.0; // Multiplier for faster translation?

// ESC settings
#define HIGH_SIGNAL 2000
#define LOW_SIGNAL 1000
#define NEUTRAL 1500

// Outputs between 0˚ and 360˚
double inverseTangent(double y, double x) {
    if (x == 0.0d) return ((y < 0.0d)? 270.0d : 90.0d);
    double output = ((DEGREES_PER_RADIAN * atan(y / x)) + ((x < 0.0d)? 180.0d : 0.0d));
    while (output < 0.0d) output += 360.0d;
    return output;
}

// Handles translation, assumes counterclockwise spinning
// Heading angle in degrees
// No lookup table yipeeeeeeeee
// We treat heading_angle as starting from 0 on the +x axis
void stopLogic(double throttle_x, double throttle_y, double heading_angle) {
  // Calculate heading unit vector
  heading_x = cos(RADIANS_PER_DEGREE * heading_angle);
  heading_y = sin(RADIANS_PER_DEGREE * heading_angle);

  // Perform dot product
  throttle_magnitude = sqrt(throttle_x*throttle_x + throttle_y*throttle_y);
  throttle_x_normalized = throttle_x;
  throttle_y_normalized = throttle_y;

  if (throttle_magnitude != 0.0d) {
    throttle_x_normalized /= throttle_magnitude;
    throttle_y_normalized /= throttle_magnitude;
  }

  if (throttle_magnitude < 30) { // Dead zone
    throttle_x_normalized = 0.0d;
    throttle_y_normalized = 0.0d;
  }

  // Now goes from -1 to +1
  throttle_multiplier_1 = throttle_x_normalized*heading_x + throttle_y_normalized*heading_y;
  throttle_multiplier_2 = -throttle_multiplier_1;

  // Wanted more translation so more aggressive profile
  throttle_multiplier_1 *= AGGRESSION;
  throttle_multiplier_2 *= AGGRESSION;

  // Renormalization sanity check
  if (abs(throttle_multiplier_1) > 2.0d) {
    throttle_multiplier_1 *= 2.0d / abs(throttle_multiplier_1);
  }
  if (abs(throttle_multiplier_2) > 2.0d) {
    throttle_multiplier_2 *= 2.0d / abs(throttle_multiplier_2);
  }

  // Gets compressed down to the right range
  throttle_multiplier_1 *= (1.0d - MOTOR_BRAKING_THROTTLE_PERCENTAGE) / 2.0d;
  throttle_multiplier_2 *= (1.0d - MOTOR_BRAKING_THROTTLE_PERCENTAGE) / 2.0d;
  // Range gets moved
  throttle_multiplier_1 += MOTOR_BRAKING_THROTTLE_PERCENTAGE + (1.0d - MOTOR_BRAKING_THROTTLE_PERCENTAGE) / 2.0d;
  throttle_multiplier_2 += MOTOR_BRAKING_THROTTLE_PERCENTAGE + (1.0d - MOTOR_BRAKING_THROTTLE_PERCENTAGE) / 2.0d;

  // Debugging
  #ifdef DISPLAY_SERIAL
    // Serial.print(heading_angle);
    // Serial.print(",");
    // Serial.print(heading_x);
    // Serial.print(",");
    // Serial.print(heading_y);
    // Serial.print(",");
    // Serial.print(throttle_x_normalized);
    // Serial.print(",");
    // Serial.print(throttle_y_normalized);
    // Serial.print(",");
    // Serial.print(throttle_multiplier_1);
    // Serial.print(",");
    // Serial.print(throttle_multiplier_2);
    // Serial.print(",");
    // Serial.print((int)((double)(min(max(channel1, LOW_SIGNAL), HIGH_SIGNAL) - 1500) * throttle_multiplier_1) + 1500);
    // Serial.print(",");
    // Serial.print((int)((double)(min(max(channel1, LOW_SIGNAL), HIGH_SIGNAL) - 1500) * throttle_multiplier_2) + 1500);
    // Serial.println();
  #endif

  // Heading LED based on direction of intended translation
  if (heading_degrees < 100 && heading_degrees > 80) { // Should point in "forward" direction
      digitalWrite(INDICATOR1, HIGH);
      digitalWrite(INDICATOR2, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
  } else {
      digitalWrite(INDICATOR1, LOW);
      digitalWrite(INDICATOR2, LOW);
      digitalWrite(LED_BUILTIN, HIGH);
  }

  // Set motor powers
  // Right (right of heading vector) wheel to throttle_multiplier_1
  // Left wheel to throttle_multiplier_2
  #ifdef MOTORS
    // esc1.writeMicroseconds((int)((double)(min(max(channel1, LOW_SIGNAL), HIGH_SIGNAL) - 1500) * throttle_multiplier_1) + 1500);
    esc2.writeMicroseconds((int)((double)(min(max(channel1, LOW_SIGNAL), HIGH_SIGNAL) - 1500) * throttle_multiplier_2) + 1500);
  #endif
}

void setup() {
    #ifdef DISPLAY_SERIAL
        Serial.begin(115200); // Start serial communication
    #endif

    // Setup receiver channels
    pinMode(CHANNEL_1, INPUT);
    pinMode(CHANNEL_2, INPUT);
    pinMode(CHANNEL_3, INPUT);
    pinMode(CHANNEL_4, INPUT);

    // Setup attachInterrupts
    attachInterrupt(digitalPinToInterrupt(CHANNEL_1), ch1Interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CHANNEL_2), ch2Interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CHANNEL_3), ch3Interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CHANNEL_4), ch4Interrupt, CHANGE);

    #ifdef MOTORS
        // Bidirectional: 1000 µs (full reverse), 1500 µs (neutral), 2000 µs (full forward)
        // esc1.attach(ESC_1, LOW_SIGNAL, HIGH_SIGNAL);
        delay(10); 
        esc2.attach(ESC_2, LOW_SIGNAL, HIGH_SIGNAL);
        delay(100); 

        // Calibrate by setting neutral since these are bidirectional
        #ifdef DISPLAY_SERIAL
            Serial.println("Calibrating Both ESCs");
        #endif
        // esc1.writeMicroseconds(NEUTRAL);
        delay(1000);
        esc2.writeMicroseconds(NEUTRAL);
        delay(1000);
    #endif

  // Set up timing and heading calculation variables
  current_micros = micros();
  last_micros = micros();
  delta_micros = 0;
  ticks_per_second = 0;
  raw_acceleration = 0.0d;
  radius_of_rotation = 0.01d;
  last_radius_of_rotation = 0.01d;
  degrees_per_second = 0.0d;
  previous_degrees_per_second = 0.0d;
  heading_degrees = 0.0d;

  // Set up ADXL 375 and/or MPU6050, we'll assume it's reliable enough LOL
  // Intended logic for later: if no ADXL375, initialize MPU accelerometer
  accel.begin();
  accel.setTrimOffsets(-3, -1, 1);
  // Either X or Z axes should work, can square root combined result for better accuracy but may not be worth the loss in overhead
  // w^2 is proportional to acceleration measured
  // When available, use ESP-Now to get the acceleration direction when fully stable in rotation, find formulas for getting radial vs tangential acceleration

    // Set up indicator LED, using the built in LED too
    pinMode(INDICATOR1, OUTPUT);
    pinMode(INDICATOR2, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    #ifdef DISPLAY_SERIAL
        Serial.println("ESCs calibrated :)");
    #endif
}

void loop() {
    // Update current time in clock
    current_micros = micros();
    delta_micros = current_micros - last_micros;
    ticks_per_second = (1000000.0d) / (double)delta_micros;
    
    // Read PWM pulse widths, has a goofy tendency to drop below 0 or go incredibly high so I've bounded the values to a reasonable range
    if (ch1Available && ((ch1End - ch1Begin) > 0) && ((ch1End - ch1Begin) < 2020)) channel1 = ch1End - ch1Begin;
    if (ch2Available && ((ch2End - ch2Begin) > 0) && ((ch2End - ch2Begin) < 2020)) channel2 = ch2End - ch2Begin;
    if (ch3Available && ((ch3End - ch3Begin) > 0) && ((ch3End - ch3Begin) < 2020)) channel3 = ch3End - ch3Begin;
    if (ch4Available && ((ch4End - ch4Begin) > 0) && ((ch4End - ch4Begin) < 2020)) channel4 = ch4End - ch4Begin;

    // Use channel 2 to check for fail-safe scenario
    if (abs(channel2 - 1000) > 10) channel2_last_non_neutral = micros();

    // Handle ADXL 375 inputs
    sensors_event_t event;
    accel.getEvent(&event);
    // Simplest mapping from receiver input to radius of rotation (left stick Y)
    // Around 2.80 mm for the 300 mAh 3s battery
    // Around 2.70 mm for the 380 mAh 3s battery
    // Lights were lighting up twice per rotation, might need to multiply all these by 4
    // 12.735
    radius_of_rotation = max(0.0d, (double)(channel2 - 1000.0d) * (1.0d) / (1000.0d)) + 6.5d;
    if (radius_of_rotation == 7.0d) radius_of_rotation = last_radius_of_rotation;
    if (radius_of_rotation != 7.0d) last_radius_of_rotation = radius_of_rotation;
    // Map the acceleration in the z direction to the rate of rotation
    raw_acceleration = event.acceleration.x;
    double processed_acceleration = (abs(raw_acceleration + ADXL_Z_ACCEL_MANUAL_OFFSET) < ACCELERATION_DEAD_ZONE)? 0 : (raw_acceleration + ADXL_Z_ACCEL_MANUAL_OFFSET);
    degrees_per_second = DEGREES_PER_RADIAN * sqrt(abs(processed_acceleration) * (1000.0d) / radius_of_rotation);
    // Low-pass filter for noise, 5% ratio for new data
    degrees_per_second *= LOW_PASS_RATIO;
    degrees_per_second += (1.0d - LOW_PASS_RATIO) * previous_degrees_per_second;
    // Start counting the new degrees_per_second
    previous_degrees_per_second = degrees_per_second;
    // Calculate new heading
    heading_degrees = heading_degrees + degrees_per_second * (double)delta_micros / (1000000.0d);
    // Mod it
    while (heading_degrees > 360.0d) heading_degrees -= 360.0d;
    
    // Print values to Serial Monitor
    #ifdef DISPLAY_SERIAL
        Serial.print("Channel 1 throttle: ");
        Serial.print(channel1);
        Serial.print(",");
        Serial.print("Channel 2 throttle: ");
        Serial.print(channel2);
        Serial.print(",");
        Serial.print("Channel 3 throttle: ");
        Serial.print(channel3);
        Serial.print(",");
        Serial.print("Channel 4 throttle: ");
        Serial.println(channel4);

        // Serial.print(event.acceleration.x + ADXL_X_ACCEL_MANUAL_OFFSET);
        // Serial.print(",");
        // Serial.print(event.acceleration.y + ADXL_Y_ACCEL_MANUAL_OFFSET);
        // Serial.print(",");
        // Serial.print(raw_acceleration + ADXL_Z_ACCEL_MANUAL_OFFSET);

        // Serial.print(delta_micros);
        // Serial.print(",");
        // Serial.println(ticks_per_second);

        // Serial.print(raw_acceleration + ADXL_Z_ACCEL_MANUAL_OFFSET);
        // Serial.print(",");
        // Serial.println(radius_of_rotation);
        // Serial.print(",");
        // Serial.print(degrees_per_second);
        // Serial.print(",");
        // Serial.println(heading_degrees);
    #endif

    // Handles failsafe scenario, using timeout system on channel 2
    if (micros() - channel2_last_non_neutral > TIMEOUT_MICROS) {
      channel1 = 1500;
      channel2 = 1000;
      channel3 = 1500;
      channel4 = 1500;
    }

    stopLogic((double) (channel4 - 1500), (double) (channel3 - 1500), heading_degrees);
    // esc1.writeMicroseconds(channel1);
    // esc2.writeMicroseconds(3000 - channel1);
    
    // Update last_time on timer
    last_micros = current_micros;
}