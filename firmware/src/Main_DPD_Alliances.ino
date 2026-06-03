/* Libraries Used */
// ADXL library and sensor libraries used
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>

// LSM library and sensor libraries used
#include <Wire.h>
#include <SPI.h>

// Motor control -- DShot! 
#include <Arduino.h>
#include <DShotRMT.h>

// Motor control (currently not DShot, just PWM)
#include <ESP32Servo.h> 

#include <FastLED.h>

// For modf
#include <cmath>

/* Conditional Macros */
// #define DISPLAY_SERIAL // Runs serial outputs
#undef DISPLAY_SERIAL
// #define ESP_NOW // Runs ESP as transmitter for ESP-Now
#undef ESP_NOW
#define MOTORS // Enables motors
// #undef MOTORS

/* Forward Declarations */
// From FS2A_Receiver.ino
bool setupReceiver();
void updateTiming();
bool readChannels();
void remapInputs();

// From ADXL375_Heading.ino
bool setupHeading();
bool updateHeading(float COR_Input);
float updateHeadingDotProduct(float throttle_x, float throttle_y);
void indicateHeading();

// From Motor_Control.ino
// bool setupMotorsPWM();
bool setupMotorsDshot(); 
bool stopLogic(float throttle_spinup, float throttle_x, float throttle_y, float heading_cosine);
// void killMotorsPWM(); 
void killMotorsDshot(); 

// Tracks whether or not setup was successful
bool setupSuccessful = true;

/* Dynamic variables and objects from each file */
// From ADXL375_Heading.ino
/* In-code objects */
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
sensors_event_t event;

/* Dynamically-Updated Variables */
// Accelerometer reading
unsigned long last_read = 0; // Last time accelerometer was read in
float raw_acceleration = 0.0f;
float processed_acceleration = 0.0f;

// COR
float radius_of_rotation = 0.0f; // Radius of rotation in millimeters
float last_radius_of_rotation = 0.0f;

// Outputs
float degrees_per_second = 0.0f; // Rotation rate
float previous_degrees_per_second = 0.0f; // Last measured rotation rate for 
float heading_degrees = 0.0f; // Should only be positive for now

// Auxillary variables for calculating dot product b/w heading and throttle
float heading_x = 0.0f;
float heading_y = 0.0f;
float throttle_magnitude = 0.0f;
float heading_translation_dot_product = 0.0f;

// From FS2A_Receiver.ino
// Channel-specific timing deltas
// Available denotes "available to be read-in"
volatile unsigned long channelBeginTimes[4];
volatile unsigned long channelEndTimes[4];
volatile bool channelAvailabilities[4];

// Raw channel data and last channel data information for low-pass
// Channel 1: Left X, left = 1000, right = 2000
// Channel 2: Left Y, down = 1000, up = 2000
// Channel 3: Right Y, down = 1000, up = 2000
// Channel 4: Right X, left = 1000, right = 2000
int rawChannelData[4]; 
int lastChannelData[4]; // No low-pass filter implemented for now, not needed
float remappedChannelData[4]; // [-1, 1] Range
int channelDefaults[4];

// General timing deltas
int current_micros = 0.0f;
int last_micros = 0.0f;
int delta_micros = 0.0f;
float ticks_per_second = 0.0f; // Only calculate if needed for telemetry
bool safe_to_read = false;

// From Motor_Control.ino
// This determines the threshold on the cosine b/w translation direction and heading to trigger braking
#define HEADING_THRESHOLD 0.96f

// Dynamically-updated variables
// Translational throttle magnitude, has a dead zone
float D = 0.0f;
float M = 0.0f;
float Modulation = 0.0f;
float Amplitude_Modulation = 0.0f;
float Motor_1_Throttle = 0.0f;
float Motor_2_Throttle = 0.0f;

// Setup :)
void setup() {
  #ifdef DISPLAY_SERIAL
    Serial.begin(115200); // Start serial communication
  #endif

  // Setup receiver channels

  // Uncomment for FS2A
  // if (!setupReceiver()) setupSuccessful = false;

  // Uncomment for ELRS
  setupELRSReceiver();

  #ifdef MOTORS
    // Motor setup -- choose setup! 
    // if (!setupMotorsPWM()) setupSuccessful = false;
    if (!setupMotorsDshot()) setupSuccessful = false;
  #endif

  // Accelerometer and heading indication setup

  // Uncomment for ADXL
  // if (!setupHeading()) setupSuccessful = false;

  // Uncomment for LSM
  if (!setupHeadingLSM()) setupSuccessful = false;

  #ifdef DISPLAY_SERIAL
      Serial.println("Setup Finished :)");
  #endif
}

void loop() {
    // Update timing and receiver inputs first
    updateTiming();
    // if (!readChannels()) {
    //   killMotors();
    //   return;
    // }
    if (!readChannelsELRS()) {
      killMotorsDshot();
      // killMotorsPWM(); 
      return;
    }
    remapInputs();

    // Handle accelerometer updates
    // updateHeading(remappedChannelData[1]);
    updateHeadingLSM(remappedChannelData[1]);
    updateHeadingDotProductLSM(remappedChannelData[3], remappedChannelData[2]);
    indicateHeadingLSM();
    
    // Motor control code
    stopLogic(remappedChannelData[0], remappedChannelData[3], remappedChannelData[2], heading_translation_dot_product);
}