/* Hardware-Related Constants */
// #define INDICATOR1 8 // Blue
#define ADXL_X_ACCEL_MANUAL_OFFSET -0.6
#define ADXL_Y_ACCEL_MANUAL_OFFSET -0.421
#define ADXL_Z_ACCEL_MANUAL_OFFSET 2.0
#define ACCELERATION_DEAD_ZONE 3.75 // Bounds absolute value of adjusted acceleration, m/s^2
#define LOW_PASS_RATIO 0.05
#define UPDATE_INTERVAL_MICROS 20000 // How often to update accelerometer, we use a simple low-pass filter

/* FASTLED constants, variables */
#define NUM_LEDS 5
#define DATA_PIN1 10 //67 :(((
#define DATA_PIN2 40 
CRGB leds_top[NUM_LEDS];
CRGB leds_bottom[NUM_LEDS];

/* Physical constants */
#define DEGREES_PER_RADIAN 57.2957f
#define RADIANS_PER_DEGREE 0.0174533f

/* Main Functions */
// Setup accelerometer and indicator LEDs
// No checksums, base code needs robot to be stationary during startup and calibration
bool setupHeading() {
  // Setup wire (I2C)
  Wire.begin();
  Wire.setClock(400000);
  
  // Setup the object in code
  accel.begin();
  accel.setTrimOffsets(-3, -1, 1);

  // Indicator LEDs
  // pinMode(INDICATOR1, OUTPUT);
  // pinMode(INDICATOR2, OUTPUT);
  // pinMode(LED_BUILTIN, OUTPUT);

  // FASTLED setup: 
  FastLED.addLeds<WS2812B, DATA_PIN1, GRB>(leds_top, NUM_LEDS);
  FastLED.addLeds<WS2812B, DATA_PIN2, GRB>(leds_bottom, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

  // Set up timing
  unsigned long last_read = micros(); // Last time accelerometer was read in

  // No checksums :)
  return true;
}

// Indicate heading
void indicateHeading() {
  // Heading LED based on perceived "forward"
  if (heading_degrees < 125.0f && heading_degrees > 145.0f) { // Should point in "forward" direction
      // digitalWrite(INDICATOR1, HIGH);
      // digitalWrite(INDICATOR2, HIGH);
      // digitalWrite(LED_BUILTIN, LOW);

      //fastLEDs ON: 
      for (int i = 0; i < NUM_LEDS; i++) {
        leds_top[i] = CRGB::Red; 
        leds_bottom[i] = CRGB::Green; 
      }
      FastLED.show();
  }

  else {
      // digitalWrite(INDICATOR1, LOW);
      // // digitalWrite(INDICATOR2, LOW);
      // digitalWrite(LED_BUILTIN, HIGH);

      //fastLEDs OFF:
      for (int i = 0; i < NUM_LEDS; i++) {
        leds_top[i] = CRGB::Black; 
        leds_bottom[i] = CRGB::Black; 
      }
      FastLED.show();
  }
}

// Updates heading
bool updateHeading(float COR_Input) { // Input in [-1, 1]
  // Get ADXL 375 inputs if 20ms has passed
  if (micros() - last_read > UPDATE_INTERVAL_MICROS) {
    accel.getEvent(&event);
    raw_acceleration = event.acceleration.z;
    last_read = micros();
  }

  // Update radius of rotation
  radius_of_rotation = 5.0f * COR_Input + 50.0f;
  if (radius_of_rotation == 50.0f) radius_of_rotation = last_radius_of_rotation;
  if (radius_of_rotation != 50.0f) last_radius_of_rotation = radius_of_rotation;

  // Apply dead zone and convert to degrees_per_second
  processed_acceleration = (abs(raw_acceleration + ADXL_Z_ACCEL_MANUAL_OFFSET) < ACCELERATION_DEAD_ZONE)? 0.0f : (raw_acceleration + ADXL_Z_ACCEL_MANUAL_OFFSET);
  degrees_per_second = DEGREES_PER_RADIAN * sqrt(abs(processed_acceleration) * (1000.0f) / radius_of_rotation);

  // Start counting the new degrees_per_second
  previous_degrees_per_second = degrees_per_second;

  // Calculate new heading
  heading_degrees = heading_degrees + degrees_per_second * (float) delta_micros / (1000000.0f);

  // Mod it
  heading_degrees = fmod(heading_degrees, 360.0f);

  // Serial outputs
  #ifdef DISPLAY_SERIAL
    // Serial.print(raw_acceleration + ADXL_Z_ACCEL_MANUAL_OFFSET);
    // Serial.print(",");
    // Serial.print(radius_of_rotation);
    // Serial.print(",");
    // Serial.print(degrees_per_second);
    // Serial.print(",");
    // Serial.println(heading_degrees);
  #endif

  // No checksums :)
  return true;
}

// Calculate dot product b/w heading and translational throttle
// Inputs are in [-1, 1]
float updateHeadingDotProduct(float throttle_x, float throttle_y) {
  // Limiting factor for speed is I2C reads, so we don't really need lookup tables here
  // Calculate vector for heading
  heading_x = cos(RADIANS_PER_DEGREE * heading_degrees);
  heading_y = sin(RADIANS_PER_DEGREE * heading_degrees);

  // Normalize throttle coordinates
  throttle_magnitude = sqrt(throttle_x*throttle_x + throttle_y*throttle_y);
  float throttle_x_normalized = throttle_x;
  float throttle_y_normalized = throttle_y;
  if (throttle_magnitude != 0.0f) {
    throttle_x_normalized /= throttle_magnitude;
    throttle_y_normalized /= throttle_magnitude;
  }

  // Calculate dot product b/w normalized throttle and return it
  heading_translation_dot_product = throttle_x_normalized*heading_x + throttle_y_normalized*heading_y;

  return heading_translation_dot_product;
}