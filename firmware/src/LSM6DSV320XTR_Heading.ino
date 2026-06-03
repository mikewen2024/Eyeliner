/* Hardware-Related Constants */
// #define INDICATOR1 8 // Blue
#define ACCELERATION_DEAD_ZONE 4.0 // Bounds absolute value of adjusted acceleration, m/s^2
#define LOW_PASS_RATIO 0.05
#define UPDATE_INTERVAL_MICROS 20000 // How often to update accelerometer, we use a simple low-pass filter

/* Physical constants */
#define DEGREES_PER_RADIAN 57.2957f
#define RADIANS_PER_DEGREE 0.0174533f

/* LSM Setup */
extern "C" {
  #include "lsm6dsv320x_reg.h"
}

// I²C device address (default 0x6A if SDO = GND, 0x6B if SDO = VDD)
#define LSM6DSV320X_I2C_ADDRESS  0x6B

// ST driver requires a handle for communication
#define DEV_ADDR NULL

#define LSM_CS 3

SPISettings imuSPI(40000000, MSBFIRST, SPI_MODE0);

// --- Platform read/write functions for the ST driver ---
int32_t spi_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    (void)handle;

    SPI.beginTransaction(imuSPI);
    digitalWrite(LSM_CS, LOW);

    // For SPI write, reg address MSB must be 0 (auto-inc works)
    SPI.transfer(reg & 0x7F);

    for (uint16_t i = 0; i < len; i++) {
        SPI.transfer(bufp[i]);
	}

    digitalWrite(LSM_CS, HIGH);
    SPI.endTransaction();
    return 0;
}

int32_t spi_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    (void)handle;

    SPI.beginTransaction(imuSPI);
    digitalWrite(LSM_CS, LOW);

    // For SPI read, set MSB = 1
    SPI.transfer(reg | 0x80);

    for (uint16_t i = 0; i < len; i++) {
        bufp[i] = SPI.transfer(0x00);
	}

    digitalWrite(LSM_CS, HIGH);
    SPI.endTransaction();
    return 0;
}

void platform_delay(uint32_t ms) {
  delay(ms);
}

// --- Global driver context ---
stmdev_ctx_t dev_ctx;

/* Main Functions */
// Setup accelerometer and indicator LEDs
// No checksums, base code needs robot to be stationary during startup and calibration
bool setupHeadingLSM() {
  // I2C??
	Wire.begin(D2, D1); // SDA=D2 (GPIO4), SCL=D1 (GPIO5)
	Wire.setClock(400000L);

  // SPI CS Setup
	pinMode(LSM_CS, OUTPUT);
  digitalWrite(LSM_CS, HIGH);

  // Start SPI
	SPI.begin();

	// Initialize driver context
	dev_ctx.read_reg = spi_read;
	dev_ctx.write_reg = spi_write;
	dev_ctx.mdelay    = platform_delay;
	dev_ctx.handle    = DEV_ADDR;

	// Wait for sensor boot time
	delay(500);

  // ID
	uint8_t whoamI = 0;
  int failCounter = 0;
	while (whoamI != LSM6DSV320X_ID) {
		lsm6dsv320x_device_id_get(&dev_ctx, &whoamI);
		delay(500);

    if (++failCounter > 20) {
      return false;
    }
	}


	lsm6dsv320x_sh_reset_set(&dev_ctx, 1);
	delay(10);
	lsm6dsv320x_sh_reset_set(&dev_ctx, 0);
	delay(10);

  lsm6dsv320x_hg_xl_data_rate_set(&dev_ctx, LSM6DSV320X_HG_XL_ODR_AT_7680Hz, 1 );
  lsm6dsv320x_hg_xl_full_scale_set(&dev_ctx, LSM6DSV320X_320g);

  // Indicator LEDs
  // pinMode(INDICATOR1, OUTPUT);
  // pinMode(INDICATOR2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Set up timing
  unsigned long last_read = micros(); // Last time accelerometer was read in

  return true; // Success :)
}

// Indicate heading
void indicateHeadingLSM() {
  // Heading LED based on perceived "forward"
  if (heading_degrees < 125.0f && heading_degrees > 145.0f) { // Should point in "forward" direction
      // digitalWrite(INDICATOR1, HIGH);
      // digitalWrite(INDICATOR2, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
  } else {
      // digitalWrite(INDICATOR1, LOW);
      // digitalWrite(INDICATOR2, LOW);
      digitalWrite(LED_BUILTIN, HIGH);
  }
}

// Updates heading
bool updateHeadingLSM(float COR_Input) { // Input in [-1, 1]
  int16_t data_raw_accel_hg[3];

  if (micros() - last_read > UPDATE_INTERVAL_MICROS) {
    lsm6dsv320x_hg_acceleration_raw_get(&dev_ctx, data_raw_accel_hg);
    raw_acceleration = (1.0f - LOW_PASS_RATIO) * raw_acceleration + LOW_PASS_RATIO * lsm6dsv320x_from_fs320_to_mg(data_raw_accel_hg[1]) * 0.00981f;
    last_read = micros();
  }

  // Update radius of rotation
  radius_of_rotation = 5.0f * COR_Input + 50.0f;
  if (radius_of_rotation == 50.0f) radius_of_rotation = last_radius_of_rotation;
  if (radius_of_rotation != 50.0f) last_radius_of_rotation = radius_of_rotation;

  // Apply dead zone and convert to degrees_per_second
  processed_acceleration = (abs(raw_acceleration) < ACCELERATION_DEAD_ZONE)? 0.0f : (raw_acceleration);
  degrees_per_second = DEGREES_PER_RADIAN * sqrt(abs(processed_acceleration) * (1000.0f) / radius_of_rotation);

  // Start counting the new degrees_per_second
  previous_degrees_per_second = degrees_per_second;

  // Calculate new heading
  heading_degrees = heading_degrees + degrees_per_second * (float) delta_micros / (1000000.0f);

  // Mod it
  heading_degrees = fmod(heading_degrees, 360.0f);

  // Serial outputs
  #ifdef DISPLAY_SERIAL
    // Serial.print(processed_acceleration);
    // Serial.print(",");
    // Serial.println(0);

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
float updateHeadingDotProductLSM(float throttle_x, float throttle_y) {
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