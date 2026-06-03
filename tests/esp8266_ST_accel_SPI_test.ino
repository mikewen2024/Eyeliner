#include <Wire.h>
#include <SPI.h>
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

void setup() {
	Serial.begin(115200);

	Wire.begin(D2, D1); // SDA=D2 (GPIO4), SCL=D1 (GPIO5)
	Wire.setClock(400000L);

	pinMode(LSM_CS, OUTPUT);
    digitalWrite(LSM_CS, HIGH);

	SPI.begin(); /* begin SPI */
	Serial.println("SPI start");
	// Initialize driver context
	dev_ctx.read_reg = spi_read;
	dev_ctx.write_reg = spi_write;
	dev_ctx.mdelay    = platform_delay;
	dev_ctx.handle    = DEV_ADDR;

	// Wait for sensor boot time
	delay(500);

	uint8_t whoamI = 0;


	while (whoamI != LSM6DSV320X_ID) {
		Serial.println("Device not found!");
		lsm6dsv320x_device_id_get(&dev_ctx, &whoamI);
		Serial.print("WHOAMI = 0x");
		Serial.println(whoamI, HEX);
		delay(500);
	}


	lsm6dsv320x_sh_reset_set(&dev_ctx, 1);
	delay(10);
	lsm6dsv320x_sh_reset_set(&dev_ctx, 0);
	delay(10);
    
    // lsm6dsv320x_xl_data_rate_set(&dev_ctx, LSM6DSV320X_ODR_AT_7680Hz );
    // lsm6dsv320x_xl_full_scale_set(&dev_ctx, LSM6DSV320X_4g);

    lsm6dsv320x_hg_xl_data_rate_set(&dev_ctx, LSM6DSV320X_HG_XL_ODR_AT_7680Hz, 1 );
    lsm6dsv320x_hg_xl_full_scale_set(&dev_ctx, LSM6DSV320X_320g);

    // lsm6dsv320x_gy_data_rate_set(&dev_ctx, LSM6DSV320X_ODR_AT_7680Hz );
    // lsm6dsv320x_gy_full_scale_set(&dev_ctx, LSM6DSV320X_4000dps);

	Serial.println("LSM6DSV320X initialized!");
}

unsigned long last_micros = 0;

void loop() {
    int16_t data_raw_accel[3];
    float accel_mg[3];
    int16_t data_raw_accel_hg[3];
    float accel_mg_hg[3];

    // Serial.print("Frame Rate: ");
    // Serial.println(1000000.0f / (float)(micros() - last_micros));
    // last_micros = micros();

    // lsm6dsv320x_acceleration_raw_get(&dev_ctx, data_raw_accel);

    // accel_mg[0] = lsm6dsv320x_from_fs4_to_mg(data_raw_accel[0]);
    // accel_mg[1] = lsm6dsv320x_from_fs4_to_mg(data_raw_accel[1]);
    // accel_mg[2] = lsm6dsv320x_from_fs4_to_mg(data_raw_accel[2]);

    lsm6dsv320x_hg_acceleration_raw_get(&dev_ctx, data_raw_accel_hg);
	
    accel_mg_hg[0] = lsm6dsv320x_from_fs320_to_mg(data_raw_accel_hg[0]);
    accel_mg_hg[1] = lsm6dsv320x_from_fs320_to_mg(data_raw_accel_hg[1]); // Use this for spin testing
    accel_mg_hg[2] = lsm6dsv320x_from_fs320_to_mg(data_raw_accel_hg[2]);
    
    // Serial.print("Accel [mg]: X=");
    // Serial.print(accel_mg[0], 2);
    // Serial.print(" Y=");
    // Serial.print(accel_mg[1], 2);
    // Serial.print(" Z=");
    // Serial.println(accel_mg[2], 2);
    // Serial.print("Accel HG [mg]: X=");
    Serial.print(accel_mg_hg[0], 2);
    Serial.print(", ");
    Serial.print(accel_mg_hg[1], 2);
    Serial.print(", ");
    Serial.println(accel_mg_hg[2], 2);

	// delay(5);
}
