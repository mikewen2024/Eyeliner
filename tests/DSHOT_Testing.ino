#include <Arduino.h>
#include <DShotRMT.h>

// Define the GPIO pin connected to the motor ESC
const gpio_num_t MOTOR_PIN = GPIO_NUM_38;
// const gpio_num_t MOTOR_PIN = GPIO_NUM_39;

// Create a DShotRMT instance for DSHOT300 with bidirectional telemetry enabled
DShotRMT motor(MOTOR_PIN, DSHOT600, false);

// Other variables used
static int throttle = 1048; // Start in dead zone
static unsigned long throttle_loop_time = 0;

void setup() {
  motor.begin();
  
  delay(1000);

  // Enable 3D mode and save it — only needs to be done once ever
  for (int i = 0; i < 100; i++) {
    motor.sendCommand(DSHOT_CMD_3D_MODE_ON);   // command 10
  }
  for (int i = 0; i < 100; i++) {
    motor.sendCommand(DSHOT_CMD_SAVE_SETTINGS); // command 12
  }

  // Re-arm after config change
  unsigned long start = millis();
  while (millis() - start < 5000) {
    motor.sendThrottle(0);
  }

  throttle_loop_time = millis();
}


void loop() {

  // Ramp forward
  if (millis() - throttle_loop_time < 1998) {
    throttle = min(198, max(48, 
                (int)((millis() - throttle_loop_time) / 2) + 48));
  } else if (millis() - throttle_loop_time < 3996) {
    throttle = min(198, max(48, 
                1047 - (int)((millis() - throttle_loop_time - 1998) / 2)));
  }

  // Ramp other way
  else if (millis() - throttle_loop_time < 5994) {
    throttle = min(1198, max(1049, 
                (int)((millis() - throttle_loop_time - 3996) / 2) + 1049));
  } else if (millis() - throttle_loop_time < 7992) {
    throttle = min(1198, max(1049, 
                2047 - (int)((millis() - throttle_loop_time - 5994) / 2)));
  }

  // Reset
  else {
    throttle_loop_time = millis();
    throttle = 1048;
  }

  motor.sendThrottle(throttle);
}