// Motor definition / declaration for PWM are done here, we only use them in this file
#define ESC_1 38
#define ESC_2 39

Servo esc1;
Servo esc2;

/* Constants used */

// // ESC settings - PWM
#define HIGH_SIGNAL 2000
#define LOW_SIGNAL 1000
#define NEUTRAL 1500

// ESC setting important variables
int MAX_THROTTLE_DEVIATION = 500.0f;

// ±10˚ off center, braking AND throttle spike
#define BRAKING_ARC_ANGLE 30.0f

// Take max(|Throttle| + 1, 1 - |Throttle|), this is how far to push dips and spikes towards that
// Goes from 0 to 1
// Throttle will be able to reverse by default
#define AGGRESSION 1.0f

// Defines whether or not to reverse throttle on braking
#define REVERSE_THROTTLE true

// Take [1000, 2000] and remap to [-1, 1] for both sticks and both axes
// This defines the dead zone applied to the output of the remapping
#define TRANSLATION_DEAD_ZONE 0.1f

// Rotational throttle has similar modulation, so we define the same dead zone
#define ROTATION_DEAD_ZONE 0.1f

/* Main Functions */
// Setup function
bool setupMotorsPWM() {
  // Setup constants
  MAX_THROTTLE_DEVIATION = (int)(((float) HIGH_SIGNAL - (float) LOW_SIGNAL) / 2.0f);

  // Bidirectional setup, just input neutral
  esc1.attach(ESC_1, LOW_SIGNAL, HIGH_SIGNAL);
  delay(2000); 
  esc2.attach(ESC_2, LOW_SIGNAL, HIGH_SIGNAL);
  delay(2000);
  esc1.writeMicroseconds(NEUTRAL);
  delay(2000);
  esc2.writeMicroseconds(NEUTRAL);

  // No checksums yet :(
  return true;
}


DShotRMT motor1(ESC_1, DSHOT600, false);
DShotRMT motor2(ESC_2, DSHOT600, false);
bool setupMotorsDshot() {
  MAX_THROTTLE_DEVIATION = (int)(((float) HIGH_SIGNAL - (float) LOW_SIGNAL) / 2.0f);
  motor1.begin();
  delay(20);

  motor2.begin(); 
  delay(20);

  // Enable 3D mode and save it — only needs to be done once ever
  for (int i = 0; i < 100; i++) {
    motor1.sendCommand(DSHOT_CMD_3D_MODE_ON);   // command 10
  }
  for (int i = 0; i < 100; i++) {
    motor1.sendCommand(DSHOT_CMD_SAVE_SETTINGS); // command 12
  }

  // Enable 3D mode and save it — only needs to be done once ever
  for (int i = 0; i < 100; i++) {
    motor2.sendCommand(DSHOT_CMD_3D_MODE_ON); 
  }
  for (int i = 0; i < 100; i++) {
    motor2.sendCommand(DSHOT_CMD_SAVE_SETTINGS);
  }

  // Re-arm after config change
  // unsigned long start = millis();
  // while (millis() - start < 5000) {
  //   motor1.sendThrottle(0);
  //   motor2.sendThrottle(0);
  // }

  delay(500);
  
  // Hold 1048 to ensure ESC is happy
  // start = millis();
  // while (millis() - start < 6000) {
  //   motor1.sendThrottle(1048);
  //   motor2.sendThrottle(1048);
  // }

  return true; 
}

int PWMtodShot(int pwm) {
  if (pwm == 1500) {
    return 1049; 
  }

  // 48 - 1047: full rev to min
  // 1049 - 2047: min to full for 

  //inputs:
  // 1000 - 1500 maps to 48 - 1047
  // 1500 - 2000 maps to 1049 - 2047

  // y = c + \frac{d - c}{b - a} \times (x - a) $$ 
  else if (pwm < 1500) { //back 
    return (int)(std::round(49.0 + (1500.0 - pwm) / (500.0) * (1047.0 - 49.0)));
  }
  else { //for
    return (int)(std::round(1049.0 + (pwm - 1500.0) / (500.0) * (2047.0 - 1049.0)));
  }
}

// Loop function to set motor throttles
// All inputs from [-1, 1]
// Heading_cosine denotes the dot product between heading and throttle vectors
bool stopLogic(float throttle_spinup, float throttle_x, float throttle_y, float heading_cosine) {
  // Get translational throttle magnitude D, not square rooted!
  D = min(1.0f, throttle_x*throttle_x + throttle_y*throttle_y);
  if (D < TRANSLATION_DEAD_ZONE) {
    D = 0.0f;
  }

  // Calculate the modulation function (determines where braking and spikes are)
  if (heading_cosine > HEADING_THRESHOLD) {
    M = 1.0f;
  } else if (-heading_cosine > HEADING_THRESHOLD) {
    M = -1.0f;
  } else {
    M = 0.0f;
  }

  // Calculate both motor throttle deviations (still outputs in range [-1, 1])
  if (REVERSE_THROTTLE) {
    Amplitude_Modulation = AGGRESSION * (abs(throttle_spinup) + 1.0f);
  } else {
    Amplitude_Modulation = -AGGRESSION;
  } 
  Modulation = M * D * throttle_spinup * Amplitude_Modulation;

  // Add to spinup throttle T and apply limits
  Motor_1_Throttle = throttle_spinup;
  Motor_2_Throttle = throttle_spinup;
  if (throttle_spinup < -0.1f || throttle_spinup > 0.1f) {
    Motor_1_Throttle += Modulation;
    Motor_2_Throttle -= Modulation;
  }
  Motor_1_Throttle = max(-1.0f, min(Motor_1_Throttle, 1.0f));
  Motor_2_Throttle = max(-1.0f, min(Motor_2_Throttle, 1.0f));

  int Motor_1_PWM_Val = (int)(Motor_1_Throttle * MAX_THROTTLE_DEVIATION + NEUTRAL); 
  int Motor_2_PWM_Val = (int)(Motor_2_Throttle * MAX_THROTTLE_DEVIATION + NEUTRAL); 

  // Rescale by multiplying by 500 and adding 1500
  // Set motor throttles
  #ifdef MOTORS
    // PWM WRITE
      // esc1.writeMicroseconds(Motor_1_PWM_Val);
      // esc2.writeMicroseconds(Motor_2_PWM_Val);

    // DSHOT WRITE
    motor1.sendThrottle(PWMtodShot(Motor_1_PWM_Val)); 
    motor2.sendThrottle(PWMtodShot(Motor_2_PWM_Val)); 


  #endif

  // Output telemetry
  #ifdef DISPLAY_SERIAL
    // Serial.println(HEADING_THRESHOLD);

    // Serial.print(heading_cosine);
    // Serial.print(", ");
    // Serial.print(M);
    // Serial.print(", ");
    // Serial.println(Modulation);

    // Serial.print(Motor_1_Throttle);
    // Serial.print(", ");
    // Serial.print(Motor_2_Throttle);
    // Serial.print(", ");
    // Serial.println(0);

    Serial.print(PWMtodShot(Motor_1_PWM_Val));
    Serial.print(", ");
    Serial.print(PWMtodShot(Motor_2_PWM_Val));
    Serial.print(", ");
    Serial.println(0);
  #endif

  // Again no checksums :(
  return true;
}

// Failsafe function
void killMotorsPWM() {
  esc1.writeMicroseconds(NEUTRAL);
  esc2.writeMicroseconds(NEUTRAL);
}

void killMotorsDshot() {
  motor1.sendThrottle(1048); 
  motor2.sendThrottle(1048); 
}
