// All timers pertaining to failsafe are here
/* Constants */
// 2-second timeout
#define TIMEOUT_MICROS 2000000UL
unsigned long lastChannelTwoTime = 0;

// Channel GPIOs
#define CHANNEL_1 1
#define CHANNEL_2 2
#define CHANNEL_3 3
#define CHANNEL_4 4

/* Interrupt functions */
void ch1Interrupt(){
  if (digitalRead(CHANNEL_1) == HIGH) {
    channelAvailabilities[0] = false;
    channelBeginTimes[0] = micros();
  } else {
    channelEndTimes[0] = micros();
    channelAvailabilities[0] = true;
  }
}

void ch2Interrupt(){
  if (digitalRead(CHANNEL_2) == HIGH) {
    channelAvailabilities[1] = false;
    channelBeginTimes[1] = micros();
  } else {
    channelEndTimes[1] = micros();
    channelAvailabilities[1] = true;
  }
}

void ch3Interrupt(){
  if (digitalRead(CHANNEL_3) == HIGH) {
    channelAvailabilities[2] = false;
    channelBeginTimes[2] = micros();
  } else {
    channelEndTimes[2] = micros();
    channelAvailabilities[2] = true;
  }
}

void ch4Interrupt(){
  if (digitalRead(CHANNEL_4) == HIGH) {
    channelAvailabilities[3] = false;
    channelBeginTimes[3] = micros();
  } else {
    channelEndTimes[3] = micros();
    channelAvailabilities[3] = true;
  }
}

/* Main Functions */
// Setup function
bool setupReceiver() {
  // Pinmodes
  pinMode(CHANNEL_1, INPUT);
  pinMode(CHANNEL_2, INPUT);
  pinMode(CHANNEL_3, INPUT);
  pinMode(CHANNEL_4, INPUT);

  // Setup attachInterrupts
  attachInterrupt(digitalPinToInterrupt(CHANNEL_1), ch1Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_2), ch2Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_3), ch3Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_4), ch4Interrupt, CHANGE);

  // Set default values
  channelDefaults[0] = 1500;
  channelDefaults[1] = 1000;
  channelDefaults[2] = 1500;
  channelDefaults[3] = 1500;
  for (int i = 0; i < 4; i++) {
    channelBeginTimes[i] = micros();
    channelEndTimes[i] = micros();
    channelAvailabilities[i] = false;
    rawChannelData[i] = channelDefaults[i];
    lastChannelData[i] = channelDefaults[i];
    remappedChannelData[i] = ((float)rawChannelData[i] - 1500.0f) * 0.002f;
  }

  // No checksums yet :(
  return true;
}

// Update timing
void updateTiming() {
  current_micros = micros();
  delta_micros = current_micros - last_micros;
  // ticks_per_second = (1000000.0d) / (float)delta_micros;
  last_micros = current_micros;
}

// Failsafed read-in function
bool readChannels() {
  // Main read portion
  noInterrupts();
  for (int i = 0; i < 4; i++) {
    if (channelAvailabilities[i]) {
      rawChannelData[i] = channelEndTimes[i] - channelBeginTimes[i];
    }
  }
  interrupts();

  // Failsafe conditions, check if very-obviously-erroneous signals are received
  safe_to_read = true;
  for (int i = 0; i < 4; i++) {
    if (rawChannelData[i] < 980 || 
      rawChannelData[i] > 2020 || 
      (micros() - channelEndTimes[i]) > TIMEOUT_MICROS) {

      rawChannelData[i] = channelDefaults[i];
      safe_to_read = false;
    }
  }

  // Timeout condition - if left stick y above 1200
  if (rawChannelData[1] > 1200) {
    lastChannelTwoTime = micros();
  }

  // If the failsafe condition is invoked, we force signals to be a certain way, can't really just have no interrupts though
  if (micros() - lastChannelTwoTime > TIMEOUT_MICROS) {
    for (int i = 0; i < 4; i++) {
      rawChannelData[i] = channelDefaults[i];
    }
  }

  // Serial output
  #ifdef DISPLAY_SERIAL
    // Serial.print("Channel 1 throttle: ");
    // Serial.print(rawChannelData[0]);
    // Serial.print(",");
    // Serial.print("Channel 2 throttle: ");
    // Serial.print(rawChannelData[1]);
    // Serial.print(",");
    // Serial.print("Channel 3 throttle: ");
    // Serial.print(rawChannelData[2]);
    // Serial.print(",");
    // Serial.print("Channel 4 throttle: ");
    // Serial.println(rawChannelData[3]);

    // Serial.print("Channel 1 throttle: ");
    // Serial.print(remappedChannelData[0]);
    // Serial.print(",");
    // Serial.print("Channel 2 throttle: ");
    // Serial.print(remappedChannelData[1]);
    // Serial.print(",");
    // Serial.print("Channel 3 throttle: ");
    // Serial.print(remappedChannelData[2]);
    // Serial.print(",");
    // Serial.print("Channel 4 throttle: ");
    // Serial.println(remappedChannelData[3]);

    // Serial.print(delta_micros);
    // Serial.print(",");
    // Serial.println(ticks_per_second);
  #endif

  // Checksum on whether or not timed out 
  return safe_to_read;
}

// Remap inputs to [-1, 1]
void remapInputs() {
  for (int i = 0; i < 4; i++) {
    remappedChannelData[i] = ((float)rawChannelData[i] - 1500.0f) * 0.002f;
  }
}