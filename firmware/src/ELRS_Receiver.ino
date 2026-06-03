// Libraries required
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

// 2-second timeout
#define TIMEOUT_MICROS 4000000UL
unsigned long lastPacketMicros = 0;

// Pins used
#define PIN_RX 41
#define PIN_TX 42

// Serial objects
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

bool setupELRSReceiver() {
  // Pinmodes
  pinMode(41, INPUT);
  pinMode(42, INPUT);

  // CRSF protocol setup
  crsfSerial.begin(420000, SERIAL_8N1, PIN_RX, PIN_TX);
  if(!crsfSerial) return false;

  crsf.begin(crsfSerial);

  // Set default values
  channelDefaults[0] = 1500;
  channelDefaults[1] = 1000;
  channelDefaults[2] = 1500;
  channelDefaults[3] = 1500;

  // Set as channel data
  for (int i = 0; i < 4; i++) {
    rawChannelData[i] = channelDefaults[i];
  }

  // Set channel remapped data
  remapInputs();

  // Set lastPacketMicros to micros() to avoid false failsafe trigger
  lastPacketMicros = micros();

  return true; // Passed :)
}

// Failsafed read-in function
bool readChannelsELRS() {
  // Main read portion
  crsf.update();
  // Channel 4 is left x -> rawChannelData[0]
  rawChannelData[0] = crsf.getChannel(4);
  // Channel 3 is left y -> rawChannelData[1]
  rawChannelData[1] = crsf.getChannel(3);
  // Channel 1 is right x -> rawChannelData[3]
  rawChannelData[3] = crsf.getChannel(1);
  // Channel 2 is right y -> rawChannelData[2]
  rawChannelData[2] = crsf.getChannel(2);

  // Failsafe conditions, check if very-obviously-erroneous signals are received
  safe_to_read = true;
  // Last set of bits received time
  if (crsfSerial.available() != 0) {
    lastPacketMicros = micros();
  }

  // If the failsafe condition is invoked, we force signals to be a certain way, can't really just have no interrupts though
  if (micros() - lastPacketMicros > TIMEOUT_MICROS) {
    for (int i = 0; i < 4; i++) {
      rawChannelData[i] = channelDefaults[i];
    }
    safe_to_read = false;
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