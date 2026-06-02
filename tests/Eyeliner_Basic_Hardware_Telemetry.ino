#include <ESP32Servo.h>

/*
MPU6050 Offsets:
XGYRO: 204
YGYRO: 36
ZGYRO: -38
*/

// Define PWM input pins for each channel
#define CHANNEL_1 1 // Left X, left = 1000, right = 2000
#define CHANNEL_2 2 // Left Y, down = 1000, up = 2000
#define CHANNEL_3 3  // Right Y, down = 1000, up = 2000
#define CHANNEL_4 4 // Right X, left = 1000, right = 2000

// ESCs
Servo esc1;
Servo esc2;

// ESC Pinouts
#define ESC_1 43
#define ESC_2 44

// ESC settings
#define HIGH_SIGNAL 1550
#define LOW_SIGNAL 1450
#define NEUTRAL 1500

void setup() {
    Serial.begin(115200); // Start serial communication
    pinMode(CHANNEL_1, INPUT);
    pinMode(CHANNEL_2, INPUT);
    pinMode(CHANNEL_3, INPUT);
    pinMode(CHANNEL_4, INPUT);

    // Bidirectional: 1000 µs (full reverse), 1500 µs (neutral), 2000 µs (full forward)
    esc1.attach(ESC_1, LOW_SIGNAL, HIGH_SIGNAL);
    delay(10); 
    esc2.attach(ESC_2, LOW_SIGNAL, HIGH_SIGNAL);
    delay(10); 

    // Calibrate by setting neutral since these are bidirectional
    Serial.println("Calibrating Both ESCs");
    esc1.writeMicroseconds(NEUTRAL);
    delay(2000);
    esc2.writeMicroseconds(NEUTRAL);
    delay(2000);

    Serial.println("ESCs calibrated :)");
}

void loop() {
    // Read PWM pulse widths
    int channel3 = pulseIn(CHANNEL_3, HIGH);
    int channel4 = pulseIn(CHANNEL_4, HIGH);
    int channel2 = pulseIn(CHANNEL_2, HIGH);
    int channel1 = pulseIn(CHANNEL_1, HIGH);

    // In-Code fail safe for an accidental power cycle to SEEED studio
    if (channel1 < 980) channel1 = 1500;
    if (channel2 < 980) channel2 = 1500;
    if (channel3 < 980) channel3 = 1500;
    if (channel4 < 980) channel4 = 1500;
    
    // Print values to Serial Monitor
    // Serial.print("Channel 1 throttle: ");
    Serial.print(min(max(channel1, LOW_SIGNAL), HIGH_SIGNAL));
    Serial.print(",");
    // Serial.print("Channel 2 throttle: ");
    Serial.print(channel2);
    Serial.print(",");
    // Serial.print("Channel 3 throttle: ");
    Serial.print(channel3);
    Serial.print(",");
    // Serial.print("Channel 4 throttle: ");
    Serial.println(min(max(channel4, LOW_SIGNAL), HIGH_SIGNAL));

    // Input signal to motors
    esc1.writeMicroseconds(min(max(channel1, LOW_SIGNAL), HIGH_SIGNAL)); // Writes right x input to esc 1
    esc2.writeMicroseconds(min(max(channel4, LOW_SIGNAL), HIGH_SIGNAL)); // Writes left x input to esc 2
    
    delay(10); // Small delay to stabilize readings
}