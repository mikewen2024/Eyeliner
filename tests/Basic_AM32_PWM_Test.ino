#include <ESP32Servo.h>

// Define the GPIO pin connected to the ESC signal wire

#define MAX_SIG 1600 //1990
#define MIN_SIG 1400  //1010
#define NEU_SIG 1500 

const uint8_t escPinLeft = 44;  
const uint8_t escPinRight = 43;  

Servo escLeft;
Servo escRight;

void setup() {
  Serial.begin(115200);

  // Bidirectional: 1000 µs (full reverse), 1500 µs (neutral), 2000 µs (full forward)
  escLeft.attach(escPinLeft, MIN_SIG, MAX_SIG);
  delay(10); 
  escRight.attach(escPinRight, MIN_SIG, MAX_SIG);
  delay(10); 


  // Calibrate the ESC (optional, depends on your ESC setup)
  Serial.println("Calibrating ESC...");
  escLeft.writeMicroseconds(NEU_SIG);  // Neutral (stop)
  delay(2000);
  escRight.writeMicroseconds(NEU_SIG);  // Neutral (stop)
  delay(2000);

  Serial.println("ESC Calibrated.");
}

void loop() {
  for (int speed = NEU_SIG; speed <= MAX_SIG; speed += (MAX_SIG-NEU_SIG)/20) {
    escLeft.writeMicroseconds(speed);  
    escRight.writeMicroseconds(speed);  
    delay(100);  
    Serial.println(speed);
  }

  // Hold full forward speed for 5 seconds
  escLeft.writeMicroseconds(MAX_SIG); 
  escRight.writeMicroseconds(MAX_SIG); 
  delay(3000);

  // Gradually slow down to neutral
  for (int speed = MAX_SIG; speed >= NEU_SIG; speed -= (MAX_SIG-NEU_SIG)/20) {
    escLeft.writeMicroseconds(speed);  
    escRight.writeMicroseconds(speed);  
    delay(100);
  }

  // Gradually increase speed in reverse direction
  for (int speed = NEU_SIG; speed >= MIN_SIG; speed -= (NEU_SIG - MIN_SIG)/20) {
    escLeft.writeMicroseconds(speed);  
    escRight.writeMicroseconds(speed);  
    delay(100);
  }

  // Hold full reverse for 5 seconds
  escLeft.writeMicroseconds(1000);  
  escRight.writeMicroseconds(1000);  
  delay(3000);

  // Gradually slow down to neutral again
  for (int speed = MIN_SIG; speed <= NEU_SIG; speed += (NEU_SIG - MIN_SIG)/20) {
    escLeft.writeMicroseconds(speed); 
    escRight.writeMicroseconds(speed); 
    delay(100);
  }

  // Hold neutral for a bit before repeating the cycle
  escLeft.writeMicroseconds(1500);  
  escRight.writeMicroseconds(1500);  
  delay(3000);
}