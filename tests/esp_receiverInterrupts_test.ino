#include <cmath>

#define TIMEOUT_MILLIS 500

#define LEFT_X_PIN 18 
#define LEFT_Y_PIN 15
#define RIGHT_X_PIN 5 
#define RIGHT_Y_PIN 4

volatile uint16_t servoPulseRX, servoPulseRY, servoPulseLX, servoPulseLY;
volatile uint32_t last_isr[] = {0, 0, 0, 0};


void setup() {
  Serial.begin(115200);

  pinMode(LEFT_X_PIN, INPUT);
  pinMode(LEFT_Y_PIN, INPUT);
  pinMode(RIGHT_X_PIN, INPUT);
  pinMode(RIGHT_Y_PIN, INPUT); 

  attachInterrupt(digitalPinToInterrupt(RIGHT_X_PIN), rightXPulseUpdate, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_X_PIN), leftXPulseUpdate, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_Y_PIN), leftThrottlePulseUpdate, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_Y_PIN), leftThrottlePulseUpdate, CHANGE);
}


void rightXPulseUpdate() {
    static uint32_t startTime = 0;
    uint32_t curTime = micros();
    
    if (digitalRead(RIGHT_X_PIN))
        startTime = curTime;
    else {
      servoPulseRX = (uint16_t)(curTime - startTime);
    }
    if (servoPulseRX <= 2200 && servoPulseRX >= 800) {
          last_isr[0] = millis();
    }
        
}

void leftXPulseUpdate() {
    static uint32_t startTime = 0;
    uint32_t curTime = micros();

    if (digitalRead(LEFT_X_PIN))
        startTime = curTime;
    else {
      servoPulseLX = (uint16_t)(curTime - startTime);
      
    }
    if (servoPulseLX <= 2200 && servoPulseLX >= 800) {
          last_isr[1] = millis();
      }
}

void leftThrottlePulseUpdate() {
    static uint32_t startTime = 0;
    uint32_t curTime = micros();

    if (digitalRead(LEFT_Y_PIN))
        startTime = curTime;
    else {
      servoPulseLY = (uint16_t)(curTime - startTime);
      
    }
    if (servoPulseLY <= 2200 && servoPulseLY >= 800) {
          last_isr[2] = millis();
      }

}

void rightYPulseUpdate() {
    static uint32_t startTime = 0;
    uint32_t curTime = micros();
    
    if (digitalRead(RIGHT_Y_PIN))
        startTime = curTime;
    else {
      servoPulseRY = (uint16_t)(curTime - startTime);
    }
    if (servoPulseRY <= 2200 && servoPulseRY >= 800) {
          last_isr[3] = millis();
    }
        
}


int invertSignal(int sig, int neu) {
  return (2 * neu) - sig; 
}

bool runFailSafe = false; 
void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Throttle - aIn: "); 
  Serial.print(servoPulseRX); 
  Serial.print(" | Throttle - pIn: "); 
  Serial.println(pulseIn(RIGHT_X_PIN, HIGH)); 
  // Serial.print(" | Delta: "); 
  // Serial.println(servoPulseLX - pulseIn(LEFT_X_PIN, HIGH)); 

  if (runFailSafe) {
    int now = millis(); 
    for (unsigned i = 0; i < 3; i++) {
      if(now - last_isr[i] > TIMEOUT_MILLIS) { 
        Serial.print("i: "); 
        Serial.print(i); 
        Serial.print(", Di: "); 
        Serial.println(now - last_isr[i]); 
        Serial.println("ITS GONNA EAT YOUR FINGERS!!!");
      } 
      delay(5); 
    }
  }
}
