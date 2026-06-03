#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "SparkFun_LIS331.h"
#include "ESP32_SoftWire.h"

#include "DShotESC.h"

#include "elrs.hpp"
#include "telemwifi.hpp"

// #define DEBUG

#define LED_PIN     D3
#define NUM_LEDS    6

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

#define DEADBAND 5

#define ESC0_PIN      GPIO_NUM_5
#define ESC1_PIN      GPIO_NUM_6
DShotESC esc0;
DShotESC esc1;
float th0 = 0;
float th1 = 0;
#define M1_REV 1
#define M0_REV -1
bool flipped = false;

#define XL_CS D1
#define XL_MOSI D10
#define XL_MISO D9
#define XL_SCK D8

#define ACCELMULT 100
LIS331 xl;
int16_t x, y, z, trim;
SoftWire Wire;
bool failsafe = false;
bool firstLoop = true;
double xAccel, yAccel, zAccel, xAcceloffset, yAcceloffset, zAcceloffset = 0;

//we read from the accelerometer much slower than the accelerometer's data rate to make sure we always get new data
//reading the same data twice could mess with the prediction algorithms
//a better method is to use the interrupt pin on the accelerometer that tells us every time new data is available
unsigned long measurementPeriod = 10000;//in microseconds. Represents 100Hz

uint16_t robotPeriod[2] = {10000,10000};//measured in microseconds per degree, with some memory for discrete integration

//this is the times we measured the accelerometer at. We keep some history for extrapolation
unsigned long accelMeasTime[2] = {0,10};

//this angle (degrees) is calculated only using the accelerometer. We keep it separate to keep our discrete integration algorithms operating smoothly
//the beacon sets our heading to 0, which would mess up the discrete integration if allowed to affect this variable directly
//instead we utilize a trim variable. In Accel control mode, the user controls trim with the encoder wheel. in hybrid mode, the beacon controls trim
uint16_t accelAngle = 0;

//in degrees, this angle is added to the accel angle as adjusted by the beacon or the driver
uint16_t accelTrim = 0;

uint16_t angleAtLastMeasurement;

enum States {
  FAILSAFE,
  DRIVING,
  MELTING,
  // AUTO
};

States state = FAILSAFE;

void setup() {
  // enableLoopWDT();
  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println("COM Serial initialized");
  #endif
  initELRS();
  // initTelem();

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  x = 0;
  y = 0;
  z = 0;

  digitalWrite(XL_MISO, HIGH);
  Wire.begin(XL_MOSI, XL_SCK, 400000);
  xl.setI2CAddr(0x19);
  xl.begin(LIS331::USE_I2C); // Selects the bus to be used and sets
                          //  the power up bit on the accelerometer.
                          //  Also zeroes out all accelerometer
                          //  registers that are user writable.
  xl.setPowerMode(LIS331::NORMAL);
  xl.setODR(LIS331::DR_1000HZ);
  failsafe = !crsf.isLinkUp();
  for(int i = 0; i < NUM_LEDS; i++){
    strip.setPixelColor(i, strip.Color(255,0,255));
  }
  strip.show();
  xl.readAxes(x,y,z);
  loopELRS();
  esc0.install(ESC0_PIN, RMT_CHANNEL_3);
  esc1.install(ESC1_PIN, RMT_CHANNEL_2);
  esc0.init();
  esc1.init();
  delay(1000);
  esc0.set3DMode(false);
  esc1.set3DMode(false);
  esc0.sendMotorStop();
  esc1.sendMotorStop();
  for(int i = 0; i < 25; i++) {
    xl.readAxes(x,y,z);
    xAcceloffset -= xl.convertToG(ACCELMULT,x);
    yAcceloffset -= xl.convertToG(ACCELMULT,y);
    zAcceloffset -= xl.convertToG(ACCELMULT,z);
    delay(10);
  }
  xAcceloffset /= 25;
  yAcceloffset /= 25;
  zAcceloffset /= 25;

  trim = 0;
}

void loop() {
  #ifdef DEBUG
  // printChannels();
  // if(failsafe)
  //   Serial.println("FAILSAFE!");
    // Serial.print("state: ");
    // // Serial.print(state);
    // Serial.print("Accel: ");
    // Serial.print(xAccel);
    // Serial.print(", ");
    // Serial.print(yAccel);
    // Serial.print(", ");
    Serial.println(zAccel);
    // Serial.print("  ");
    // Serial.print("Throttle: ");
    // Serial.print(th0);
    // Serial.print(", ");
    // Serial.println(th1);
    // Serial.print("Angle: ");
    // Serial.println(trim);
  #endif

  //stuff to loop even in failsafe
  failsafe = !crsf.isLinkUp();
  strip.show();
  int targetAngle = abs(accelAngle - trim) % 360;
  //this uses blocking I2C, which makes it relatively slow. But given that we run our I2C ar 1.8MHz we will likely be okay
  if(micros() - accelMeasTime[0] > measurementPeriod | firstLoop)  {
    trim += (crsf.getChannel(4) - 1500)/25;
    if(trim > 360){
      trim -= 360;
    }
    if(trim < 0){
      trim += 360;
    }
    //shift all of the old values down
    for(int i=1; i>0; i--) {
      accelMeasTime[i] = accelMeasTime[i-1];
    }
    //put in the new value
    accelMeasTime[0] = micros();
    
    uint8_t accelBuf[6];
  
    xl.readAxes(x,y,z);
    xAccel = (xl.convertToG(ACCELMULT,x) + xAcceloffset) * -1;
    yAccel = -1 * (xl.convertToG(ACCELMULT,y) + yAcceloffset) * -1;
    zAccel = xl.convertToG(ACCELMULT,z) + zAcceloffset;
    flipped = crsf.getChannel(8) > 1500;
    //shift all of the old values down
    for(int i=1; i>0; i--) {
      robotPeriod[i] = robotPeriod[i-1];
    }
    //put in the new value
    //this equation has been carefully calibrated for this bot. See here for explanation:
    //https://www.swallenhardware.io/battlebots/2018/8/12/halo-pt-9-accelerometer-calibration
    if(yAccel > 1) {
      robotPeriod[0] = (uint16_t)(528.6449 / sqrt(yAccel * (((double)(crsf.getChannel(6)-1500))/1000 + 1)));

      //find the new angle
      //TRIANGULAR INTEGRATION
      uint32_t deltaT = accelMeasTime[0] - accelMeasTime[1];
      angleAtLastMeasurement = (angleAtLastMeasurement + (deltaT/robotPeriod[0] + deltaT/robotPeriod[1])/2) % 360;

      accelAngle = angleAtLastMeasurement;
    }
  } else {//if it isn't time to check the accelerometer, predict our current heading
    //predict the current velocity by extrapolating old data
    uint32_t newTime = micros();
    uint32_t periodPredicted = robotPeriod[1] + (newTime - accelMeasTime[1]) * (robotPeriod[0] - robotPeriod[1]) / (accelMeasTime[0] - accelMeasTime[1]);

    //predict the current robot heading by triangular integration up to the extrapolated point
    uint32_t deltaT = newTime - accelMeasTime[0];
    accelAngle = (angleAtLastMeasurement + (deltaT/periodPredicted + deltaT/robotPeriod[0])/2) % 360;
  }
  if(targetAngle + (crsf.getChannel(7)-1500)/10 < 5 && targetAngle + (crsf.getChannel(7)-1500)/10 > -5) {
    strip.setBrightness(255);
    for(int i = 0; i < NUM_LEDS; i++)
      strip.setPixelColor(i, strip.Color(0,0,255));
  } else {
    for(int i = 0; i < NUM_LEDS; i++)
      strip.setPixelColor(i, strip.Color(0,0,0));
  }

  loopELRS();
  if(failsafe) {
    state = FAILSAFE;
    strip.setBrightness(50);
    for(int i = 0; i < NUM_LEDS; i++)
      strip.setPixelColor(i, strip.Color(255,0,0));
    esc0.sendMotorStop();
    esc1.sendMotorStop();
    esc0.beep(2);
    esc1.beep(2);
    firstLoop = false;
    return;
  }
  th0 = 0;
  th1 = 0;
  if(crsf.getChannel(5) > 1600) {
    state = MELTING;
  } else {
    state = DRIVING;
  }
  if(state == DRIVING) {
    th0 = (crsf.getChannel(2)-1500)/5 + (crsf.getChannel(1)-1500)/20;
    th1 = (crsf.getChannel(2)-1500)/5 - (crsf.getChannel(1)-1500)/20;
  }
  
  if(state == MELTING) {
    int throttle = crsf.getChannel(3) - 950;
    
    if(targetAngle < 45 && targetAngle > 315) {
      th0 = (crsf.getChannel(2)-1500);
      th1 = (crsf.getChannel(2)-1500);
    }
    else if(targetAngle < 225 && targetAngle > 135) {
      th0 = -1*(crsf.getChannel(2)-1500);
      th1 = -1*(crsf.getChannel(2)-1500);
    }
    else if(targetAngle < 135 && targetAngle > 45) {
      th0 = (crsf.getChannel(1)-1500);
      th1 = (crsf.getChannel(1)-1500);
    }
    else if(targetAngle < 315 && targetAngle > 225) {
      th0 = -1*(crsf.getChannel(1)-1500);
      th1 = -1*(crsf.getChannel(1)-1500);
    }
    if(throttle > 0) {
      if(th0 < 0)
        th0 = 0;
      if(th1 > 0)
        th1 = 0;
    } else if(throttle < 0) {
      if(th0 > 0)
        th0 = 0;
      if(th1 < 0)
        th1 = 0;
    }
    th0 += throttle;
    th1 += -throttle;
    if(flipped) {
      th0 = -th0;
      th1 = -th1;
    }
    
  }
  //stuff to not loop if in failsafe
  if(th0 < DEADBAND && th0 > -1*DEADBAND)
    esc0.sendMotorStop();
  else
    esc0.sendThrottle3D(th0 * M0_REV);
  if(th1 < DEADBAND && th1 > -1*DEADBAND)
    esc1.sendMotorStop();
  else
    esc1.sendThrottle3D(th1 * M1_REV);
  firstLoop = false;
  return;
}
