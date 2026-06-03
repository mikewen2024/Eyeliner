#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#define PIN_RX D6
#define PIN_TX D7

#define PIN_SNS_VIN 1

#define RESISTOR1 15000.0
#define RESISTOR2 2200.0
#define ADC_RES 8192.0
#define ADC_VLT 3.3

extern AlfredoCRSF crsf;

void initELRS();
void loopELRS();
static void sendBattTelem(float voltage, float current, float capacity, float remaining);
void printChannels();
