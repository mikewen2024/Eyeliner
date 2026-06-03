#include "elrs.hpp"

HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;
float cap;

void initELRS() {
    crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
    if (!crsfSerial) 
        while (1) 
            Serial.println("Invalid crsfSerial configuration");
    crsf.begin(crsfSerial);
    cap = 0;
}

void loopELRS() {
    // Must call crsf.update() in loop() to process data
    crsf.update();
  
    // int snsVin = analogRead(PIN_SNS_VIN);
    // float batteryVoltage = ((float)snsVin * ADC_VLT / ADC_RES) * ((RESISTOR1 + RESISTOR2) / RESISTOR2);
    // sendBattTelem(batteryVoltage, 1.2, cap += 10, 50);
}

static void sendBattTelem(float voltage, float current, float capacity, float remaining)
{
  crsf_sensor_battery_t crsfBatt = { 0 };

  // Values are MSB first (BigEndian)
  crsfBatt.voltage = htobe16((uint16_t)(voltage * 10.0));   //Volts
  crsfBatt.current = htobe16((uint16_t)(current * 10.0));   //Amps
  crsfBatt.capacity = htobe16((uint16_t)(capacity)) << 8;   //mAh (with this implemetation max capacity is 65535mAh)
  crsfBatt.remaining = (uint8_t)(remaining);                //percent
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
}

void printChannels()
{
  for (int ChannelNum = 1; ChannelNum <= 8; ChannelNum++)
  {
    Serial.print(crsf.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println(" ");
}