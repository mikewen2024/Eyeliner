#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>


struct TelemetryData {
    float sensor1;
    float sensor2;
};

extern TelemetryData telemetryData;


String getTelemetryJSON();
void initTelem();
void wifiTask(void* pvParameters);