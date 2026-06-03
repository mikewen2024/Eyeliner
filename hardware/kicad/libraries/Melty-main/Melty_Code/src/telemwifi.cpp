#include "telemwifi.hpp"

const char* ssid = "walnut";
const char* password = "walnut123456";
AsyncWebServer server(80);
TelemetryData telemetryData;

// Function to get telemetry data as JSON
String getTelemetryJSON() {
    StaticJsonDocument<200> jsonDoc;
    jsonDoc["sensor1"] = telemetryData.sensor1;
    jsonDoc["sensor2"] = telemetryData.sensor2;
    
    String jsonString;
    serializeJson(jsonDoc, jsonString);
    return jsonString;
}

void initTelem() {

    // Set up ESP32 as an Access Point (AP)
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);  // Start Access Point
    Serial.print("Access Point Started. IP Address: ");
    Serial.println(WiFi.softAPIP()); // Get the IP address of the AP

    // Route to serve JSON telemetry data
    server.on("/telemetry", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "application/json", getTelemetryJSON());
    });

    // Start the web server
    server.begin();
}





