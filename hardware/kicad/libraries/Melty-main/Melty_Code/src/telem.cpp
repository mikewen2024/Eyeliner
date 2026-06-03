// #include "telem.hpp"

// NimBLECharacteristic *pCharacteristic;
// NimBLEServer *pServer;
// NimBLEAdvertising *pAdvertising;
// TaskHandle_t telemetryTaskHandle;

// TelemetryData telemetryData;

// class MyServerCallbacks : public NimBLEServerCallbacks {
//     void onConnect(NimBLEServer* pServer) {
//         Serial.println("Device connected");
//     }
//     void onDisconnect(NimBLEServer* pServer) {
//         Serial.println("Device disconnected");
//     }
// };

// void initTelem() {
//     // Explicitly enable the Bluetooth controller
//     esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
//     NimBLEDevice::init("Walnut");
//     NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Set maximum power
//     pServer = NimBLEDevice::createServer();
//     pServer->setCallbacks(new MyServerCallbacks());
//     NimBLEService* pService = pServer->createService("WALL");

//     pCharacteristic = pService->createCharacteristic(
//         CHARACTERISTIC_UUID
//     );

//     pService->start();
//     pAdvertising = NimBLEDevice::getAdvertising();
//     pAdvertising->addServiceUUID(SERVICE_UUID);
//     pAdvertising->start();
//     Serial.println("BLE Telemetry Service Started");

//     xTaskCreatePinnedToCore(
//         telemetryTask,      // Task function
//         "Telemetry Task",  // Name of the task
//         4096,               // Stack size
//         (void*)&telemetryData, // Pass telemetry data struct
//         1,                  // Priority
//         &telemetryTaskHandle, // Task handle
//         0                   // Core (1 = second core)
//     );
// }

// void sendTelem(String msg) {
//     uint8_t data[10];
//     data[5] = 5;
//     pCharacteristic->setValue(data,10);
// }

// void telemetryTask(void *parameter) {
//     TelemetryData* data = (TelemetryData*)parameter;
//     while (true) {
//         // Create a structured JSON-like payload
//         String payload = "{ \"sensor1\": " + String(data->sensor1) + 
//                           ", \"sensor2\": " + String(data->sensor2) + " }";
        
//         pCharacteristic->setValue(payload.c_str());
//         pCharacteristic->notify();
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//         if (pAdvertising->start()) {
//         Serial.println("BLE Advertising Started Successfully!");
//         } else {
//             Serial.println("BLE Advertising Failed!");
//         }
//     }
// }



