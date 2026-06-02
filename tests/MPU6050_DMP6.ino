// Define PWM input pins for each channel
#define PIN_1 2
#define PIN_4 4
#define PIN_3 17
#define PIN_2 16

void setup() {
    Serial.begin(115200); // Start serial communication
    pinMode(PIN_1, INPUT);
    pinMode(PIN_2, INPUT);
    pinMode(PIN_3, INPUT);
    pinMode(PIN_4, INPUT);
}

void loop() {
    // Read PWM pulse widths
    int pin_1_value = pulseIn(PIN_1, HIGH);
    int pin_2_value = pulseIn(PIN_2, HIGH);
    int pin_3_value = pulseIn(PIN_3, HIGH);
    int pin_4_value = pulseIn(PIN_4, HIGH);
    
    // Print values to Serial Monitor
    Serial.print("Pin_1:");
    Serial.print(pin_1_value);
    Serial.print(",Pin_2:");
    Serial.print(pin_2_value);
    Serial.print(",Pin_3:");
    Serial.print(pin_3_value);
    Serial.print(",Pin_4:");
    Serial.println(pin_4_value);
    
    delay(10); // Small delay to stabilize readings
}