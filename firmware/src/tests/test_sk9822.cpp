/**
 * @file test_sk9822.cpp
 * @brief PCA9570 I/O expander probe + SK9822 LED blink test (red, 1 s on / 1 s off).
 *
 * Build/flash: pio run -e test_sk9822 -t upload
 * Monitor:     pio device monitor -e test_sk9822
 *
 * Board wiring (hardware/Eyeliner_PCB_V3/Top_Board_Netlist.net):
 *   I2C SDA  = XIAO D0 = GPIO1, SCL = XIAO D1 = GPIO2 (1k pull-ups R13/R10)
 *   IC1 PCA9570 @0x24: P0 -> LEDs_EN -> IC2 (74AHCT2G125) ~1OE
 *                      P1 -> Temp_Sel1, P2 -> Temp_Sel2, P3 -> Q1 gate
 *   IC2 buffer 1: 1A = SCL (GPIO2)      -> 1Y = Conn_LEDs1 pin 3 (SK9822 CLK, 10k pull-down)
 *   IC2 buffer 2: 2A = GPIO40 (MTDO)    -> 2Y = Conn_LEDs1 pin 2 (SK9822 DATA), ~2OE tied to GND
 *
 * So the SK9822 clock is the I2C SCL line, gated by PCA9570 P0 (LOW = enabled).
 * Both the I2C transactions (open-drain, ~100 kHz) and the LED clock
 * (push-pull) are bit-banged on the same pins, so no I2C driver has to be torn
 * down and rebuilt around every LED frame. SDA is left high while clocking
 * the LED, so the IMUs never see a START; with the gate open the LED sees the
 * I2C clock pulses too, but DATA (GPIO40) is held low then, so they only
 * lengthen the next start frame.
 *
 * Serial commands:
 *   s     re-run the bus scan + PCA9570 report (same as at boot)
 *   r     read the PCA9570 output register
 *   e/d   enable / disable the LED clock gate (P0 low / high)
 *   c     cycle red -> green -> blue on successive blinks (default)
 *   R G B W  lock the blink colour to red / green / blue / white
 *   h ?   help. Other bytes are ignored.
 */
#include <Arduino.h>

#include "board_pins.h"

#ifndef SK_SDA_PIN
#define SK_SDA_PIN PIN_I2C_SDA          // GPIO1
#endif
#ifndef SK_SCL_PIN
#define SK_SCL_PIN PIN_I2C_SCL          // GPIO2, also the SK9822 clock
#endif
#ifndef SK_DATA_PIN
#define SK_DATA_PIN 40                  // MTDO, net /Top_Board/LEDs
#endif

#ifndef PCA9570_ADDR
#define PCA9570_ADDR 0x24
#endif
#ifndef PCA9570_LED_EN_BIT
#define PCA9570_LED_EN_BIT 0            // P0
#endif
// 74AHCT2G125 ~1OE is active low: P0 = 0 enables the LED clock
#ifndef PCA9570_LED_EN_ACTIVE_LOW
#define PCA9570_LED_EN_ACTIVE_LOW 1
#endif

#ifndef SK_NUM_LEDS
#define SK_NUM_LEDS 1
#endif
#ifndef SK_BRIGHTNESS
#define SK_BRIGHTNESS 8                 // 5-bit global brightness, 0..31
#endif
#ifndef SK_BLINK_MS
#define SK_BLINK_MS 1000
#endif
// Half clock period of the LED bit-bang, in microseconds
#ifndef SK_HALF_PERIOD_US
#define SK_HALF_PERIOD_US 1
#endif
// Half clock period of the bit-banged I2C, in microseconds (5 = ~100 kHz)
#ifndef I2C_HALF_PERIOD_US
#define I2C_HALF_PERIOD_US 5
#endif

#define HELP "'s' = rescan + report, 'r' = read PCA9570, 'e'/'d' = enable/disable LED clock gate, 'c' = cycle RGB, 'R'/'G'/'B'/'W' = lock colour"

static uint8_t g_pcaShadow = 0x0F;      // PCA9570 power-on default is all outputs high

// --- bit-banged I2C (open-drain) on SDA/SCL --------------------------------

static inline void i2cDelay() { delayMicroseconds(I2C_HALF_PERIOD_US); }
static inline void sdaRelease() { pinMode(SK_SDA_PIN, INPUT); }
static inline void sclRelease() { pinMode(SK_SCL_PIN, INPUT); }
static inline void sdaLow() { pinMode(SK_SDA_PIN, OUTPUT_OPEN_DRAIN); digitalWrite(SK_SDA_PIN, LOW); }
static inline void sclLow() { pinMode(SK_SCL_PIN, OUTPUT_OPEN_DRAIN); digitalWrite(SK_SCL_PIN, LOW); }

// Release SCL and wait for it to actually go high (clock stretching / stuck line)
static bool sclHigh() {
  sclRelease();
  for (int i = 0; i < 200; i++) {
    if (digitalRead(SK_SCL_PIN)) return true;
    delayMicroseconds(1);
  }
  return false;
}

static void i2cStart() { sdaRelease(); sclHigh(); i2cDelay(); sdaLow(); i2cDelay(); sclLow(); i2cDelay(); }
static void i2cStop()  { sdaLow(); i2cDelay(); sclHigh(); i2cDelay(); sdaRelease(); i2cDelay(); }

// Returns true on ACK
static bool i2cWriteByte(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    if ((b >> i) & 1) sdaRelease(); else sdaLow();
    i2cDelay(); sclHigh(); i2cDelay(); sclLow();
  }
  sdaRelease(); i2cDelay(); sclHigh(); i2cDelay();
  bool ack = !digitalRead(SK_SDA_PIN);
  sclLow(); i2cDelay();
  return ack;
}

static uint8_t i2cReadByte(bool ack) {
  uint8_t b = 0;
  sdaRelease();
  for (int i = 7; i >= 0; i--) {
    i2cDelay(); sclHigh(); i2cDelay();
    b = (b << 1) | (digitalRead(SK_SDA_PIN) ? 1 : 0);
    sclLow();
  }
  if (ack) sdaLow(); else sdaRelease();
  i2cDelay(); sclHigh(); i2cDelay(); sclLow(); sdaRelease(); i2cDelay();
  return b;
}

static bool i2cProbe(uint8_t addr) {
  i2cStart();
  bool ack = i2cWriteByte((addr << 1) | 0);
  i2cStop();
  return ack;
}

static bool i2cLinesIdleHigh() {
  sdaRelease(); sclRelease(); delayMicroseconds(10);
  bool sda = digitalRead(SK_SDA_PIN), scl = digitalRead(SK_SCL_PIN);
  if (!sda || !scl) Serial.printf("WARNING: bus not idle (SDA=%d SCL=%d)\n", sda, scl);
  return sda && scl;
}

// --- PCA9570 ------------------------------------------------------------------

static bool pcaWrite(uint8_t v) {
  i2cStart();
  bool a = i2cWriteByte((PCA9570_ADDR << 1) | 0);
  bool d = a && i2cWriteByte(v & 0x0F);
  i2cStop();
  if (!a) { Serial.println("PCA9570 write: NACK on address"); return false; }
  if (!d) { Serial.println("PCA9570 write: NACK on data"); return false; }
  g_pcaShadow = v & 0x0F;
  return true;
}

static bool pcaRead(uint8_t *v) {
  i2cStart();
  bool a = i2cWriteByte((PCA9570_ADDR << 1) | 1);
  uint8_t b = a ? i2cReadByte(false) : 0xFF;
  i2cStop();
  if (!a) { Serial.println("PCA9570 read: NACK on address"); return false; }
  *v = b & 0x0F;
  return true;
}

static void pcaPrint(uint8_t v) {
  Serial.printf("  P3..P0 = %d%d%d%d  (P0 LEDs_EN=%d, P1 Temp_Sel1=%d, P2 Temp_Sel2=%d, P3 Q1 gate=%d)\n",
                (v >> 3) & 1, (v >> 2) & 1, (v >> 1) & 1, v & 1,
                v & 1, (v >> 1) & 1, (v >> 2) & 1, (v >> 3) & 1);
}

static void i2cScan() {
  Serial.print("I2C scan:");
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (i2cProbe(a)) { Serial.printf(" 0x%02X", a); found++; }
  }
  if (!found) Serial.print(" (nothing)");
  Serial.println();
}

static bool pcaSetLedGate(bool enable) {
  uint8_t v = g_pcaShadow;
  bool bit = PCA9570_LED_EN_ACTIVE_LOW ? !enable : enable;
  if (bit) v |= (1 << PCA9570_LED_EN_BIT);
  else     v &= ~(1 << PCA9570_LED_EN_BIT);
  if (!pcaWrite(v)) return false;
  uint8_t rb;
  if (!pcaRead(&rb)) return false;
  Serial.printf("PCA9570 wrote 0x%02X, read back 0x%02X -> LED clock gate %s%s\n", v, rb,
                enable ? "ENABLED" : "disabled", rb == v ? "" : "  (MISMATCH)");
  pcaPrint(rb);
  return rb == v;
}

// --- SK9822 bit-bang on GPIO2 (clock, push-pull) / GPIO40 (data) ------------------

static inline void skDelay() {
#if SK_HALF_PERIOD_US > 0
  delayMicroseconds(SK_HALF_PERIOD_US);
#endif
}

static void skBegin() {
  digitalWrite(SK_DATA_PIN, LOW);
  pinMode(SK_DATA_PIN, OUTPUT);
  // Push-pull on SCL: the 1k pull-up is no problem for a GPIO driving low (3.3 mA)
  digitalWrite(SK_SCL_PIN, LOW);
  pinMode(SK_SCL_PIN, OUTPUT);
}

static void skEnd() {
  digitalWrite(SK_SCL_PIN, LOW);
  digitalWrite(SK_DATA_PIN, LOW);
  sclRelease();   // back to the pull-up so the I2C bus idles high
}

static void skByte(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SK_DATA_PIN, (b >> i) & 1);
    skDelay();
    digitalWrite(SK_SCL_PIN, HIGH);     // SK9822 samples on the rising edge
    skDelay();
    digitalWrite(SK_SCL_PIN, LOW);
  }
}

// SK9822 frame: 32x0 start, per-LED {111bbbbb, B, G, R}, then 32x0 which the SK9822
// treats as the latch ("reset") frame, plus >= n/2 more clocks of zeros to push the
// data through a longer strip. Do NOT send an APA102-style 0xFF end frame: the
// SK9822 reads the 32 zeros as a new start frame and the ones as an LED frame
// (brightness 31, B=G=R=255), i.e. the strip goes solid white.
static void skShow(uint8_t r, uint8_t g, uint8_t b, uint8_t bright) {
  skBegin();
  for (int i = 0; i < 4; i++) skByte(0x00);
  for (int n = 0; n < SK_NUM_LEDS; n++) {
    skByte(0xE0 | (bright & 0x1F));
    skByte(b);
    skByte(g);
    skByte(r);
  }
  for (int i = 0; i < 4 + (SK_NUM_LEDS + 15) / 16; i++) skByte(0x00);
  skEnd();
}

// --- main ----------------------------------------------------------------------

static void report() {
  Serial.println("=== PCA9570 + SK9822 test ===");
  Serial.printf("I2C (bit-bang) SDA=GPIO%d SCL=GPIO%d, PCA9570 @0x%02X\n", SK_SDA_PIN, SK_SCL_PIN, PCA9570_ADDR);
  Serial.printf("SK9822 CLK=GPIO%d (via 74AHCT2G125 gated by P%d, active %s), DATA=GPIO%d\n",
                SK_SCL_PIN, PCA9570_LED_EN_BIT, PCA9570_LED_EN_ACTIVE_LOW ? "low" : "high", SK_DATA_PIN);
  if (!i2cLinesIdleHigh()) {
    Serial.println("  SDA/SCL stuck low: check VCC_3V3 at the 1k pull-ups (R10/R13). Skipping the scan,");
    Serial.println("  every address would 'ACK' (SDA never rises) and reads would return 0x00.");
    Serial.println(HELP);
    return;
  }
  i2cScan();
  uint8_t v;
  if (pcaRead(&v)) {
    Serial.printf("PCA9570 found, output register = 0x%02X\n", v);
    pcaPrint(v);
    g_pcaShadow = v;
    pcaSetLedGate(true);
  } else {
    Serial.println("PCA9570 NOT responding at 0x24; blinking anyway");
  }
  Serial.println(HELP);
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 3000) delay(10);

  // Keep the LED data line quiet while we talk I2C
  digitalWrite(SK_DATA_PIN, LOW);
  pinMode(SK_DATA_PIN, OUTPUT);
  sdaRelease();
  sclRelease();

  Serial.println();
  report();
}

static const struct { const char *name; uint8_t r, g, b; } kColours[] = {
  {"RED", 255, 0, 0}, {"GREEN", 0, 255, 0}, {"BLUE", 0, 0, 255}, {"WHITE", 255, 255, 255}};
static int  g_colour = 0;       // index into kColours
static bool g_cycle  = true;    // advance colour on every "on" period

void loop() {
  static uint32_t last = 0;
  static bool on = false;

  delay(1);   // yield: a non-yielding loop stalls the USB CDC driver

  if (millis() - last >= SK_BLINK_MS) {
    last = millis();
    on = !on;
    if (on) {
      const auto &c = kColours[g_colour];
      skShow(c.r, c.g, c.b, SK_BRIGHTNESS);
      Serial.printf("LED: %s\n", c.name);
      if (g_cycle) g_colour = (g_colour + 1) % 3;
    } else {
      skShow(0, 0, 0, SK_BRIGHTNESS);
      Serial.println("LED: off");
    }
  }

  while (Serial.available()) {
    char c = Serial.read();
    uint8_t v;
    switch (c) {
      case 's': report(); break;
      case 'r':
        if (pcaRead(&v)) { Serial.printf("PCA9570 output register = 0x%02X\n", v); pcaPrint(v); }
        break;
      case 'e': pcaSetLedGate(true); break;
      case 'd': pcaSetLedGate(false); break;
      case 'c': g_cycle = true; Serial.println("cycling red/green/blue"); break;
      case 'R': g_cycle = false; g_colour = 0; Serial.println("locked RED"); break;
      case 'G': g_cycle = false; g_colour = 1; Serial.println("locked GREEN"); break;
      case 'B': g_cycle = false; g_colour = 2; Serial.println("locked BLUE"); break;
      case 'W': g_cycle = false; g_colour = 3; Serial.println("locked WHITE"); break;
      case 'h': case '?': Serial.println(HELP); break;
      default: break;   // ignore stray bytes (e.g. esptool sync packets)
    }
  }
}
