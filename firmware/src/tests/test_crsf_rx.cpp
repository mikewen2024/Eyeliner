/**
 * @file test_crsf_rx.cpp
 * @brief Bench check of an RC receiver (CRSF / ELRS) on a UART, using
 *        lib/AlfredoCRSF.
 *
 * Build/flash: pio run -e test_crsf_rx -t upload
 * Monitor:     pio device monitor -e test_crsf_rx
 *
 * What it does:
 *   1. Probes both receiver pins as plain inputs for a moment: idle level and
 *      edge count. A CRSF receiver streams channel frames continuously, so its
 *      TX line idles high and toggles constantly even without a radio link
 *      (link statistics frames keep coming). This tells us which wire is which
 *      before any UART is touched. With CRSF_AUTO_PINS=1 (default) the pins are
 *      swapped automatically if only the configured TX pin shows traffic.
 *   2. Opens UART1 at CRSF_BAUD (420000 8N1) on the chosen pins and feeds it
 *      to AlfredoCRSF.
 *   3. Prints, twice a second: raw bytes received, bytes/s, link state,
 *      channels 1..16 in us, and the link statistics the receiver reports.
 *
 * Serial commands (single key):
 *   d   toggle hex dump of every raw byte (gap > 2 ms starts a new line)
 *   x   swap RX/TX pins and restart the UART
 *   b   cycle baud 420000 -> 416666 -> 115200 -> 921600 -> 420000
 *   p   re-run the pin probe (UART is stopped and restarted)
 *   l   print full link statistics once
 *
 * Wiring (bench setup this was written for):
 *   GPIO43 (D6)  <- receiver TX   (MCU RX)
 *   GPIO6  (D5)  -> receiver RX   (MCU TX, telemetry to the receiver)
 *   GND shared, receiver powered from 5V.
 */
#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

/* ---------------------------------------------------------------------------
 * Configuration (override any of these from build_flags)
 * ------------------------------------------------------------------------- */
#ifndef CRSF_RX_PIN
#define CRSF_RX_PIN 43         // D6, receiver TX -> MCU
#endif
#ifndef CRSF_TX_PIN
#define CRSF_TX_PIN 6          // D5, MCU -> receiver RX
#endif
#ifndef CRSF_BAUD
#define CRSF_BAUD CRSF_BAUDRATE
#endif
#ifndef CRSF_AUTO_PINS
#define CRSF_AUTO_PINS 1       // swap pins if only the TX pin shows traffic
#endif
#ifndef CRSF_PROBE_MS
#define CRSF_PROBE_MS 300
#endif
#ifndef CRSF_STATUS_MS
#define CRSF_STATUS_MS 500
#endif

/* ---------------------------------------------------------------------------
 * Stream wrapper: counts bytes and optionally hex-dumps them on the way to
 * the CRSF parser.
 * ------------------------------------------------------------------------- */
class TapStream : public Stream {
public:
    explicit TapStream(Stream &inner) : _in(inner) {}
    uint32_t rxBytes = 0;
    uint32_t txBytes = 0;
    bool dump = false;

    int available() override { return _in.available(); }
    int peek() override { return _in.peek(); }
    int read() override {
        int b = _in.read();
        if (b >= 0) {
            rxBytes++;
            if (dump) {
                uint32_t now = micros();
                if (now - _lastDumpUs > 2000) Serial.println();
                _lastDumpUs = now;
                Serial.printf("%02X ", b);
            }
        }
        return b;
    }
    size_t write(uint8_t b) override { txBytes++; return _in.write(b); }
    size_t write(const uint8_t *buf, size_t n) override { txBytes += n; return _in.write(buf, n); }
    void flush() override { _in.flush(); }

private:
    Stream &_in;
    uint32_t _lastDumpUs = 0;
};

static const uint32_t kBauds[] = { 420000, 416666, 115200, 921600 };

static HardwareSerial rxSerial(1);
static TapStream tap(rxSerial);
static AlfredoCRSF crsf;

static int g_rxPin = CRSF_RX_PIN;
static int g_txPin = CRSF_TX_PIN;
static uint32_t g_baud = CRSF_BAUD;
static int g_baudIdx = 0;
static uint32_t g_lastBytes = 0;   // for the bytes/s figure in the status line

static volatile uint32_t g_edgesA = 0, g_edgesB = 0;
static void IRAM_ATTR isrA() { g_edgesA++; }
static void IRAM_ATTR isrB() { g_edgesB++; }

/* Probe both pins as inputs: idle level + edge count over CRSF_PROBE_MS. */
static void probePins()
{
    rxSerial.end();
    pinMode(g_rxPin, INPUT);
    pinMode(g_txPin, INPUT);
    delay(2);
    int lvlRx = digitalRead(g_rxPin);
    int lvlTx = digitalRead(g_txPin);

    g_edgesA = g_edgesB = 0;
    attachInterrupt(digitalPinToInterrupt(g_rxPin), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(g_txPin), isrB, CHANGE);
    delay(CRSF_PROBE_MS);
    detachInterrupt(digitalPinToInterrupt(g_rxPin));
    detachInterrupt(digitalPinToInterrupt(g_txPin));
    uint32_t eRx = g_edgesA, eTx = g_edgesB;

    Serial.printf("[probe] GPIO%d (cfg RX): idle=%s edges=%lu in %d ms\n",
                  g_rxPin, lvlRx ? "HIGH" : "LOW", (unsigned long)eRx, CRSF_PROBE_MS);
    Serial.printf("[probe] GPIO%d (cfg TX): idle=%s edges=%lu in %d ms\n",
                  g_txPin, lvlTx ? "HIGH" : "LOW", (unsigned long)eTx, CRSF_PROBE_MS);

    if (eRx == 0 && eTx == 0) {
        Serial.println("[probe] no activity on either pin: receiver unpowered, "
                       "not talking, or wired elsewhere");
    } else if (eRx == 0 && eTx > 0) {
        Serial.println("[probe] traffic only on the configured TX pin");
#if CRSF_AUTO_PINS
        Serial.println("[probe] swapping RX/TX");
        int t = g_rxPin; g_rxPin = g_txPin; g_txPin = t;
#endif
    } else if (eRx > 0 && eTx > 0) {
        Serial.println("[probe] traffic on both pins (receiver may be echoing, "
                       "or both are receiver outputs)");
    }
}

static void startUart()
{
    rxSerial.end();
    rxSerial.setRxBufferSize(1024);
    rxSerial.begin(g_baud, SERIAL_8N1, g_rxPin, g_txPin);
    tap.rxBytes = tap.txBytes = 0;
    g_lastBytes = 0;
    crsf.begin(tap);
    Serial.printf("[uart] UART1 %lu 8N1  RX=GPIO%d  TX=GPIO%d\n",
                  (unsigned long)g_baud, g_rxPin, g_txPin);
}

static void printLinkStats()
{
    const crsfLinkStatistics_t *ls = crsf.getLinkStatistics();
    Serial.printf("[link] up=%d  RSSI1=-%u RSSI2=-%u LQ=%u%% SNR=%d  ant=%u mode=%u txpwr=%u  "
                  "dlRSSI=-%u dlLQ=%u%% dlSNR=%d\n",
                  crsf.isLinkUp(), ls->uplink_RSSI_1, ls->uplink_RSSI_2,
                  ls->uplink_Link_quality, ls->uplink_SNR, ls->active_antenna,
                  ls->rf_Mode, ls->uplink_TX_Power, ls->downlink_RSSI,
                  ls->downlink_Link_quality, ls->downlink_SNR);
}

void setup()
{
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);
    delay(200);

    Serial.println();
    Serial.println("=== CRSF receiver test ===");
    probePins();
    startUart();
}

void loop()
{
    crsf.update();

    while (Serial.available()) {
        char c = Serial.read();
        switch (c) {
        case 'd':
            tap.dump = !tap.dump;
            Serial.printf("\n[cmd] hex dump %s\n", tap.dump ? "ON" : "OFF");
            break;
        case 'x': {
            int t = g_rxPin; g_rxPin = g_txPin; g_txPin = t;
            Serial.println("[cmd] swapping pins");
            startUart();
            break;
        }
        case 'b':
            g_baudIdx = (g_baudIdx + 1) % (int)(sizeof(kBauds) / sizeof(kBauds[0]));
            g_baud = kBauds[g_baudIdx];
            startUart();
            break;
        case 'p':
            probePins();
            startUart();
            break;
        case 'l':
            printLinkStats();
            break;
        default:
            break;
        }
    }

    static uint32_t lastStatus = 0;
    uint32_t now = millis();
    if (now - lastStatus >= CRSF_STATUS_MS) {
        uint32_t dt = now - lastStatus;
        uint32_t bytes = tap.rxBytes;
        uint32_t bps = (bytes - g_lastBytes) * 1000UL / (dt ? dt : 1);
        g_lastBytes = bytes;
        lastStatus = now;

        if (tap.dump) Serial.println();
        Serial.printf("rx=%lu B (%lu B/s) link=%s | ",
                      (unsigned long)bytes, (unsigned long)bps,
                      crsf.isLinkUp() ? "UP  " : "down");
        for (int ch = 1; ch <= 8; ch++) Serial.printf("%4d ", crsf.getChannel(ch));
        Serial.print("| ");
        for (int ch = 9; ch <= 16; ch++) Serial.printf("%4d ", crsf.getChannel(ch));
        const crsfLinkStatistics_t *ls = crsf.getLinkStatistics();
        Serial.printf("| LQ=%u%% RSSI=-%u SNR=%d\n",
                      ls->uplink_Link_quality, ls->uplink_RSSI_1, ls->uplink_SNR);
    }
}
