/*
 * ERS — Seeed XIAO nRF52840 BLE Emergency Advertiser
 * Uses Adafruit Bluefruit library (comes with Seeed nRF52 board package)
 */

#include <bluefruit.h>

// ── Config ────────────────────────────────────────────────────────────────────

#define TRIGGER_PIN          D2
#define ADVERT_DURATION_MS   10000

#define BLE_DEVICE_NAME      "ERS-PERSONAL-EMERGENCY"

// Custom 128-bit UUIDs — note: Bluefruit wants LSB-first byte arrays
// 19B10000-E8F2-537E-4F6C-D104768A1214
const uint8_t ERS_SERVICE_UUID[] = {
    0x14, 0x12, 0x8A, 0x76, 0x04, 0xD1, 0x6C, 0x4F,
    0x7E, 0x53, 0xF2, 0xE8, 0x00, 0x00, 0xB1, 0x19
};
// 19B10001-E8F2-537E-4F6C-D104768A1214
const uint8_t ERS_CHAR_UUID[] = {
    0x14, 0x12, 0x8A, 0x76, 0x04, 0xD1, 0x6C, 0x4F,
    0x7E, 0x53, 0xF2, 0xE8, 0x01, 0x00, 0xB1, 0x19
};

// ── BLE objects ───────────────────────────────────────────────────────────────

BLEService        ersService(ERS_SERVICE_UUID);
BLECharacteristic emergencyChar(ERS_CHAR_UUID, BLERead | BLENotify, 1);  // 1 byte

// ── State ─────────────────────────────────────────────────────────────────────

bool          isAdvertising = false;
unsigned long advertStart   = 0;
bool          lastPinState  = LOW;

// ── Helpers ───────────────────────────────────────────────────────────────────

void startAdvertising() {
    uint8_t val = 1;
    emergencyChar.write(&val, 1);

    Bluefruit.Advertising.start(0);   // 0 = advertise indefinitely (we stop manually)
    isAdvertising = true;
    advertStart   = millis();

    digitalWrite(LED_RED,   LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE,  HIGH);
    Serial.println("[BLE] Advertising started — 10 second window");
}

void stopAdvertising() {
    Bluefruit.Advertising.stop();

    uint8_t val = 0;
    emergencyChar.write(&val, 1);
    isAdvertising = false;

    digitalWrite(LED_RED,   HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE,  HIGH);
    Serial.println("[BLE] Advertising stopped");
}

// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    pinMode(TRIGGER_PIN, INPUT);

    pinMode(LED_RED,   OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE,  OUTPUT);
    digitalWrite(LED_RED,   HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE,  HIGH);

    // Bluefruit init
    Bluefruit.begin();
    Bluefruit.setName(BLE_DEVICE_NAME);
    Bluefruit.setTxPower(4);   // -40, -20, -16, -12, -8, -4, 0, 4 dBm

    // Set up service + characteristic
    ersService.begin();
    emergencyChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    emergencyChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    emergencyChar.setFixedLen(1);
    emergencyChar.begin();

    uint8_t zero = 0;
    emergencyChar.write(&zero, 1);

    // Configure advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(ersService);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.setIntervalMS(100, 100);  // fast advertising

    Serial.println("[ERS] BLE Emergency Advertiser ready");
    Serial.print("[ERS] Monitoring trigger on pin D");
    Serial.println(TRIGGER_PIN);

    digitalWrite(LED_GREEN, LOW);  delay(300);
    digitalWrite(LED_GREEN, HIGH);
}

// ── Loop ──────────────────────────────────────────────────────────────────────

void loop() {
    bool pinState = (digitalRead(TRIGGER_PIN) == HIGH);

    if (pinState && !lastPinState && !isAdvertising) {
        startAdvertising();
    }
    lastPinState = pinState;

    if (isAdvertising && (millis() - advertStart >= ADVERT_DURATION_MS)) {
        stopAdvertising();
    }

    delay(10);
}