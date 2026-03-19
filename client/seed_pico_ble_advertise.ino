#include <bluefruit.h>

bool advertising = false;
unsigned long adStartTime = 0;
const unsigned long AD_DURATION = 5000;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("XIAOBeacon");

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.setInterval(80, 80);
  Bluefruit.Advertising.setFastTimeout(0);

  Serial.println("Ready - waiting for signal from Pi...");
}

void loop() {
  if (Serial1.available()) {
    String msg = Serial1.readStringUntil('\n');
    msg.trim();
    if (msg == "START") {
      Serial.println("Received START - advertising for 5 seconds...");
      Bluefruit.Advertising.start(0);
      advertising = true;
      adStartTime = millis();
    }
  }

  if (advertising && millis() - adStartTime >= AD_DURATION) {
    Bluefruit.Advertising.stop();
    advertising = false;
    Serial.println("5 seconds done - stopped advertising.");
  }
}
