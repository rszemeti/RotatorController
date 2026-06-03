// Minimal blinky sketch with heartbeat LED and normal Serial2 setup.

#include <Arduino.h>

const unsigned long STATUS_INTERVAL_MS = 1000;
const int serialTxPin = 4;
const int serialRxPin = 5;

unsigned long lastStatusMs = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(19200);
  unsigned long usbStart = millis();
  while (!Serial && (millis() - usbStart < 2000)) {
    delay(10);
  }

  Serial2.setTX(serialTxPin);
  Serial2.setRX(serialRxPin);
  Serial2.begin(19200);

  if (Serial) {
    Serial.println("LOOPBACK BOOT OK");
  }
}

void loop() {
  if (millis() - lastStatusMs >= STATUS_INTERVAL_MS) {
    if (Serial) {
      Serial.println("BLINKY ALIVE");
    }
    lastStatusMs = millis();
  }
}
