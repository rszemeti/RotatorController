#include <Arduino.h>

const unsigned long STATUS_INTERVAL_MS = 1000;
const int serialTxPin = 4;
const int serialRxPin = 5;

unsigned long lastStatusMs = 0;
unsigned long testCounter = 1;
String rxBuffer;

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
    Serial2.print("TEST ");
    Serial2.println(testCounter++);
    if (Serial) {
      Serial.println("TX TEST SENT");
    }
    lastStatusMs = millis();
  }

  while (Serial2.available() > 0) {
    char incoming = (char)Serial2.read();
    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      if (rxBuffer.length() > 0 && Serial) {
        Serial.print("RX LOOPBACK: ");
        Serial.println(rxBuffer);
      }
      rxBuffer = "";
    } else {
      rxBuffer += incoming;
    }
  }
}
