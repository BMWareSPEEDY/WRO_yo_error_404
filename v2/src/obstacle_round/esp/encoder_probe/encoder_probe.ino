/*
 * encoder_probe - ESP32-C3 Super Mini (V2 bot)
 *
 * Finds out which GPIOs the wheel encoder is actually wired to. The motor is
 * never driven, so nothing moves: spin a wheel BY HAND and the two pins that
 * count up are the encoder's A and B channels.
 *
 * Every candidate pin is read as INPUT_PULLUP and every edge is counted. A
 * table is printed twice a second. Pins that sit still show 0.
 *
 * PATCH NOTES
 *   PATCH 1 (2026-09-21) First version. Written because IN1 (GPIO0) and a
 *           reported encoder channel (GPIO0) cannot both own the same pin.
 */
#include <Arduino.h>

// every pin broken out on the C3 Super Mini that could plausibly carry a
// channel, including 0 and 1 (the motor is never driven here, so they are
// safe to read)
const uint8_t PINS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21};
const uint8_t N = sizeof(PINS);

volatile uint32_t edges[N];
uint8_t lastLevel[N];
uint32_t lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  for (uint8_t i = 0; i < N; i++) {
    pinMode(PINS[i], INPUT_PULLUP);
    lastLevel[i] = digitalRead(PINS[i]);
    edges[i] = 0;
  }
  Serial.println();
  Serial.println("encoder_probe - motor is OFF. SPIN A WHEEL BY HAND.");
  Serial.println("The two pins whose counts climb are the encoder channels.");
}

void loop() {
  // poll fast; an encoder on a hand-spun wheel is only a few hundred Hz
  for (uint8_t i = 0; i < N; i++) {
    uint8_t lv = digitalRead(PINS[i]);
    if (lv != lastLevel[i]) {
      lastLevel[i] = lv;
      edges[i]++;
    }
  }

  uint32_t now = millis();
  if (now - lastPrint >= 500) {
    lastPrint = now;
    Serial.print("edges:");
    for (uint8_t i = 0; i < N; i++) {
      if (edges[i] == 0) continue;            // only show pins that moved
      Serial.print("  GPIO");
      Serial.print(PINS[i]);
      Serial.print("=");
      Serial.print(edges[i]);
    }
    Serial.println();
  }
}
