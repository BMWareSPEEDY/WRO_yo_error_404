/*
 * motor_stop - ESP32-C3 Super Mini (V2 bot) - SAFE STATE
 *
 * Does one thing: holds the drive motor off, forever. Flash this whenever the
 * car needs to be made safe in a hurry.
 *
 * The very first statements in setup() make IN1/IN2 outputs and write them
 * LOW, before anything else runs. That ordering matters: the MD10112 reads
 * IN1 as its PWM input, so any moment those pins are left as inputs they are
 * pulled HIGH and the driver sees 100% duty = full speed forward.
 *
 * PATCH NOTES
 *   PATCH 1 (2026-09-21) First version, written after encoder_probe set the
 *           motor pins to INPUT_PULLUP and drove the car off at full throttle.
 */
#include <Arduino.h>

#define IN1_PIN 1    // MD10112 PWM   - HIGH here = full speed, never leave floating
#define IN2_PIN 0    // MD10112 DIR

void setup() {
  // FIRST. Before serial, before anything.
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(IN1_PIN, 0);

  Serial.begin(115200);
  delay(200);
  Serial.println("motor_stop - drive motor held OFF. Car is safe.");
}

void loop() {
  analogWrite(IN1_PIN, 0);
  digitalWrite(IN2_PIN, LOW);
  delay(100);
}
