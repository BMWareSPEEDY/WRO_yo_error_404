/*
 * dir_probe - ESP32-C3 Super Mini (V2 bot)
 *
 * Finds out why the motor will not reverse. Drives four combinations of
 * GPIO0/GPIO1 in turn, 1.5 s each, and reports the SIGNED encoder movement of
 * every one. Nothing has to be guessed afterwards - the numbers say which pin
 * is PWM, which is DIR, and whether DIR does anything at all.
 *
 *   A  GPIO0 = PWM 150, GPIO1 = HIGH     the firmware's "forward"
 *   B  GPIO0 = PWM 150, GPIO1 = LOW      the firmware's "reverse"
 *   C  GPIO1 = PWM 150, GPIO0 = HIGH     as if the two were wired swapped
 *   D  GPIO1 = PWM 150, GPIO0 = LOW      swapped, other direction
 *
 * READING IT
 *   A moves, B zero          -> DIR never reaches the driver: broken wire /
 *                               joint on GPIO1, or the reverse half of the
 *                               bridge is dead. (A floating DIR input reads
 *                               HIGH, which is why forward still works.)
 *   A and B move opposite    -> DIR is fine, the fault is elsewhere.
 *   C or D move              -> the pins are wired the other way round.
 *   nothing moves            -> PWM is not getting through either.
 *
 * THE CAR MOVES DURING THIS - put it on a stand or leave clear space, and note
 * that it runs at 150 PWM in whichever direction each combination produces.
 *
 * PATCH NOTES
 *   PATCH 1 (2026-09-21) First version. Written after reverse produced exactly
 *           0 encoder ticks at every speed while the ESP reported applying the
 *           negative value, and forward worked normally in the same test.
 */
#include <Arduino.h>

#define PIN_A      0      // firmware calls this IN1 / PWM
#define PIN_B      1      // firmware calls this IN2 / DIR
#define ENC_A_PIN  4
#define ENC_B_PIN  5

#define TEST_PWM   150
#define PHASE_MS   1500
#define SETTLE_MS  800

volatile int32_t encPos = 0;      // signed: direction matters here

void IRAM_ATTR encISR() {
  if (digitalRead(ENC_B_PIN)) encPos++;
  else                        encPos--;
}

int32_t encRead() {
  noInterrupts();
  int32_t v = encPos;
  interrupts();
  return v;
}

void allOff() {
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
}

void phase(const char *name, int pwmPin, int dirPin, int dirLevel) {
  allOff();
  delay(SETTLE_MS);
  int32_t before = encRead();

  digitalWrite(dirPin, dirLevel);
  analogWrite(pwmPin, TEST_PWM);
  uint32_t t0 = millis();
  while (millis() - t0 < PHASE_MS) delay(5);

  analogWrite(pwmPin, 0);
  allOff();
  delay(SETTLE_MS);

  int32_t moved = encRead() - before;
  Serial.print(name);
  Serial.print(": PWM on GPIO");
  Serial.print(pwmPin);
  Serial.print(", GPIO");
  Serial.print(dirPin);
  Serial.print(dirLevel ? " HIGH" : " LOW ");
  Serial.print("  ->  moved ");
  Serial.print(moved);
  Serial.print(" ticks (");
  Serial.print(moved / 30.41, 1);
  Serial.println(" cm)");
}

void setup() {
  allOff();                       // motor pins safe before anything else
  Serial.begin(115200);
  delay(400);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encISR, RISING);

  Serial.println();
  Serial.println("dir_probe - 4 phases, 1.5 s each. THE CAR WILL MOVE.");
  Serial.println("starting in 3 s...");
  delay(3000);

  phase("A (fw as coded) ", PIN_A, PIN_B, HIGH);
  phase("B (rev as coded)", PIN_A, PIN_B, LOW);
  phase("C (swapped high)", PIN_B, PIN_A, HIGH);
  phase("D (swapped low) ", PIN_B, PIN_A, LOW);

  Serial.println();
  Serial.println("A moves, B zero  -> DIR never reaches the driver (wire/joint on GPIO1, or dead bridge half)");
  Serial.println("A and B opposite -> DIR is fine, look elsewhere");
  Serial.println("C or D move      -> PWM and DIR are wired the other way round");
  Serial.println("done - motor held off.");
}

void loop() {
  allOff();
  delay(200);
}
