/*
 * calibrate_ticks - ESP32-C3 Super Mini (V2 bot)
 *
 * Measures how many encoder ticks this car actually produces per metre.
 *
 * The inherited TICKS_PER_METER = 90 came from the old classic-ESP32 bot and
 * is badly wrong here: at full throttle this encoder was seen producing
 * ~3450 edges/s, so a 27-tick target was hit almost instantly and the car
 * moved only a few centimetres.
 *
 * HOW TO USE
 *   1. Put the car on the floor with clear space ahead and a tape measure
 *      lined up at its starting point.
 *   2. Press START. The car drives straight for RUN_TIME_MS, then stops.
 *   3. Read the tick total it prints, measure the distance it actually
 *      travelled, and:  TICKS_PER_METER = ticks / distance_in_metres
 *   4. Press START again to repeat. Average a few runs.
 *
 * The drivetrain cannot be back-driven by hand, so driving for a fixed time
 * is the only way to get a known distance to divide by.
 *
 * PATCH NOTES
 *   PATCH 1 (2026-09-21) First version.
 *   PATCH 2 (2026-09-21) Too slow and too short a run. Speed 150 -> 170 and
 *           run time 1.5 s -> 5 s. 170 is the team hard cap; the old sketch's
 *           BASE_SPEED of 180 is over it and is not used. A 5 s run at full
 *           speed covers a LOT of floor - make sure the space is clear.
 */
#include <Arduino.h>
#include <ESP32Servo.h>

#define IN1_PIN 1    // MD10112 PWM
#define IN2_PIN 0

// Direction polarity: LOW = forward on this car. Verified by tick sign -
// positive ticks have always meant forward, so +speed must produce them.
#define MOTOR_FORWARD_LEVEL LOW
#define MOTOR_REVERSE_LEVEL HIGH    // MD10112 DIR (HIGH = forward)
#define SERVO_PIN      6
#define START_BTN_PIN 10
#define ENCODER_A      4
#define ENCODER_B      5

#define DRIVE_SPEED   170   // team hard cap. The old sketch's 180 is over it.
#define MAX_PWM       170
#define SERVO_CENTER   90
#define RUN_TIME_MS  5000   // drive this long, then stop and report
#define BUTTON_DEBOUNCE_MS 60

Servo steeringServo;
enum State { WAITING, DRIVING, DONE };
State state = WAITING;

volatile long encoderTicks = 0;
uint32_t runStartMs = 0, lastBeatMs = 0;
bool lastButton = HIGH;

void IRAM_ATTR encoderISR() {
  if (digitalRead(ENCODER_B) == HIGH) encoderTicks++;
  else                                encoderTicks--;
}

void driveMotor(int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  if (speed == 0) { analogWrite(IN1_PIN, 0); digitalWrite(IN2_PIN, LOW); }
  else { digitalWrite(IN2_PIN, speed > 0 ? MOTOR_FORWARD_LEVEL : MOTOR_REVERSE_LEVEL); analogWrite(IN1_PIN, abs(speed)); }
}

void setup() {
  // Motor pins safe first, digitalWrite only - no analogWrite before the servo
  // attaches, or the servo shares its LEDC timer and spins (move_30cm PATCH 4).
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);

  Serial.begin(115200);
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  steeringServo.write(SERVO_CENTER);
  driveMotor(0);

  Serial.println("calibrate_ticks ready. Press START to drive 5 s at 170 PWM.");
}

void loop() {
  bool btnNow = digitalRead(START_BTN_PIN);
  bool btnPressed = (btnNow == LOW && lastButton == HIGH);
  lastButton = btnNow;

  uint32_t nowMs = millis();
  if (nowMs - lastBeatMs >= 500) {
    lastBeatMs = nowMs;
    noInterrupts(); long t = encoderTicks; interrupts();
    Serial.print("state=");
    Serial.print(state == WAITING ? "WAITING" : (state == DRIVING ? "DRIVING" : "DONE"));
    Serial.print("  ticks=");
    Serial.println(t);
  }

  switch (state) {
    case WAITING:
      if (btnPressed) {
        delay(BUTTON_DEBOUNCE_MS);
        if (digitalRead(START_BTN_PIN) == LOW) {
          encoderTicks = 0;
          runStartMs = millis();
          state = DRIVING;
          Serial.println("GO - driving 5 s at 170 PWM");
        }
      }
      break;

    case DRIVING: {
      if (btnPressed) {                       // press again = emergency stop
        driveMotor(0);
        state = DONE;
        Serial.println("ABORTED by button");
        break;
      }
      driveMotor(DRIVE_SPEED);
      if (millis() - runStartMs >= RUN_TIME_MS) {
        driveMotor(0);
        state = DONE;
        noInterrupts(); long t = encoderTicks; interrupts();
        Serial.print(">>> RUN DONE: ");
        Serial.print(labs(t));
        Serial.println(" ticks");
        Serial.println(">>> Measure the distance travelled, then:");
        Serial.println(">>>   TICKS_PER_METER = ticks / distance_in_metres");
        Serial.println(">>> Press START to run again.");
      }
      break;
    }

    case DONE:
      driveMotor(0);
      if (btnPressed) {
        delay(BUTTON_DEBOUNCE_MS);
        encoderTicks = 0;
        state = WAITING;
        Serial.println("RE-ARMED");
      }
      break;
  }
}
