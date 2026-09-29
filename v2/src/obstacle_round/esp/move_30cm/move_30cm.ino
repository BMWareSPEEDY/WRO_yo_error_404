/*
 * ============================================================================
 *  WRO 2026 Future Engineers - Team Yo_Error_404
 *  move_30cm - ESP32-C3 Super Mini (V2 bot) - encoder distance test
 * ============================================================================
 *
 *  WHAT IT DOES
 *    Press START. The car drives straight forward until the wheel encoder has
 *    counted 30 cm, then stops. That is all. No ToF, no BNO055, no camera, no
 *    obstacle logic, no turns.
 *
 *  Ported from the classic-ESP32 loop sketch (2500 mm -> 90 deg turn x12):
 *  the encoder maths and the 90 ticks/metre calibration are kept, everything
 *  else is stripped out and the pins/motor driver are changed to the V2 bot.
 *
 *  PATCH NOTES
 *    PATCH 1  (2026-09-21)  First version. Pins moved from the classic ESP32
 *             (IN1 27 / IN2 26 / ENA 14 / servo 13 / encoder 18,19 / btn 15)
 *             to the ESP32-C3 Super Mini V2 pinout. Motor driver changed from
 *             the L298-style IN1+IN2+ENA to the MD10112's PWM+DIR pair, which
 *             is how open_slave.ino drives this car. Encoder moved OFF GPIO18/
 *             19 - see the warning below. Added a hard timeout so the car can
 *             never run away if the encoder is silent.
 *    PATCH 2  (2026-09-21)  Removed the NeoPixel strip - this car does not have
 *             one. Status is reported over serial only. GPIO7 is now free.
 *    PATCH 3  (2026-09-21)  Encoder moved to GPIO4 (A) and GPIO5 (B) - proved
 *             on the bench with encoder_probe, not guessed. Motor pins are now
 *             driven LOW as the very FIRST thing in setup(), before serial or
 *             anything else: encoder_probe left IN1 as INPUT_PULLUP, the
 *             MD10112 read that HIGH as 100% duty and the car ran away at full
 *             speed. A pin that commands the motor is never left an input.
 *    PATCH 4  (2026-09-21)  Servo spun continuously on boot. PATCH 3 had moved
 *             driveMotor(0) - an analogWrite - ahead of the servo attach, and
 *             on ESP32 core 3.x analogWrite claims an LEDC timer at ~1 kHz.
 *             ESP32Servo then shared that timer instead of getting a clean
 *             50 Hz one, so the servo saw nonsense pulse widths and ran away.
 *             Order now matches open_slave.ino, which is known good: the motor
 *             pins are still made safe first, but with digitalWrite LOW only
 *             (no LEDC involved), the servo attaches next and claims its 50 Hz
 *             timer, and analogWrite is not called until after that.
 *    PATCH 5  (2026-09-21)  Pressing START produced no reaction and no serial
 *             output at all, so there was no way to tell whether the button,
 *             the encoder or the motor was at fault. Added a 2 Hz heartbeat
 *             that prints the raw START pin level and the live tick count, so
 *             the board is never silent again. Drive speed raised 120 -> 150:
 *             120 PWM may simply be below the stall torque of this drivetrain.
 *    PATCH 6  (2026-09-21)  "Nothing moves when I press START" - because the
 *             run had ALREADY completed (29 ticks vs the 27 target) and DONE
 *             was a dead end: the button was only ever read in WAITING, so the
 *             sketch could run exactly once per power cycle. The button is now
 *             read in every state - a press in DONE re-arms for another run,
 *             and a press while DRIVING is an emergency stop.
 *    PATCH 7  (2026-09-21)  Calibrated. run_5s_ticks measured 2913 ticks over
 *             ~1.1 m, so TICKS_PER_METER goes 90 -> 2648 and the 30 cm target
 *             goes 27 -> 794 ticks. The inherited 90 was from the old bot and
 *             ~29x too small, which is why the car twitched and stopped.
 *             Also: the encoder's signed position counts NEGATIVE when driving
 *             forward on this car, so direction logic must not assume +ve is
 *             forward. This sketch uses labs() and is unaffected.
 *             NOTE: 1.1 m was a rough tape measurement, and the run includes
 *             ramp-up and coast, so treat 2648 as a first cut - re-measure
 *             over a longer distance if 30 cm comes out off.
 *    PATCH 8  (2026-09-21)  It came out off: 794 ticks measured 27 cm on the
 *             floor, 10% short. Recalibrated from that real result instead of
 *             the eyeballed 1.1 m - 794 / 0.27 = 2941 ticks/m, so the target
 *             becomes 882 ticks. This figure is measured over the actual 30 cm
 *             move, so it also folds in the ramp-up and the coast after the
 *             motor cuts, which a long free run does not represent.
 *    PATCH 9  (2026-09-21)  882 ticks measured 29 cm - 1 cm short. Scaled once
 *             more: 882 / 0.29 = 3041 ticks/m, target 912 ticks. Convergence
 *             so far: 794->27 cm, 882->29 cm, 912->30 cm expected. Note 1 cm
 *             is close to tape-measure and coast-variation noise, so chasing
 *             it further has limited value - coast distance also shifts with
 *             battery charge, which no fixed tick target can compensate for.
 *
 *  !! ENCODER PIN WARNING !!
 *    On the ESP32-C3, GPIO18 and GPIO19 ARE the native USB D-/D+ lines. They
 *    are what the Pi talks to this board over. Wiring the encoder there kills
 *    the USB link and the board can no longer be flashed or read normally, so
 *    the encoder CANNOT stay on 18/19 the way it was on the classic ESP32.
 *    The encoder on this car is on GPIO4 (A) and GPIO5 (B). That was measured
 *    with the encoder_probe sketch, which counted edges on every pin while the
 *    wheels turned - 4 and 5 were the only two that moved.
 *
 *  SAFETY
 *    Speed is capped at 170 PWM (team hard limit). The run also aborts after
 *    RUN_TIMEOUT_MS even if no ticks ever arrive, so a disconnected encoder
 *    stops the car instead of letting it drive off.
 *
 *  UPLOAD
 *    arduino-cli compile --upload -p <port> \
 *      --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc esp/move_30cm
 *    Libraries: ESP32Servo
 *
 *  NOTE: flashing this REPLACES open_slave.ino on the board. Reflash
 *  esp/open_slave/open_slave.ino.merged.bin to put the robot back to normal.
 * ============================================================================
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// ---------------------------------------------------------------- pins (V2 bot, ESP32-C3)
#define IN1_PIN 1    // MD10112 PWM
#define IN2_PIN 0

// Direction polarity: LOW = forward on this car. Verified by tick sign -
// positive ticks have always meant forward, so +speed must produce them.
#define MOTOR_FORWARD_LEVEL LOW
#define MOTOR_REVERSE_LEVEL HIGH    // MD10112 DIR (HIGH = forward)
#define SERVO_PIN      6
#define START_BTN_PIN 10    // to GND, internal pull-up

#define ENCODER_A      4    // proved with encoder_probe on the bench
#define ENCODER_B      5

// ---------------------------------------------------------------- distance
// Calibrated against a measured run, not a tape estimate: 794 ticks produced
// exactly 27 cm on the floor, so 794 / 0.27 = 2941 ticks per metre.
// (Cross-check: at 2941, the earlier 5 s / 2913-tick run was ~99 cm, not the
// 1.1 m estimated by eye - the same 10% the first target came up short by.)
const float TICKS_PER_METER = 2078.0;                      // 2494 ticks / 1.2 m
const float TARGET_MM       = 300.0;                       // <-- the 30 cm
const long  TARGET_TICKS    = (long)(TICKS_PER_METER * TARGET_MM / 1000.0);   // = 623

// ---------------------------------------------------------------- motion
#define DRIVE_SPEED     120   // PWM (hard cap 170)
#define MAX_PWM         170
#define SERVO_CENTER     90
#define RUN_TIMEOUT_MS  4000  // runaway guard: stop even if no ticks arrive
#define BUTTON_DEBOUNCE_MS 60

Servo steeringServo;

enum State { WAITING, DRIVING, DONE };
State state = WAITING;

volatile long encoderTicks = 0;
uint32_t runStartMs = 0;
bool lastButton = HIGH;
uint32_t lastBeatMs = 0;

void IRAM_ATTR encoderISR() {
  if (digitalRead(ENCODER_B) == HIGH) encoderTicks++;
  else                                encoderTicks--;
}

// MD10112: PWM on IN1, direction on IN2 - same as open_slave.ino.
void driveMotor(int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  if (speed == 0) {
    analogWrite(IN1_PIN, 0);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN2_PIN, speed > 0 ? MOTOR_FORWARD_LEVEL : MOTOR_REVERSE_LEVEL);
    analogWrite(IN1_PIN, abs(speed));
  }
}

void setup() {
  // Motor pins FIRST, driven low, before serial or any other setup. Left as
  // inputs they pull HIGH and the MD10112 reads that as full speed forward.
  // Motor pins safe FIRST - but with digitalWrite only. Left as inputs they
  // pull HIGH and the MD10112 reads that as 100% duty = full speed forward.
  // Deliberately NOT analogWrite here: that would claim an LEDC timer before
  // the servo can get its 50 Hz one, and the servo would spin (see PATCH 4).
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);

  Serial.begin(115200);
  pinMode(START_BTN_PIN, INPUT_PULLUP);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  // Servo before any analogWrite, exactly as open_slave.ino does it.
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  steeringServo.write(SERVO_CENTER);        // wheels dead straight, never moved again

  driveMotor(0);                            // first analogWrite, after the servo has its timer

  Serial.print("move_30cm ready - target ");
  Serial.print(TARGET_MM, 0);
  Serial.print(" mm = ");
  Serial.print(TARGET_TICKS);
  Serial.println(" ticks. Press START.");
}

void loop() {
  // One button edge-detector for every state, so the sketch is never stuck.
  bool btnNow = digitalRead(START_BTN_PIN);
  bool btnPressed = (btnNow == LOW && lastButton == HIGH);
  lastButton = btnNow;

  // Heartbeat: the board is never silent, so a dead button or a dead encoder
  // is visible immediately instead of looking like "nothing happened".
  uint32_t nowMs = millis();
  if (nowMs - lastBeatMs >= 500) {
    lastBeatMs = nowMs;
    noInterrupts();
    long t = encoderTicks;
    interrupts();
    Serial.print("state=");
    Serial.print(state == WAITING ? "WAITING" : (state == DRIVING ? "DRIVING" : "DONE"));
    Serial.print("  START_pin=");
    Serial.print(digitalRead(START_BTN_PIN) == LOW ? "PRESSED" : "up");
    Serial.print("  ticks=");
    Serial.print(t);
    Serial.print("/");
    Serial.println(TARGET_TICKS);
  }

  switch (state) {

    case WAITING: {
      if (btnPressed) {
        delay(BUTTON_DEBOUNCE_MS);
        if (digitalRead(START_BTN_PIN) == LOW) {
          encoderTicks = 0;
          runStartMs = millis();
          state = DRIVING;
          Serial.println("GO");
        }
      }
      break;
    }

    case DRIVING: {
      if (btnPressed) {                      // press again = emergency stop
        driveMotor(0);
        state = DONE;
        Serial.println("ABORTED by button - motor stopped");
        break;
      }
      driveMotor(DRIVE_SPEED);

      noInterrupts();
      long ticks = encoderTicks;
      interrupts();

      bool arrived = labs(ticks) >= TARGET_TICKS;
      bool timedOut = (millis() - runStartMs) > RUN_TIMEOUT_MS;

      if (arrived || timedOut) {
        driveMotor(0);
        state = DONE;
        Serial.print(timedOut ? "TIMEOUT - stopped at " : "ARRIVED - stopped at ");
        Serial.print(labs(ticks));
        Serial.print(" ticks = ");
        Serial.print(labs(ticks) * 1000.0 / TICKS_PER_METER, 0);
        Serial.println(" mm");
        if (timedOut) Serial.println("  (no/low ticks -> check the encoder wiring on GPIO2/GPIO3)");
      }
      break;
    }

    case DONE:
      driveMotor(0);
      if (btnPressed) {                      // re-arm for another run
        delay(BUTTON_DEBOUNCE_MS);
        encoderTicks = 0;
        state = WAITING;
        Serial.println("RE-ARMED - press START to run again");
      }
      break;
  }
}
