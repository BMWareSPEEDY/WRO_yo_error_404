/*
 * ============================================================================
 *  tick_m_calculate - WRO 2026 Future Engineers, Team Yo_Error_404
 *  ESP32-C3 Super Mini (V2 bot)
 *
 *  WORKS OUT HOW MANY ENCODER TICKS THE CAR PRODUCES PER METRE.
 * ============================================================================
 *
 *  WHAT IT DOES
 *    Press START. The car drives straight forward for exactly 3 seconds, stops,
 *    and prints the total number of encoder ticks it counted. You measure how
 *    far it actually travelled, and that pair of numbers gives you the
 *    calibration. Press START again to repeat.
 *
 *  HOW TO USE IT
 *    1. Put the car on the floor with at least 1.5 m of clear space ahead.
 *       Line the BACK of the car up with a tape measure, or mark the floor.
 *    2. Press the START button. It drives for 3 seconds and stops itself.
 *    3. Read the tick total off the serial monitor (115200 baud).
 *    4. Measure how far it moved, from the same point on the car.
 *    5. Work it out:
 *
 *           TICKS_PER_METER  =  ticks  /  (distance_in_cm / 100)
 *
 *       Worked example: 2740 ticks over 90 cm
 *           2740 / 0.90  =  3044 ticks per metre
 *
 *    6. Repeat 2-3 times and average them. Tyre slip and the coast after the
 *       motor cuts make any single run a couple of percent out.
 *    7. Put the answer in obstacle_round/pi/config.py:
 *
 *           TICKS_PER_METER = <your number>
 *
 *  WHY IT MATTERS
 *    Every distance the car drives - dodge legs, corner backups, the finish
 *    push - is counted in ticks. If this number is wrong, every one of those
 *    distances is wrong by the same proportion. It is geometry (counts per
 *    wheel revolution), so it does NOT change with battery or speed; it only
 *    changes if the wheels, gearing or encoder change.
 *
 *  READING THE OUTPUT
 *    ticks           what you divide by your measured distance
 *    ticks/second    sanity check - should be steady between runs
 *    implied cm      what the run measures IF the current 3041 ticks/m is
 *                    right. If that disagrees with your tape, the tape wins
 *                    and 3041 needs updating.
 *
 *  NOTE
 *    Flashing this REPLACES open_slave on the board. Put the robot back with:
 *      ~/esp_flash/venv/bin/esptool --chip esp32c3 -p /dev/serial/by-id/usb-Espressif* \
 *          write-flash 0x0 ~/esp_flash/open_slave.ino.merged.bin
 *
 *  UPLOAD
 *    arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc tick_m_calculate
 *    Libraries: none beyond the ESP32 core.
 *
 *  PATCH NOTES
 *    PATCH 1 (2026-09-21) First version. Pins are the CORRECTED ones: PWM is
 *            GPIO1 and DIR is GPIO0 (proved with esp/dir_probe). Sketches
 *            written before that fix had them the other way round, which made
 *            the car run at full throttle whatever speed was asked for - so any
 *            calibration taken with them is suspect and worth redoing here.
 * ============================================================================
 */
#include <Arduino.h>

// ---------------------------------------------------------------- pins
#define IN1_PIN        1    // MD10112 PWM   (corrected: PWM is GPIO1)
#define IN2_PIN        0    // MD10112 DIR   (corrected: DIR is GPIO0)

// Direction polarity: LOW = forward on this car. Verified by tick sign -
// positive ticks have always meant forward, so +speed must produce them.
// Swap these two if the car ever drives backwards on a forward command.
#define MOTOR_FORWARD_LEVEL LOW
#define MOTOR_REVERSE_LEVEL HIGH
#define START_BTN_PIN 10    // to GND, internal pull-up
#define ENC_A_PIN      4    // encoder channel A - interrupt
#define ENC_B_PIN      5    // encoder channel B - direction

// ---------------------------------------------------------------- test
#define DRIVE_MS      3000UL   // <-- the 3 seconds
#define DRIVE_SPEED    120     // PWM, hard cap 170
#define MAX_PWM        170
#define BUTTON_DEBOUNCE_MS 60

// Only used to print the "implied cm" line. Your tape measure overrules it.
#define ASSUMED_TICKS_PER_METER 3041.0

volatile int32_t encTicks = 0;      // signed: forward counts up

enum State { WAITING, DRIVING, DONE };
State state = WAITING;

uint32_t runStart = 0, lastBeat = 0;
int32_t  startTicks = 0, runTicks = 0;
bool     lastButton = HIGH;

void IRAM_ATTR encISR() {
  if (digitalRead(ENC_B_PIN)) encTicks++;
  else                        encTicks--;
}

int32_t encRead() {
  noInterrupts();
  int32_t v = encTicks;
  interrupts();
  return v;
}

void motorWrite(int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  if (speed == 0) {
    analogWrite(IN1_PIN, 0);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN2_PIN, speed > 0 ? MOTOR_FORWARD_LEVEL : MOTOR_REVERSE_LEVEL);
    analogWrite(IN1_PIN, abs(speed));
  }
}

void report(int32_t ticks, uint32_t elapsed) {
  float secs = elapsed / 1000.0f;
  float tps = secs > 0 ? ticks / secs : 0;
  Serial.println();
  Serial.println("=========================================");
  Serial.print("  ticks          : ");
  Serial.println(ticks);
  Serial.print("  over           : ");
  Serial.print(secs, 2);
  Serial.println(" s");
  Serial.print("  ticks/second   : ");
  Serial.println(tps, 0);
  Serial.print("  implied cm     : ");
  Serial.print(ticks * 100.0f / ASSUMED_TICKS_PER_METER, 1);
  Serial.print("   (if ");
  Serial.print(ASSUMED_TICKS_PER_METER, 0);
  Serial.println(" ticks/m is right)");
  Serial.println("  -----------------------------------");
  Serial.println("  NOW MEASURE how far the car actually moved, then:");
  Serial.print("     TICKS_PER_METER = ");
  Serial.print(ticks);
  Serial.println(" / (your_cm / 100)");
  Serial.println("  e.g. if you measured 90 cm:");
  Serial.print("     ");
  Serial.print(ticks);
  Serial.print(" / 0.90 = ");
  Serial.print(ticks / 0.90f, 0);
  Serial.println(" ticks per metre");
  Serial.println("=========================================");
  Serial.println("press START to run again");
}

void setup() {
  // Motor pins safe FIRST, digitalWrite only - an input here floats HIGH and
  // the driver reads that as throttle wide open.
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);

  Serial.begin(115200);
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encISR, RISING);

  motorWrite(0);

  Serial.println();
  Serial.println("tick_m_calculate - drives forward 3 s, then reports the tick count.");
  Serial.println("Clear 1.5 m ahead, mark where the car starts, then press START.");
}

void loop() {
  bool btnNow = digitalRead(START_BTN_PIN);
  bool btnPressed = (btnNow == LOW && lastButton == HIGH);
  lastButton = btnNow;

  uint32_t now = millis();

  switch (state) {

    case WAITING:
      if (btnPressed) {
        delay(BUTTON_DEBOUNCE_MS);
        if (digitalRead(START_BTN_PIN) == LOW) {
          startTicks = encRead();
          runStart = now;
          state = DRIVING;
          Serial.println("GO - 3 s forward");
        }
      }
      break;

    case DRIVING: {
      if (btnPressed) {                      // press again = emergency stop
        motorWrite(0);
        runTicks = encRead() - startTicks;
        state = DONE;
        Serial.println("ABORTED by button");
        report(abs(runTicks), now - runStart);
        break;
      }
      motorWrite(DRIVE_SPEED);
      if (now - lastBeat >= 500) {
        lastBeat = now;
        Serial.print("  ");
        Serial.print((now - runStart) / 1000.0, 1);
        Serial.print(" s  ticks=");
        Serial.println(abs(encRead() - startTicks));
      }
      if (now - runStart >= DRIVE_MS) {
        motorWrite(0);
        uint32_t elapsed = now - runStart;
        runTicks = encRead() - startTicks;
        state = DONE;
        report(abs(runTicks), elapsed);
      }
      break;
    }

    case DONE:
      motorWrite(0);
      if (btnPressed) {
        delay(BUTTON_DEBOUNCE_MS);
        state = WAITING;
        Serial.println("re-armed - press START for another run");
      }
      break;
  }
}
