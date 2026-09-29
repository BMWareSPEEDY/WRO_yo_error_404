/*
 * parking_exit.ino - WRO 2026 Future Engineers, Team Yo_Error_404
 * ESP32-C3 Super Mini (V2 bot)
 *
 * STANDALONE parking exit routine.
 *
 * Sequence:
 *   0. LOOK: Read Left and Right ToF 3 times.
 *      - Wall on LEFT  (L <= R) -> Exit RIGHT (+90° / out, short straight, back, align 0°)
 *      - Wall on RIGHT (R < L)  -> Exit LEFT  (-90° / out, short straight, back, align 0°)
 *   1. TURN OUT: Full lock away from wall until IMU turns TURN_OUT_DEG (bounded by encoder)
 *   2. STRAIGHT: Drive forward STRAIGHT_CM holding heading with IMU P-term
 *   3. TURN BACK: Opposite full lock until IMU turns back towards 0° (bounded by encoder)
 *   4. ALIGN: Square up to start heading (0°) using IMU heading hold until |error| < ALIGN_DONE_DEG
 *   5. COMPLETE: Stop motor, center servo, set LED to blue.
 *
 * Safety & Hardware:
 *   - GPIO 0 and GPIO 1 driven OUTPUT and LOW first thing in setup()
 *   - Servo attached before analogWrite to avoid LEDC timer conflict
 *   - BNO055 in OPERATION_MODE_IMUPLUS (no magnetometer, relative yaw)
 *   - Left/Right ToF channels: Left is Mux 1, Right is Mux 2, Front is Mux 0
 *   - Servo straight = 87° (-3° trim), limits [30, 140]
 *   - Debounced start button (GPIO 10); press during run aborts cleanly
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>

// ============================================================================
// CALIBRATED HARDWARE PINS & CONSTANTS (V2 Bot)
// ============================================================================
#define IN1_PIN            1       // MD10112 PWM
#define IN2_PIN            0       // MD10112 DIR
#define FORWARD_LEVEL      LOW     // Measured on V2: LOW = forward, HIGH = reverse
#define REVERSE_LEVEL      HIGH
#define SERVO_PIN          6
#define LED_PIN            7
#define NEOPIXEL_COUNT     16
#define BTN_PIN            10      // Active LOW start button with internal pullup
#define ENC_A_PIN          4
#define ENC_B_PIN          5
#define SDA_PIN            8
#define SCL_PIN            9

#define TCAADDR            0x70
#define TOF_FRONT_CH       0
#define TOF_LEFT_CH        1       // Chassis swapped: Left is ch1, Right is ch2
#define TOF_RIGHT_CH       2
#define BNO_CH             4
#define BNO_ADDR           0x28

#define SERVO_STRAIGHT     87      // Measured: 87 is true straight (90 is not)
#define SERVO_MIN          30      // Mechanical stops at 28
#define SERVO_MAX         140      // Mechanical stops at 142
#define MAX_PWM           110      // Hard safety cap on motor PWM
#define TICKS_PER_CM    20.78f     // 2078 ticks/m, measured

// ============================================================================
// TUNABLE PARKING PARAMETERS
// ============================================================================
#define EXIT_SPEED         75      // PWM out of 255 (gentle, controlled exit)
#define STEER_LOCK_DEG     45      // Servo offset for turn (87+45=132 Right, 87-45=42 Left)
#define TURN_OUT_DEG     45.0f     // Degrees to turn out away from wall
#define STRAIGHT_CM       8.5f     // Forward displacement leg (85 mm as per spec)
#define TURN_MAX_CM      70.0f     // Encoder safety bound per turn step (cm)
#define HOLD_KP           1.5f     // Proportional gain for heading hold
#define ALIGN_DONE_DEG    2.5f     // Alignment tolerance in degrees
#define BTN_DEBOUNCE_MS     40     // Button debounce time

// ============================================================================
// OBJECTS & GLOBALS
// ============================================================================
Adafruit_VL53L0X  tof[3];
Adafruit_BNO055   bno = Adafruit_BNO055(55, BNO_ADDR, &Wire);
Servo             steeringServo;
Adafruit_NeoPixel led(NEOPIXEL_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

volatile long encoderTicks = 0;

void IRAM_ATTR encISR() {
  if (digitalRead(ENC_B_PIN) == HIGH) encoderTicks--;
  else                                encoderTicks++;
}

long readTicks() {
  long t;
  noInterrupts();
  t = encoderTicks;
  interrupts();
  return t;
}

float cmSince(long refTicks) {
  return (float)labs(readTicks() - refTicks) / TICKS_PER_CM;
}

// ============================================================================
// HARDWARE HELPERS
// ============================================================================
void tcaselect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < led.numPixels(); i++) {
    led.setPixelColor(i, led.Color(r, g, b));
  }
  led.show();
}

void driveMotor(int pwm, bool forward) {
  pwm = constrain(pwm, 0, MAX_PWM);
  if (pwm == 0) {
    analogWrite(IN1_PIN, 0);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN2_PIN, forward ? FORWARD_LEVEL : REVERSE_LEVEL);
    analogWrite(IN1_PIN, pwm);
  }
}

void setServo(int deg) {
  int clamped = constrain(deg, SERVO_MIN, SERVO_MAX);
  steeringServo.write(clamped);
}

float readHeading() {
  tcaselect(BNO_CH);
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

float wrap180(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

float getHeadingError(float target, float current) {
  return wrap180(target - current);
}

float getDistance(uint8_t muxChannel) {
  tcaselect(muxChannel);
  VL53L0X_RangingMeasurementData_t measure;
  tof[muxChannel].rangingTest(&measure, false);
  if (measure.RangeStatus == 4 || measure.RangeMilliMeter > 2000) return 999.0f;
  return measure.RangeMilliMeter / 10.0f;
}

bool isButtonPressed() {
  if (digitalRead(BTN_PIN) != LOW) return false;
  uint32_t t0 = millis();
  while (millis() - t0 < BTN_DEBOUNCE_MS) {
    if (digitalRead(BTN_PIN) != LOW) return false;
    delay(2);
  }
  return true;
}

bool checkAbort() {
  if (!isButtonPressed()) return false;
  while (digitalRead(BTN_PIN) == LOW) delay(10);
  return true;
}

// ============================================================================
// PARKING EXIT SEQUENCE
// ============================================================================
void runParkingExit() {
  setLed(60, 60, 0); // Yellow: preparing

  // 1. LOOK: Average Left and Right ToF 3 times
  float L = 0, R = 0;
  for (int k = 0; k < 3; k++) {
    L += getDistance(TOF_LEFT_CH);
    R += getDistance(TOF_RIGHT_CH);
    delay(20);
  }
  L /= 3.0f;
  R /= 3.0f;

  // Decide exit direction: away from nearer wall
  // Wall on LEFT (L <= R) -> exit RIGHT (turnDir = +1)
  // Wall on RIGHT (R < L)  -> exit LEFT  (turnDir = -1)
  int turnDir = (L <= R) ? +1 : -1;

  float startHdg = readHeading();
  long startTicks = readTicks();
  uint32_t runStartMs = millis();

  Serial.printf("\n=== STARTING PARKING EXIT ===\n");
  Serial.printf("Sensors: L=%.1f cm, R=%.1f cm, F=%.1f cm\n", L, R, getDistance(TOF_FRONT_CH));
  Serial.printf("Wall on %s -> Exiting %s\n",
                (turnDir > 0) ? "LEFT" : "RIGHT",
                (turnDir > 0) ? "RIGHT" : "LEFT");
  Serial.printf("Locked Start Heading: %.1f deg\n\n", startHdg);

  setLed(0, 60, 0); // Green: Running

  // -------------------------------------------------------------
  // STAGE 1: TURN OUT (steer away from wall)
  // -------------------------------------------------------------
  Serial.println("STAGE 1: TURN OUT");
  int outSteer = SERVO_STRAIGHT + turnDir * STEER_LOCK_DEG;
  setServo(outSteer);
  driveMotor(EXIT_SPEED, true);
  long stageRef = readTicks();

  while (true) {
    if (checkAbort()) { Serial.println("ABORTED by button"); return; }

    float curHdg = readHeading();
    float turned = turnDir * wrap180(curHdg - startHdg);
    float distCm = cmSince(stageRef);

    if (turned >= TURN_OUT_DEG) {
      Serial.printf("Stage 1 done: turned %.1f deg (traveled %.1f cm)\n", turned, distCm);
      break;
    }
    if (distCm >= TURN_MAX_CM) {
      Serial.printf("Stage 1 distance bound: %.1f cm (turned %.1f deg)\n", distCm, turned);
      break;
    }
    delay(20);
  }

  // -------------------------------------------------------------
  // STAGE 2: STRAIGHT LEG (hold turn heading for short distance)
  // -------------------------------------------------------------
  Serial.printf("STAGE 2: STRAIGHT FORWARD (%.1f cm)\n", STRAIGHT_CM);
  float legTargetHdg = wrap180(startHdg + turnDir * TURN_OUT_DEG);
  stageRef = readTicks();

  while (cmSince(stageRef) < STRAIGHT_CM) {
    if (checkAbort()) { Serial.println("ABORTED by button"); return; }

    float curHdg = readHeading();
    float herr = getHeadingError(legTargetHdg, curHdg);
    int steer = SERVO_STRAIGHT + constrain((int)(HOLD_KP * herr), -20, 20);
    setServo(steer);
    driveMotor(EXIT_SPEED, true);
    delay(20);
  }
  Serial.printf("Stage 2 done: traveled %.1f cm\n", cmSince(stageRef));

  // -------------------------------------------------------------
  // STAGE 3: TURN BACK / RECENTER (opposite steer to bring heading back)
  // -------------------------------------------------------------
  Serial.println("STAGE 3: TURN BACK / RECENTER");
  int backSteer = SERVO_STRAIGHT - turnDir * STEER_LOCK_DEG;
  setServo(backSteer);
  driveMotor(EXIT_SPEED, true);
  stageRef = readTicks();

  while (true) {
    if (checkAbort()) { Serial.println("ABORTED by button"); return; }

    float curHdg = readHeading();
    float turned = turnDir * wrap180(curHdg - startHdg);
    float distCm = cmSince(stageRef);

    if (turned <= 10.0f) {
      Serial.printf("Stage 3 done: turned back to %.1f deg (traveled %.1f cm)\n", turned, distCm);
      break;
    }
    if (distCm >= TURN_MAX_CM) {
      Serial.printf("Stage 3 distance bound: %.1f cm (turned %.1f deg)\n", distCm, turned);
      break;
    }
    delay(20);
  }

  // -------------------------------------------------------------
  // STAGE 4: FORWARD 55 CM & ALIGN (drive 55 cm forward holding heading)
  // -------------------------------------------------------------
  Serial.println("STAGE 4: FORWARD 55 CM & ALIGN");
  stageRef = readTicks();

  while (cmSince(stageRef) < 55.0f) {
    if (checkAbort()) { Serial.println("ABORTED by button"); return; }

    float curHdg = readHeading();
    float herr = getHeadingError(startHdg, curHdg);
    int steer = SERVO_STRAIGHT + constrain((int)(HOLD_KP * herr), -25, 25);
    setServo(steer);
    driveMotor(EXIT_SPEED, true);
    delay(20);
  }
  Serial.printf("Stage 4 done: moved forward %.1f cm (heading error = %.1f deg)\n", cmSince(stageRef), getHeadingError(startHdg, readHeading()));

  // HALT AND STOP
  driveMotor(0, true);
  setServo(SERVO_STRAIGHT);
  setLed(0, 0, 60); // Blue: finished / halted
  float totalDist = cmSince(startTicks);
  Serial.printf("\n=== PARKING EXIT COMPLETE - HALTED ===\n");
  Serial.printf("Final Heading: %.1f deg (error: %.1f deg)\n", readHeading(), wrap180(readHeading() - startHdg));
  Serial.printf("Total Distance: %.1f cm in %.1f s\n", totalDist, (millis() - runStartMs) / 1000.0f);
  Serial.println("Car is HALTED. Press START to run again.\n");
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // HAZARD 1: Set motor pins OUTPUT and LOW first statement
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);

  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  led.begin();
  setLed(60, 0, 0); // Red during init

  // HAZARD 2: Attach servo before any analogWrite
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  setServo(SERVO_STRAIGHT);
  driveMotor(0, true);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encISR, RISING);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // Initialize ToF sensors
  for (int i = 0; i < 3; i++) {
    tcaselect(i);
    if (!tof[i].begin()) {
      Serial.printf("ToF channel %d missing!\n", i);
    }
  }

  // Initialize BNO055 in IMUPLUS mode
  tcaselect(BNO_CH);
  while (!bno.begin()) {
    Serial.println("BNO055 missing! Retrying...");
    setLed(60, 60, 0);
    delay(400);
  }
  delay(100);
  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_IMUPLUS);
  delay(30);

  Serial.println("parking_exit ready. Press START button.");
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  driveMotor(0, true);
  setServo(SERVO_STRAIGHT);
  setLed(0, 0, 60); // Blue: waiting

  uint32_t lastPrint = 0;
  while (!isButtonPressed()) {
    if (millis() - lastPrint >= 1000) {
      lastPrint = millis();
      float h = readHeading();
      float f = getDistance(TOF_FRONT_CH);
      float l = getDistance(TOF_LEFT_CH);
      float r = getDistance(TOF_RIGHT_CH);
      Serial.printf("IDLE | btn=%d | Hdg=%.1f | F=%.0f L=%.0f R=%.0f cm\n",
                    digitalRead(BTN_PIN), h, f, l, r);
    }
    delay(10);
  }

  // Button pressed: wait for release
  Serial.println("START pressed!");
  while (digitalRead(BTN_PIN) == LOW) delay(10);
  delay(60);

  runParkingExit();

  driveMotor(0, true);
  setServo(SERVO_STRAIGHT);
}
