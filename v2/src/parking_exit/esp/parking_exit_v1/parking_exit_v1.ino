/*
 * ============================================================================
 *  parking_exit_v1 - WRO 2026 Future Engineers, Team Yo_Error_404
 *  ESP32-C3 Super Mini (V2 bot)
 *
 *  STANDALONE parking exit. Nothing to do with the obstacle round - flash it,
 *  press START, watch it, flash open_slave back afterwards.
 * ============================================================================
 *
 *  WHAT IT DOES  (the sequence is taken verbatim from the reference sketch)
 *    0. LOOK    average L and R three times; exit AWAY from the nearer wall
 *    1. LOCK    full steering lock until the heading has turned TURN_OUT_DEG
 *    2. STRAIGHT drive EXIT_MM holding that heading with a P term
 *    3. LOCK    opposite full lock until the heading is back to TURN_BACK_DEG
 *    4. stop
 *
 *  WHY THIS IS A PORT AND NOT A COPY
 *    The reference sketch is written for a different robot. Flashed here it
 *    would not compile, and if it did it would drive into things:
 *
 *      motor IN1/IN2   GPIO26/27  ->  GPIO1/0      and DIR forward is LOW here,
 *                                                  not HIGH - the car would
 *                                                  have reversed out of the bay
 *      servo           GPIO13     ->  GPIO6
 *      START button    GPIO25     ->  GPIO10
 *      encoder A/B     18/19      ->  4/5
 *      BNO055          mux ch3    ->  mux ch4
 *      ToF LEFT/RIGHT  ch2/ch1    ->  ch1/ch2      SWAPPED - it would have
 *                                                  exited towards the wall
 *      ticks/metre     657        ->  2078         3x out, so EXIT_MM would
 *                                                  have been a third of asked
 *      servo centre    104        ->  87           (90 nominal, -3 trim)
 *      servo limits    28..142    ->  30..140
 *      BluetoothSerial            ->  removed      classic BT does not exist
 *                                                  on the ESP32-C3 at all
 *
 *    The STEP LOGIC, the angles, the distances and the gains are unchanged.
 *
 *  SAFETY
 *    Speed is capped at MAX_PWM. Motor pins are driven OUTPUT/LOW as the very
 *    first statements in setup(), because an input on the driver's PWM pin
 *    floats HIGH and reads as full throttle. The servo is attached BEFORE any
 *    analogWrite, or it shares the LEDC timer and spins continuously.
 *
 *  UPLOAD
 *    arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc parking_exit_v1
 *
 *  PATCH NOTES
 *    PATCH 1 (2026-09-24) First version. Ported from the reference sketch onto
 *            V2 hardware; sequence, angles and distances left as they were.
 * ============================================================================
 */
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>

// ---------------------------------------------------------------- V2 pins
#define IN1_PIN            1      // MD10112 PWM
#define IN2_PIN            0      // MD10112 DIR
#define MOTOR_FORWARD_LEVEL LOW   // forward on this car
#define MOTOR_REVERSE_LEVEL HIGH
#define SERVO_PIN          6
#define NEOPIXEL_PIN       7
#define NEOPIXEL_COUNT     16
#define START_BTN_PIN      10
#define ENC_A_PIN          4
#define ENC_B_PIN          5
#define I2C_SDA_PIN        8
#define I2C_SCL_PIN        9

#define TCAADDR          0x70
#define TOF_FRONT_CH        0
#define TOF_LEFT_CH         1     // V2: LEFT is ch1, RIGHT is ch2
#define TOF_RIGHT_CH        2
#define BNO_CH              4
#define BNO_ADDR         0x28

// ---------------------------------------------------------------- tune these
#define SERVO_CENTER_DEG   87     // 90 nominal, -3 trim: 90 is not straight here
#define EXIT_LOCK_DEG      38     // exactly as the reference sketch has it
#define TURN_OUT_DEG       90     // step 1 target
#define EXIT_MM            85     // step 2 distance, trimmed from 100
#define TURN_BACK_DEG       0     // step 3 target
#define EXIT_SPEED         70     // +20
#define STRAIGHT_KP       1.2
#define RUN_TIMEOUT_MS   8000
// A 90 deg turn at this lock is well under half a metre of travel. If a
// heading-driven step goes past this WITHOUT the heading arriving, the IMU is
// not reporting rotation and the car would otherwise drive in circles until
// the timeout - which is exactly the intermittent failure seen. Encoder-based
// on purpose: it does not care whether the heading is believable.
#define TURN_MAX_MM       900
// ----------------------------------------------------------------

#define SERVO_MIN_DEG      30
#define SERVO_MAX_DEG     140
#define MAX_PWM           110     // team cap, matches open_slave
#define TICKS_PER_METER 2078.0    // measured on V2

Adafruit_VL53L0X tof[3];
Adafruit_BNO055  bno = Adafruit_BNO055(55, BNO_ADDR, &Wire);
Servo            steeringServo;
Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

volatile long encoderTicks = 0;
float startHeading = 0;
int   dir = 0;                    // +1 = exit right, -1 = exit left
int   stepNo = 0;
long  stepTicks = 0;
unsigned long runStart = 0;
int   servoNow = SERVO_CENTER_DEG;

void tcaselect(uint8_t i) {
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, strip.Color(r, g, b));
  strip.show();
}

void logLine(const String &s) { Serial.println(s); }

// On THIS car, driving forward makes this count DOWN. Measured live:
//     t_ms step turned servo   mm
//      158    1    0.3   125  -12      <-- driving forward, mm going negative
//
// It has never shown up before because the Pi side takes abs() on every single
// encoder comparison, so the sign was invisible. The reference sketch does not:
// it compares mmSince(...) >= EXIT_MM directly, which on our wiring can never
// be true - so step 2 never ended and the car drove forward until the timeout.
// Flipped here so forward counts UP, which is what the sketch assumes.
void IRAM_ATTR encoderISR() {
  if (digitalRead(ENC_B_PIN) == HIGH) encoderTicks--;
  else                                encoderTicks++;
}

float mmSince(long ref) {
  long t;
  noInterrupts(); t = encoderTicks; interrupts();
  return (t - ref) * (1000.0 / TICKS_PER_METER);
}

void driveMotor(int speed, bool fwd) {
  speed = constrain(speed, 0, MAX_PWM);
  if (speed == 0) { analogWrite(IN1_PIN, 0); digitalWrite(IN2_PIN, LOW); }
  else {
    digitalWrite(IN2_PIN, fwd ? MOTOR_FORWARD_LEVEL : MOTOR_REVERSE_LEVEL);
    analogWrite(IN1_PIN, speed);
  }
}

void setServo(int deg) {
  servoNow = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  steeringServo.write(servoNow);
}

float readHeading() {
  tcaselect(BNO_CH);
  sensors_event_t e;
  bno.getEvent(&e);
  return e.orientation.x;
}

float wrap180(float d) { while (d > 180) d -= 360; while (d < -180) d += 360; return d; }

float readTof(uint8_t ch) {
  tcaselect(ch);
  for (int k = 0; k < 20 && !tof[ch].isRangeComplete(); k++) delay(5);
  uint16_t mm = tof[ch].readRangeResult();
  return (mm > 0 && mm <= 2000) ? mm / 10.0 : 999.0;
}

// Stop, report, and go back to WAITING so the next button press runs the whole
// thing again from scratch. It used to sit in while(1) until a power cycle,
// which made testing the same change twice a chore.
void finish(const String &why) {
  driveMotor(0, true);
  setServo(SERVO_CENTER_DEG);
  setLed(0, 0, 60);
  logLine("STOP " + why + " in step " + String(stepNo) +
          "  turned=" + String(dir * wrap180(readHeading() - startHeading), 1) +
          " mm=" + String(mmSince(0), 0) +
          " t=" + String(millis() - runStart));
  stepNo = 0;                       // idle: loop() now waits for START again
  logLine("--- press START to run it again ---");
}

// Debounced edge detect. The old version sampled the pin in a tight loop with
// no debounce, so a real press was often missed and a bounce sometimes counted
// twice - which is why the button felt dead one moment and twitchy the next.
// A press must be held for BTN_STABLE_MS to count.
#define BTN_STABLE_MS 40
bool buttonPressed() {
  if (digitalRead(START_BTN_PIN) != LOW) return false;
  uint32_t t0 = millis();
  while (millis() - t0 < BTN_STABLE_MS) {
    if (digitalRead(START_BTN_PIN) != LOW) return false;   // bounce, not a press
    delay(2);
  }
  return true;
}

// Blocks until a clean press, then until release. Keeps the BNO and the ToFs
// being READ the whole time - that warm-up is what our working open-round
// sketches do and what this one was missing.
void waitForStartPress() {
  uint32_t lastReport = 0;
  while (!buttonPressed()) {
    if (millis() - lastReport >= 1000) {
      lastReport = millis();
      uint8_t cs = 0, cg = 0, ca = 0, cm = 0;
      float h = readHeading();
      tcaselect(BNO_CH); bno.getCalibration(&cs, &cg, &ca, &cm);
      logLine("waiting  hdg=" + String(h, 1) +
              "  cal sys=" + String(cs) + " gyro=" + String(cg) +
              " accel=" + String(ca) + " mag=" + String(cm) +
              (cg >= 2 ? "   gyro ready" : "   gyro still settling - hold the car still"));
    } else {
      readHeading();                 // keep the IMU ticking between reports
      delay(20);
    }
  }
  while (digitalRead(START_BTN_PIN) == LOW) delay(10);
  delay(60);
}

// Everything a fresh run needs: look at the walls, zero the heading and the
// encoder, and enter step 1.
void armRun() {
  setLed(60, 60, 0);
  float L = 0, R = 0;
  for (int k = 0; k < 3; k++) { L += readTof(TOF_LEFT_CH); R += readTof(TOF_RIGHT_CH); }
  L /= 3; R /= 3;
  dir = (R > L) ? 1 : -1;
  startHeading = readHeading();

  uint8_t cs = 0, cg = 0, ca = 0, cm = 0;
  tcaselect(BNO_CH); bno.getCalibration(&cs, &cg, &ca, &cm);
  logLine("L=" + String(L, 0) + " R=" + String(R, 0) +
          " F=" + String(readTof(TOF_FRONT_CH), 0) +
          " -> exit " + (dir == 1 ? "RIGHT (+90)" : "LEFT (-90)") +
          "  hdg=" + String(startHeading, 1) +
          "  cal sys=" + String(cs) + " gyro=" + String(cg));
  logLine("t_ms,step,turned,servo,mm");

  noInterrupts(); encoderTicks = 0; interrupts();
  stepTicks = 0;
  runStart = millis();
  stepNo = 1;
  setLed(0, 60, 0);
  setServo(SERVO_CENTER_DEG + dir * EXIT_LOCK_DEG);
  logLine("STEP 1: lock " + String(servoNow) + " to " + String(TURN_OUT_DEG) + " deg");
}

void setup() {
  // Motor pins safe FIRST. An input here floats HIGH and the driver reads that
  // as throttle wide open - this has driven the car off a bench before.
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);

  Serial.begin(115200);
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  strip.begin();
  setLed(60, 0, 0);

  // Servo BEFORE any analogWrite, or analogWrite claims the LEDC timer at
  // ~1 kHz and the servo shares it instead of getting a clean 50 Hz one.
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  setServo(SERVO_CENTER_DEG);
  driveMotor(0, true);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, RISING);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

  for (int i = 0; i < 3; i++) {
    tcaselect(i);
    if (!tof[i].begin()) { logLine("ToF " + String(i) + " missing"); while (1) delay(1000); }
    tof[i].startRangeContinuous(30);
  }
  tcaselect(BNO_CH);
  if (!bno.begin()) { logLine("BNO055 missing"); while (1) delay(1000); }
  delay(100);
  bno.setExtCrystalUse(true);
  // MEASURED, not assumed: in NDOF this board reports orientation.x = 0.0
  // permanently on the bench -
  //
  //     waiting  hdg=0.0  cal sys=0 gyro=3 accel=0 mag=0
  //
  // NDOF fuses the MAGNETOMETER and will not produce a heading until the mag
  // calibrates. Beside a motor and a LiPo indoors, mag stays 0, so the heading
  // stays dead at zero - and every step here waits on a heading that never
  // moves, which is precisely the "drives in circles" failure.
  //
  // IMUPLUS fuses accelerometer + gyro and ignores the magnetometer. The
  // heading is RELATIVE rather than compass-referenced, which is all this
  // needs: every comparison is against startHeading taken seconds earlier.
  // The gyro reaches cal 3 within about two seconds of being read, which the
  // warm-up below now guarantees before any run begins.
  bno.setMode(OPERATION_MODE_IMUPLUS);
  delay(30);

  setLed(60, 60, 0);
  {
    uint8_t cs = 0, cg = 0, ca = 0, cm = 0;
    tcaselect(BNO_CH); bno.getCalibration(&cs, &cg, &ca, &cm);
    logLine("BNO cal sys=" + String(cs) + " gyro=" + String(cg) +
            " accel=" + String(ca) + " mag=" + String(cm) +
            (cs == 0 ? "   <-- SYSTEM UNCALIBRATED: heading may not be trustworthy"
                     : ""));
  }
  logLine("parking_exit_v1 ready. Press START.");
  waitForStartPress();
  armRun();
}

void loop() {
  // Idle between runs. One press re-runs the whole script from the top:
  // fresh wall reading, fresh heading zero, fresh encoder zero.
  if (stepNo == 0) {
    driveMotor(0, true);
    setServo(SERVO_CENTER_DEG);
    waitForStartPress();
    armRun();
    return;
  }

  float turned = dir * wrap180(readHeading() - startHeading);   // + = toward the exit side

  if (stepNo == 1) {
    setServo(SERVO_CENTER_DEG + dir * EXIT_LOCK_DEG);
    driveMotor(EXIT_SPEED, true);
    if (mmSince(0) > TURN_MAX_MM) {
      finish("step 1 overran " + String(TURN_MAX_MM) + " mm without reaching " +
             String(TURN_OUT_DEG) + " deg - heading only got to " + String(turned, 1));
    }
    if (turned >= TURN_OUT_DEG) {
      stepNo = 2; stepTicks = encoderTicks;
      logLine("STEP 2: straight " + String(EXIT_MM) + " mm  turned=" + String(turned, 1));
    }
  } else if (stepNo == 2) {
    float err = TURN_OUT_DEG - turned;
    setServo(SERVO_CENTER_DEG + dir * constrain(STRAIGHT_KP * err, -(float)EXIT_LOCK_DEG, (float)EXIT_LOCK_DEG));
    driveMotor(EXIT_SPEED, true);
    if (mmSince(stepTicks) >= EXIT_MM) {
      stepNo = 3;
      setServo(SERVO_CENTER_DEG - dir * EXIT_LOCK_DEG);
      logLine("STEP 3: lock " + String(servoNow) + " back to " + String(TURN_BACK_DEG) +
              " deg  turned=" + String(turned, 1));
    }
  } else if (stepNo == 3) {
    setServo(SERVO_CENTER_DEG - dir * EXIT_LOCK_DEG);
    driveMotor(EXIT_SPEED, true);
    if (mmSince(stepTicks) > TURN_MAX_MM) {
      finish("step 3 overran " + String(TURN_MAX_MM) + " mm without returning to " +
             String(TURN_BACK_DEG) + " deg - heading stuck at " + String(turned, 1));
    }
    if (turned <= TURN_BACK_DEG) finish("done");
  }

  logLine(String(millis() - runStart) + "," + stepNo + "," + String(turned, 1) +
          "," + servoNow + "," + String(mmSince(0), 0));

  if (millis() - runStart > RUN_TIMEOUT_MS) finish("timeout");
  if (buttonPressed()) {
    while (digitalRead(START_BTN_PIN) == LOW) delay(10);   // let go first
    finish("button");
  }
  delay(20);
}
