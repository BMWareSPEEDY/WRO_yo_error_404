/*
 * ============================================================================
 *  WRO 2026 Future Engineers - Team Yo_Error_404
 *  ESP32-C3 Super Mini - 5 s STRAIGHT RUN + ENCODER TICK COUNT
 * ============================================================================
 *
 *  Press START  -> wheels centre, motor ramps to DRIVE_SPEED, holds for
 *                  DRIVE_MS, ramps down, stops, then prints the encoder
 *                  totals. Press again to abort at any time.
 *
 *  Motor/servo/button/LED handling is lifted unchanged from
 *  obstacle_round/esp/open_slave. Added: a quadrature encoder on ENC_A_PIN
 *  (interrupt) and ENC_B_PIN (direction), counted only while the run is live.
 *
 *  SAFETY: IN1/IN2 are OUTPUT + LOW as the first statements in setup().
 *  IN1 is the MD10112 PWM input - floating it pulls HIGH = 100% duty.
 *  The encoder pins are the ONLY pins here that get INPUT_PULLUP.
 *
 *  TELEMETRY (USB CDC, 115200)
 *    "TICKS,<ticks>,<pos>,<appliedSpeed>\n"      10 Hz while running
 *    "RESULT,<elapsed_ms>,<ticks>,<pos>,<ticks_per_s>\n"   once at the end
 *
 *  RULES
 *    11.10  Wi-Fi and Bluetooth off.
 *    9.14   Nothing moves before START.
 *
 *  UPLOAD
 *    arduino-cli compile --upload -p /dev/cu.usbmodem* \
 *      --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc <this sketch folder>
 *    Libraries: ESP32Servo, Adafruit NeoPixel
 *
 *  PATCH NOTES
 *    PATCH 1 (2026-09-21) First version.
 *    PATCH 2 (2026-09-21) Removed the servo sweep. open_slave's centerWheels()
 *            swings full left, then full right, then centre on every boot;
 *            that is pointless for a straight-line tick count and it is the
 *            servo movement seen at start-up. The wheels are now simply put
 *            straight once at boot and never moved again. The
 *            writeSteering(SERVO_CENTER) calls in startRun()/stopRun() stay,
 *            but they return early because the angle is already centred, so
 *            they produce no movement.
 *    PATCH 3 (2026-09-21) Servo STILL turned continuously after PATCH 2, and
 *            the cause was not the sweep at all: setup() called
 *            analogWrite(IN1_PIN, 0) before steeringServo.attach(). On ESP32
 *            core 3.x analogWrite claims an LEDC timer at ~1 kHz, and
 *            ESP32Servo then shares that timer instead of getting a clean
 *            50 Hz one, so the servo is fed pulse widths it cannot read and
 *            runs away - which also drags on the drivetrain and stops the car
 *            driving straight. That early analogWrite is removed; digitalWrite
 *            LOW already holds the motor off safely, and the first analogWrite
 *            is now motorWrite(0) AFTER the attach, exactly as open_slave
 *            does it. Same root cause as move_30cm PATCH 4.
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>

// ============================================================================
// PINS  (motor/servo/LED/button identical to the slave sketch)
// ============================================================================
#define IN1_PIN 1    // MD10112 PWM
#define IN2_PIN 0    // MD10112 DIR
#define SERVO_PIN      6
#define NEOPIXEL_PIN   7
#define START_BTN_PIN 10    // to GND, internal pull-up

#define ENC_A_PIN      4    // encoder channel A - interrupt
#define ENC_B_PIN      5    // encoder channel B - direction only

// ============================================================================
// ACTUATORS  (identical to the slave sketch)
// ============================================================================
#define MOTOR_FORWARD_LEVEL LOW
#define MOTOR_REVERSE_LEVEL HIGH
#define MAX_PWM            170   // hard speed cap
#define RAMP_STEP           30
#define RAMP_DELAY_MS        8

#define SERVO_CENTER   90
#define SERVO_MIN      30
#define SERVO_MAX     140

#define NEOPIXEL_COUNT 16

// ============================================================================
// TIMING / TEST PARAMETERS
// ============================================================================
#define DRIVE_SPEED         170     // forward PWM, clamped to MAX_PWM
#define DRIVE_MS           5000UL   // 5 s measured from the button press
#define TICK_REPORT_MS      100     // live tick stream, 10 Hz
#define BUTTON_DEBOUNCE_MS   60

// ============================================================================
// STATE
// ============================================================================
Servo steeringServo;
Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

volatile uint32_t encTicks = 0;   // every A edge, direction ignored
volatile int32_t  encPos   = 0;   // signed, +ve = forward

bool     started = false;
bool     holdDone = false;        // 5 s window over, ramping down
int      targetSpeed = 0;
int      appliedSpeed = 0;
int      appliedSteer = SERVO_CENTER;

uint32_t runStartMs = 0;
uint32_t lastRampMs = 0;
uint32_t lastTickMsgMs = 0;
uint32_t btnLowSinceMs = 0;
bool     btnWasPressed = false;
uint32_t ledColor = 0xFFFFFFFF;

// ============================================================================
// ENCODER
// ============================================================================
void IRAM_ATTR encISR() {
  encTicks++;
  if (digitalRead(ENC_B_PIN)) encPos--;
  else                        encPos++;
}

void encReset() {
  noInterrupts();
  encTicks = 0;
  encPos   = 0;
  interrupts();
}

void encRead(uint32_t *ticks, int32_t *pos) {
  noInterrupts();
  *ticks = encTicks;
  *pos   = encPos;
  interrupts();
}

// ============================================================================
// HELPERS
// ============================================================================
void setLed(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t c = strip.Color(r, g, b);
  if (c == ledColor) return;
  ledColor = c;
  for (int i = 0; i < NEOPIXEL_COUNT; i++) strip.setPixelColor(i, c);
  strip.show();
}

void sendLine(const char *s) {
  size_t n = strlen(s);
  if ((size_t)Serial.availableForWrite() >= n) Serial.write((const uint8_t *)s, n);
}

// ============================================================================
// MOTOR & SERVO  (same as driveMotor()/writeSteering() in the slave sketch)
// ============================================================================
void motorWrite(int speed) {
  if (speed == 0) {
    analogWrite(IN1_PIN, 0);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN2_PIN, speed > 0 ? MOTOR_FORWARD_LEVEL : MOTOR_REVERSE_LEVEL);
    analogWrite(IN1_PIN, abs(speed));
  }
}

void updateMotor(uint32_t now, int target) {
  if (target == 0 && !started) {          // aborts are immediate
    appliedSpeed = 0;
    motorWrite(0);
    return;
  }
  if (now - lastRampMs < RAMP_DELAY_MS) return;
  lastRampMs = now;
  if (appliedSpeed < target)      appliedSpeed = min(appliedSpeed + RAMP_STEP, target);
  else if (appliedSpeed > target) appliedSpeed = max(appliedSpeed - RAMP_STEP, target);
  motorWrite(appliedSpeed);
}

void writeSteering(int angle) {
  angle = constrain(angle, SERVO_MIN, SERVO_MAX);
  if (angle == appliedSteer) return;
  appliedSteer = angle;
  steeringServo.write(angle);
}

// The open-round left/right/centre sweep is deliberately NOT done here - the
// wheels are only ever put straight, never swung. See PATCH 2.
void centerWheels() {
  steeringServo.write(SERVO_CENTER);
  delay(300);
  appliedSteer = SERVO_CENTER;
}

// ============================================================================
// RUN CONTROL
// ============================================================================
void startRun(uint32_t now) {
  started = true;
  holdDone = false;
  targetSpeed = constrain(DRIVE_SPEED, -MAX_PWM, MAX_PWM);
  runStartMs = now;
  lastRampMs = now;
  lastTickMsgMs = now;
  encReset();
  writeSteering(SERVO_CENTER);
  setLed(0, 255, 0);
  sendLine("START,forward 5 s, counting ticks\n");
}

void stopRun(uint32_t now, const char *why) {
  uint32_t elapsed = now - runStartMs;
  uint32_t ticks; int32_t pos;
  encRead(&ticks, &pos);

  started = false;
  holdDone = false;
  targetSpeed = 0;
  appliedSpeed = 0;
  motorWrite(0);
  writeSteering(SERVO_CENTER);
  setLed(0, 0, 255);

  float tps = elapsed ? ((float)ticks * 1000.0f / (float)elapsed) : 0.0f;
  char msg[96];
  snprintf(msg, sizeof(msg), "RESULT,%s,%lu,%lu,%ld,%.2f\n",
           why, (unsigned long)elapsed, (unsigned long)ticks, (long)pos, tps);
  sendLine(msg);
}

// ============================================================================
// BUTTON
// ============================================================================
void pollButton(uint32_t now) {
  if (digitalRead(START_BTN_PIN) == HIGH) {
    btnLowSinceMs = 0;
    btnWasPressed = false;
    return;
  }
  if (btnLowSinceMs == 0) btnLowSinceMs = now;
  if (!btnWasPressed && (now - btnLowSinceMs >= BUTTON_DEBOUNCE_MS)) {
    btnWasPressed = true;
    if (!started) startRun(now);
    else          stopRun(now, "BUTTON");
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // FIRST. Before serial, before anything. IN1 floating = full throttle.
  // digitalWrite ONLY here - deliberately no analogWrite until after the servo
  // has attached and claimed its 50 Hz LEDC timer. See PATCH 3.
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);

  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  WiFi.mode(WIFI_OFF);                     // rule 11.10
  btStop();

  pinMode(START_BTN_PIN, INPUT_PULLUP);
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  strip.begin();
  setLed(255, 0, 0);

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  centerWheels();
  motorWrite(0);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encISR, RISING);

  setLed(0, 0, 255);
  sendLine("INFO,READY - press START for a 5 s forward run\n");
}

// ============================================================================
// LOOP  - never blocks
// ============================================================================
void loop() {
  uint32_t now = millis();

  pollButton(now);

  if (started && !holdDone && (now - runStartMs >= DRIVE_MS)) {
    holdDone = true;
    targetSpeed = 0;
    sendLine("INFO,5 s elapsed - ramping down\n");
  }

  updateMotor(now, started ? targetSpeed : 0);

  if (started && now - lastTickMsgMs >= TICK_REPORT_MS) {
    lastTickMsgMs = now;
    uint32_t ticks; int32_t pos;
    encRead(&ticks, &pos);
    char line[48];
    snprintf(line, sizeof(line), "TICKS,%lu,%ld,%d\n",
             (unsigned long)ticks, (long)pos, appliedSpeed);
    sendLine(line);
  }

  // Run ends once the ramp-down has actually reached zero.
  if (started && holdDone && appliedSpeed == 0) {
    stopRun(now, "RUN_COMPLETE");
  }

  if (!started) setLed(0, 0, 255);
  else          setLed(0, 255, 0);
}
