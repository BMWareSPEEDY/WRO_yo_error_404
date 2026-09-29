/*
 * parking_exit_v2 - WRO 2026, Team Yo_Error_404, V2 bot (ESP32-C3)
 *
 *   Wall on the LEFT  ->  turn RIGHT 90, forward 80 cm, back 20 cm, turn LEFT 90
 *   Wall on the RIGHT ->  mirrored
 *
 * Angles end on the BNO055. Distances end on the encoder. Nothing ends on a clock.
 * Press START to run. Press again to stop. Press again to run it from scratch.
 */
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>

// ---- tune these ----
#define TURN_DEG        90      // how far to swing out, and back
#define FORWARD_CM      80      // straight leg after the turn
#define BACK_CM         20      // reverse before straightening up
#define SPEED           90      // PWM
#define STEER_LOCK      53      // full lock here: 87+53 = 140, 87-53 = 34
#define HOLD_KP        1.2      // heading hold on the straight legs
// --------------------

// ---- V2 hardware (do not change without checking the bot) ----
#define IN1_PIN  1              // PWM
#define IN2_PIN  0              // DIR, forward = LOW
#define FORWARD_LEVEL  LOW
#define REVERSE_LEVEL  HIGH
#define SERVO_PIN      6
#define LED_PIN        7
#define BTN_PIN       10
#define ENC_A          4
#define ENC_B          5
#define SDA_PIN        8
#define SCL_PIN        9
#define MUX         0x70
#define CH_FRONT       0
#define CH_LEFT        1
#define CH_RIGHT       2
#define CH_BNO         4

#define SERVO_STRAIGHT 87       // 90 is NOT straight on this car
#define SERVO_MIN      30
#define SERVO_MAX     140
#define MAX_PWM       110
#define TICKS_PER_CM 20.78      // 2078 ticks/m, measured
// Forward makes this encoder count DOWN, so distance is taken as a magnitude.

Adafruit_VL53L0X  tof[3];
Adafruit_BNO055   bno(55, 0x28, &Wire);
Servo             steer;
Adafruit_NeoPixel led(16, LED_PIN, NEO_GRB + NEO_KHZ800);

volatile long ticks = 0;
float startHdg = 0;
int   turnDir  = 0;             // +1 = swing right, -1 = swing left

void IRAM_ATTR encISR() { digitalRead(ENC_B) ? ticks++ : ticks--; }

long readTicks() { long t; noInterrupts(); t = ticks; interrupts(); return t; }
float cmSince(long ref) { return fabs(readTicks() - ref) / TICKS_PER_CM; }

void mux(uint8_t ch) { Wire.beginTransmission(MUX); Wire.write(1 << ch); Wire.endTransmission(); }
void setLed(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < led.numPixels(); i++) led.setPixelColor(i, led.Color(r, g, b));
  led.show();
}
void drive(int pwm, bool fwd) {
  pwm = constrain(pwm, 0, MAX_PWM);
  if (!pwm) { analogWrite(IN1_PIN, 0); digitalWrite(IN2_PIN, LOW); return; }
  digitalWrite(IN2_PIN, fwd ? FORWARD_LEVEL : REVERSE_LEVEL);
  analogWrite(IN1_PIN, pwm);
}
void setSteer(float deg) { steer.write((int)constrain(deg, SERVO_MIN, SERVO_MAX)); }
float heading() { mux(CH_BNO); sensors_event_t e; bno.getEvent(&e); return e.orientation.x; }
float wrap180(float d) { while (d > 180) d -= 360; while (d < -180) d += 360; return d; }
float turnedSoFar() { return turnDir * wrap180(heading() - startHdg); }
float readSide(uint8_t ch) {
  mux(ch);
  for (int k = 0; k < 20 && !tof[ch].isRangeComplete(); k++) delay(5);
  uint16_t mm = tof[ch].readRangeResult();
  return (mm > 0 && mm <= 2000) ? mm / 10.0 : 999.0;
}
// A press must be held to count, so bounce is not mistaken for a press.
bool pressed() {
  if (digitalRead(BTN_PIN) != LOW) return false;
  for (int i = 0; i < 20; i++) { if (digitalRead(BTN_PIN) != LOW) return false; delay(2); }
  return true;
}
bool abortNow() {
  if (!pressed()) return false;
  while (digitalRead(BTN_PIN) == LOW) delay(10);
  return true;
}

// ---- the four moves -------------------------------------------------------

// Swing until the heading CROSSES `target`, not until it lands inside a window
// around it. The IMU is read about every 20 ms while the car is turning, so it
// steps straight over any window - 86 then 97 - and once past it, the distance
// to target only grows while the steering stays hard over. That is what made it
// spin in circles. A threshold cannot be missed: once crossed, it stays crossed.
//
//   rising = true   swinging away from 0, stop at >= target
//   rising = false  swinging back to 0,   stop at <= target
bool swingTo(float target, bool rising, int steerDir, const char *label) {
  Serial.printf("%s: swinging %s %.0f deg\n", label, rising ? "out to" : "back to", target);
  uint32_t lastLog = 0;
  while (true) {
    if (abortNow()) return false;
    float turned = turnedSoFar();
    if (rising ? (turned >= target) : (turned <= target)) {
      Serial.printf("%s: done at %.1f deg\n", label, turned);
      return true;
    }
    if (millis() - lastLog > 250) { lastLog = millis(); Serial.printf("   %.1f deg\n", turned); }
    setSteer(SERVO_STRAIGHT + steerDir * STEER_LOCK);
    drive(SPEED, true);
    delay(20);
  }
}

// Drive `cm`, holding whatever heading we are meant to be on.
bool straight(float cm, float holdAt, bool fwd, const char *label) {
  long ref = readTicks();
  Serial.printf("%s: %.0f cm %s\n", label, cm, fwd ? "forward" : "back");
  while (cmSince(ref) < cm) {
    if (abortNow()) return false;
    float err = holdAt - turnedSoFar();
    // Reversing inverts the steering, so the correction flips with it.
    setSteer(SERVO_STRAIGHT + (fwd ? 1 : -1) * turnDir * constrain(HOLD_KP * err, -20.0f, 20.0f));
    drive(SPEED, fwd);
    delay(20);
  }
  Serial.printf("%s: done, %.1f cm\n", label, cmSince(ref));
  return true;
}

void runOnce() {
  float L = 0, R = 0;
  for (int k = 0; k < 3; k++) { L += readSide(CH_LEFT); R += readSide(CH_RIGHT); }
  L /= 3; R /= 3;
  turnDir  = (L < R) ? +1 : -1;             // wall on the left -> swing right
  startHdg = heading();
  ticks    = 0;

  Serial.printf("\nL=%.0f R=%.0f -> wall on the %s, swinging %s. start hdg %.1f\n",
                L, R, (L < R) ? "LEFT" : "RIGHT", (turnDir > 0) ? "RIGHT" : "LEFT", startHdg);
  setLed(0, 60, 0);

  if (!swingTo(TURN_DEG, true,  turnDir, "1 TURN OUT"))                    { Serial.println("stopped"); return; }
  if (!straight(FORWARD_CM, TURN_DEG, true,  "2 FORWARD"))          { Serial.println("stopped"); return; }
  if (!straight(BACK_CM,    TURN_DEG, false, "3 BACK"))             { Serial.println("stopped"); return; }
  if (!swingTo(0,        false, -turnDir, "4 STRAIGHTEN"))                        { Serial.println("stopped"); return; }

  Serial.printf("DONE. final heading %.1f deg off start\n", turnedSoFar());
}

void setup() {
  // Motor pins safe first: an input here floats high and reads as full throttle.
  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);

  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  led.begin(); setLed(60, 0, 0);

  // Servo before any analogWrite, or they fight over the LEDC timer.
  steer.setPeriodHertz(50); steer.attach(SERVO_PIN, 500, 2400);
  setSteer(SERVO_STRAIGHT); drive(0, true);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, RISING);

  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(100000);
  for (int i = 0; i < 3; i++) {
    mux(i);
    if (tof[i].begin()) tof[i].startRangeContinuous(30);
    else Serial.printf("ToF %d missing\n", i);
  }
  mux(CH_BNO);
  while (!bno.begin()) { Serial.println("BNO055 missing"); setLed(60,60,0); delay(400); }
  delay(100); bno.setExtCrystalUse(true);

  Serial.println("parking_exit_v2 ready. Press START.");
}

void loop() {
  drive(0, true); setSteer(SERVO_STRAIGHT); setLed(0, 0, 60);

  // Idle: keep reading the IMU so it is settled by the time a run starts, and
  // say so once a second. If this heartbeat stops, the board is wedged - almost
  // always on the I2C read - rather than ignoring the button. btn=0 means the
  // pin is being pulled LOW, i.e. the board CAN see the button.
  // The button is polled FAST and on its own. The IMU is read only occasionally
  // here: an I2C read that stalls would otherwise slow this loop to a crawl and
  // the button would be sampled too rarely to catch a press at all.
  uint32_t beat = 0, polls = 0, imuUs = 0;
  float h = 0;
  while (!pressed()) {
    if (millis() - beat > 1000) {
      beat = millis();
      uint32_t t0 = micros();
      h = heading();
      imuUs = micros() - t0;
      Serial.printf("idle  btn=%d  hdg=%.1f  imu_read=%lu us  btn_polls/s=%lu\n",
                    digitalRead(BTN_PIN), h, (unsigned long)imuUs, (unsigned long)polls);
      polls = 0;
    }
    // Report the instant the pin CHANGES, so even a 10 ms glitch is visible.
    // If nothing ever prints here, the button is not pulling GPIO10 low at all.
    static int lastPin = HIGH;
    int nowPin = digitalRead(BTN_PIN);
    if (nowPin != lastPin) {
      lastPin = nowPin;
      Serial.printf("  >>> PIN CHANGED to %s at %lu ms\n",
                    nowPin == LOW ? "LOW (pressed)" : "HIGH (released)", (unsigned long)millis());
    }
    polls++;
    delay(5);
  }
  Serial.println("START pressed");
  while (digitalRead(BTN_PIN) == LOW) delay(10);
  delay(80);

  runOnce();

  drive(0, true); setSteer(SERVO_STRAIGHT);
  Serial.println("--- press START to run again ---");
}
