/*
 * ============================================================================
 *  WRO 2026 Future Engineers - Team Yo_Error_404
 *  ESP32-C3 Super Mini SLAVE  (the Raspberry Pi 5 is the master)
 * ============================================================================
 *
 *  ROLE
 *    The Pi (obstacle_round/pi/main.py) makes every driving decision. This
 *    board only:
 *      1. reads 4x VL53L0X ToF (TCA9548A ch0 front, ch1 LEFT, ch2 RIGHT,
 *         ch3 rear - left/right swapped on the car 2026-09-22) and the BNO055 (ch4)
 *      2. streams everything to the Pi at 50 Hz
 *      3. applies the Pi's speed + steering command to the motor and servo
 *      4. reports the START button
 *      5. stops the car if the Pi goes quiet for 500 ms (watchdog)
 *
 *  Pins, mux channels, servo limits and the centring sweep are copied from
 *  the working open-round sketch (open_round/Working_Open_Round_V2_FE).
 *
 *  SERIAL PROTOCOL  (USB CDC, 115200 baud, one ASCII line per message)
 *    Pi  -> ESP32   "<speed>,<steering>\n"
 *                     speed    -170..170   (negative = reverse, 0 = stop; capped here too)
 *                     steering   30..140   (servo degrees, 90 straight, >90 right)
 *    ESP32 -> Pi    "DATA,<F>,<L>,<R>,<B>,<yaw>,<sys>,<gyro>,<accel>,<mag>,<spd>,<str>,<ticks>\n"  50 Hz
 *                     F/L/R/B  ToF distance in cm, 999 = no valid reading
 *                     yaw      BNO055 heading 0.0..359.9 deg, clockwise positive
 *                     sys..mag BNO055 calibration 0..3
 *                     spd/str  what the motor and servo are actually getting
 *                     ticks    free-running wheel encoder count, never reset.
 *                              The Pi takes differences, so rollover is
 *                              harmless. 3041 ticks = 1 metre on this car.
 *
 *  PATCH NOTES
 *    PATCH 5 (2026-09-24) MAX_PWM 100 -> 110 so the Pi's 110 reaches the motor.
 *            This board caps independently of the Pi, so leaving it at 100
 *            would silently clip every command back and the change would look
 *            like it had no effect. Still inside the team's 170 limit.
 *    PATCH 4 (2026-09-24) MAX_PWM back to 100 at the user's call. Keeping the
 *            note below because the measurement stands: if the car ever moves
 *            in jerks again, suspect the PWM number before the control loop.
 *    PATCH 3 (2026-09-24) MAX_PWM 100 -> 140. The Pi was commanding 90 and
 *            being capped at 100, which is ~35% duty on the core's ~1 kHz PWM -
 *            where a geared motor cogs rather than turns, so the car moved in
 *            jerks. Nothing was wrong with the loops: the Pi held 48 Hz, this
 *            board held a 20 ms median with no stall over 66 ms, commanded
 *            speed never reached zero and the servo never hit its stops. The
 *            number itself was too low. Rohan had already fixed the identical
 *            problem in the open round (d324d96); the obstacle round never got
 *            it. 140 is still well inside the team's 170 PWM limit.
 *    PATCH 2 (2026-09-24) Stop the board going silent, and stop it blocking
 *            before loop() ever runs. Three changes, all from a real failure
 *            where the ESP32 was enumerated on USB but sent nothing for
 *            minutes, and a DTR/RTS reset fixed it:
 *              a) sendLine() dropped telemetry whenever the USB buffer was
 *                 full, silently and with no recovery. It now counts drops,
 *                 frees the endpoint when they pile up, and reports how many
 *                 were lost once the link is back.
 *              b) Sensor init was `while (!present)` with no exit, so a ToF or
 *                 IMU that would not start meant setup() never returned: no
 *                 telemetry, no watchdog, no response to the Pi. Bounded to
 *                 INIT_ATTEMPTS now; the board carries on degraded and says so.
 *              c) Absent sensors are retried in the background by pollTof and
 *                 pollBno, so one that shows up late rejoins without a reboot,
 *                 and a missing BNO is no longer read every loop for a yaw of 0.
 *            LED now distinguishes booting (red), retrying init (yellow blink),
 *            running (blue) and running degraded (orange).
 *    PATCH 1 (2026-09-21) Added the wheel encoder: GPIO4 channel A (interrupt,
 *            RISING) and GPIO5 channel B (read in the ISR for direction), and
 *            a 13th <ticks> field on every DATA line. This is what lets the Pi
 *            drive by measured DISTANCE instead of by timers - timed
 *            manoeuvres stretch and shrink with battery charge, tick counts do
 *            not. Encoder pins are the only INPUT_PULLUP pins added; never
 *            point a pull-up at GPIO0/GPIO1, which are the motor's PWM and DIR
 *            inputs (a pulled-up IN1 reads as 100% duty = full speed).
 *    ESP32 -> Pi    "START\n"   START pressed. Repeated every second until the
 *                               Pi's first drive command arrives.
 *    ESP32 -> Pi    "INFO,..." / "ERR,..."   status text
 *
 *  LED   red = initialising / error, blue = waiting for START, green = running
 *
 *  RULES
 *    11.10  Wi-Fi and Bluetooth are switched off in setup().
 *    9.14   Nothing moves before START: drive commands before it are ignored.
 *
 *  UPLOAD
 *    arduino-cli compile --upload -p /dev/cu.usbmodem* \
 *      --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc obstacle_round/esp/open_slave
 *    Libraries: ESP32Servo, Adafruit BNO055, Adafruit Unified Sensor,
 *               Adafruit_VL53L0X, Adafruit NeoPixel
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>

// ============================================================================
// PINS  (same as the open round)
// ============================================================================
// PWM is on GPIO1 and DIR on GPIO0 - proved with esp/dir_probe, which drove
// every pin combination and measured signed encoder movement for each. With
// these the other way round the throttle sits wide open, the commanded PWM
// lands on the direction pin, anything under 128 drives backwards, and reverse
// cannot work at all.
#define IN1_PIN        1    // MD10112 PWM
#define IN2_PIN        0    // MD10112 DIR
#define SERVO_PIN      6
#define NEOPIXEL_PIN   7
#define I2C_SDA_PIN    8
#define I2C_SCL_PIN    9
#define START_BTN_PIN 10    // to GND, internal pull-up

#define ENC_A_PIN      4    // wheel encoder channel A - interrupt, RISING
#define ENC_B_PIN      5    // wheel encoder channel B - direction only

// ============================================================================
// I2C
// ============================================================================
#define TCA_ADDR     0x70
#define BNO_ADDR     0x28
#define BNO_MUX_CH   4
#define I2C_CLOCK_HZ 100000

// Mux channels. LEFT and RIGHT were SWAPPED on the car 2026-09-22 during a
// hardware fix, so the sensor on channel 1 is now the left one and channel 2
// is the right. Fixing it here means everything downstream - telemetry, the
// Pi's wall centring, the corner side-gap checks - is correct without any of
// them needing to know.
enum { TOF_FRONT = 0, TOF_LEFT = 1, TOF_RIGHT = 2, TOF_REAR = 3, TOF_COUNT = 4 };  // = mux channel
const char *TOF_NAME[TOF_COUNT] = {"FRONT", "LEFT", "RIGHT", "REAR"};
#define TOF_PERIOD_MS    33     // continuous ranging, ~30 Hz per sensor
#define TOF_MAX_MM     2000     // same cut-off as the open round
#define TOF_NO_READING  999
#define TOF_STALE_MS    500     // no new measurement this long -> 999 + restart the sensor

// ============================================================================
// TIMING
// ============================================================================
#define SENSOR_PERIOD_MS     10   // 100 Hz poll
#define TELEMETRY_PERIOD_MS  20   // 50 Hz DATA
#define CALIB_PERIOD_MS     200
#define WATCHDOG_MS         500

// Sensor init gives up after this many tries instead of blocking setup()
// forever. pollTof keeps retrying in the background afterwards.
#define INIT_ATTEMPTS        12
#define INIT_ATTEMPTS_OPTIONAL 2   // the rear ToF - do not stall the boot for it
// Consecutive dropped telemetry lines before sendLine tries to rescue the USB
// endpoint and, once through, reports how many were lost.
#define TX_DROP_WARN         50
#define TX_KICK_PERIOD_MS    250
// How often to retry a sensor that was absent at boot. Slow on purpose.
#define ABSENT_RETRY_MS      1000

// LED states, so "booting", "stuck in init" and "running degraded" can be told
// apart without a laptop attached.
#define LED_BOOT_R      255
#define LED_BOOT_G      0
#define LED_BOOT_B      0
#define LED_INIT_R      255
#define LED_INIT_G      255
#define LED_INIT_B      0
#define LED_DEGRADED_R  255
#define LED_DEGRADED_G  40
#define LED_DEGRADED_B  0
#define START_REPEAT_MS    1000
#define BUTTON_DEBOUNCE_MS   60

// ============================================================================
// ACTUATORS
// ============================================================================
// The motor is driven EXACTLY like the working open-round sketch:
// analogWrite() on IN1 (default Arduino PWM) and DIR HIGH = forward.
// (An ESP32PWM 20 kHz channel here left the motor dead while the servo worked.)
// LOW = forward on this car. Verified by tick sign: positive ticks have always
// meant forward, and with HIGH a +120 command produced NEGATIVE ticks.
#define MOTOR_FORWARD_LEVEL LOW
#define MOTOR_REVERSE_LEVEL HIGH
#define MAX_PWM            110   // hard speed cap, forward and reverse, whatever the Pi sends
#define RAMP_STEP           30   // soft start: 0 -> 255 in ~70 ms
#define RAMP_DELAY_MS        8

#define SERVO_CENTER   90
#define SERVO_MIN      30        // 2 deg inside the linkage's mechanical stops (28..142),
#define SERVO_MAX     140        // so the servo never stalls against them

#define NEOPIXEL_COUNT 16

// ============================================================================
// STATE
// ============================================================================
Adafruit_VL53L0X tof[TOF_COUNT];
bool     tofPresent[TOF_COUNT];
uint16_t tofCm[TOF_COUNT];
uint32_t tofLastMs[TOF_COUNT];
uint32_t tofRetryMs[TOF_COUNT];

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDR, &Wire);
float   yawDeg = 0.0f;
uint8_t calSys = 0, calGyro = 0, calAccel = 0, calMag = 0;

Servo    steeringServo;
Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

uint8_t  muxChannel = 0xFF;

bool     bnoPresent = false;   // false = no heading; the Pi is told, not guessed at
bool     degraded = false;     // something is absent - shown on the LED and said once
uint32_t bnoRetryMs = 0;

bool     started = false;
bool     piHasCommanded = false;
bool     watchdogTripped = false;
uint32_t lastCmdMs = 0;
int      targetSpeed = 0;
int      appliedSpeed = 0;
int      steerCmd = SERVO_CENTER;
int      appliedSteer = SERVO_CENTER;

char     rxBuf[48];
uint8_t  rxLen = 0;
bool     rxOverflow = false;

// Wheel encoder. Free-running: never reset here, the Pi takes differences.
volatile int32_t encTicks = 0;

uint32_t lastSensorMs = 0, lastTelemetryMs = 0, lastCalibMs = 0;
uint32_t lastRampMs = 0, lastStartMsgMs = 0, btnLowSinceMs = 0;
bool     btnWasPressed = false;
uint32_t ledColor = 0xFFFFFFFF;

// ============================================================================
// HELPERS
// ============================================================================
// Counts forward travel as POSITIVE. This car's B channel reads HIGH when
// going forward, which is why the test sketches saw a negative position -
// the sign is flipped here so the Pi can trust "more ticks = further on".
void IRAM_ATTR encISR() {
  if (digitalRead(ENC_B_PIN)) encTicks++;
  else                        encTicks--;
}

int32_t encRead() {
  noInterrupts();
  int32_t t = encTicks;
  interrupts();
  return t;
}

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t c = strip.Color(r, g, b);
  if (c == ledColor) return;
  ledColor = c;
  for (int i = 0; i < NEOPIXEL_COUNT; i++) strip.setPixelColor(i, c);
  strip.show();
}

// Only write when the USB buffer has room, so a Pi that is not reading can
// never stall the loop. A dropped DATA line is replaced 20 ms later.
// Telemetry is dropped rather than blocked on: a full USB buffer must never
// stall the control loop. But dropping SILENTLY AND FOREVER is how the board
// went completely quiet for minutes at a time - alive, driving, and saying
// nothing, because availableForWrite() had been stuck at 0 since the host last
// closed the port. So count the drops and rescue the link when they pile up.
uint32_t txDrops = 0;          // consecutive lines dropped
uint32_t txDropsTotal = 0;     // since boot, reported once the link is back
uint32_t lastCdcKickMs = 0;

void sendLine(const char *s) {
  size_t n = strlen(s);
  if ((size_t)Serial.availableForWrite() >= n) {
    Serial.write((const uint8_t *)s, n);
    if (txDrops >= TX_DROP_WARN) {
      char msg[72];
      snprintf(msg, sizeof(msg),
               "ERR,USB_TX_RECOVERED after %lu dropped lines (%lu total)\n",
               (unsigned long)txDrops, (unsigned long)txDropsTotal);
      size_t m = strlen(msg);
      if ((size_t)Serial.availableForWrite() >= m) Serial.write((const uint8_t *)msg, m);
    }
    txDrops = 0;
    return;
  }

  txDrops++;
  txDropsTotal++;

  // Nothing is draining the endpoint. Clearing the buffer frees it for the
  // next line, so one wedged moment cannot silence the board permanently.
  uint32_t now = millis();
  if (txDrops >= TX_DROP_WARN && now - lastCdcKickMs >= TX_KICK_PERIOD_MS) {
    lastCdcKickMs = now;
    Serial.flush();          // returns immediately when the host is absent
    Serial.clearWriteError();
  }
}

bool tcaSelect(uint8_t ch) {
  if (ch == muxChannel) return true;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  bool ok = (Wire.endTransmission() == 0);
  muxChannel = ok ? ch : 0xFF;
  return ok;
}

// ============================================================================
// MOTOR & SERVO
// ============================================================================
// Same as driveMotor() in the open-round sketch.
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
  if (target == 0 && (watchdogTripped || !started)) {   // stops are immediate
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

// Same sweep as the open round (full left, full right, centre), kept inside the safe limits.
void centerWheels() {
  steeringServo.write(SERVO_MIN);    delay(400);
  steeringServo.write(SERVO_MAX);    delay(400);
  steeringServo.write(SERVO_CENTER); delay(500);
  appliedSteer = SERVO_CENTER;
}

// ============================================================================
// SENSORS
// ============================================================================
bool initTof(uint8_t i) {
  if (!tcaSelect(i)) return false;
  if (!tof[i].begin(0x29, false, &Wire)) return false;
  if (!tof[i].startRangeContinuous(TOF_PERIOD_MS)) return false;
  tofLastMs[i] = millis();
  return true;
}

void pollTof(uint8_t i, uint32_t now) {
  // An absent sensor is retried here rather than blocking setup(), so one that
  // was not ready at boot - or has been unplugged and put back - rejoins on its
  // own. Slowly: a missing sensor must not cost the loop any real time.
  if (!tofPresent[i]) {
    if (now - tofRetryMs[i] < ABSENT_RETRY_MS) return;
    tofRetryMs[i] = now;
    if (initTof(i)) {
      tofPresent[i] = true;
      char msg[48];
      snprintf(msg, sizeof(msg), "INFO,TOF_%s_BACK\n", TOF_NAME[i]);
      sendLine(msg);
    }
    return;
  }

  // A sensor that stops producing data has left continuous mode - restart it
  // instead of reporting a frozen number.
  if (now - tofLastMs[i] > TOF_STALE_MS) {
    tofCm[i] = TOF_NO_READING;
    if (now - tofRetryMs[i] > 500) {
      tofRetryMs[i] = now;
      bool ok = initTof(i);
      char msg[40];
      snprintf(msg, sizeof(msg), "ERR,TOF_%s_%s\n", TOF_NAME[i], ok ? "RESTARTED" : "NO_DATA");
      sendLine(msg);
    }
    return;
  }

  if (!tcaSelect(i)) return;
  bool ready = tof[i].isRangeComplete();
  if (tof[i].Status != VL53L0X_ERROR_NONE) { muxChannel = 0xFF; return; }
  if (!ready) return;

  uint16_t mm = tof[i].readRangeResult();                 // 0xFFFF = out of range
  if (tof[i].Status != VL53L0X_ERROR_NONE) { muxChannel = 0xFF; return; }
  tofLastMs[i] = now;
  tofCm[i] = (mm == 0xFFFF || mm > TOF_MAX_MM) ? TOF_NO_READING : (mm + 5) / 10;
}

bool initBno() {
  tcaSelect(BNO_MUX_CH);
  if (!bno.begin()) return false;          // NDOF
  delay(100);
  bno.setExtCrystalUse(true);
  return true;
}

void pollBno(uint32_t now) {
  // Same treatment as the ToFs: retry gently instead of blocking the boot, and
  // never read a device that is not answering - getEvent() on a missing BNO
  // burns I2C time every single loop for a yaw of 0.
  if (!bnoPresent) {
    if (now - bnoRetryMs < ABSENT_RETRY_MS) return;
    bnoRetryMs = now;
    if (initBno()) {
      bnoPresent = true;
      sendLine("INFO,BNO055_BACK\n");
    }
    return;
  }
  tcaSelect(BNO_MUX_CH);
  sensors_event_t event;
  bno.getEvent(&event);
  yawDeg = event.orientation.x;
  if (now - lastCalibMs >= CALIB_PERIOD_MS) {
    lastCalibMs = now;
    bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
  }
}

void sendTelemetry() {
  char line[128];
  snprintf(line, sizeof(line), "DATA,%u,%u,%u,%u,%.1f,%u,%u,%u,%u,%d,%d,%ld\n",
           tofCm[TOF_FRONT], tofCm[TOF_LEFT], tofCm[TOF_RIGHT], tofCm[TOF_REAR],
           yawDeg, calSys, calGyro, calAccel, calMag, appliedSpeed, appliedSteer,
           (long)encRead());
  sendLine(line);
}

// ============================================================================
// COMMANDS FROM THE PI
// ============================================================================
void handleLine(char *line, uint32_t now) {
  if (strcmp(line, "STOP") == 0 || strcmp(line, "RESET") == 0) {
    started = false;
    piHasCommanded = false;
    targetSpeed = 0;
    appliedSpeed = 0;
    motorWrite(0);
    steerCmd = SERVO_CENTER;
    writeSteering(SERVO_CENTER);
    setLed(0, 0, 255);
    sendLine("INFO,STOPPED\n");
    return;
  }
  if (strcmp(line, "START") == 0) {
    started = true;
    piHasCommanded = false;
    lastCmdMs = now;
    lastStartMsgMs = now;
    sendLine("START\n");
    return;
  }

  char *end;
  long speed = strtol(line, &end, 10);
  if (end == line || *end != ',') return;
  char *p = end + 1;
  long steer = strtol(p, &end, 10);
  if (end == p || (*end != '\0' && *end != '\r')) return;

  lastCmdMs = now;
  if (!started) return;                    // rule 9.14
  piHasCommanded = true;
  targetSpeed = constrain(speed, -MAX_PWM, MAX_PWM);
  steerCmd = constrain(steer, SERVO_MIN, SERVO_MAX);
}

void readSerial(uint32_t now) {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (!rxOverflow) { rxBuf[rxLen] = '\0'; handleLine(rxBuf, now); }
      rxLen = 0;
      rxOverflow = false;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxOverflow = true;
    }
  }
}

void pollButton(uint32_t now) {
  if (digitalRead(START_BTN_PIN) == HIGH) {
    btnLowSinceMs = 0;
    btnWasPressed = false;
    return;
  }
  if (btnLowSinceMs == 0) btnLowSinceMs = now;
  if (!btnWasPressed && (now - btnLowSinceMs >= BUTTON_DEBOUNCE_MS)) {
    btnWasPressed = true;
    if (!started) {
      started = true;
      piHasCommanded = false;
      lastCmdMs = now;                       // give the Pi a full watchdog window
      lastStartMsgMs = now;
      sendLine("START\n");
    } else {
      started = false;
      piHasCommanded = false;
      targetSpeed = 0;
      appliedSpeed = 0;
      motorWrite(0);
      steerCmd = SERVO_CENTER;
      writeSteering(SERVO_CENTER);
      setLed(0, 0, 255);
      sendLine("STOP\n");
    }
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  WiFi.mode(WIFI_OFF);                     // rule 11.10
  btStop();

  // Same order as the open-round sketch: pins, LEDs, servo, THEN the first motor write.
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  strip.begin();
  setLed(255, 0, 0);

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  centerWheels();
  motorWrite(0);

  // After the servo has its 50 Hz LEDC timer and the motor is safely at rest.
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encISR, RISING);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);

  // Sensor init is BOUNDED. These loops used to be `while (!present)` with no
  // way out, so a front, left or right ToF that would not start meant setup()
  // never returned and loop() never ran: no telemetry, no watchdog, no
  // response to the Pi, just a blinking board on the start line. A car that
  // drives with one dead sensor and keeps SAYING so is far more useful than a
  // car that is silent, so give up after INIT_ATTEMPTS and carry on degraded.
  setLed(LED_BOOT_R, LED_BOOT_G, LED_BOOT_B);
  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    tofCm[i] = TOF_NO_READING;
    tofPresent[i] = false;
    // The rear is optional and not fitted on this car, so it gets a token
    // couple of tries: twelve would add three seconds to every power-on
    // waiting for hardware that is not there. pollTof still retries it in the
    // background, so fitting one later needs no reflash.
    const uint8_t tries = (i == TOF_REAR) ? INIT_ATTEMPTS_OPTIONAL : INIT_ATTEMPTS;
    for (uint8_t att = 0; att < tries && !tofPresent[i]; att++) {
      tofPresent[i] = initTof(i);
      if (tofPresent[i]) break;
      char msg[56];
      snprintf(msg, sizeof(msg), "ERR,TOF_%s_MISSING attempt %u/%u\n",
               TOF_NAME[i], att + 1, tries);
      sendLine(msg);
      setLed(LED_INIT_R, LED_INIT_G, LED_INIT_B); delay(120);
      setLed(0, 0, 0); delay(120);
    }
    if (!tofPresent[i]) {
      char msg[72];
      // The rear sensor is not fitted on this car. Counting it as degraded
      // would leave the LED permanently orange, which costs the indicator all
      // of its meaning - the point is to show when something is WRONG.
      snprintf(msg, sizeof(msg), "ERR,TOF_%s_ABSENT%s\n", TOF_NAME[i],
               i == TOF_REAR ? " (optional, continuing)" : " - continuing without it");
      sendLine(msg);
      if (i != TOF_REAR) degraded = true;
    }
  }

  for (uint8_t att = 0; att < INIT_ATTEMPTS && !bnoPresent; att++) {
    bnoPresent = initBno();
    if (bnoPresent) break;
    char msg[48];
    snprintf(msg, sizeof(msg), "ERR,BNO055_MISSING attempt %u/%u\n", att + 1, INIT_ATTEMPTS);
    sendLine(msg);
    setLed(LED_INIT_R, LED_INIT_G, LED_INIT_B); delay(120);
    setLed(0, 0, 0); delay(120);
  }
  if (!bnoPresent) {
    sendLine("ERR,BNO055_ABSENT - no heading, the Pi cannot steer on gyro\n");
    degraded = true;
  }

  // pollTof retries an absent sensor in the background from here on, so a
  // sensor that comes back later rejoins without a reboot.
  for (uint8_t i = 0; i < TOF_COUNT; i++) tofLastMs[i] = millis();

  setLed(degraded ? LED_DEGRADED_R : 0, degraded ? LED_DEGRADED_G : 0,
         degraded ? LED_DEGRADED_B : 255);
  if (degraded) sendLine("INFO,READY (DEGRADED) - waiting for START\n");
  else          sendLine("INFO,READY - waiting for START\n");
}

// ============================================================================
// LOOP  - never blocks
// ============================================================================
void loop() {
  uint32_t now = millis();

  pollButton(now);
  readSerial(now);

  if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = now;
    for (uint8_t i = 0; i < TOF_COUNT; i++) pollTof(i, now);
    pollBno(now);
  }

  bool wasTripped = watchdogTripped;
  watchdogTripped = started && (now - lastCmdMs > WATCHDOG_MS);
  if (watchdogTripped && !wasTripped) sendLine("ERR,WATCHDOG - no command for 500 ms, motor stopped\n");
  if (!watchdogTripped && wasTripped) sendLine("INFO,WATCHDOG_CLEARED\n");

  updateMotor(now, (started && !watchdogTripped) ? targetSpeed : 0);
  writeSteering((started && !watchdogTripped) ? steerCmd : SERVO_CENTER);

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }

  if (started && !piHasCommanded && now - lastStartMsgMs >= START_REPEAT_MS) {
    lastStartMsgMs = now;
    sendLine("START\n");
  }

  if (!started)             setLed(0, 0, 255);
  else if (watchdogTripped) setLed(255, 0, 0);
  else                      setLed(0, 255, 0);
}
