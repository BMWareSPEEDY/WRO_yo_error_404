/*
 * ============================================================================
 *  WRO 2026 Future Engineers - Team Yo_Error_404
 *  ESP32-C3 Super Mini SLAVE  (the Raspberry Pi 5 is the master)
 * ============================================================================
 *
 *  ROLE
 *    The Pi (obstacle_round/pi/main.py) makes every driving decision. This
 *    board only:
 *      1. reads 4x VL53L0X ToF (TCA9548A ch0..3) and the BNO055 (ch4)
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
 *    ESP32 -> Pi    "DATA,<F>,<L>,<R>,<B>,<yaw>,<sys>,<gyro>,<accel>,<mag>,<spd>,<str>\n"  50 Hz
 *                     F/L/R/B  ToF distance in cm, 999 = no valid reading
 *                     yaw      BNO055 heading 0.0..359.9 deg, clockwise positive
 *                     sys..mag BNO055 calibration 0..3
 *                     spd/str  what the motor and servo are actually getting
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
#define IN1_PIN        0    // MD10112 PWM
#define IN2_PIN        1    // MD10112 DIR
#define SERVO_PIN      6
#define NEOPIXEL_PIN   7
#define I2C_SDA_PIN    8
#define I2C_SCL_PIN    9
#define START_BTN_PIN 10    // to GND, internal pull-up

// ============================================================================
// I2C
// ============================================================================
#define TCA_ADDR     0x70
#define BNO_ADDR     0x28
#define BNO_MUX_CH   4
#define I2C_CLOCK_HZ 100000

enum { TOF_FRONT = 0, TOF_RIGHT = 1, TOF_LEFT = 2, TOF_REAR = 3, TOF_COUNT = 4 };  // = mux channel
const char *TOF_NAME[TOF_COUNT] = {"FRONT", "RIGHT", "LEFT", "REAR"};
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
#define START_REPEAT_MS    1000
#define BUTTON_DEBOUNCE_MS   60

// ============================================================================
// ACTUATORS
// ============================================================================
// The motor is driven EXACTLY like the working open-round sketch:
// analogWrite() on IN1 (default Arduino PWM) and DIR HIGH = forward.
// (An ESP32PWM 20 kHz channel here left the motor dead while the servo worked.)
#define MOTOR_FORWARD_LEVEL HIGH
#define MOTOR_REVERSE_LEVEL LOW
#define MAX_PWM            170   // hard speed cap, forward and reverse, whatever the Pi sends
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

uint32_t lastSensorMs = 0, lastTelemetryMs = 0, lastCalibMs = 0;
uint32_t lastRampMs = 0, lastStartMsgMs = 0, btnLowSinceMs = 0;
bool     btnWasPressed = false;
uint32_t ledColor = 0xFFFFFFFF;

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

// Only write when the USB buffer has room, so a Pi that is not reading can
// never stall the loop. A dropped DATA line is replaced 20 ms later.
void sendLine(const char *s) {
  size_t n = strlen(s);
  if ((size_t)Serial.availableForWrite() >= n) Serial.write((const uint8_t *)s, n);
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
  if (!tofPresent[i]) return;

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
  char line[96];
  snprintf(line, sizeof(line), "DATA,%u,%u,%u,%u,%.1f,%u,%u,%u,%u,%d,%d\n",
           tofCm[TOF_FRONT], tofCm[TOF_LEFT], tofCm[TOF_RIGHT], tofCm[TOF_REAR],
           yawDeg, calSys, calGyro, calAccel, calMag, appliedSpeed, appliedSteer);
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

  strip.begin();
  setLed(255, 0, 0);

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  centerWheels();
  motorWrite(0);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);

  // Front, right and left are needed to drive: keep retrying them. Rear is optional.
  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    tofCm[i] = TOF_NO_READING;
    tofPresent[i] = initTof(i);
    while (!tofPresent[i] && i != TOF_REAR) {
      char msg[32];
      snprintf(msg, sizeof(msg), "ERR,TOF_%s_MISSING\n", TOF_NAME[i]);
      sendLine(msg);
      setLed(255, 255, 0); delay(250); setLed(0, 0, 0); delay(250);
      tofPresent[i] = initTof(i);
    }
    if (!tofPresent[i]) sendLine("ERR,TOF_REAR_MISSING (optional, continuing)\n");
  }

  while (!initBno()) {
    sendLine("ERR,BNO055_MISSING\n");
    setLed(255, 255, 0); delay(250); setLed(0, 0, 0); delay(250);
  }

  setLed(0, 0, 255);
  sendLine("INFO,READY - waiting for START\n");
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
