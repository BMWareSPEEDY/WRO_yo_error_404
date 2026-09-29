/*
 * ============================================================================
 *  working_open_round - WRO 2026 Future Engineers, Team Yo_Error_404
 *  ESP32-C3 Super Mini - OPEN ROUND, FULLY STANDALONE
 * ============================================================================
 *
 *  THE CAR DRIVES ITSELF. The Raspberry Pi is NOT required - not to start, not
 *  to steer, not to finish. Power the ESP32, press START, and it runs the whole
 *  open round: 12 corners, lap direction locked at the first one, then a finish push.
 *
 *  IT STREAMS TELEMETRY AT 50 HZ FOR THE PI DASHBOARD:
 *      DATA,<front>,<left>,<right>,<rear>,<yaw>,<sys>,<gyro>,<accel>,<mag>,<spd>,<str>
 *  and periodic status:
 *      T:<turn> | F:<f> L:<l> R:<r> | H_Err:<err> | Ang:<ang> | State: <state>
 *  so the Pi dashboard at http://error404.local:8080 shows live ToFs, yaw,
 *  state, turns, and speed.
 *
 *  Non-blocking writes (sendLine) ensure the car never stalls when the Pi is
 *  disconnected or sleeping.
 * ============================================================================
 */
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>

// ==============================
// ESP32-C3 Hardware Pins
// ==============================
// Motor: PWM on GPIO1, DIRECTION on GPIO0.
#define IN1_PIN 1               // PWM
#define IN2_PIN 0               // DIR
#define MOTOR_FORWARD_LEVEL LOW  // measured: HIGH drove the car backwards
#define MOTOR_REVERSE_LEVEL HIGH
#define SERVO_PIN 6    
#define NEOPIXEL_PIN 7 
#define START_BTN_PIN 10 

// ==============================
// I2C Multiplexer Routing
// ==============================
#define TCAADDR 0x70
#define TOF_FRONT_CH 0
// LEFT and RIGHT were physically swapped on the car 2026-09-22, so these two
// channels are the other way round now.
#define TOF_RIGHT_CH 2
#define TOF_LEFT_CH 1
#define BNO_CH 4

// ==============================
// Objects & State
// ==============================
Adafruit_VL53L0X tof[3]; 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Servo steeringServo;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Turn Stop Settings
int TURN = 0;
#define STOP_AFTER_TIME 6500
#define STOP_BOT_AFTER_TURNS 12
#define FINISH_LINE_TIME_MS 1800 // Push forward this long after the 12th turn to cross the section
unsigned long turnStartTime = 0;
unsigned long lastTurnEndTime = 0; 

// ==============================
// 1. TURN SMOOTHNESS PARAMETERS
// ==============================
#define SERVO_CENTER_DEG 90
#define SERVO_MIN_DEG 28
#define SERVO_MAX_DEG 142       
#define SERVO_SHIFT_LEFT 60
#define SERVO_SHIFT_RIGHT 120

#define CORNER_KP        0.55   // servo degrees off centre per degree of error
#define CORNER_MIN_OFF   7.0    // never less, or the turn stalls
#define CORNER_MAX_OFF  30.0    // never more (matches SERVO_SHIFT_*)

// ==============================
// 2. TURN DECISION PARAMETERS
// ==============================
#define TURN_THRESHOLD_CM 65.0   // Front distance that starts a corner
#define SIDE_GAP_THRESHOLD 50.0  // Confirm a real corner and ignore noise
#define TURN_COOLDOWN_MS 750    // Mandatory straight drive time after a turn

#define BASE_SPEED 110           // ~half duty: smooth
#define TURN_SPEED 110           // speed through the corner
#define REVERSE_SPEED 110

float target_heading = 0.0;
bool isTurning = false;
bool isReversing = false;        
int turnDirection = 0; 
int lockedTurnDirection = 0; 
int currentServoAngle = SERVO_CENTER_DEG;

// ---------------------------------------------------------------- telemetry
// Sent for the Pi dashboard's benefit only. The car never reads anything back
// and never waits for anyone, so none of this affects the driving.
#define TELEMETRY_PERIOD_MS 20        // 50 Hz, same as the slave firmware
unsigned long lastTelemetryMs = 0;
uint8_t calSys = 0, calGyro = 0, calAccel = 0, calMag = 0;
float lastDistF = 999, lastDistL = 999, lastDistR = 999;
int   lastAppliedSpeed = 0;

// Only writes when the USB buffer has room. This is what makes the sketch
// independent of the Pi: with nothing reading, the buffer fills, the write is
// skipped, and the control loop carries on at full speed.
void sendLine(const char *s) {
  size_t n = strlen(s);
  if ((size_t)Serial.availableForWrite() >= n) Serial.write((const uint8_t *)s, n);
}

void sendTelemetry(float yaw) {
  char line[110];
  snprintf(line, sizeof(line), "DATA,%d,%d,%d,%d,%.1f,%u,%u,%u,%u,%d,%d\n",
           (int)lastDistF, (int)lastDistL, (int)lastDistR, 999,
           yaw, calSys, calGyro, calAccel, calMag,
           lastAppliedSpeed, currentServoAngle);
  sendLine(line);
}

unsigned long lastSerialTime = 0;

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setNeoPixels(uint8_t r, uint8_t g, uint8_t b) {
  for(int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

float getDistance(uint8_t muxChannel) {
  tcaselect(muxChannel);
  VL53L0X_RangingMeasurementData_t measure;
  tof[muxChannel].rangingTest(&measure, false); 
  
  if (measure.RangeStatus == 4 || measure.RangeMilliMeter > 2000) return 999.0; 
  return measure.RangeMilliMeter / 10.0;      
}

float getHeadingError(float target, float current) {
  float diff = target - current;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

void driveMotor(int speed, bool forward) {
  lastAppliedSpeed = forward ? speed : -speed;   // telemetry only
  if (speed == 0) {
    analogWrite(IN1_PIN, 0);       
    digitalWrite(IN2_PIN, MOTOR_FORWARD_LEVEL);
  } else {
    digitalWrite(IN2_PIN, forward ? MOTOR_FORWARD_LEVEL : MOTOR_REVERSE_LEVEL);
    analogWrite(IN1_PIN, speed);                 
  }
}

void centerWheels() {
  Serial.println("Running wheel centering sweep...");
  steeringServo.write(SERVO_MIN_DEG);
  delay(400);
  steeringServo.write(SERVO_MAX_DEG);
  delay(400);
  steeringServo.write(SERVO_CENTER_DEG);
  delay(500); 
  Serial.println("Wheels centered.");
}

void setup() {
  Serial.begin(115200);
  
  // Drive both motor pins LOW the moment they become outputs.
  pinMode(IN1_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN2_PIN, LOW);
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  
  strip.begin();
  setNeoPixels(255, 0, 0); 

  Wire.begin(8, 9); 

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400); 
  centerWheels();
  driveMotor(0, true);

  // Keep RETRYING a sensor that does not answer, rather than hanging forever.
  for (int i = 0; i < 3; i++) {
    tcaselect(i);
    unsigned long firstFail = 0;
    while (!tof[i].begin()) {
      if (firstFail == 0) {
        firstFail = millis();
        Serial.print("ERROR: ToF Channel "); Serial.print(i);
        Serial.println(" not answering - retrying (power-cycle the car if it stays)");
      }
      setNeoPixels(255, 255, 0); delay(150); setNeoPixels(0, 0, 0); delay(150);
      tcaselect(i);
    }
    if (firstFail) {
      Serial.print("ToF Channel "); Serial.print(i);
      Serial.print(" recovered after "); Serial.print((millis() - firstFail) / 1000.0, 1);
      Serial.println(" s");
    }
  }

  tcaselect(BNO_CH);
  while (!bno.begin()) {                 // retry, never hang
    Serial.println("ERROR: No BNO055 detected - retrying");
    setNeoPixels(255, 255, 0); delay(150); setNeoPixels(0, 0, 0); delay(150);
    tcaselect(BNO_CH);
  }
  delay(100);
  bno.setExtCrystalUse(true);

  setNeoPixels(0, 255, 0); 
  Serial.println("Hardware Ready. Waiting for START button...");

  // Waiting for START. Keep streaming telemetry while we wait, so the
  // dashboard shows live sensor values before the run.
  while (digitalRead(START_BTN_PIN) == HIGH) {
    if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
      lastTelemetryMs = millis();
      lastDistF = getDistance(TOF_FRONT_CH);
      lastDistR = getDistance(TOF_RIGHT_CH);
      lastDistL = getDistance(TOF_LEFT_CH);
      tcaselect(BNO_CH);
      sensors_event_t ev;
      bno.getEvent(&ev);
      bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
      sendTelemetry(ev.orientation.x);
    }
    delay(5);
  }
  delay(50); 
  while (digitalRead(START_BTN_PIN) == LOW) { delay(10); }

  tcaselect(BNO_CH);
  sensors_event_t event;
  bno.getEvent(&event);
  target_heading = event.orientation.x;
  
  setNeoPixels(0, 255, 255); 
  sendLine("START\n");                 // dashboard shows the run as live
  Serial.print("Run Started! Target Heading locked at: ");
  Serial.println(target_heading);
}

// Starts the corner itself: latch the new heading target and throw the servo to full lock.
void beginTurn() {
  isTurning = true;
  turnStartTime = millis();
  setNeoPixels(255, 165, 0);

  if (lockedTurnDirection == 1) {
    turnDirection = 1;
    target_heading += 90.0;
    currentServoAngle = SERVO_SHIFT_RIGHT;
  } else {
    turnDirection = -1;
    target_heading -= 90.0;
    currentServoAngle = SERVO_SHIFT_LEFT;
  }

  if (target_heading >= 360.0) target_heading -= 360.0;
  if (target_heading < 0.0) target_heading += 360.0;
}

void loop() {
  if (TURN >= STOP_BOT_AFTER_TURNS) {
    Serial.print("12 Turns Complete! Actively centering for ");
    Serial.print(FINISH_LINE_TIME_MS / 1000);
    Serial.println(" seconds to cross the section...");
    setNeoPixels(255, 0, 255); 
    
    unsigned long finishStartTime = millis();
    
    // Run the centering loop actively for FINISH_LINE_TIME_MS
    while (millis() - finishStartTime < FINISH_LINE_TIME_MS) {
      float distR = getDistance(TOF_RIGHT_CH);
      float distL = getDistance(TOF_LEFT_CH);
      float distF = getDistance(TOF_FRONT_CH);
      lastDistF = distF; lastDistL = distL; lastDistR = distR;
      
      tcaselect(BNO_CH);
      sensors_event_t event;
      bno.getEvent(&event);
      float headingError = getHeadingError(target_heading, event.orientation.x);

      float wallCentering = 0;
      float Kp_Wall = 1.2; 
      float targetWallDist = 42.0; 

      if (distL < 80.0 && distR < 80.0) {
        wallCentering = (distR - distL) * (Kp_Wall * 0.5);
      } else if (lockedTurnDirection == 1) {
        if (distR < 80.0) wallCentering -= (targetWallDist - distR) * Kp_Wall;
        else if (distL < 80.0) wallCentering += (targetWallDist - distL) * Kp_Wall;
      } else if (lockedTurnDirection == -1) {
        if (distL < 80.0) wallCentering += (targetWallDist - distL) * Kp_Wall;
        else if (distR < 80.0) wallCentering -= (targetWallDist - distR) * Kp_Wall;
      } else {
        if (distR < 80.0) wallCentering -= (targetWallDist - distR) * Kp_Wall;
        else if (distL < 80.0) wallCentering += (targetWallDist - distL) * Kp_Wall;
      }

      float correction = (headingError * 1.2) + wallCentering; 
      int finishAngle = SERVO_CENTER_DEG + correction;
      finishAngle = constrain(finishAngle, SERVO_MIN_DEG, SERVO_MAX_DEG);
      
      steeringServo.write(finishAngle);
      currentServoAngle = finishAngle;
      driveMotor(BASE_SPEED, true);

      if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
        lastTelemetryMs = millis();
        bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
        sendTelemetry(event.orientation.x);
      }
      
      delay(15); 
    }
    
    driveMotor(0, true);
    setNeoPixels(255, 0, 0); 
    sendLine("STOP\n");
    sendLine("T:12 | State: COMPLETE\n");
    Serial.println("RACE COMPLETE - System Halted.");
    while(1) {
      if (millis() - lastTelemetryMs >= 100) {
        lastTelemetryMs = millis();
        lastDistF = getDistance(TOF_FRONT_CH);
        lastDistR = getDistance(TOF_RIGHT_CH);
        lastDistL = getDistance(TOF_LEFT_CH);
        tcaselect(BNO_CH);
        sensors_event_t event;
        bno.getEvent(&event);
        bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
        sendTelemetry(event.orientation.x);
      }
      delay(50);
    } 
  }

  float distF = getDistance(TOF_FRONT_CH);
  float distR = getDistance(TOF_RIGHT_CH);
  float distL = getDistance(TOF_LEFT_CH);
  lastDistF = distF; lastDistL = distL; lastDistR = distR;   // telemetry only

  tcaselect(BNO_CH);
  sensors_event_t event;
  bno.getEvent(&event);
  float current_heading = event.orientation.x;
  float headingError = getHeadingError(target_heading, current_heading);

  // Telemetry for the Pi dashboard. Skipped silently if nothing is listening.
  if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = millis();
    bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
    sendTelemetry(current_heading);
  }

  if (!isTurning) {
    bool shouldTurn = false;

    // Trigger turn ONLY when distance is <= 65cm
    if (distF <= TURN_THRESHOLD_CM && distF > 2.0 && (millis() - lastTurnEndTime > TURN_COOLDOWN_MS)) {
      
      bool rightOpen = (distR > SIDE_GAP_THRESHOLD); 
      bool leftOpen = (distL > SIDE_GAP_THRESHOLD);

      // Lap 1 Direction Lock
      if (lockedTurnDirection == 0) {
        if (rightOpen && !leftOpen) {
          lockedTurnDirection = 1; 
          shouldTurn = true;
          Serial.println("LOCKED LAP DIRECTION: CLOCKWISE");
        } else if (leftOpen && !rightOpen) {
          lockedTurnDirection = -1; 
          shouldTurn = true;
          Serial.println("LOCKED LAP DIRECTION: ANTI-CLOCKWISE");
        } else if (distF <= 40.0) { 
          // Emergency Override increased to 40 to prevent getting too close to the wall on Lap 1
          lockedTurnDirection = (distR > distL) ? 1 : -1;
          shouldTurn = true;
          Serial.print("EMERGENCY BLIND LOCK: ");
          Serial.println(lockedTurnDirection == 1 ? "CW" : "CCW");
        }
      } 
      // Subsequent laps logic
      else {
        // Restored safety checks so the bot doesn't turn on false ToF readings
        if (lockedTurnDirection == 1 && (rightOpen || distF <= 40.0)) shouldTurn = true;
        if (lockedTurnDirection == -1 && (leftOpen || distF <= 40.0)) shouldTurn = true;
      }
    }

    if (shouldTurn) {
      beginTurn();

    } else {
      // Straight-Line Wall Centering
      float wallCentering = 0;
      float Kp_Wall = 1.2; 
      float targetWallDist = 42.0; 

      if (distL < 80.0 && distR < 80.0) {
        wallCentering = (distR - distL) * (Kp_Wall * 0.5);
      } else if (lockedTurnDirection == 1) {
        if (distR < 80.0) wallCentering -= (targetWallDist - distR) * Kp_Wall;
        else if (distL < 80.0) wallCentering += (targetWallDist - distL) * Kp_Wall;
      } else if (lockedTurnDirection == -1) {
        if (distL < 80.0) wallCentering += (targetWallDist - distL) * Kp_Wall;
        else if (distR < 80.0) wallCentering -= (targetWallDist - distR) * Kp_Wall;
      } else {
        if (distR < 80.0) wallCentering -= (targetWallDist - distR) * Kp_Wall;
        else if (distL < 80.0) wallCentering += (targetWallDist - distL) * Kp_Wall;
      }

      float correction = (headingError * 1.2) + wallCentering; 
      currentServoAngle = SERVO_CENTER_DEG + correction;
      currentServoAngle = constrain(currentServoAngle, SERVO_MIN_DEG, SERVO_MAX_DEG);
      
      steeringServo.write(currentServoAngle);
      driveMotor(BASE_SPEED, true);
    }
  } else {
    // ==========================================
    // AUTONOMOUS 3-POINT TURN CRASH RECOVERY
    // ==========================================
    
    // Hysteresis: Start reversing if too close, stop reversing only when safe
    if (distF < 12.0) {
      isReversing = true;
    } else if (distF > 20.0) {
      isReversing = false; 
    }

    if (isReversing) {
      // Wedged against the wall: INVERT steering and REVERSE hard
      int reverseAngle = (turnDirection == 1) ? SERVO_SHIFT_LEFT : SERVO_SHIFT_RIGHT;
      steeringServo.write(reverseAngle);
      currentServoAngle = reverseAngle;
      
      driveMotor(REVERSE_SPEED, false); 
    } else {
      // Smooth turn: the angle follows what is LEFT to turn, so it eases out.
      float off = fabs(headingError) * CORNER_KP;
      off = constrain(off, CORNER_MIN_OFF, CORNER_MAX_OFF);
      currentServoAngle = SERVO_CENTER_DEG + (turnDirection == 1 ? off : -off);
      currentServoAngle = constrain(currentServoAngle, SERVO_MIN_DEG, SERVO_MAX_DEG);
      steeringServo.write(currentServoAngle);
      driveMotor(TURN_SPEED, true);
    }
    
    if (abs(headingError) < 8.0 || (millis() - turnStartTime > STOP_AFTER_TIME)) {
      isTurning = false;
      isReversing = false; 
      TURN++;
      lastTurnEndTime = millis(); 
      
      currentServoAngle = SERVO_CENTER_DEG;
      steeringServo.write(currentServoAngle);
      setNeoPixels(0, 255, 255); 
    }
  }

  // Update serial print to reflect the new state variable (non-blocking for telemetry relay)
  if (millis() - lastSerialTime > 250) {
    lastSerialTime = millis();
    char sline[120];
    const char *stateStr = "STRAIGHT";
    if (isTurning) {
      if (isReversing) stateStr = "CRASH RECOVERY (REVERSE)";
      else stateStr = (turnDirection == 1) ? "TURNING CW" : "TURNING CCW";
    }
    snprintf(sline, sizeof(sline), "T:%d | F:%.0f L:%.0f R:%.0f | H_Err:%.1f | Ang:%d | State: %s\n",
             TURN, distF, distL, distR, headingError, currentServoAngle, stateStr);
    sendLine(sline);
  }
}