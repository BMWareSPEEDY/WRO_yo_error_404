/*
 * ============================================================================
 *  surprise_rule - WRO 2026 Future Engineers, Team Yo_Error_404
 *  ESP32-C3 Super Mini - MODULAR SURPRISE RULE & CUSTOM MANEUVER PLAYGROUND
 * ============================================================================
 *
 *  Duplicate / modular extension of final_open_round_working.ino with rich,
 *  reusable primitive functions to quickly script surprise rules without AI:
 *
 *  MOTION FUNCTIONS:
 *    - moveForwardCm(cm, [speed], [useHeadingHold])       -> Drive forward X cm (gyro locked)
 *    - moveBackwardCm(cm, [speed], [useHeadingHold])      -> Drive reverse X cm (gyro locked)
 *    - turnRightDeg(deg, [speed], [maxSteer])             -> Turn right X degrees (BNO yaw)
 *    - turnLeftDeg(deg, [speed], [maxSteer])              -> Turn left X degrees (BNO yaw)
 *    - turn90Right() / turn90Left()                       -> Quick 90 deg corner turns
 *    - moveForwardUntilWall(stopDistCm, [speed], [t_out]) -> Drive forward until front ToF <= stopDistCm
 *    - moveForwardWithWallCentering(cm, [speed], [tgt])   -> Drive X cm while holding wall distance
 *    - dodgeObstacle(passRight, [outCm], [fwdCm], [inCm]) -> Swerve around unexpected obstacle
 *    - alignWithWalls([targetWallDist], [timeoutMs])      -> Center between walls using side ToFs
 *    - halt([reason])                                     -> Safe motor stop + keep telemetry streaming
 *    - executeAutonomousOpenRound()                       -> The full standard 12-corner open round
 *
 *  Continuous 50 Hz telemetry (DATA,...) is maintained during every maneuver
 *  so the Pi dashboard at http://error404.local:8080 stays live and responsive.
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
#define IN1_PIN 1               // PWM
#define IN2_PIN 0               // DIR
#define MOTOR_FORWARD_LEVEL LOW  // measured: HIGH drove the car backwards
#define MOTOR_REVERSE_LEVEL HIGH
#define SERVO_PIN 6    
#define NEOPIXEL_PIN 7 
#define START_BTN_PIN 10 
#define ENC_A_PIN 4             // Wheel encoder channel A (interrupt, RISING)
#define ENC_B_PIN 5             // Wheel encoder channel B (direction)

// ==============================
// I2C Multiplexer Routing
// ==============================
#define TCAADDR 0x70
#define TOF_FRONT_CH 0
#define TOF_RIGHT_CH 2          // Physical chassis swap: Right is ch2, Left is ch1
#define TOF_LEFT_CH 1
#define BNO_CH 4

// ==============================
// Calibrations & Constants
// ==============================
#define TICKS_PER_CM 20.78f     // 2078 ticks/m, measured with encoder test
#define SERVO_CENTER_DEG 90
#define SERVO_MIN_DEG 28
#define SERVO_MAX_DEG 142       
#define SERVO_SHIFT_LEFT 60
#define SERVO_SHIFT_RIGHT 120

#define CORNER_KP 0.55f         // servo deg error per deg yaw error
#define CORNER_MIN_OFF 7.0f
#define CORNER_MAX_OFF 30.0f

#define TURN_THRESHOLD_CM 65.0f
#define SIDE_GAP_THRESHOLD 50.0f
#define TURN_COOLDOWN_MS 750

#define BASE_SPEED 110
#define TURN_SPEED 110
#define REVERSE_SPEED 110
#define STOP_AFTER_TIME 6500
#define STOP_BOT_AFTER_TURNS 12
#define FINISH_LINE_TIME_MS 1800

// ==============================
// Objects & Globals
// ==============================
Adafruit_VL53L0X tof[3]; 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Servo steeringServo;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

volatile long encoderTicks = 0;
float target_heading = 0.0f;
bool isTurning = false;
bool isReversing = false;        
int turnDirection = 0; 
int lockedTurnDirection = 0; 
int currentServoAngle = SERVO_CENTER_DEG;
int TURN = 0;
unsigned long turnStartTime = 0;
unsigned long lastTurnEndTime = 0; 

// Telemetry
#define TELEMETRY_PERIOD_MS 20  // 50 Hz
unsigned long lastTelemetryMs = 0;
unsigned long lastSerialTime = 0;
uint8_t calSys = 0, calGyro = 0, calAccel = 0, calMag = 0;
float lastDistF = 999.0f, lastDistL = 999.0f, lastDistR = 999.0f;
int lastAppliedSpeed = 0;

// ============================================================================
// ENCODER ISR & UTILITIES
// ============================================================================
void IRAM_ATTR encISR() {
  if (digitalRead(ENC_B_PIN) == HIGH) encoderTicks--;
  else                                encoderTicks++;
}

long readEncoderTicks() {
  noInterrupts();
  long t = encoderTicks;
  interrupts();
  return t;
}

void resetEncoder() {
  noInterrupts();
  encoderTicks = 0;
  interrupts();
}

float getDistanceMovedCm(long startTicks) {
  return (float)labs(readEncoderTicks() - startTicks) / TICKS_PER_CM;
}

// ============================================================================
// TELEMETRY & HARDWARE HELPERS
// ============================================================================
void sendLine(const char *s) {
  size_t n = strlen(s);
  if ((size_t)Serial.availableForWrite() >= n) Serial.write((const uint8_t *)s, n);
}

void sendTelemetry(float yaw) {
  char line[128];
  snprintf(line, sizeof(line), "DATA,%d,%d,%d,%d,%.1f,%u,%u,%u,%u,%d,%d,%ld\n",
           (int)lastDistF, (int)lastDistL, (int)lastDistR, 999,
           yaw, calSys, calGyro, calAccel, calMag,
           lastAppliedSpeed, currentServoAngle, readEncoderTicks());
  sendLine(line);
}

void tickTelemetry(float currentYaw) {
  if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = millis();
    bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
    sendTelemetry(currentYaw);
  }
}

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
  
  if (measure.RangeStatus == 4 || measure.RangeMilliMeter > 2000) return 999.0f; 
  return measure.RangeMilliMeter / 10.0f;      
}

float readCurrentHeading() {
  tcaselect(BNO_CH);
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

float getHeadingError(float target, float current) {
  float diff = target - current;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

void setServo(int deg) {
  currentServoAngle = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  steeringServo.write(currentServoAngle);
}

void driveMotor(int speed, bool forward) {
  lastAppliedSpeed = forward ? speed : -speed;
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
  setServo(SERVO_MIN_DEG);
  delay(400);
  setServo(SERVO_MAX_DEG);
  delay(400);
  setServo(SERVO_CENTER_DEG);
  delay(500); 
  Serial.println("Wheels centered.");
}

// ============================================================================
// MODULAR SURPRISE RULE BUILDING BLOCKS
// ============================================================================

/**
 * Move forward by X centimeters with closed-loop gyro heading-hold.
 */
void moveForwardCm(float cm, int speed = BASE_SPEED, bool useHeadingHold = true) {
  long startTicks = readEncoderTicks();
  float startHeading = readCurrentHeading();
  unsigned long startMs = millis();
  unsigned long timeoutMs = (unsigned long)((cm / 15.0f) * 1000.0f) + 3000;

  setNeoPixels(0, 255, 0);

  while (getDistanceMovedCm(startTicks) < cm && (millis() - startMs < timeoutMs)) {
    float curHeading = readCurrentHeading();
    float headingErr = getHeadingError(startHeading, curHeading);
    
    int steer = SERVO_CENTER_DEG;
    if (useHeadingHold) {
      steer = SERVO_CENTER_DEG + (int)(headingErr * 1.5f);
    }
    setServo(steer);
    driveMotor(speed, true);

    lastDistF = getDistance(TOF_FRONT_CH);
    lastDistL = getDistance(TOF_LEFT_CH);
    lastDistR = getDistance(TOF_RIGHT_CH);
    tickTelemetry(curHeading);

    delay(10);
  }
  driveMotor(0, true);
  setServo(SERVO_CENTER_DEG);
  delay(50);
}

/**
 * Move backward by X centimeters with gyro heading-hold.
 */
void moveBackwardCm(float cm, int speed = REVERSE_SPEED, bool useHeadingHold = true) {
  long startTicks = readEncoderTicks();
  float startHeading = readCurrentHeading();
  unsigned long startMs = millis();
  unsigned long timeoutMs = (unsigned long)((cm / 15.0f) * 1000.0f) + 3000;

  setNeoPixels(255, 80, 0);

  while (getDistanceMovedCm(startTicks) < cm && (millis() - startMs < timeoutMs)) {
    float curHeading = readCurrentHeading();
    float headingErr = getHeadingError(startHeading, curHeading);
    
    int steer = SERVO_CENTER_DEG;
    if (useHeadingHold) {
      // In reverse, steering direction is inverted
      steer = SERVO_CENTER_DEG - (int)(headingErr * 1.5f);
    }
    setServo(steer);
    driveMotor(speed, false);

    lastDistF = getDistance(TOF_FRONT_CH);
    lastDistL = getDistance(TOF_LEFT_CH);
    lastDistR = getDistance(TOF_RIGHT_CH);
    tickTelemetry(curHeading);

    delay(10);
  }
  driveMotor(0, false);
  setServo(SERVO_CENTER_DEG);
  delay(50);
}

/**
 * Turn right by X degrees using BNO055 yaw tracking and proportional corner easing.
 */
void turnRightDeg(float deg, int speed = TURN_SPEED, float maxSteer = SERVO_SHIFT_RIGHT) {
  float initialHeading = readCurrentHeading();
  float target = initialHeading + deg;
  while (target >= 360.0f) target -= 360.0f;
  while (target < 0.0f) target += 360.0f;

  unsigned long startMs = millis();
  unsigned long timeoutMs = 6000;

  setNeoPixels(255, 165, 0);

  while (millis() - startMs < timeoutMs) {
    float curHeading = readCurrentHeading();
    float headingErr = getHeadingError(target, curHeading);

    if (fabs(headingErr) < 4.0f) {
      break;
    }

    float off = fabs(headingErr) * CORNER_KP;
    off = constrain(off, CORNER_MIN_OFF, CORNER_MAX_OFF);
    int steer = SERVO_CENTER_DEG + (int)off;
    setServo(steer);
    driveMotor(speed, true);

    lastDistF = getDistance(TOF_FRONT_CH);
    lastDistL = getDistance(TOF_LEFT_CH);
    lastDistR = getDistance(TOF_RIGHT_CH);
    tickTelemetry(curHeading);

    delay(10);
  }

  driveMotor(0, true);
  setServo(SERVO_CENTER_DEG);
  target_heading = target;
  setNeoPixels(0, 255, 255);
  delay(50);
}

/**
 * Turn left by X degrees using BNO055 yaw tracking and proportional corner easing.
 */
void turnLeftDeg(float deg, int speed = TURN_SPEED, float maxSteer = SERVO_SHIFT_LEFT) {
  float initialHeading = readCurrentHeading();
  float target = initialHeading - deg;
  while (target >= 360.0f) target -= 360.0f;
  while (target < 0.0f) target += 360.0f;

  unsigned long startMs = millis();
  unsigned long timeoutMs = 6000;

  setNeoPixels(255, 165, 0);

  while (millis() - startMs < timeoutMs) {
    float curHeading = readCurrentHeading();
    float headingErr = getHeadingError(target, curHeading);

    if (fabs(headingErr) < 4.0f) {
      break;
    }

    float off = fabs(headingErr) * CORNER_KP;
    off = constrain(off, CORNER_MIN_OFF, CORNER_MAX_OFF);
    int steer = SERVO_CENTER_DEG - (int)off;
    setServo(steer);
    driveMotor(speed, true);

    lastDistF = getDistance(TOF_FRONT_CH);
    lastDistL = getDistance(TOF_LEFT_CH);
    lastDistR = getDistance(TOF_RIGHT_CH);
    tickTelemetry(curHeading);

    delay(10);
  }

  driveMotor(0, true);
  setServo(SERVO_CENTER_DEG);
  target_heading = target;
  setNeoPixels(0, 255, 255);
  delay(50);
}

/**
 * Standard 90-degree right turn.
 */
void turn90Right(int speed = TURN_SPEED) {
  turnRightDeg(90.0f, speed, SERVO_SHIFT_RIGHT);
}

/**
 * Standard 90-degree left turn.
 */
void turn90Left(int speed = TURN_SPEED) {
  turnLeftDeg(90.0f, speed, SERVO_SHIFT_LEFT);
}

/**
 * Drive forward with heading-hold until the front wall is closer than stopDistCm.
 */
void moveForwardUntilWall(float stopDistCm = 45.0f, int speed = BASE_SPEED, float timeoutMs = 7000) {
  float startHeading = readCurrentHeading();
  unsigned long startMs = millis();

  setNeoPixels(0, 255, 0);

  while (millis() - startMs < timeoutMs) {
    lastDistF = getDistance(TOF_FRONT_CH);
    lastDistL = getDistance(TOF_LEFT_CH);
    lastDistR = getDistance(TOF_RIGHT_CH);

    if (lastDistF <= stopDistCm && lastDistF > 2.0f) {
      break;
    }

    float curHeading = readCurrentHeading();
    float headingErr = getHeadingError(startHeading, curHeading);
    int steer = SERVO_CENTER_DEG + (int)(headingErr * 1.5f);
    setServo(steer);
    driveMotor(speed, true);

    tickTelemetry(curHeading);
    delay(10);
  }

  driveMotor(0, true);
  setServo(SERVO_CENTER_DEG);
  delay(50);
}

/**
 * Drive forward X cm while maintaining distance from walls using side ToFs + gyro.
 */
void moveForwardWithWallCentering(float cm, int speed = BASE_SPEED, float targetWallDist = 42.0f) {
  long startTicks = readEncoderTicks();
  float startHeading = readCurrentHeading();
  unsigned long startMs = millis();
  unsigned long timeoutMs = (unsigned long)((cm / 15.0f) * 1000.0f) + 3000;
  float Kp_Wall = 1.2f;

  while (getDistanceMovedCm(startTicks) < cm && (millis() - startMs < timeoutMs)) {
    lastDistF = getDistance(TOF_FRONT_CH);
    lastDistL = getDistance(TOF_LEFT_CH);
    lastDistR = getDistance(TOF_RIGHT_CH);

    float curHeading = readCurrentHeading();
    float headingError = getHeadingError(startHeading, curHeading);

    float wallCentering = 0;
    if (lastDistL < 80.0f && lastDistR < 80.0f) {
      wallCentering = (lastDistR - lastDistL) * (Kp_Wall * 0.5f);
    } else if (lastDistR < 80.0f) {
      wallCentering -= (targetWallDist - lastDistR) * Kp_Wall;
    } else if (lastDistL < 80.0f) {
      wallCentering += (targetWallDist - lastDistL) * Kp_Wall;
    }

    int steer = SERVO_CENTER_DEG + (int)((headingError * 1.2f) + wallCentering);
    setServo(steer);
    driveMotor(speed, true);

    tickTelemetry(curHeading);
    delay(10);
  }

  driveMotor(0, true);
  setServo(SERVO_CENTER_DEG);
  delay(50);
}

/**
 * Autonomous dodge sequence around an unexpected obstacle on the track.
 * passRight = true -> swerve right around block, false -> swerve left.
 */
void dodgeObstacle(bool passRight, float outwardCm = 25.0f, float forwardCm = 35.0f, float inwardCm = 25.0f) {
  int steerOut = passRight ? (SERVO_CENTER_DEG + 35) : (SERVO_CENTER_DEG - 35);
  int steerIn  = passRight ? (SERVO_CENTER_DEG - 35) : (SERVO_CENTER_DEG + 35);

  setNeoPixels(255, 0, 255); // Magenta / Dodge

  // Leg 1: Steer outward across the lane
  setServo(steerOut);
  moveForwardCm(outwardCm, BASE_SPEED, false);

  // Leg 2: Drive straight past the obstacle
  setServo(SERVO_CENTER_DEG);
  moveForwardCm(forwardCm, BASE_SPEED, true);

  // Leg 3: Steer inward back into the lane
  setServo(steerIn);
  moveForwardCm(inwardCm, BASE_SPEED, false);

  // Settle straight
  setServo(SERVO_CENTER_DEG);
  setNeoPixels(0, 255, 255);
}

/**
 * Clean system halt: stop motor, center steering, send STOP & status to Pi dashboard.
 */
void halt(const char* reason = "COMPLETE") {
  driveMotor(0, true);
  setServo(SERVO_CENTER_DEG);
  setNeoPixels(255, 0, 0);

  sendLine("STOP\n");
  char sline[80];
  snprintf(sline, sizeof(sline), "T:%d | State: %s\n", TURN, reason);
  sendLine(sline);
  Serial.print("System Halted: "); Serial.println(reason);

  while (1) {
    if (millis() - lastTelemetryMs >= 100) {
      lastTelemetryMs = millis();
      lastDistF = getDistance(TOF_FRONT_CH);
      lastDistR = getDistance(TOF_RIGHT_CH);
      lastDistL = getDistance(TOF_LEFT_CH);
      float h = readCurrentHeading();
      bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
      sendTelemetry(h);
    }
    delay(50);
  }
}

// ============================================================================
// FULL ORIGINAL OPEN ROUND (12 TURNS) - PRESERVED & AVAILABLE
// ============================================================================
void beginTurn() {
  isTurning = true;
  turnStartTime = millis();
  setNeoPixels(255, 165, 0);

  if (lockedTurnDirection == 1) {
    turnDirection = 1;
    target_heading += 90.0f;
    setServo(SERVO_SHIFT_RIGHT);
  } else {
    turnDirection = -1;
    target_heading -= 90.0f;
    setServo(SERVO_SHIFT_LEFT);
  }

  if (target_heading >= 360.0f) target_heading -= 360.0f;
  if (target_heading < 0.0f) target_heading += 360.0f;
}

void executeAutonomousOpenRound() {
  if (TURN >= STOP_BOT_AFTER_TURNS) {
    Serial.print("12 Turns Complete! Actively centering for ");
    Serial.print(FINISH_LINE_TIME_MS / 1000);
    Serial.println(" seconds to cross the section...");
    setNeoPixels(255, 0, 255); 
    
    unsigned long finishStartTime = millis();
    while (millis() - finishStartTime < FINISH_LINE_TIME_MS) {
      float distR = getDistance(TOF_RIGHT_CH);
      float distL = getDistance(TOF_LEFT_CH);
      float distF = getDistance(TOF_FRONT_CH);
      lastDistF = distF; lastDistL = distL; lastDistR = distR;
      
      float curH = readCurrentHeading();
      float headingError = getHeadingError(target_heading, curH);

      float wallCentering = 0;
      float Kp_Wall = 1.2f; 
      float targetWallDist = 42.0f; 

      if (distL < 80.0f && distR < 80.0f) {
        wallCentering = (distR - distL) * (Kp_Wall * 0.5f);
      } else if (lockedTurnDirection == 1) {
        if (distR < 80.0f) wallCentering -= (targetWallDist - distR) * Kp_Wall;
        else if (distL < 80.0f) wallCentering += (targetWallDist - distL) * Kp_Wall;
      } else if (lockedTurnDirection == -1) {
        if (distL < 80.0f) wallCentering += (targetWallDist - distL) * Kp_Wall;
        else if (distR < 80.0f) wallCentering -= (targetWallDist - distR) * Kp_Wall;
      } else {
        if (distR < 80.0f) wallCentering -= (targetWallDist - distR) * Kp_Wall;
        else if (distL < 80.0f) wallCentering += (targetWallDist - distL) * Kp_Wall;
      }

      int finishAngle = SERVO_CENTER_DEG + (int)((headingError * 1.2f) + wallCentering);
      setServo(finishAngle);
      driveMotor(BASE_SPEED, true);
      tickTelemetry(curH);
      delay(15); 
    }
    
    halt("RACE COMPLETE");
  }

  float distF = getDistance(TOF_FRONT_CH);
  float distR = getDistance(TOF_RIGHT_CH);
  float distL = getDistance(TOF_LEFT_CH);
  lastDistF = distF; lastDistL = distL; lastDistR = distR;

  float current_heading = readCurrentHeading();
  float headingError = getHeadingError(target_heading, current_heading);
  tickTelemetry(current_heading);

  if (!isTurning) {
    bool shouldTurn = false;

    if (distF <= TURN_THRESHOLD_CM && distF > 2.0f && (millis() - lastTurnEndTime > TURN_COOLDOWN_MS)) {
      bool rightOpen = (distR > SIDE_GAP_THRESHOLD); 
      bool leftOpen = (distL > SIDE_GAP_THRESHOLD);

      if (lockedTurnDirection == 0) {
        if (rightOpen && !leftOpen) {
          lockedTurnDirection = 1; 
          shouldTurn = true;
          Serial.println("LOCKED LAP DIRECTION: CLOCKWISE");
        } else if (leftOpen && !rightOpen) {
          lockedTurnDirection = -1; 
          shouldTurn = true;
          Serial.println("LOCKED LAP DIRECTION: ANTI-CLOCKWISE");
        } else if (distF <= 40.0f) { 
          lockedTurnDirection = (distR > distL) ? 1 : -1;
          shouldTurn = true;
          Serial.print("EMERGENCY BLIND LOCK: ");
          Serial.println(lockedTurnDirection == 1 ? "CW" : "CCW");
        }
      } else {
        if (lockedTurnDirection == 1 && (rightOpen || distF <= 40.0f)) shouldTurn = true;
        if (lockedTurnDirection == -1 && (leftOpen || distF <= 40.0f)) shouldTurn = true;
      }
    }

    if (shouldTurn) {
      beginTurn();
    } else {
      float wallCentering = 0;
      float Kp_Wall = 1.2f; 
      float targetWallDist = 42.0f; 

      if (distL < 80.0f && distR < 80.0f) {
        wallCentering = (distR - distL) * (Kp_Wall * 0.5f);
      } else if (lockedTurnDirection == 1) {
        if (distR < 80.0f) wallCentering -= (targetWallDist - distR) * Kp_Wall;
        else if (distL < 80.0f) wallCentering += (targetWallDist - distL) * Kp_Wall;
      } else if (lockedTurnDirection == -1) {
        if (distL < 80.0f) wallCentering += (targetWallDist - distL) * Kp_Wall;
        else if (distR < 80.0f) wallCentering -= (targetWallDist - distR) * Kp_Wall;
      } else {
        if (distR < 80.0f) wallCentering -= (targetWallDist - distR) * Kp_Wall;
        else if (distL < 80.0f) wallCentering += (targetWallDist - distL) * Kp_Wall;
      }

      int steer = SERVO_CENTER_DEG + (int)((headingError * 1.2f) + wallCentering);
      setServo(steer);
      driveMotor(BASE_SPEED, true);
    }
  } else {
    // 3-point recovery hysteresis
    if (distF < 12.0f) {
      isReversing = true;
    } else if (distF > 20.0f) {
      isReversing = false; 
    }

    if (isReversing) {
      int reverseAngle = (turnDirection == 1) ? SERVO_SHIFT_LEFT : SERVO_SHIFT_RIGHT;
      setServo(reverseAngle);
      driveMotor(REVERSE_SPEED, false); 
    } else {
      float off = fabs(headingError) * CORNER_KP;
      off = constrain(off, CORNER_MIN_OFF, CORNER_MAX_OFF);
      int steer = SERVO_CENTER_DEG + (turnDirection == 1 ? (int)off : -(int)off);
      setServo(steer);
      driveMotor(TURN_SPEED, true);
    }
    
    if (fabs(headingError) < 8.0f || (millis() - turnStartTime > STOP_AFTER_TIME)) {
      isTurning = false;
      isReversing = false; 
      TURN++;
      lastTurnEndTime = millis(); 
      setServo(SERVO_CENTER_DEG);
      setNeoPixels(0, 255, 255); 
    }
  }

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

// ============================================================================
// ARDUINO SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  pinMode(IN1_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN2_PIN, LOW);
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encISR, RISING);
  
  strip.begin();
  setNeoPixels(255, 0, 0); 

  Wire.begin(8, 9); 

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400); 
  centerWheels();
  driveMotor(0, true);

  // Init ToFs with retry
  for (int i = 0; i < 3; i++) {
    tcaselect(i);
    unsigned long firstFail = 0;
    while (!tof[i].begin()) {
      if (firstFail == 0) {
        firstFail = millis();
        Serial.print("ERROR: ToF Channel "); Serial.print(i);
        Serial.println(" not answering - retrying...");
      }
      setNeoPixels(255, 255, 0); delay(150); setNeoPixels(0, 0, 0); delay(150);
      tcaselect(i);
    }
  }

  // Init BNO055
  tcaselect(BNO_CH);
  while (!bno.begin()) {
    Serial.println("ERROR: No BNO055 detected - retrying...");
    setNeoPixels(255, 255, 0); delay(150); setNeoPixels(0, 0, 0); delay(150);
    tcaselect(BNO_CH);
  }
  delay(100);
  bno.setExtCrystalUse(true);

  setNeoPixels(0, 255, 0); 
  Serial.println("Hardware Ready. Waiting for START button...");

  // Stream live telemetry while waiting for button press
  while (digitalRead(START_BTN_PIN) == HIGH) {
    if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
      lastTelemetryMs = millis();
      lastDistF = getDistance(TOF_FRONT_CH);
      lastDistR = getDistance(TOF_RIGHT_CH);
      lastDistL = getDistance(TOF_LEFT_CH);
      float h = readCurrentHeading();
      bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
      sendTelemetry(h);
    }
    delay(5);
  }
  delay(50); 
  while (digitalRead(START_BTN_PIN) == LOW) { delay(10); }

  target_heading = readCurrentHeading();
  resetEncoder();
  
  setNeoPixels(0, 255, 255); 
  sendLine("START\n");
  Serial.print("Run Started! Target Heading locked at: ");
  Serial.println(target_heading);
}

// ============================================================================
// ARDUINO LOOP - SURPRISE RULE / CUSTOM SEQUENCE PLAYGROUND
// ============================================================================
void loop() {
  // --------------------------------------------------------------------------
  // OPTION A: Run the standard autonomous open round (12 turns)
  // --------------------------------------------------------------------------
  executeAutonomousOpenRound();

  // --------------------------------------------------------------------------
  // OPTION B: Write custom sequence here for the WRO Surprise Rule.
  // Example:
  //   moveForwardCm(120);              // Drive 120 cm forward on gyro lock
  //   turn90Right();                   // Turn 90 degrees right
  //   moveForwardUntilWall(40.0f);     // Drive until 40 cm from wall
  //   turn90Left();                    // Turn 90 degrees left
  //   moveBackwardCm(30.0f);           // Back up 30 cm
  //   dodgeObstacle(true);             // Swerve around an obstacle on right
  //   halt("MISSION COMPLETE");        // Stop cleanly with dashboard live
  // --------------------------------------------------------------------------
}
