#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>

// ==== PIN Definitions ====
#define BUTTON_PIN 5  // Push button connected to D5 and GND

#define MOTOR_IN1 18  // Forward
#define MOTOR_IN2 19  // Reverse

#define SERVO_PIN 23
#define SERVO_LEFT 30       // Mechanical Turn Limit (Max Left)
#define SERVO_STRAIGHT 90   // Straight Position
#define SERVO_RIGHT 150     // Mechanical Turn Limit (Max Right)

// Front Ultrasonic
#define TRIG_FRONT 12
#define ECHO_FRONT 14

// Right Ultrasonic
#define TRIG_RIGHT 33 
#define ECHO_RIGHT 32 

// Left Ultrasonic
#define TRIG_LEFT 27 
#define ECHO_LEFT 26 

// ==== Tuning Parameters ====
const int REVERSE_DURATION_MS = 400;    // Reduced from 600ms for faster turns
const int FINAL_FORWARD_DRIVE_MS = 450; // Extra ~10cm drive forward at mission completion

// PI Controller for Yaw
float Kp_yaw = 5.0;   
float Ki_yaw = 0.15;   
const float MAX_CORRECTION_YAW = 22.0; 
const float INTEGRAL_LIMIT_YAW = 150.0; 
const float DEADBAND_YAW = 0.5;    

// Cascaded Lane Centering Parameters
float K_lane_to_yaw = 0.4;             // Degrees of heading shift per cm of lane offset
const float MAX_YAW_HEADING_SHIFT = 12.0;// Max degrees the lane logic can alter the target heading
const float MAX_VALID_SIDE_WALL = 75.0; // Distances above 75cm are corner openings, NOT walls
const float LANE_DEADBAND_CM = 2.0;     // 2cm deadband to prevent oscillation

// Speeds
const int DRIVE_SPEED = 255;
const int TURN_SPEED = 255; 
const int REVERSE_SPEED = 255;

// ==== Global Variables ====
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo steeringservo;


int turn_count = 0;
int lap_count = 0;
int turns_per_lap = 4;
bool first_lap = true;

bool returning_to_start = false;
bool inTurn = false; 

float yawOffset = 0;
float targetYaw = 0;
float yawErrorIntegral = 0;
float starting_front_distance = 0;
bool starting_position_recorded = false;

unsigned long lastLogTime = 0; // For non-blocking logging


// ==== Fast Raw Distance Reader ====
float getRawUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Timeout reduced to 10000us (~1.7m max distance) for high speed
  long duration = pulseIn(echoPin, HIGH, 10000); 
  if (duration == 0) return 999.0;
  float distance = duration * 0.034 / 2.0;
  return (distance > 0 && distance < 400) ? distance : 999.0;
}

// ==== Fast Filtered Ultrasonic Reader ====
float readUltrasonicDistance(int trigPin, int echoPin) {
  float readings[3];
  for(int i = 0; i < 3; i++) {
    readings[i] = getRawUltrasonicDistance(trigPin, echoPin);
    delayMicroseconds(4000); // Reduced delay between ultrasound pulses
  }
  
  // Median Bubble Sort
  if (readings[0] > readings[1]) { float t = readings[0]; readings[0] = readings[1]; readings[1] = t; }
  if (readings[1] > readings[2]) { float t = readings[1]; readings[1] = readings[2]; readings[2] = t; }
  if (readings[0] > readings[1]) { float t = readings[0]; readings[0] = readings[1]; readings[1] = t; }
  
  return readings[1]; 
}

float getCompensatedYaw() {
  sensors_event_t orientationData;
  if (!bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER)) {
    return targetYaw; 
  }
  
  float currentYaw = orientationData.orientation.x;
  if (currentYaw > 180.0) currentYaw -= 360.0;
  if (currentYaw < -180.0) currentYaw += 360.0;
  
  return currentYaw - yawOffset;
}

void setMotor(int speed1, int speed2) {
  analogWrite(MOTOR_IN1, speed1);
  analogWrite(MOTOR_IN2, speed2);
}

void driveForward(int speed) { setMotor(speed, 0); }
void driveBackward(int speed) { setMotor(0, speed); }
void stopMotors() { setMotor(0, 0); }

// Cascaded Lane Adjustment: Computes desired heading modification (in degrees)
float calculateHeadingShift(float distLeft, float distRight) {
  bool validLeft = (distLeft <= MAX_VALID_SIDE_WALL);
  bool validRight = (distRight <= MAX_VALID_SIDE_WALL);

  if (validLeft && validRight) {
    float laneError = (distRight - distLeft) / 2.0; 

    if (fabs(laneError) < LANE_DEADBAND_CM) {
      return 0.0;
    }

    float shift = laneError * K_lane_to_yaw;
    return constrain(shift, -MAX_YAW_HEADING_SHIFT, MAX_YAW_HEADING_SHIFT);
  } 

  return 0.0;
}

void reverseBeforeTurn() {
  driveBackward(REVERSE_SPEED);
  delay(REVERSE_DURATION_MS);
  stopMotors();
  delay(20); 
}

void maintainHeading(float currentYaw, float distLeft, float distRight) {
  if (inTurn) return; 

  float headingShift = calculateHeadingShift(distLeft, distRight);
  float adjustedTargetYaw = targetYaw + headingShift;
  if (adjustedTargetYaw > 180.0) adjustedTargetYaw -= 360.0;
  if (adjustedTargetYaw < -180.0) adjustedTargetYaw += 360.0;

  float yawError = adjustedTargetYaw - currentYaw;
  if (yawError > 180.0) yawError -= 360.0;
  if (yawError < -180.0) yawError += 360.0;

  float totalCorrection = 0;
  if (fabs(yawError) >= DEADBAND_YAW) {
    yawErrorIntegral += yawError;
    if (yawErrorIntegral > INTEGRAL_LIMIT_YAW) yawErrorIntegral = INTEGRAL_LIMIT_YAW;
    if (yawErrorIntegral < -INTEGRAL_LIMIT_YAW) yawErrorIntegral = -INTEGRAL_LIMIT_YAW;
    
    totalCorrection = (Kp_yaw * yawError) + (Ki_yaw * yawErrorIntegral);
    if (totalCorrection > MAX_CORRECTION_YAW) totalCorrection = MAX_CORRECTION_YAW;
    if (totalCorrection < -MAX_CORRECTION_YAW) totalCorrection = -MAX_CORRECTION_YAW;
  } else {
    yawErrorIntegral *= 0.9;
  }

  int steeringAngle = SERVO_STRAIGHT + (int)round(totalCorrection);
  
  if (steeringAngle < SERVO_LEFT) steeringAngle = SERVO_LEFT;
  if (steeringAngle > SERVO_RIGHT) steeringAngle = SERVO_RIGHT;

  steeringservo.write(steeringAngle);
}

void turnRight90Degrees() {
  inTurn = true;

  steeringservo.write(SERVO_RIGHT);
  delay(30); 
  driveForward(TURN_SPEED);
  
  float turnTarget = targetYaw + 89.5;
  if (turnTarget > 180.0) turnTarget -= 360.0;
  
  unsigned long turnStart = millis();
  while(true) {
    float currentYaw = getCompensatedYaw();
    float error = turnTarget - currentYaw;
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;

    

    if (fabs(error) < 10.0) break; 
    
    if (millis() - turnStart > 3000) { 
      break;
    }
    delay(5); 
  }

  stopMotors();
  delay(20); 
  steeringservo.write(SERVO_STRAIGHT);
  delay(30); 

  targetYaw += 90.0;
  if (targetYaw > 180.0) targetYaw -= 360.0;
  
  yawErrorIntegral = 0; 
  turn_count++;
  inTurn = false;
  
}

void turnLeft90Degrees() {
  inTurn = true;

  steeringservo.write(SERVO_LEFT);
  delay(30); 
  driveForward(TURN_SPEED);

  float turnTarget = targetYaw - 89.5;
  if (turnTarget < -180.0) turnTarget += 360.0;

  unsigned long turnStart = millis();
  while(true) {
    float currentYaw = getCompensatedYaw();
    float error = turnTarget - currentYaw;
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;

    if (fabs(error) < 10.0) break;
    
    if (millis() - turnStart > 3000) {
      break;
    }
    delay(5); 
  }
  
  stopMotors();
  delay(20); 
  steeringservo.write(SERVO_STRAIGHT);
  delay(30); 

  targetYaw -= 90.0;
  if (targetYaw < -180.0) targetYaw += 360.0;
  
  yawErrorIntegral = 0; 
  turn_count++;
  inTurn = false;
  
}

void moveBackwardToSafeDistance() {
  driveBackward(REVERSE_SPEED);
  
  unsigned long startTime = millis();
  while (readUltrasonicDistance(TRIG_FRONT, ECHO_FRONT) < 40) {
    if(millis() - startTime > 1500) break; 
    delay(10); 
  }

  stopMotors();
  delay(20); 
}

void completeFirstLap() {
  if (first_lap) {
    turns_per_lap = turn_count;
    first_lap = false;
    lap_count = 1;
  }
}

void checkLapCompletion() {
  if (first_lap && turn_count >= 4) {
    completeFirstLap();
    return;
  }
  if (first_lap) return;

  if (turn_count >= turns_per_lap * (lap_count + 1)) {
    lap_count++;
    
    if (lap_count >= 3) {
      returning_to_start = true;
    }
  }
}

void recordStartingPosition() {
  starting_front_distance = readUltrasonicDistance(TRIG_FRONT, ECHO_FRONT);
  starting_position_recorded = true;
}

void returnToStartNavigation(float distFront, float distRight) {
  if (distFront <= (starting_front_distance + 8.0)) {
    
    driveForward(DRIVE_SPEED);
    delay(FINAL_FORWARD_DRIVE_MS);
    stopMotors();
    while (true) { delay(1000); } 
  }
  
  driveForward(DRIVE_SPEED);
  maintainHeading(getCompensatedYaw(), readUltrasonicDistance(TRIG_LEFT, ECHO_LEFT), readUltrasonicDistance(TRIG_RIGHT, ECHO_RIGHT));
}

void setup() {
  Serial.begin(115200);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);

  steeringservo.attach(SERVO_PIN, 500, 2500);
  steeringservo.write(SERVO_STRAIGHT);

  Wire.begin(21, 22); 
  if (!bno.begin()) {
    while (1);
  }
  bno.setMode(OPERATION_MODE_NDOF);
  delay(500); 

  yawOffset = getCompensatedYaw();
  targetYaw = getCompensatedYaw();
  yawErrorIntegral = 0;


  while (digitalRead(BUTTON_PIN) == HIGH) {
    delay(10); 
  }

  delay(1000); // 1-second delay maintained

  recordStartingPosition();
}

void loop() {
  float yawNow = getCompensatedYaw();
  float distFront = readUltrasonicDistance(TRIG_FRONT, ECHO_FRONT);
  float distRight = readUltrasonicDistance(TRIG_RIGHT, ECHO_RIGHT);
  float distLeft = readUltrasonicDistance(TRIG_LEFT, ECHO_LEFT);

  // Rate limit main sensor output to prevent log spam blocking execution
  if (millis() - lastLogTime >= 150) {
    lastLogTime = millis();
    }

  if (returning_to_start) {
    returnToStartNavigation(distFront, distRight);
    return;
  }

  checkLapCompletion();

  if (distFront < 22) { 
    moveBackwardToSafeDistance();
    return;
  }
  
  if (distFront < 65 && distRight > 85 && distLeft < 85) {
    reverseBeforeTurn();
    turnRight90Degrees();
  } else if (distFront < 65 && distLeft > 85 && distRight < 85) {
    reverseBeforeTurn();
    turnLeft90Degrees();
  } else {
    driveForward(DRIVE_SPEED);
    maintainHeading(yawNow, distLeft, distRight);
  }
}