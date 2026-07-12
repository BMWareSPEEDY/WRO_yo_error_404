#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>

// ==============================
// Pin definitions
// ==============================
#define BUTTON_PIN 5  // Push button connected to D5 and GND

#define MOTOR_IN1 18  // Forward
#define MOTOR_IN2 19  // Reverse

#define SERVO_PIN 23
#define SERVO_LEFT 30
#define SERVO_STRAIGHT 90
#define SERVO_RIGHT 150

// Ultrasonic sensors
#define TRIG_FRONT 12
#define ECHO_FRONT 14

#define TRIG_RIGHT 33
#define ECHO_RIGHT 32

#define TRIG_LEFT 27
#define ECHO_LEFT 26

// ==============================
// Globals
// ==============================
Servo steeringServo;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
bool systemStarted = false;
bool imuReady = false;

// Current motor and servo states
int currentSpeed = 0;
int currentSteering = SERVO_STRAIGHT;

// Timing for non-blocking operations
unsigned long lastSensorRead = 0;
unsigned long lastStatusSend = 0;
const unsigned long SENSOR_INTERVAL = 50;  // Read sensors every 50ms
const unsigned long STATUS_INTERVAL = 50;  // Send status every 50ms
const unsigned long MOTOR_COMMAND_TIMEOUT = 500; // Stop if Pi commands disappear

// Latest sensor readings
long frontDistance = 999;
long leftDistance = 999;
long rightDistance = 999;
float imuYaw = -999.0;
uint8_t imuSystemCalibration = 0;
uint8_t imuGyroCalibration = 0;
uint8_t imuAccelCalibration = 0;
uint8_t imuMagCalibration = 0;

// Command buffer to handle latest command only
String latestCommand = "";
bool hasNewCommand = false;
unsigned long lastMotorCommand = 0;

// ==============================
// Helper Functions
// ==============================
long readUltrasonicNonBlocking(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Use shorter timeout to prevent blocking
  long duration = pulseIn(echo, HIGH, 15000); // 15ms timeout instead of 25ms
  if (duration == 0) return 999;              // no echo
  return duration * 0.034 / 2;                // distance in cm
}

void setMotor(int speed) {
  if (speed > 0) {
    analogWrite(MOTOR_IN1, constrain(speed, 0, 255));
    analogWrite(MOTOR_IN2, 0);
  } else if (speed < 0) {
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, constrain(-speed, 0, 255));
  } else {
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
  }
  currentSpeed = speed;
}

void setSteering(int angle) {
  angle = constrain(angle, SERVO_LEFT, SERVO_RIGHT);
  steeringServo.write(angle);
  currentSteering = angle;
}

void readAllSensors() {
  // Read all three sensors quickly
  frontDistance = readUltrasonicNonBlocking(TRIG_FRONT, ECHO_FRONT);
  leftDistance = readUltrasonicNonBlocking(TRIG_LEFT, ECHO_LEFT);
  rightDistance = readUltrasonicNonBlocking(TRIG_RIGHT, ECHO_RIGHT);

  // The ESP32 owns the BNO055 and sends its heading to the Raspberry Pi.
  if (imuReady) {
    sensors_event_t orientationData;
    if (bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER)) {
      imuYaw = orientationData.orientation.x;
    } else {
      imuYaw = -999.0;
    }

    bno.getCalibration(
      &imuSystemCalibration,
      &imuGyroCalibration,
      &imuAccelCalibration,
      &imuMagCalibration
    );
  }
}

void processLatestCommand() {
  if (!hasNewCommand) return;
  
  String cmd = latestCommand;
  latestCommand = "";
  hasNewCommand = false;
  
  if (cmd.length() > 0) {
    int commaIndex = cmd.indexOf(',');
    if (commaIndex > 0) {
      int speed = cmd.substring(0, commaIndex).toInt();
      int steering = cmd.substring(commaIndex + 1).toInt();

      // Apply commands immediately
      setMotor(speed);
      setSteering(steering);
      lastMotorCommand = millis();
      
      // Optional: Send acknowledgment for critical commands
      if (abs(speed) > 200 || abs(steering - SERVO_STRAIGHT) > 40) {
        Serial.print("ACK:");
        Serial.print(speed);
        Serial.print(",");
        Serial.println(steering);
      }
    }
  }
}

void handleSerialInput() {
  // Process all available serial data to get the latest command
  while (Serial.available()) {
    String incomingCmd = Serial.readStringUntil('\n');
    incomingCmd.trim();
    
    if (incomingCmd.length() > 0) {
      // Always keep only the latest command (overwrite previous)
      latestCommand = incomingCmd;
      hasNewCommand = true;
    }
  }
}

// ==============================
// Setup
// ==============================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1); // Very short timeout for non-blocking reads

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  // BNO055 is connected to the ESP32 I2C bus: SDA=21, SCL=22.
  Wire.begin(21, 22);
  imuReady = bno.begin();
  if (imuReady) {
    bno.setMode(OPERATION_MODE_NDOF);
    delay(500);
    Serial.println("BNO055_READY");
  } else {
    Serial.println("ERROR_BNO055_NOT_FOUND");
  }

  steeringServo.attach(SERVO_PIN, 500, 2500);
  setSteering(SERVO_STRAIGHT);
  setMotor(0);

  Serial.println("ESP32_V2_READY");
  Serial.flush();
}

// ==============================
// Main Loop - Non-blocking approach
// ==============================
void loop() {
  unsigned long currentTime = millis();

  // Wait for the ESP32 start button, then tell the Raspberry Pi to begin
  if (!systemStarted) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      systemStarted = true;
      Serial.println("START");
      Serial.flush();

      // Initialize sensor readings
      readAllSensors();
      lastSensorRead = currentTime;
      lastStatusSend = currentTime;
      lastMotorCommand = currentTime;
    } else {
      // Discard movement commands until the start button is pressed
      while (Serial.available()) {
        Serial.read();
      }
    }

    delay(10);
    return;
  }

  // 1. HIGHEST PRIORITY: Handle incoming commands immediately
  handleSerialInput();
  processLatestCommand();

  // Fail safe: never continue driving on an old command if the Pi/link stops.
  if (currentSpeed != 0 && currentTime - lastMotorCommand > MOTOR_COMMAND_TIMEOUT) {
    setMotor(0);
    setSteering(SERVO_STRAIGHT);
    Serial.println("WATCHDOG_STOP");
    Serial.flush();
  }

  // 2. MEDIUM PRIORITY: Read sensors at regular intervals
  if (currentTime - lastSensorRead >= SENSOR_INTERVAL) {
    readAllSensors();
    lastSensorRead = currentTime;
  }

  // 3. LOW PRIORITY: Send every sensor reading at regular intervals
  if (currentTime - lastStatusSend >= STATUS_INTERVAL) {
    // DATA,front,left,right,yaw,system,gyro,accel,mag
    Serial.print("DATA,");
    Serial.print(frontDistance);
    Serial.print(",");
    Serial.print(leftDistance);
    Serial.print(",");
    Serial.print(rightDistance);
    Serial.print(",");
    Serial.print(imuYaw, 2);
    Serial.print(",");
    Serial.print(imuSystemCalibration);
    Serial.print(",");
    Serial.print(imuGyroCalibration);
    Serial.print(",");
    Serial.print(imuAccelCalibration);
    Serial.print(",");
    Serial.print(imuMagCalibration);
    Serial.println();
    Serial.flush(); // Ensure data is sent immediately
    
    lastStatusSend = currentTime;
  }

  // Small delay to prevent overwhelming the system
  delay(1); // Very small delay - allows for ~1000 Hz loop rate
}
