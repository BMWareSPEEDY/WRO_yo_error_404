#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#define IN1_PIN 0
#define IN2_PIN 1
#define SERVO_PIN 6
#define NEOPIXEL_PIN 7
#define START_BTN_PIN 10
#define TCAADDR 0x70
#define TOF_FRONT_CH 0
#define TOF_RIGHT_CH 1
#define TOF_LEFT_CH 2
#define TOF_REAR_CH 3
#define BNO_CH 4
#define SERVO_CENTER_DEG 90
#define SERVO_MIN_DEG 28
#define SERVO_MAX_DEG 142
Adafruit_VL53L0X tof[4];
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Servo steeringServo;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
bool systemStarted = false;
bool imuReady = false;
bool tofReady[4] = {false, false, false, false};
int currentSpeed = 0;
int currentSteering = SERVO_CENTER_DEG;
unsigned long lastSensorRead = 0;
unsigned long lastStatusSend = 0;
const unsigned long SENSOR_INTERVAL = 50;
const unsigned long STATUS_INTERVAL = 50;
const unsigned long MOTOR_COMMAND_TIMEOUT = 500;
float frontDistance = 999.0;
float leftDistance = 999.0;
float rightDistance = 999.0;
float rearDistance = 999.0;
float imuYaw = -999.0;
uint8_t imuSystemCalibration = 0;
uint8_t imuGyroCalibration = 0;
uint8_t imuAccelCalibration = 0;
uint8_t imuMagCalibration = 0;
String latestCommand = "";
bool hasNewCommand = false;
unsigned long lastMotorCommand = 0;
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
void setNeoPixels(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}
float getDistance(uint8_t muxChannel) {
  if (muxChannel > 3 || !tofReady[muxChannel]) return 999.0;
  tcaselect(muxChannel);
  VL53L0X_RangingMeasurementData_t measure;
  tof[muxChannel].rangingTest(&measure, false);
  if (measure.RangeStatus == 4) return 999.0;
  return measure.RangeMilliMeter / 10.0;
}
void setMotor(int speed) {
  currentSpeed = speed;
  if (speed == 0) {
    analogWrite(IN1_PIN, 0);
    digitalWrite(IN2_PIN, LOW);
  } else if (speed > 0) {
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(IN1_PIN, constrain(speed, 0, 255));
  } else {
    digitalWrite(IN2_PIN, LOW);
    analogWrite(IN1_PIN, constrain(-speed, 0, 255));
  }
}
void setSteering(int angle) {
  angle = constrain(angle, SERVO_MIN_DEG, SERVO_MAX_DEG);
  steeringServo.write(angle);
  currentSteering = angle;
}
void readAllSensors() {
  frontDistance = getDistance(TOF_FRONT_CH);
  rightDistance = getDistance(TOF_RIGHT_CH);
  leftDistance = getDistance(TOF_LEFT_CH);
  rearDistance = getDistance(TOF_REAR_CH);
  if (imuReady) {
    tcaselect(BNO_CH);
    sensors_event_t orientationData;
    if (bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER)) {
      imuYaw = orientationData.orientation.x;
    } else {
      imuYaw = -999.0;
    }
    bno.getCalibration(&imuSystemCalibration, &imuGyroCalibration, &imuAccelCalibration, &imuMagCalibration);
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
      setMotor(speed);
      setSteering(steering);
      lastMotorCommand = millis();
    }
  }
}
void handleSerialInput() {
  while (Serial.available()) {
    String incomingCmd = Serial.readStringUntil('\n');
    incomingCmd.trim();
    if (incomingCmd.length() > 0) {
      latestCommand = incomingCmd;
      hasNewCommand = true;
    }
  }
}
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  strip.begin();
  setNeoPixels(255, 0, 0);
  Wire.begin(8, 9);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  setSteering(SERVO_CENTER_DEG);
  setMotor(0);
  for (int i = 0; i < 4; i++) {
    tcaselect(i);
    if (tof[i].begin()) {
      tofReady[i] = true;
    }
  }
  tcaselect(BNO_CH);
  imuReady = bno.begin();
  if (imuReady) {
    delay(100);
    bno.setExtCrystalUse(true);
    Serial.println("BNO055_READY");
  } else {
    Serial.println("ERROR_BNO055_NOT_FOUND");
  }
  setNeoPixels(0, 255, 0);
  Serial.println("ESP32_C3_READY");
}
void loop() {
  if (!systemStarted) {
    if (digitalRead(START_BTN_PIN) == LOW) {
      delay(50);
      if (digitalRead(START_BTN_PIN) == LOW) {
        systemStarted = true;
        setNeoPixels(0, 255, 255);
        Serial.println("START");
        lastMotorCommand = millis();
      }
    }
  }
  handleSerialInput();
  processLatestCommand();
  if (systemStarted && (millis() - lastMotorCommand > MOTOR_COMMAND_TIMEOUT)) {
    setMotor(0);
    setSteering(SERVO_CENTER_DEG);
    setNeoPixels(255, 165, 0);
  }
  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = millis();
    readAllSensors();
  }
  if (millis() - lastStatusSend >= STATUS_INTERVAL) {
    lastStatusSend = millis();
    Serial.print("DATA,");
    Serial.print(frontDistance, 1); Serial.print(",");
    Serial.print(leftDistance, 1); Serial.print(",");
    Serial.print(rightDistance, 1); Serial.print(",");
    Serial.print(rearDistance, 1); Serial.print(",");
    Serial.print(imuYaw, 1); Serial.print(",");
    Serial.print(imuSystemCalibration); Serial.print(",");
    Serial.print(imuGyroCalibration); Serial.print(",");
    Serial.print(imuAccelCalibration); Serial.print(",");
    Serial.println(imuMagCalibration);
  }
}
