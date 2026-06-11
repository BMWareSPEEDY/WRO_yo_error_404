#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// --- BNO055 IMU SETUP ---
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
float headingOffset = 0.0;
float pitchOffset   = 0.0;
float rollOffset    = 0.0;

// --- SERVO PIN ---
const int SERVO_PIN = 23;
Servo steeringServo;

// --- MOTOR PINS ---
const int MOTOR_PIN1 = 18; // Forward pin
const int MOTOR_PIN2 = 19; // Reverse pin

// --- ULTRASONIC PINS ---
#define TRIG_PIN_1 12
#define ECHO_PIN_1 14

#define TRIG_PIN_2 27
#define ECHO_PIN_2 26

#define TRIG_PIN_3 33
#define ECHO_PIN_3 32

#define SOUND_SPEED 0.0343

// Function to read distance from an ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000); 
  return (duration * SOUND_SPEED) / 2.0;
}

// Function to control motor direction and speed (0 - 255)
void setMotor(int speed1, int speed2) {
  analogWrite(MOTOR_PIN1, speed1);
  analogWrite(MOTOR_PIN2, speed2);
}

// Function to print zero-calibrated BNO055 orientation
void printIMUData() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  float rawHeading = euler.x();
  float rawPitch   = euler.z();
  float rawRoll    = euler.y();

  float heading = rawHeading - headingOffset;
  float pitch   = rawPitch - pitchOffset;
  float roll    = rawRoll - rollOffset;

  // Keep heading bounded within 0 to 360 degrees
  if (heading < 0.0) {
    heading += 360.0;
  } else if (heading >= 360.0) {
    heading -= 360.0;
  }

  Serial.print("IMU -> Yaw: ");
  Serial.print(heading, 1);
  Serial.print("° | Pitch: ");
  Serial.print(pitch, 1);
  Serial.print("° | Roll: ");
  Serial.print(roll, 1);
  Serial.println("°");
}

void setup() {
  Serial.begin(115200);

  // Configure Motor Pins
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);

  // Configure Ultrasonic Pins
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);

  // Configure Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);

  // Initialize BNO055 IMU
  Serial.println("\nInitializing BNO055...");
  if (!bno.begin()) {
    Serial.println("Error: No BNO055 detected! Check SDA (GPIO 21) & SCL (GPIO 22).");
    while (1);
  }
  bno.setExtCrystalUse(true);

  // Calibrate Zero Position for IMU
  Serial.println("Keep the sensor still. Zeroing orientation...");
  delay(2000); // Allow sensor fusion engine to stabilize

  imu::Vector<3> initEuler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  headingOffset = initEuler.x();
  pitchOffset   = initEuler.z();
  rollOffset    = initEuler.y();

  Serial.println("--- All Systems Initialized & Calibrated ---\n");
}

void loop() {
  Serial.println("==========================================");

  // --- 1. Read Ultrasonic Sensors ---
  float dist1 = getDistance(TRIG_PIN_1, ECHO_PIN_1);
  delay(10);
  float dist2 = getDistance(TRIG_PIN_2, ECHO_PIN_2);
  delay(10);
  float dist3 = getDistance(TRIG_PIN_3, ECHO_PIN_3);

  Serial.print("Ultrasonics (cm) -> S1: "); Serial.print(dist1, 1);
  Serial.print(" | S2: "); Serial.print(dist2, 1);
  Serial.print(" | S3: "); Serial.println(dist3, 1);

  // --- 2. Read BNO055 IMU ---
  printIMUData();

  // --- 3. Test Steering Servo Sequence ---
  Serial.println("\n[Servo Test]");
  Serial.println("Action: Steering Left (45°)");
  steeringServo.write(45);
  delay(500);

  Serial.println("Action: Steering Center (90°)");
  steeringServo.write(90);
  delay(500);

  Serial.println("Action: Steering Right (135°)");
  steeringServo.write(135);
  delay(500);

  // --- 4. Test DC Motor Sequence ---
  Serial.println("\n[Motor Test]");
  Serial.println("Action: Motor Forward");
  setMotor(200, 0);
  delay(1000);

  Serial.println("Action: Motor Stop");
  setMotor(0, 0);
  delay(500);

  Serial.println("Action: Motor Reverse");
  setMotor(0, 200);
  delay(1000);

  Serial.println("Action: Motor Stop");
  setMotor(0, 0);
  delay(1000);
}