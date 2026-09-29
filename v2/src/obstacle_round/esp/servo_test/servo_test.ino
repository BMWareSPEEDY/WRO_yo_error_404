/*
 * servo_test - sweep the steering servo, nothing else.
 *
 * Bench test for the DS3218 on the V2 car. It drives ONLY the servo: the motor
 * pins are set OUTPUT and LOW first, because left as inputs the pull-up on the
 * PWM pin reads as full throttle and the car drives off on its own.
 *
 * Turns the wheels FULL LEFT, back to centre, FULL RIGHT, back to centre, over
 * and over, holding each position for a second so you can watch and measure it.
 * The limits sit 2 degrees inside the linkage's mechanical stops (28..142) so
 * the servo never stalls against them. For a fixed angle instead, call
 * steering.write(<angle>) in setup() and leave loop() empty.
 *
 * This REPLACES open_slave on the ESP32, so the car stops answering the Pi
 * until the slave firmware is flashed back.
 *
 *   arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc servo_test
 */
#include <ESP32Servo.h>

#define SERVO_PIN      6     // DS3218 steering servo
#define MOTOR_PWM_PIN  1     // hold the drive motor off while testing
#define MOTOR_DIR_PIN  0

#define SERVO_MIN     30     // inside the mechanical stops (28..142)
#define SERVO_MAX    140
#define SERVO_CENTER  90

Servo steering;

void setup() {
  Serial.begin(115200);

  // Motor pins OUTPUT and LOW before anything else.
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  digitalWrite(MOTOR_PWM_PIN, LOW);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, LOW);

  steering.setPeriodHertz(50);
  steering.attach(SERVO_PIN, 500, 2400);
  steering.write(SERVO_CENTER);
  delay(500);
  Serial.println("centred at 90 - sweeping");
}

void hold(const char *what, int angle, int ms) {
  Serial.print(what);
  Serial.print(" -> servo ");
  Serial.println(angle);
  steering.write(angle);
  delay(ms);
}

void loop() {
  hold("LEFT  ", SERVO_MIN, 1200);
  hold("centre", SERVO_CENTER, 800);
  hold("RIGHT ", SERVO_MAX, 1200);
  hold("centre", SERVO_CENTER, 800);
}
