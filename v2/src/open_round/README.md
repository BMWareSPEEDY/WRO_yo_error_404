# Open Round Source Code (`src/open_round`)

This directory contains the firmware, control algorithms, and calibration routines for the **ERROR 404** vehicle during the Open Round challenge.

---

## Directory Structure & Files

### 1. `Open_Round.ino`
The main autonomous control script responsible for navigation, wall-following algorithms, IMU heading correction, ToF distance measurement, and reactive turn control during Open Round competition runs.

#### Hardware & Pin Requirements
- **ESP32-C3 Microcontroller**
 - `IN1_PIN` (GPIO 0): Motor PWM Speed Control
 - `IN2_PIN` (GPIO 1): Motor Direction Control
 - `SERVO_PIN` (GPIO 6): Steering Servo PWM
 - `NEOPIXEL_PIN` (GPIO 7): WS2812B NeoPixel LED Strip Data
 - `START_BTN_PIN` (GPIO 10): Push-button start switch
 - `Wire` I2C SDA (GPIO 8), SCL (GPIO 9)
- **TCA9548A I2C Multiplexer** (`0x70` address)
 - Channel 0: Front VL53L0X ToF Distance Sensor
 - Channel 1: Right VL53L0X ToF Distance Sensor
 - Channel 2: Left VL53L0X ToF Distance Sensor
 - Channel 4: Adafruit BNO055 9-Axis IMU
- **Adafruit NeoPixel (WS2812B, 16 LEDs)**
 - Red: Initialization / Emergency Halt
 - Green: Hardware Ready / Waiting for Start Button
 - Cyan: Running / Straight Track Mode
 - Orange: Executing Turn Sequence
 - Magenta: 12-Turn Race Complete Push

#### Software Setup & Installation Instructions
1. **Install Arduino IDE** (v2.0 or higher recommended).
2. **Install ESP32 Board Package**:
 - Open Arduino IDE $\rightarrow$ `Preferences` $\rightarrow$ Add Additional Board Manager URL for Espressif Systems.
 - Go to `Board Manager`, search for **ESP32 by Espressif Systems**, and click **Install**.
3. **Install Required Libraries**:
 - `ESP32Servo.h`
 - `Adafruit_VL53L0X.h`
 - `Adafruit_BNO055.h`
 - `Adafruit_NeoPixel.h`
 - `Adafruit_Sensor.h`
 - `Wire.h`
4. **Flashing Firmware**:
 - Open `src/open_round/Open_Round.ino` in Arduino IDE.
 - Select Board: **ESP32C3 Dev Module** (or your specific ESP32-C3 target).
 - Select Port and click **Upload**.
 - Open **Serial Monitor** (115200 Baud Rate) for diagnostic logs.

#### Autonomous Control Algorithm Overview
1. **Boot Initialization**: Initializes I2C bus on GPIO 8/9, sweeps steering servo to calibrate endpoints, initializes all 3 ToF channels and BNO055 IMU through the TCA9548A multiplexer, and sets NeoPixels to Green.
2. **Start Signal**: Waits for `START_BTN_PIN` press, locks the baseline target heading from the BNO055 orientation sensor, and transitions NeoPixels to Cyan.
3. **Straight Track Navigation**: Runs heading error PID correction combined with dual-wall centering algorithms (outer/inner wall buffers: 55 cm / 45 cm) to keep the vehicle centered.
4. **Turn Execution**: Triggers when front ToF distance `distF <= 30.0 cm` (with 600 ms turn debouncing). Automatically locks lap direction (Clockwise vs Anti-Clockwise), adjusts target heading by $\pm 90^\circ$, shifts servo angle to turn limits (`32°` left / `142°` right), and engages apex wall-avoidance steering.
5. **Race Completion**: Tracks total completed turns `TURN`. Upon reaching `STOP_BOT_AFTER_TURNS = 12`, centers steering, pushes forward across the finish line for `750 ms`, halts motor, and turns NeoPixels Red.
