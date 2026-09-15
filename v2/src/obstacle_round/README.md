# Obstacle Round Source Code (`src/obstacle_round`)

## Directory Overview

| File | Target Controller | Primary Job |
|:-----|:------------------|:------------|
| `ESP_Code_Slave.ino` | ESP32-C3 Super Mini | Real-time hardware co-processor: polls 4x ToF sensors & IMU, executes low-level PID control, and drives motor/servo PWM |
| `Raspberry_Pi_Code_Master.py` | Raspberry Pi 5 (4GB) | Visual autonomy brain: processes 60 FPS camera input, tracks lane pillars via OpenCV, computes target navigation state |
| `testing_calibration_codes/` | Both Controllers | Diagnostic, serial echo, and HSV color tuning utilities |

The Raspberry Pi 5 acts as the master decision engine while the ESP32-C3 Super Mini functions as the deterministic hardware controller.

---

## Inter-Controller Communication Protocol

The Raspberry Pi 5 and ESP32-C3 Super Mini communicate via high-speed Serial UART at 115200 baud.

1. **Pi 5 Command Output**: Streams `speed,steering` packets (e.g. `140,90`), where speed ranges from `-255` to `255` and steering ranges from `30` (full left) to `150` (full right), with `90` centered.
2. **ESP32-C3 Telemetry Output**: Transmits periodic telemetry (`DATA,front,left,right,rear,yaw,sys,gyro,accel,mag`) at 20 Hz, reporting ToF distances in cm, BNO055 absolute yaw in degrees, and sensor fusion status.

If serial connectivity drops for >500 ms, the ESP32-C3 safety watchdog halts motor propulsion and centers the steering servo.

---

## Hardware Architecture & Components

### Hardware Checklist
- **Primary Compute**: Raspberry Pi 5 (4GB) + Active Cooler
- **Co-Processor**: ESP32-C3 Super Mini Microcontroller
- **Vision Sensor**: Raspberry Pi Camera Module 3 (Wide Angle)
- **Spatial Sensing**: 4× VL53L0X Time-of-Flight (ToF) Sensors (Front, Left, Right, Rear)
- **I2C Routing**: TCA9548A 8-Channel I2C Multiplexer (`0x70` Address)
- **Orientation Sensing**: Adafruit BNO055 9-DOF IMU
- **Propulsion Motor**: JGB37-520 DC Motor with Quadrature Encoder (320 RPM, 1.8 kg·cm torque)
- **Motor Driver**: 7semi MD10112 High-Current MOSFET Driver
- **Steering Actuator**: 20kg DS3218 Digital Servo
- **Power System**: Isolated 3-Domain Power System (12V Direct Pack, Dual Buck Converters for 7.4V Servo & 3.3V Logic)

### ESP32-C3 Super Mini Pin Wiring Table

| Subsystem | ESP32-C3 Pin / Multiplexer Channel |
|:----------|:-----------------------------------|
| Motor Speed (PWM) | `GPIO 0` (`IN1_PIN`) |
| Motor Direction | `GPIO 1` (`IN2_PIN`) |
| Steering Servo PWM | `GPIO 6` (`SERVO_PIN`) |
| Status NeoPixel Strip | `GPIO 7` (`NEOPIXEL_PIN`) |
| Start Push Button | `GPIO 10` (`START_BTN_PIN`, `INPUT_PULLUP`) |
| I2C Bus (`SDA`/`SCL`) | `GPIO 8` / `GPIO 9` |
| Front VL53L0X ToF | `TCA9548A Channel 0` |
| Right VL53L0X ToF | `TCA9548A Channel 1` |
| Left VL53L0X ToF | `TCA9548A Channel 2` |
| Rear VL53L0X ToF | `TCA9548A Channel 3` |
| BNO055 9-DOF IMU | `TCA9548A Channel 4` |

---

## Raspberry_Pi_Code_Master.py

The master visual navigation software running on the Raspberry Pi 5.

### Software Dependencies & Setup

1. **Install picamera2 & OpenCV Dependencies**:
   ```bash
   sudo apt update
   sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-serial
   ```
2. **Configure Serial Permissions**:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
3. **Execution**:
   ```bash
   python3 Raspberry_Pi_Code_Master.py
   ```

### Navigation & Pillar Avoidance Rules
- **Red Traffic Pillars**: Must be passed on the **right side** (car steers right).
- **Green Traffic Pillars**: Must be passed on the **left side** (car steers left).
- Steering angle correction scales dynamically based on detected contour pixel area (10° for distant pillars, up to 60° for close pillars).

---

## Diagnostic & Calibration Tools (`testing_calibration_codes/`)

- `rx_tx_test.ino`: Hardware serial echo test verifying USB UART data integrity between Raspberry Pi 5 and ESP32-C3.
- `hsv_tuner.py`: Interactive real-time HSV color segmentation tuner with presets for Red (Pillars), Green (Pillars), and Magenta (Parking Limiters).
