# Source Code & Software Architecture (`/src`)

This directory houses all firmwares, autonomous control loops, computer vision software pipelines, and hardware testing scripts engineered for the **Team Yo_Error_404** autonomous vehicle competing in the WRO Future Engineers 2026 challenge.

---

## Directory Overview & Structure

```
src/
├── open_round/
│   ├── Open_Round.ino            # Standalone 100 Hz firmware for Open Challenge Round
│   └── README.md                 # Detailed documentation for Open Round control algorithms
└── obstacle_round/
    ├── ESP_Code_Slave.ino        # Real-time hardware co-processor firmware (100 Hz loop)
    ├── Raspberry_Pi_Code_Master.py# Master visual decision engine (60 FPS OpenCV pipeline)
    ├── README.md                 # Detailed documentation for Obstacle Round control algorithms
    └── testing_calibration_codes/
        ├── rx_tx_test.ino        # Serial UART communication echo test between Pi 5 & ESP32-C3
        └── hsv_tuner.py          # Interactive real-time HSV color threshold tuning tool
```

---

## Hardware Architecture & Controller Hierarchy

The software architecture is split into a **dual-controller hierarchy**:

```
 ┌─────────────────────────────────────────────────────────────────┐
 │                   Raspberry Pi 5 (4GB) Master                   │
 │  - Video Capture: 60 FPS via Pi Camera Module 3 (Wide Angle)   │
 │  - OpenCV Vision Pipeline: Red/Green Pillar & Limit Detection  │
 │  - Master State Machine & Trajectory Planning                  │
 └────────────────────────────────┬────────────────────────────────┘
                                  │ Serial UART (115200 Baud)
                                  ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │                   ESP32-C3 Super Mini Slave                     │
 │  - 100 Hz Hardware Real-Time Loop                               │
 │  - Spatial Sensing: 4× VL53L0X ToF via TCA9548A Multiplexer    │
 │  - Heading Hold: Adafruit BNO055 9-DOF Orientation Fusion       │
 │  - Actuation: 7semi MD10112 Driver (Motor) & 20kg Servo (Pwm)  │
 └─────────────────────────────────────────────────────────────────┘
```

---

## Challenge Round Breakdown

### 1. Open Challenge Round (`src/open_round/`)

In the Open Round, the vehicle operates in standalone mode using the **ESP32-C3 Super Mini** co-processor as the sole controller:

- **Firmware File**: [`src/open_round/Open_Round.ino`](./open_round/Open_Round.ino)
- **Primary Objective**: Complete 12 continuous track laps in the shortest time without hitting walls.
- **Control Strategy**:
  - **Wall-Centering PD Algorithm**: Maintains track dead-center by calculating distance difference between Left and Right VL53L0X ToF sensors:
    $$\text{Error} = d_{\text{left}} - d_{\text{right}}$$
    $$\text{Steering Correction} = K_p \cdot \text{Error} + K_d \cdot \frac{d(\text{Error})}{dt}$$
  - **BNO055 IMU Heading Hold**: Locks target yaw angle ($\Theta_{\text{target}}$) and applies PID heading correction during straightaways to eliminate drift.
  - **Gated 90° Turn Execution**: When Front ToF distance drops below $30.0\text{ cm}$, the controller executes a debounced 90° turn sequence, locking lap direction (clockwise vs. counter-clockwise) and updating baseline target heading by $\pm 90^\circ$.
  - **Race Finish Sequence**: Automatically counts turns. Upon completing turn 12, centers steering, pushes forward across the finish line for 750 ms, and engages active motor braking.

---

### 2. Obstacle Challenge Round (`src/obstacle_round/`)

In the Obstacle Round, the vehicle operates on the **Dual-Controller Architecture**:

- **Master Visual Engine**: [`src/obstacle_round/Raspberry_Pi_Code_Master.py`](./obstacle_round/Raspberry_Pi_Code_Master.py)
  - Captures 60 FPS video frames ($640 \times 480$ resolution).
  - Executes HSV color space thresholding to isolate:
    - **Red Pillars**: Must be bypassed on the **Right** side.
    - **Green Pillars**: Must be bypassed on the **Left** side.
    - **Magenta Lines**: Track boundary and parking space limiters.
  - Streams high-level steering and speed byte packets (`speed,steering`) over Serial UART at 115200 baud.
- **Hardware Slave Co-Processor**: [`src/obstacle_round/ESP_Code_Slave.ino`](./obstacle_round/ESP_Code_Slave.ino)
  - Executes received serial commands, generates high-frequency PWM signals for the 7semi MD10112 driver and 20kg DS3218 Digital Servo.
  - Polls 4× VL53L0X ToF sensors and BNO055 IMU at 100 Hz to prevent physical collision.
  - Transmits continuous status telemetry (`DATA,front,left,right,rear,yaw,sys,gyro,accel,mag`) at 20 Hz back to the Pi 5.

---

## Inter-Controller Communication Protocol

The Raspberry Pi 5 and ESP32-C3 exchange formatted ASCII strings over Serial UART at 115200 baud:

1. **Pi 5 to ESP32-C3 (Command Packet)**:
   ```
   <speed>,<steering>\n
   ```
   - `speed`: Integer range `-255` to `255` (Negative = Reverse, `0` = Stop, Positive = Forward).
   - `steering`: Servo angle integer range `30` (Full Left) to `150` (Full Right), centered at `90`.

2. **ESP32-C3 to Pi 5 (Telemetry Packet)**:
   ```
   DATA,<distF>,<distL>,<distR>,<distB>,<yaw>,<sys_cal>,<gyro_cal>,<accel_cal>,<mag_cal>\n
   ```
   - Reports ToF laser distances (cm), absolute BNO055 yaw heading (degrees), and sensor calibration metrics.

---

## Diagnostics & Calibration Tools (`src/obstacle_round/testing_calibration_codes/`)

- **`rx_tx_test.ino`**: Hardware serial loopback test verifying bi-directional UART communication integrity and packet loss rates.
- **`hsv_tuner.py`**: Interactive GUI tool running on the Pi 5 to fine-tune HSV color min/max bounds under changing competition lighting conditions.

---

## Programming Languages & Dependencies

- **C++ (Arduino IDE / ESP32-C3)**:
  - `Wire.h` (I2C Bus Communication)
  - `ESP32Servo.h` (Hardware Servo PWM)
  - `Adafruit_VL53L0X.h` (Time-of-Flight Sensor API)
  - `Adafruit_BNO055.h` (IMU Sensor Fusion API)
  - `Adafruit_NeoPixel.h` (WS2812B Status Indicator Control)
- **Python 3 (Raspberry Pi 5)**:
  - `opencv-python` (`cv2`) (Computer Vision)
  - `numpy` (Matrix & Spatial Operations)
  - `pyserial` (Serial UART Communication)
  - `picamera2` (Raspberry Pi Camera Module 3 Hardware Interface)

---

## Failsafe Mechanisms & Safety Watchdogs

1. **Serial Watchdog Timeout**: If the ESP32-C3 receives no serial command packets from the Pi 5 for $>500\text{ ms}$, the watchdog triggers an emergency motor halt (`PWM = 0`) and centers the steering servo.
2. **Obstacle Collision Override**: If the Front VL53L0X ToF distance drops below $15.0\text{ cm}$, the ESP32-C3 overrides incoming Pi 5 speed commands and applies maximum dynamic braking.
3. **Visual LED Telemetry (WS2812B NeoPixels)**:
   - **Red**: Initialization / Emergency Halt
   - **Green**: Systems Ready / Waiting for Start Button
   - **Cyan**: Normal Running / Straight Away Drive
   - **Orange**: Turn Sequence Active
   - **Magenta**: Race Finish / Parking Maneuver Complete
