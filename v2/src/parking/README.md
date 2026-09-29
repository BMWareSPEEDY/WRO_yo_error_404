# Parking Exit Module — Team Yo_Error_404
**WRO 2026 Future Engineers (V2 Vehicle)**

A completely isolated, self-contained module for developing, testing, and running the autonomous parking bay exit.

---

## Folder Structure

```
parking/
├── config.py                 # Tunable parking constants (angles, distances, speeds, gains)
├── parking_master.py         # Main Pi controller: manages start/stop, states, sensor checks
├── README.md                 # This guide
└── esp/
    ├── open_slave/           # Default ESP32 slave firmware (communicates with Pi over serial)
    └── parking_exit_standalone/  # Standalone ESP32 sketch (runs 100% on the ESP without Pi)
```

---

## How It Works

### Flow (Purely Encoder-Driven)
1. **Waiting (Standstill):**
   - The robot sits at standstill in the parking bay against one of the walls.
   - Live telemetry (Front, Left, Right ToF, IMU Yaw, Encoder ticks, Gyro cal) is streamed at 50 Hz.
2. **Press START** (either the physical button on GPIO 10 or the web dashboard button at `http://error404.local:8080/`):
   - **LOOK:** Reads Left and Right ToF sensors.
     - If Left $\le$ Right $\rightarrow$ Wall is on the **LEFT** $\rightarrow$ Exits **RIGHT** (`exit_dir = +1`).
     - If Right < Left $\rightarrow$ Wall is on the **RIGHT** $\rightarrow$ Exits **LEFT** (`exit_dir = -1`).
   - **STAGE 1: TURN OUT (25 cm)**
     - Sharp steering lock away from wall (`SERVO_CENTER + exit_dir * 50°`, i.e., 137° right or 37° left).
     - Drives forward until traveling **25.0 cm** as measured by the wheel encoder (`abs(ticks - stage_ticks0) / 20.78`).
     - Smoothly unwinds back toward center during the final 60% of the leg.
   - **STAGE 2: STRAIGHT ACROSS LANE (55 cm)**
     - Holds angle towards perpendicular (~80° relative to wall) using BNO055 heading hold P-controller (`HOLD_KP = 1.5`).
     - Drives forward **55.0 cm** across the lane as measured by the wheel encoder.
   - **STAGE 3: TURN BACK / RECENTER (25 cm)**
     - Steers opposite lock (`SERVO_CENTER - exit_dir * 50°`).
     - Drives forward **25.0 cm** as measured by the wheel encoder to swing parallel to the lane.
     - Smoothly unwinds back toward center during the final 60% of the leg.
   - **STAGE 4: FORWARD TO SQUARE UP (20 cm)**
     - Steers using IMU heading hold P-controller against initial heading to square up with lane walls.
     - Drives forward **20.0 cm** as measured by the wheel encoder.
   - **STAGE 5: HALT & RESET**
     - Completely stops motor (`speed = 0`), straightens wheels (`steer = 87`).
     - Prints distance, time, and final heading metrics.
     - Returns to WAITING state ready for next command.

### Run & Stop Controls
- **Stop during run:** Pressing the button at any time while the car is moving immediately cuts the motor and straightens the servo.
- **Repeat runs:** Pressing the button after stopping or after completion immediately starts a fresh parking run from scratch!

---

## Architecture & Hardware Settings

- **Motor PWM:** Speed 75 / 255.
- **Steering Limits:** `SERVO_CENTER = 87` (-3° trim), `STEER_LOCK_DEG = 50` (37° left / 137° right, well within hardware safe limits `[30, 140]`).
- **Wheel Encoder:** 20.78 ticks/cm (2078 ticks/m). Traveled distance is calculated as `abs(ticks - stage_ticks0) / TICKS_PER_CM`.
- **IMU:** BNO055 on TCA9548A Mux Ch 4 operating in `OPERATION_MODE_IMUPLUS` (accel + gyro relative yaw).

---

## Running on the Pi

### Option A: As a Systemd Service (Autostart on Boot)
The parking service is installed as `/etc/systemd/system/parking.service` and runs in tandem with `dashboard.service`.

To enable and start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable parking.service
sudo systemctl start parking.service
```

To check logs or status:
```bash
systemctl status parking.service
journalctl -u parking.service -f
```

To stop:
```bash
sudo systemctl stop parking.service
```

### Option B: Running Interactively in Terminal
Make sure `parking.service` and `master.service` are stopped first so they don't fight over the serial port:
```bash
sudo systemctl stop parking.service master.service
```

Then run directly:
```bash
python3 /home/pi/parking/parking_master.py
```

Then press the START button on GPIO 10 or click "Start Run" on `http://error404.local:8080/`.
