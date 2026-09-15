# Testing Log (`docs/Testing.md`)

Empirical testing records, validation benchmarks, hardware failure analysis, and tuning logs for the **Yo_Error_404** autonomous vehicle.

---

## 1. 10 June — Power Distribution & 3-Domain Power Isolation Benchmark

**Objective.** Verify voltage stability and rail isolation under peak motor acceleration and steering servo stall conditions.

**Test Procedure**:
- Measured voltage levels on the logic rail (3.3V / 5V) during simultaneous hard motor reversal and 20kg DS3218 steering servo stall sweeps.

**Results**:
- *Single-Rail Setup (Legacy)*: Voltage dropped to **4.62V**, triggering Raspberry Pi reboots and IMU heading corruption.
- *Isolated 3-Domain Setup (Final)*: Logic rail remained rock-solid at **5.02V / 3.31V** with **0 brownouts** across 100+ continuous load stress cycles.

---

## 2. 28 June — Spatial Sensing: VL53L0X ToF Laser vs. HC-SR04 Ultrasonic Sensors

**Objective.** Benchmark distance sensing accuracy, field of view (FOV), and refresh rates in narrow track corridors and angled wall entries.

**Results**:
- *HC-SR04 Ultrasonics*: Wide acoustic cone caused false reflections off angled walls in 32% of corner approaches.
- *VL53L0X ToF Laser Sensors*: Narrow laser photon beam delivered **millimeter accuracy**, zero false wall reflections, and sub-millisecond sensor response times.

---

## 3. 2 July — TCA9548A I2C Multiplexer Sub-Channel Switching Test

**Objective.** Validate I2C bus stability and channel switching throughput for 4x VL53L0X ToF sensors (`0x29`) and the BNO055 IMU on channel 4.

**Results**:
- Achieved stable 100 Hz polling loop across all 4 ToF channels and IMU with zero address collisions or I2C bus lockups.

---

## 4. 15 July — Drive Shaft Torsional Durability Test (PETG vs PLA)

**Objective.** Evaluate 3D-printed drive shaft coupler durability under repeated full-speed motor acceleration and instant braking cycles.

**Results**:
- *PLA Filament (100% Infill)*: Fractured along print layer lines after **8–12 acceleration cycles**.
- *PETG Filament (100% Infill)*: Survived **>100 continuous high-torque start/stop cycles and motor stall tests** with zero mechanical damage or layer separation.

---

## 5. 25 July — Steering Servo Holding Torque & Deflection Test

**Objective.** Benchmark front parallel steering alignment and angle deflection under high-speed cornering load.

**Results**:
- *MG996R / EMAX Servos*: Suffered 8°–12° steering deflection under chassis weight at speed.
- *20kg DS3218 Digital Servo (7.4V Rail)*: Zero measurable steering deflection, maintaining exact heading angles throughout high-speed corner entry.

---

## 6. 10 August — Closed-Loop Distance Engine & IMU Heading Fusion Benchmark

**Objective.** Verify distance stopping precision and turn execution accuracy using JGB37-520 magnetic quadrature encoder counts and BNO055 IMU orientation fusion ($\Delta\Theta$).

**Results**:
- *Distance Precision*: Vehicle stopped within **±2 mm** of target distance (2500 mm test benchmark).
- *Heading Accuracy*: Absolute BNO055 IMU orientation fusion executed exact 90° turns with zero cumulative drift over 12 continuous laps.
