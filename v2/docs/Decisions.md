# Design Decisions (`docs/Decisions.md`)

This is a comprehensive record of the architectural choices, component selections, design iterations, and engineering trade-offs made while constructing the **Yo_Error_404** autonomous vehicle for the WRO Future Engineers 2026 competition.

---

## 1. 10 June — Power Distribution & Isolated 3-Domain Architecture

**Decision.** Use an isolated 3-domain power architecture with dual buck converters and a central power distribution board instead of a single shared rail.

**Why.** Initial testing with a single shared power rail revealed severe system instability: heavy current spikes drawn by the steering servo under load induced voltage drops on the logic rail, dragging it below 4.75V and triggering reboots on the Raspberry Pi 5. Furthermore, motor PWM noise induced signal corruption on the shared I2C bus, causing IMU heading drift.

**Implementation**:
- **12V High-Current Rail**: Direct battery pack power to the 7semi MD10112 MOSFET driver and JGB37-520 DC motor.
- **7.4V High-Torque Rail**: Stepped down via Buck Converter 1 to power the 20kg DS3218 Digital Servo, eliminating logic brownouts.
- **3.3V Low-Noise Logic Rail**: Stepped down via Buck Converter 2 to power the ESP32-C3 Super Mini, TCA9548A Multiplexer, VL53L0X ToF sensors, and BNO055 IMU.

---

## 2. 11 June — Three Li-ion Cells (3S 11.1V Pack)

**Decision.** Power the vehicle using three 18650 Li-ion cells in series (11.1V nominal) in a rigid chassis-mounted holder instead of a LiPo battery pack.

**Why.** While LiPo batteries offer slightly higher peak discharge rates, 18650 Li-ion cells provide superior durability, safer handling, and fixed mechanical mounting. Individual cells can be serviced easily, and the 11.1V nominal supply delivers ample continuous torque for the JGB37-520 motor while allowing buck converters to step down to logic levels efficiently without overheating.

---

## 3. 28 June — 4× VL53L0X Time-of-Flight (ToF) Sensors over HC-SR04 Ultrasonics

**Decision.** Replace wide-angle HC-SR04 ultrasonic sensors with four narrow-beam VL53L0X Time-of-Flight (ToF) laser sensors mounted at the Front, Left, Right, and Rear.

**Why.** HC-SR04 ultrasonic sensors emit sound waves that spread in a wide acoustic cone and bounce unpredictably off angled walls or corner openings, producing false distance readings and wall-following instability. VL53L0X ToF sensors emit invisible laser photons, measuring exact flight time. This delivers millimeter precision, a narrow field of view (FOV), and sub-millisecond refresh rates essential for high-speed wall centering.

---

## 4. 2 July — TCA9548A I2C Multiplexer (`0x70`) for Address Resolution

**Decision.** Integrate a TCA9548A 8-Channel I2C Multiplexer to route the four VL53L0X ToF sensors.

**Why.** All four VL53L0X sensors ship with a fixed default I2C address (`0x29`). Manually toggling XSHUT shutdown pins would require four extra GPIO lines on the compact ESP32-C3 Super Mini. The TCA9548A multiplexer allows all four sensors to communicate over a single I2C bus (`GPIO 8` SDA / `GPIO 9` SCL) at address `0x70` with sub-millisecond channel switching.

---

## 5. 10 July — Servo-Actuated Parallel Steering & Rear Differential over Ackermann Linkage

**Decision.** Implement a high-torque parallel steering linkage paired with high-grip tires and rear differential action instead of a complex Ackermann steering linkage.

**Why.** Ackermann steering linkages were tested but abandoned due to excessive mechanical complexity, joint play in 3D-printed pivots, and a large structural footprint. Parallel steering powered by a 20kg DS3218 Digital Servo provided crisp, high-torque steering response with significantly lower mechanical mass, making it far easier to maneuver around tight wall edges and narrow track gaps.

---

## 6. 18 July — JGB37-520 DC Motor (320 RPM) with Quadrature Encoder

**Decision.** Choose the JGB37-520 DC Gear Motor with an integrated magnetic quadrature encoder (320 RPM, 1.8 kg·cm torque) driven by a 7semi MD10112 MOSFET driver.

**Why.** Evaluated alternatives:
- *EV3 Large Servo Motor (160 RPM)*: Rejected due to bulky form factor and plastic gear wear under load.
- *Yellow TT Motor (200 RPM)*: Rejected due to low torque (0.8 kg·cm), plastic gear stripping, and lack of encoders.

The JGB37-520 delivers high continuous torque, robust metal gearing, and physical encoder count tracking for exact distance control (e.g. stopping dead at 2500 mm).

---

## 7. 25 July — 3D-Printed PETG Filament for Drive Shafts (100% Infill)

**Decision.** Print all transmission drive shafts and output couplers in PETG filament at 100% infill rather than standard PLA.

**Why.** Standard PLA drive shafts split along layer lines after 8–12 high-torque motor start cycles due to PLA's torsional rigidity. PETG provides superior layer bonding and torsional ductility, withstanding >100 continuous high-torque start cycles and motor stall tests without damage.

---

## 8. 5 August — ESP32-C3 Super Mini + Raspberry Pi 5 Dual-Controller Architecture

**Decision.** Split compute tasks between a Raspberry Pi 5 (4GB) master visual brain and an ESP32-C3 Super Mini real-time hardware co-processor communicating via Serial UART (115200 baud).

**Why.** Running computer vision (OpenCV at 60 FPS) and real-time control (100 Hz sensor polling & PWM generation) on a single processor caused OS latency spikes that dropped the control loop from 100 Hz to 45 Hz. Splitting tasks guarantees deterministic 100 Hz motor and steering control on the ESP32-C3 while allowing the Pi 5 to run high-level visual tracking without lag.
