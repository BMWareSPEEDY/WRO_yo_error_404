# Electrical Schematics & Hardware Architecture (`/schemes`)

This directory contains the electrical circuit schematics detailing the power distribution, microcontrollers, sensor interfaces, and motor driver connections for the **ERROR 404** autonomous vehicle. All schematics were designed using **KiCad 10.0.4**.

---

## ⚡ Schematics Inventory

Each round has both a rendered PNG for viewing in the browser and the original PDF for printing or zooming in on a detail.

- **[`open_round/schematic.png`](./open_round/schematic.png)** ([PDF](./open_round/schematic.pdf)):
  The complete electrical wiring diagram for the Open Round challenge configuration, detailing inter-component wiring, power rails, and signal lines. Everything on this sheet hangs off the ESP32, because in this round the ESP32 is the only controller.

- **[`obstacle_round/schematic.png`](./obstacle_round/schematic.png)** ([PDF](./obstacle_round/schematic.pdf)):
  The wiring diagram for the Obstacle Round challenge configuration. It is the Open Round circuit plus the Raspberry Pi, which joins the ESP32 over serial and takes its 5V from the same distribution board. The Camera Module 3 connects through the Pi's dedicated CSI ribbon connector and therefore does not appear as GPIO wiring on the diagram.

### Power rails on the sheet

Both sheets show two independent battery packs, each with its own switch:

| Reference | What it is | Feeds |
|:---|:---|:---|
| **BT2 / SW3** | 12V 3-cell Li-ion pack and its SPST switch | The J3 distribution board (12V → 5V), which supplies the ESP32, the three HC-SR04s and, in the obstacle round, the Raspberry Pi. The raw 12V also goes to the L298N. |
| **BT1 / SW2** | 7.4V 2-cell pack and its SPST switch | The steering servo only, on its own rail so that a servo current spike cannot dip the logic supply. |
| **SW1** | Momentary push button | Not a power switch — it is the start button, read by the ESP32. |

- **[`esp32_pin_mapping.jpg`](./esp32_pin_mapping.jpg)**:
  A visual pin map of the ESP32 DevKit V1, colour-coded by subsystem, showing which GPIO each sensor, the servo, the motor driver and the start button connect to. The matching pin table — taken directly from the firmware and authoritative for wiring — is in the [main README](../README.md#esp32-pin-mapping).

- **[`system_block_diagram.jpg`](./system_block_diagram.jpg)**:
  The architecture at a glance: the power, sensing, control, computer vision and motion blocks, and how data and power flow between them.

---

## 🔌 Hardware Bill of Materials (BOM)

Reference designators match the labels on the schematic sheets.

| Component Reference | Part / Module Name | Primary Role & Description |
| :--- | :--- | :--- |
| **J1** | Raspberry Pi | High-level computer vision processing, state decision logic, and master controller. Obstacle Round only. *(Drawn with the Pi 4 symbol from the KiCad library; the vehicle uses a Raspberry Pi 5.)* |
| **U7** | ESP32-DEVKIT-V1 | Low-level real-time hardware controller executing PWM steering, motor control, and sensor polling |
| **U6** | L298N Motor Driver | Dual H-Bridge motor driver regulating drive motor speed and direction |
| **BNO055** | BNO055 IMU | 9-axis absolute orientation sensor delivering real-time yaw heading compensation, on the I²C bus |
| **HC-SR04 ×3** | Ultrasonic Sensors | Distance sensors positioned on the front, left, and right channels |
| **M1** | DC Gear Motor | Rear-wheel propulsion, driven from the L298N |
| **M2** | Steering Servo | Front-wheel Ackermann steering actuator, on its own 7.4V rail |
| **CAM1** | Pi Camera Module 3 | Front-facing camera on the Pi CSI port providing the vision feed for traffic sign detection (Obstacle Round). Not drawn on the sheet — it is a ribbon connection, not GPIO wiring. |
| **J3** | Power Distribution Board | Steps the 12V pack down to the regulated 5V rail and fans it out to the controllers and sensors |
| **BT2** | Li-ion Battery Pack | 3× 18650 cells in series, 11.1V nominal, powering the whole vehicle |
| **BT1** | Li-ion Battery Pack | 2× cells in series, 7.4V, powering the steering servo alone |
| **SW3** | SPST Switch | Master power switch on the 12V pack |
| **SW2** | SPST Switch | Power switch on the 7.4V servo pack |
| **SW1** | Momentary Push Button | Start button, read by the ESP32 — triggers the autonomous routine |

---

> 📖 *For detailed hardware configuration instructions and firmware integration, please refer to the [Open Round Source README](../src/open_round/README.md).*
