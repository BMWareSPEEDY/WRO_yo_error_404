# Electrical Schematics & Hardware Architecture (`/schemes`)

This directory contains the electrical circuit schematics detailing the power distribution, microcontrollers, sensor interfaces, and motor driver connections for the **ERROR 404** autonomous vehicle. All schematics were designed using **KiCad 9.0.1**.

---

## ⚡ Schematics Inventory

- **[`open_round/schematic.png`](./open_round/schematic.png)**:
  The complete electrical wiring diagram for the Open Round challenge configuration, detailing inter-component wiring, power rails, and signal lines.

- **[`obstacle_round/schematic.png`](./obstacle_round/schematic.png)**:
  The electrical wiring diagram for the Obstacle Round challenge configuration. The circuit is electrically identical to the Open Round schematic — the only hardware addition is the Raspberry Pi Camera Module 3, which connects through the Pi 5's dedicated CSI ribbon connector and therefore does not appear as GPIO wiring on the diagram.

---

## 🔌 Hardware Bill of Materials (BOM)

| Component Reference | Part / Module Name | Primary Role & Description |
| :--- | :--- | :--- |
| **J1** | Raspberry Pi 5 | High-level computer vision processing, state decision logic, and main controller |
| **U6** | ESP32-DEVKIT-V1 | Low-level real-time hardware controller executing PWM steering, motor control, and sensor polling |
| **U1** | BNO055 IMU | 9-axis absolute orientation sensor delivering real-time yaw heading compensation |
| **U2** | L298N Motor Driver | Dual H-Bridge motor driver regulating drive motor speed and direction |
| **U4 / U5 / U7** | HC-SR04 Sensors | Ultrasonic distance sensors positioned on the front, left, and right channels |
| **M1** | DC Gear Motor | Main rear-wheel propulsion motor |
| **M2** | Steering Servo | Front-wheel Ackermann steering actuator |
| **CAM1** | Pi Camera Module 3 | Front-facing camera on the Pi 5 CSI port providing the vision feed for traffic sign detection (Obstacle Round) |
| **SW1** | Start Button | Physical momentary switch triggering the main autonomous routine |
| **SW2** | Push Button | Mode selection and emergency reset switch |
| **BT1** | Li-ion Battery Pack | 3x 18650 cell configuration supplying 11.1V nominal system power |

---

> 📖 *For detailed hardware configuration instructions and firmware integration, please refer to the [Open Round Source README](../src/open_round/README.md).*
