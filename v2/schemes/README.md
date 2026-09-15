# Electrical Schematics & Hardware Architecture (`/schemes`)

This directory contains the electrical circuit schematics, microcontroller pin diagrams, sensor interfaces, and power distribution diagrams for the **Yo_Error_404** autonomous vehicle.

---

## Hardware Schematics & Pin Mapping Files

- **[`esp32_pin_mapping.jpg`](./esp32_pin_mapping.jpg)**:
  Visual pin diagram for the **ESP32-C3 Super Mini** microcontroller, displaying GPIO assignments for PWM motor speed/direction control, steering servo, NeoPixel status strip, start button, and TCA9548A I2C multiplexer lines (`SDA`/`SCL`).

- **[`pi5_pin_mapping.jpeg`](./pi5_pin_mapping.jpeg)**:
  Visual pin diagram for the **Raspberry Pi 5 (4GB)** primary compute unit, highlighting GPIO header pins, power inputs, USB ports, and CSI camera ribbon interface.

- **[`system_subsystem_interface.jpeg`](./system_subsystem_interface.jpeg)**:
  Full system hardware integration and communication topology diagram illustrating power routing, I2C multiplexer channels, UART serial telemetry, and motor/servo actuation paths.

- **[`open_round/schematic.png`](./open_round/schematic.png)** ([PDF](./open_round/schematic.pdf)):
  Complete electrical wiring schematic for the Open Round challenge configuration.

- **[`obstacle_round/schematic.png`](./obstacle_round/schematic.png)** ([PDF](./obstacle_round/schematic.pdf)):
  Complete electrical wiring schematic for the Obstacle Round challenge configuration, combining the Raspberry Pi 5 vision brain with the ESP32-C3 Super Mini hardware co-processor over Serial UART.

---

## System Hardware Bill of Materials (BOM)

| Component Reference | Part / Module Name | Primary Role & Description |
|:-------------------|:-------------------|:---------------------------|
| **Compute Brain** | Raspberry Pi 5 (4GB) | High-level computer vision processing, HSV color segmentation, state decision engine |
| **Co-Processor** | ESP32-C3 Super Mini | Low-level real-time hardware controller executing 100 Hz sensor polling, PID loops, and PWM output |
| **Motor Driver** | 7semi MD10112 | High-current MOSFET H-bridge driver regulating drive motor speed and direction |
| **I2C Multiplexer** | TCA9548A | 8-Channel I2C multiplexer (`0x70`) resolving address conflicts across 4 ToF sensors |
| **Distance Sensors** | VL53L0X ToF Sensors (×4) | High-precision laser Time-of-Flight sensors (Front, Left, Right, Rear) |
| **Orientation Sensor** | Adafruit BNO055 IMU | 9-DOF absolute orientation sensor delivering real-time yaw heading fusion |
| **Propulsion Motor** | JGB37-520 DC Motor | 320 RPM, 1.8 kg·cm rear-wheel drive motor with integrated quadrature encoder |
| **Steering Servo** | 20kg DS3218 Digital Servo | High-torque front parallel steering linkage actuator |
| **Camera** | Pi Camera Module 3 (Wide) | Front-facing camera on CSI ribbon for traffic pillar recognition |
| **Buck Converters** | Dual Buck Converters | Isolated voltage step-down (12V → 7.4V Servo, 12V → 3.3V Logic) |
| **Battery Pack** | 3S Li-ion Battery Pack | Main 12V power supply |
