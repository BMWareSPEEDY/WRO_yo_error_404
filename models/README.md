# 3D Printable Mechanical Models (`/models`)

This directory houses the custom 3D-printable STL files engineered to construct the physical chassis frame, sensor mounts, and drivetrain transmission components for the **ERROR 404** autonomous vehicle.

---

## 🛠️ Model Inventory & Functional Breakdown

- **[`Ultrasonic_Sensor_Mount.stl`](./Ultrasonic_Sensor_Mount.stl)**:
  A custom-designed mounting bracket engineered to securely house and position three HC-SR04 ultrasonic sensors across the vehicle's perimeter (front, left, and right channels) for reliable wall-following and distance measurement.

- **[`drive_shaft.stl`](./drive_shaft.stl)**:
  A custom transmission coupler (Ø10 × 27 mm) that bridges the BO gear motor output axle with the carbon fiber drive shaft, ensuring smooth torque transfer to the rear wheels. One end is a double-D socket matching the flats on the motor shaft; the other is a round bore for the carbon fiber rod, with a cross hole for a locking pin. This part took roughly 20 prints across 5 designs to get the tolerances right — the story is in the [main README](../README.md#3d-printed-parts).

  <img src="./renders/drive_shaft.png" width="520" alt="Drive shaft coupler, both ends">

- **[`drive_motor_holder.stl`](./drive_motor_holder.stl)**:
  A rigid structural bracket designed to lock the BO DC propulsion motors firmly onto the lower chassis plate, minimizing motor vibration and shaft misalignment during operation.

  <img src="./renders/drive_motor_holder.png" width="200" alt="Drive motor holder">

- **Chassis plates** — the two structural levels of the vehicle, held apart by spacers.

  - **[`Chassis_bottom_view.stl`](./Chassis_bottom_view.stl)** (269.5 × 120 × 5 mm):
    The primary base plate housing the drivetrain assembly, steering mechanism, and lower structural mounts for vehicle stability.

  - **[`Chassis_top_view.stl`](./Chassis_top_view.stl)** (199.5 × 120 × 5 mm):
    The upper plate, providing designated mounting points and structural support for onboard electronics, sensor arrays, and battery housing.

  <img src="./renders/chassis_bottom.png" width="380" alt="Lower chassis plate">
  <img src="./renders/chassis_top.png" width="320" alt="Upper chassis plate">

- **Pi Camera 3 Mount Assembly** — printed as three separate parts that bolt together at the front of the vehicle for the Obstacle Round vision system:

  - **[`pi_camera_3_mount.stl`](./pi_camera_3_mount.stl)**:
    The main front housing that holds the Raspberry Pi Camera Module 3 board and shields the lens, positioning the camera to face forward down the track.

  - **[`pi_camera_3_mount_2.stl`](./pi_camera_3_mount_2.stl)**:
    A slim 3 mm backing plate that sandwiches the camera board against the main housing, locking the module in place without putting stress on the PCB.

  - **[`pi_camera_3_mount_3.stl`](./pi_camera_3_mount_3.stl)**:
    The chassis mounting arm that fixes the complete camera assembly onto the upper chassis plate and sets the camera's forward viewing angle for detecting the traffic sign pillars.

---

## 🖼️ Renders

[`renders/`](./renders/) holds a PNG preview of each of the main printed parts, generated directly from the STL files in this folder so they can be viewed without opening a slicer.

---

> ℹ️ *All components were modeled custom to fit our vehicle geometry and printed using high-durability filament.*
