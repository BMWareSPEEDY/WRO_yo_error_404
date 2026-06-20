# 3D Printable Mechanical Models (`/models`)

This directory houses the custom 3D-printable STL files engineered to construct the physical chassis frame, sensor mounts, and drivetrain transmission components for the **ERROR 404** autonomous vehicle.

---

## 🛠️ Model Inventory & Functional Breakdown

- **[`Ultrasonic_Sensor_Mount.stl`](./Ultrasonic_Sensor_Mount.stl)**:
  A custom-designed mounting bracket engineered to securely house and position three HC-SR04 ultrasonic sensors across the vehicle's perimeter (front, left, and right channels) for reliable wall-following and distance measurement.

- **[`Drive_Shaft.stl`](./Drive_Shaft.stl)**:
  A custom transmission coupler that bridges the DC gear motor output axle with the primary carbon fiber drive shaft, ensuring smooth torque transfer to the rear wheels.

- **[`Drive_Motor_Holder.stl`](./Drive_Motor_Holder.stl)**:
  A rigid structural bracket designed to lock the main DC propulsion motor firmly onto the lower chassis plate, minimizing motor vibration and shaft misalignment during operation.

- **[`Chassis_top_view.stl`](./Chassis_top_view.stl)**:
  The upper structural chassis plate, providing designated mounting points and structural support for onboard electronics, sensor arrays, and battery housing.

- **[`Chassis_bottom_view.stl`](./Chassis_bottom_view.stl)**:
  The primary base chassis plate housing the drivetrain assembly, steering mechanism, and lower structural mounts for vehicle stability.

- **Pi Camera 3 Mount Assembly** — printed as three separate parts that bolt together at the front of the vehicle for the Obstacle Round vision system:

  - **[`pi_camera_3_mount.stl`](./pi_camera_3_mount.stl)**:
    The main front housing that holds the Raspberry Pi Camera Module 3 board and shields the lens, positioning the camera to face forward down the track.

  - **[`pi_camera_3_mount_2.stl`](./pi_camera_3_mount_2.stl)**:
    A slim 3 mm backing plate that sandwiches the camera board against the main housing, locking the module in place without putting stress on the PCB.

  - **[`pi_camera_3_mount_3.stl`](./pi_camera_3_mount_3.stl)**:
    The chassis mounting arm that fixes the complete camera assembly onto the upper chassis plate and sets the camera's forward viewing angle for detecting the traffic sign pillars.

---

> ℹ️ *All components were modeled custom to fit our vehicle geometry and printed using high-durability filament.*
