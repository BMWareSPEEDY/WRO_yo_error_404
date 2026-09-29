# Open Challenge (V2): working, do not modify

`Working_Open_Round_V2_FE/Working_Open_Round_V2_FE.ino` runs the whole open round on the ESP32-C3. It needs no Pi. The file is unchanged from the first commit. It was only moved into a folder of the same name, which the Arduino IDE requires before it will open a sketch.

How it works:

1. After boot the wheels do a centering sweep. The LED turns green and the car waits for START (GPIO10).
2. It reads the front, right and left VL53L0X (mux ch0/1/2) and the BNO055 (mux ch4) on every loop.
3. A corner is detected when front ≤ 60 cm and one side is > 50 cm. The first corner latches CW or CCW for the whole run.
4. Each turn uses full lock (45 or 135) until the heading is within 8° of target ±90°. If front < 12 cm during a turn, it reverses with opposite lock.
5. On the straights it holds wall-centering P plus heading hold (Kp_wall 1.2, heading 1.2, 42 cm single-wall standoff).
6. After 12 turns it drives on for 3 s, centered, then stops.

Libraries: ESP32Servo, Adafruit BNO055, Adafruit Unified Sensor, Adafruit_VL53L0X, Adafruit NeoPixel.
Board: ESP32C3 Dev Module, **USB CDC On Boot: Enabled**.
