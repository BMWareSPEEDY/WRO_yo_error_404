# Obstacle Round Source Code

## testing_calibration_codes/

The two tools we used to get the obstacle round hardware working — one to check that the Pi and the ESP32 can actually talk to each other, and one to tune the camera's colour detection.

### rx_tx_test.ino

An echo test for the ESP32. You send it a line of text from the Pi over USB and it sends the same line back with `ECHO:` in front of it. If that works, the cable, the port and the baud rate are all fine, which is worth knowing before you start sending real speed and steering commands.

**What you need**

- ESP32 dev board (ESP32-DEVKIT-V1)
- Raspberry Pi 5
- A USB data cable, Pi USB-A to ESP32 micro-USB. A charge-only cable will not work.

**Running it**

Open `rx_tx_test.ino` in the Arduino IDE and flash it. There are no libraries to install for this one. Board is "ESP32 Dev Module", pick your port, hit upload.

Now plug the ESP32 into the Pi and open the serial port at 115200 baud. You'll need pyserial (`pip install pyserial`):

```bash
python3 -c "import serial; s=serial.Serial('/dev/ttyUSB0',115200,timeout=2); s.write(b'hello\n'); print(s.readline().decode())"
```

On boot the ESP32 prints `ESP32 ready over USB!`. After that, every line you send should come straight back as `ECHO: <your message>`.

If nothing comes back, check the port name with `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`, make sure you're on 115200, and try a different cable.

### hsv_tuner.py

Tunes the HSV colour thresholds for the camera. It shows you three windows side by side — the camera feed, the black and white mask, and the mask applied to the feed — plus six sliders for the upper and lower HSV bounds. Drag the sliders until the thing you're trying to detect shows up solid white in the mask.

There are presets for the three colours we care about: the red and green traffic sign pillars, and the magenta parking lot limiters.

**What you need**

- Raspberry Pi 5 with a Camera Module 3. A webcam or a recorded video file works too.
- A monitor plugged into the Pi, or a VNC session. It opens OpenCV preview windows, so it won't run headless.

**Running it**

```bash
pip install opencv-python numpy

python3 hsv_tuner.py                                       # live camera, green preset
python3 hsv_tuner.py --source block_test.mp4 --color red   # recorded video, red preset
```

**Presets**

| Key | Colour | Lower (H, S, V) | Upper (H, S, V) | What it's for |
|:---:|:-------|:----------------|:----------------|:--------------|
| `1` | Red | `(0, 70, 60)` | `(10, 255, 255)` | Red traffic sign pillar |
| `2` | Green | `(45, 60, 40)` | `(90, 255, 255)` | Green traffic sign pillar |
| `3` | Magenta | `(135, 70, 60)` | `(175, 255, 255)` | Parking lot limiter |

**Keys**

| Key | What it does |
|:---:|:-------------|
| `1` `2` `3` | Load the red / green / magenta preset |
| `p` | Print the current ranges to the terminal |
| `s` | Save the current ranges to `hsv_values.txt` |
| `q` or `ESC` | Quit |

**Tuning**

Put the pillar (or the parking limiter) in front of the camera under the same lighting the car will actually run in — this matters more than anything else, and it's why we re-tune at the venue rather than trusting values from home. Load the closest preset with 1, 2 or 3, then drag the sliders until the mask shows the object as solid white without picking up the background.

Once it looks right, press `p` to print the ranges or `s` to write them to `hsv_values.txt`, then copy them into the detection code.
