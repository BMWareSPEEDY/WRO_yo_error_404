# ESP32-C3 slave: `open_slave/`

The Pi is the master. This sketch only reads the sensors and applies the Pi's commands. It has no driving logic.

- Sends `DATA,F,L,R,B,yaw,sys,gyro,accel,mag,spd,str` at 50 Hz (ToF in cm, 999 = no reading). `spd`/`str` are what the motor and servo are actually getting.
- Takes `<speed>,<steering>` from the Pi. **Speed is capped at ±170 PWM** and **steering at 30..140** (2° inside the 28..142 mechanical stops). The cap holds whatever the Pi sends.
- Sends `START` when the button is pressed and ignores commands before that (rule 9.14).
- Stops the motor and centres the steering if the Pi is silent for 500 ms.
- LED: red = init/error, yellow blinking = ToF/IMU missing, blue = waiting for START, green = running.
- Pins, mux channels (ToF ch0 front, ch1 right, ch2 left, ch3 rear, BNO055 ch4) and the centring sweep are the same as the open round.
- The motor is driven exactly like the open round: `analogWrite` on GPIO0, DIR (GPIO1) **HIGH = forward** (`MOTOR_FORWARD_LEVEL`).

Upload from the Mac (repo root):

```bash
arduino-cli compile --upload -p /dev/cu.usbmodem* --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc obstacle_round/esp/open_slave
```

Or flash it on the Pi, where the ESP32 is plugged in (the services must be stopped because they hold the port):

```bash
sudo systemctl stop master.service dashboard.service
~/esp_flash/venv/bin/esptool --chip esp32c3 -p /dev/serial/by-id/usb-Espressif* write-flash 0x0 ~/esp_flash/open_slave.ino.merged.bin
sudo systemctl start dashboard.service master.service
```
