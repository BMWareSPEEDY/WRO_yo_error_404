# Handoff prompt

Copy everything below the line into a new AI session to bring it up to speed on this robot.

---

You're picking up work on our WRO 2026 Future Engineers robot (Team Yo_Error_404, "V2 bot").
Read all of this before touching anything.

──────
## How to work with me

- Keep replies SHORT: one short paragraph saying what changed, why, and what it does.
- **ESP Reflash Protocol:** if a change touches the ESP32 sketch, the reply must START with `(REFLASH NEEDED)` or `(no reflash)`. Better still, flash it yourself from the Pi over SSH and confirm you did.
- **Git Hygiene:** after EVERY code edit, commit AND push to GitHub (`origin main`) with a clear message, and update `CHANGELOG.md`. Keep the READMEs current.
- **No simulations:** build for the real physical robot and test on the real bot/Pi.
- **Hard Safety Limits:**
  - Motor speed capped at ±170 PWM, forward and reverse, enforced in both the Pi code and the ESP32 firmware.
  - Steering servo clamped to 30°..140° (mechanical stops are 28°..142°; 90° = straight, >90° = right, <90° = left).
- **Deployment:** deploy to the Pi yourself over SSH. Don't hand me terminal commands to paste.
- **Network Resilience:** the Pi is at `pi@error404.local` / `192.168.1.53`. It drops off Wi-Fi when the battery is swapped or power-cycled, and it has browned out repeatedly under motor load — wait and retry rather than assuming failure.
- **Step-by-Step Delivery:** small verified steps with my green light before each phase, not massive unchecked rewrites.
- **Verify, don't assume.** Measure on the car and show me the numbers. Several long debugging detours in this project came from trusting a value instead of testing it.

──────
## Hardware

- **Master:** Raspberry Pi 5, `pi@error404.local` (`192.168.1.53`), key auth, sudo password `pi`. Raspberry Pi OS 64-bit Bookworm.
- **Slave:** ESP32-C3 Super Mini over USB (`/dev/serial/by-id/usb-Espressif_*` → `/dev/ttyACM0`).
  - **Drive motor:** MD10112 — **IN1 = GPIO0 (PWM)**, **IN2 = GPIO1 (DIR, HIGH = forward)**.
  - **Steering:** DS3218 servo on GPIO6, 50 Hz, 500–2400 µs.
  - **Sensors:** TCA9548A mux (0x70) on SDA GPIO8 / SCL GPIO9 —
    **ch0 front ToF, ch1 LEFT ToF, ch2 RIGHT ToF** (left/right were physically swapped on the car, the firmware compensates), ch3 rear ToF, ch4 BNO055 IMU (NDOF, yaw 0–359.9°, clockwise positive).
  - **Wheel encoder:** GPIO4 channel A (interrupt), GPIO5 channel B (direction).
  - START button GPIO10 (pull-up); 16× WS2812B on GPIO7.
- **Camera:** Pi Camera Module 3 Wide, 1024×576 RGB888 @ 30 fps.
  **CURRENTLY NOT DETECTED** — `rpicam-hello --list-cameras` reports "No cameras available", no overlay loads, and nothing answers at I²C 0x1a. That is below the software layer, so it is a ribbon cable, the module, or the CSI port. **With no camera there is no pillar detection at all.**

──────
## Repository

`github.com/BMWareSPEEDY/wro_fe_26_codes` — everything is committed and pushed.

| Path | What it is |
|---|---|
| `open_round/working_open_round/` | **The working open round.** Fully standalone: the ESP32 drives the whole round with no Pi. Also streams telemetry so the dashboard can display it when the Pi happens to be on. |
| `open_round/Working_Open_Round_V2_FE/` | The original proven open-round sketch. Do not modify. |
| `obstacle_round/esp/open_slave/` | Pure slave firmware: streams sensors, applies the Pi's commands, 500 ms watchdog. **Has no driving logic — useless without the Pi.** |
| `obstacle_round/pi/` | Pi master: `main.py` (control), `dashboard.py` (web UI + serial relay), `vision.py` (YOLO), `config.py`, `params.json`. |
| `obstacle_round/esp/tick_m_calculate/` | Calibration: drives 3 s, reports encoder ticks. Instructions in the sketch header. |
| `obstacle_round/esp/dir_probe/` | Diagnostic: drives every PWM/DIR pin combination and reports signed encoder movement. |
| `obstacle_round/esp/motor_stop/` | Safety: holds the motor off. Flash to make the car safe fast. |
| `parking_exit/` | Stand-alone parking exit test. |
| `pi_setup/` | `deploy.sh` and the systemd units. |

──────
## Two ways the car can run — know which one you're in

**1. Standalone open round** (`working_open_round` flashed)
The ESP32 does everything. Press START, it drives 12 corners, locks lap direction at the first, then a 3 s finish push. **The Pi is not required.** It still streams `DATA,...` at 50 Hz, so if the Pi is on the dashboard shows live values; if the Pi is off the car is unaffected. Writes are skipped when the USB buffer is full, so nothing can stall the control loop.

**2. Obstacle round** (`open_slave` flashed + Pi services running)
`open_slave` has zero driving logic. `dashboard.py` is the ONLY process that opens the serial port, and `main.py` talks to the ESP32 *through it* over UDP on 127.0.0.1:5005. **So `dashboard.py` must be running or the car will not move**, regardless of whether anyone is looking at the web page. Without a camera this degrades to the open round — corners only, no pillar dodging.

──────
## Running it

```bash
ssh pi@192.168.1.53

# services (both enabled at boot)
sudo systemctl status  dashboard.service master.service
sudo systemctl restart dashboard.service master.service
journalctl -u master.service -f          # watch decisions live

# dashboard
#   http://192.168.1.53:8080/   or   http://error404.local:8080/
```

Each run writes a CSV to `~/obstacle_round/logs/run_<timestamp>.csv` with per-tick sensor values, stage, decision, speed and steering. **Read the CSV before theorising** — it has caught several bugs that the live log hid.

### Flashing the ESP32

`arduino-cli` is installed **on the Pi** at `~/bin/arduino-cli`, with the esp32 core and the needed libraries. Sketches are staged in `~/sketches/`. Compile ~45 s, flash ~15 s.

```bash
ssh pi@192.168.1.53
export PATH=$HOME/bin:$PATH
cd ~/sketches
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc --output-dir build_x <sketch>

sudo systemctl stop master.service dashboard.service     # they own the serial port
cp build_x/<sketch>.ino.merged.bin ~/esp_flash/
~/esp_flash/venv/bin/esptool --chip esp32c3 -p /dev/serial/by-id/usb-Espressif* \
    write-flash 0x0 ~/esp_flash/<sketch>.ino.merged.bin
sudo systemctl start dashboard.service master.service
```

**Stopping the services first is not optional** — `dashboard.py` holds `/dev/ttyACM0` and esptool fails mid-write otherwise. arduino-cli conveniently emits a single `.merged.bin` that drops straight into that command.

──────
## Tuning

`params.json` on the Pi, with live sliders on the dashboard (drag, then **Save to Disk** to persist):
`pillar_trigger_dist_cm`, `pillar_steer_left`/`right`, `turn_hold_pct`, `dodge_short_cm`, `dodge_long_cm`, `dodge_return_extra_cm`, `corner_steer_deg`, `front_turn_cm`, `recenter_target_cm`, `recenter_k`, `pillar_focal_px`.

Fixed constants live in `obstacle_round/pi/config.py`.

──────
## Hard-won gotchas — each of these cost hours

- **Never leave GPIO0 or GPIO1 as an input.** IN1 is the motor driver's PWM input; a pull-up on it reads as 100% duty and the car drives off at full speed with no code commanding it. Set them `OUTPUT` and `LOW` as the first statements in `setup()`.
- **Attach the servo before the first `analogWrite`.** On ESP32 core 3.x `analogWrite` claims an LEDC timer at ~1 kHz; if it runs first, `ESP32Servo` shares that timer instead of getting a clean 50 Hz one and the servo spins continuously.
- **Check PWM and DIR are on the pins you think.** They were swapped in firmware for weeks. The symptom was bizarre: full speed regardless of the commanded value, reverse producing exactly zero movement, and the car appearing not to respond to speed changes. `esp/dir_probe` diagnoses it in 30 seconds.
- **999 means two different things.** The firmware reports 999 both for "nothing in range" and "the sensor errored", and the driving logic reads 999 as an OPEN side — which is what triggers a corner. A flaky sensor invents corners. Measure the dropout rate before trusting a side reading.
- **Stop the services before flashing**, or esptool dies mid-write.
- **The Pi browns out under motor load.** It has rebooted mid-command many times. If SSH dies during a drive, that's why — wait and retry.
- **Re-measure `TICKS_PER_METER` after any wheel, gearing or encoder change** with `tick_m_calculate`. It was wrong by 46% at one point, so every distance the car drove was 46% longer than its label.
- The BNO055 needs a figure-eight to calibrate. Heading hold drifts until the `cal` values come off zero, and every corner targets an absolute heading.

──────
## State right now

- `working_open_round` is the current open-round firmware — standalone, with telemetry.
- The obstacle round's Pi code is at the last known-good state: turn at 45 cm, encoder-measured dodge legs, no reverse manoeuvres.
- **Reverse is not usable** on the current firmware pin assignment, so nothing that drives the motor backwards will work.
- **The camera is undetected**, so there is no pillar detection.

## What's left to build

1. Get the camera detected again — almost certainly the ribbon cable — and confirm pillar detection returns.
2. Lane centring on the straights. Currently just heading hold with a ±20° ToF nudge, so the car drifts into walls between corners. The proven logic is in `Working_Open_Round_V2_FE`, but its constants assume the real WRO corridor width and saturate the steering on a narrow test surface.
3. Multi-obstacle chaining across all four sections, once vision is back.
