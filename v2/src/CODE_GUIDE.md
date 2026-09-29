# How our code works: a guide for editing it yourselves

This guide is for making changes to the V2 code **without help**. It walks
through the important lines of each program: what they do, why they are
written that way, and what the syntax means. It covers three things:

1. **[Parking](#part-1-parking-on-its-own)**: the parking exit system configured in `flasher.py` (`open_slave.ino` on the ESP32 + `parking_master.py` on the Pi)
2. **[Obstacle round](#part-2-the-obstacle-round-on-its-own)**: the Pi + ESP32 system that drives laps and dodges pillars
3. **[Both combined](#part-3-parking--obstacle-round-combined)**: how `main.py` integrates the parking exit and obstacle round together into one unified run
4. **[Every parameter](#part-4-every-parameter-explained)**: what each slider and constant does, and which ones to change to get the behaviour you want

Read [Part 0](#part-0-things-you-need-before-anything-else) first. It covers
the syntax used everywhere else, and the sections after it assume you know it.

Line numbers such as `main.py:457` are correct as of commit `5a1de4d`. If they
have moved a little, search for the function name instead.

For **running** the car on competition day (services, flashing, SSH), use
[`RUNBOOK.md`](RUNBOOK.md). This guide explains the code, not the operating steps.

---

## Contents

- [Part 0: things you need before anything else](#part-0-things-you-need-before-anything-else)
  - [0.1 The hardware in one picture](#01-the-hardware-in-one-picture)
  - [0.2 Units and sign conventions](#02-units-and-sign-conventions-memorise-these)
  - [0.3 Arduino / C++ syntax you will meet](#03-arduino--c-syntax-you-will-meet)
  - [0.4 Python syntax you will meet](#04-python-syntax-you-will-meet)
  - [0.5 The one idea everything is built on: blocking vs tick-based](#05-the-one-idea-everything-is-built-on-blocking-vs-tick-based)
- [Part 1: parking on its own](#part-1-parking-on-its-own)
  - [1.1 Architecture in flasher.py](#11-architecture-in-flasherpy)
  - [1.2 ESP32 slave firmware: open_slave.ino](#12-esp32-slave-firmware-open_slaveino)
  - [1.3 Pi controller: parking_master.py](#13-pi-controller-parking_masterpy)
  - [1.4 Tunable parameters: parking/config.py](#14-tunable-parameters-parkingconfigpy)
  - [1.5 How to run and test parking](#15-how-to-run-and-test-parking)
  - [1.6 Historical standalone sketches](#16-historical-standalone-sketches)
- [Part 2: the obstacle round on its own](#part-2-the-obstacle-round-on-its-own)
- [Part 3: parking + obstacle round combined](#part-3-parking--obstacle-round-combined)
- [**Part 4: every parameter, explained (and which to change for what)**](#part-4-every-parameter-explained)
- [Part 5: how to change things safely (recipes)](#part-5-how-to-change-things-safely-recipes)
- [Part 6: when it misbehaves, where to look](#part-6-when-it-misbehaves-where-to-look)

---

# Part 0: things you need before anything else

## 0.1 The hardware in one picture

```
                 USB cable, 115200 baud, one text line per message
 ┌──────────────────────┐  ──── "90,87\n"  (speed,steering) ───▶ ┌─────────────────────────┐
 │  Raspberry Pi 5      │                                          │  ESP32-C3 Super Mini    │
 │  - camera + YOLO     │ ◀─── "DATA,52,40,61,999,93.4,...,1234\n" │  - 4 ToF via mux        │
 │  - all decisions     │                                          │  - BNO055 IMU (mux ch4) │
 │  - dashboard :8080   │ ◀─── "START\n" / "STOP\n"                │  - motor, servo, encoder│
 └──────────────────────┘                                          │  - START button GPIO10  │
                                                                   └─────────────────────────┘
```

| Thing | Pin / channel | Notes |
|---|---|---|
| Motor PWM (speed) | GPIO **1** | `analogWrite(1, 0..255)` |
| Motor DIR | GPIO **0** | `LOW` = forward, `HIGH` = reverse |
| Steering servo | GPIO 6 | degrees; physical stops 28..142, we use **30..140** |
| NeoPixel LEDs | GPIO 7 | 16 LEDs |
| I²C | SDA 8, SCL 9 | everything goes through the TCA9548A mux at `0x70` |
| ToF front / left / right / rear | mux ch **0 / 1 / 2 / 3** | left and right were swapped on the car, so ch1 = LEFT |
| BNO055 IMU | mux ch **4** | select ch4 before reading it |
| Wheel encoder | A = GPIO4 (interrupt), B = GPIO5 | **2078 ticks = 1 metre** |
| START button | GPIO10 to GND | reads `LOW` when pressed (internal pull-up) |

## 0.2 Units and sign conventions (memorise these)

These cause more bugs than anything else in this code.

| Quantity | Convention |
|---|---|
| **Servo angle** | Degrees. **90 is not straight on this car. 87 is.** The Pi writes that as `SERVO_CENTER (90) + servo_trim_deg (-3)`. **Bigger number = wheels RIGHT.** |
| **Yaw / heading** | BNO055, 0..359.9°, **clockwise = increasing**. Turning right makes yaw go up. |
| **Heading error** | `heading_error(target, current)` gives -180..+180. **Positive = you need to turn RIGHT.** |
| **Speed** | PWM, -110..+110 (`MAX_PWM = 110`). Negative = reverse. Below ~80 the motor may not move at all. |
| **Distance** | cm everywhere. From the encoder: `cm = abs(ticks_now - ticks_at_start) / 20.78`. |
| **ToF** | cm. **999 means "nothing in range"**, i.e. an open side or a gap. |
| **Lap direction** (`self.lock`) | `+1` = clockwise (CW) = every corner is a **right** turn. `-1` = anticlockwise (CCW) = **left** turns. `0` = not decided yet. |
| **Pillar colour** | **Red is passed on the RIGHT** (the car goes right). **Green is passed on the LEFT.** |
| **Reversing** | Reversing **flips the steering effect**. The same servo angle rotates the car the other way when the wheels go backwards, so every "hold heading while reversing" formula has a **minus** where the forward one has a plus. |

## 0.3 Arduino / C++ syntax you will meet

```cpp
#define SPEED 90          // A text substitution done before compiling. Everywhere the
                          // word SPEED appears, the compiler sees 90. No '=' and no ';'.

volatile long ticks = 0;  // 'volatile' = "this can change behind your back" (an interrupt
                          // changes it), so the compiler must re-read it every time.

void IRAM_ATTR encISR() { ... }   // An Interrupt Service Routine. The hardware calls it by
                                  // itself on every encoder pulse. IRAM_ATTR keeps it in fast RAM.
                                  // Keep ISRs tiny: no Serial, no delay.

attachInterrupt(digitalPinToInterrupt(4), encISR, RISING);
                          // "Call encISR every time pin 4 goes from LOW to HIGH."

noInterrupts(); t = ticks; interrupts();
                          // Pause interrupts while copying a multi-byte value, so the ISR
                          // cannot change it half-way through the copy.

digitalRead(ENC_B) ? ticks++ : ticks--;
                          // Ternary: condition ? do_if_true : do_if_false

constrain(x, 30, 140)     // Clamp x into [30, 140].
fabs(x)                   // Absolute value of a float. abs() is for ints.
millis()                  // Milliseconds since power-on. Used for non-blocking timing.
delay(20)                 // Freeze EVERYTHING for 20 ms. Fine in simple sketches, banned in open_slave.

Serial.printf("L=%.0f R=%.0f\n", L, R);
                          // Formatted print. %d int, %ld long, %.1f float with 1 decimal,
                          // %s string, %u unsigned. \n = newline.

snprintf(buf, sizeof(buf), "DATA,%u,...", a, ...);
                          // Same as printf, but writes into a char array instead of the port.

long v = strtol(p, &end, 10);
                          // Parse a base-10 integer from text. 'end' is left pointing just
                          // after the number, which is how the code finds the ',' next.

bool f(...);  float g(...);  void h(...);
                          // Function return types: true/false, a decimal number, nothing.

const char *label         // A text string passed into a function, e.g. "1 TURN OUT".
Adafruit_VL53L0X tof[3];  // An array of 3 sensor objects: tof[0], tof[1], tof[2].
1 << ch                   // Bit shift: 1<<0=1, 1<<1=2, 1<<4=16. The mux takes a
                          // bitmask where bit N means "open channel N".
```

Every Arduino sketch has two required functions:

- `setup()` runs **once** at power-on or reset.
- `loop()` runs **forever**, over and over, after `setup()` finishes.

## 0.4 Python syntax you will meet

```python
class ObstacleRound:              # A class groups state (variables) and behaviour (functions).
    def __init__(self, start_yaw, now):   # The constructor. Runs once when you do
        self.target = start_yaw           #   ctl = ObstacleRound(yaw, now)
                                          # 'self.x' is a variable that lives as long as the object.

    def _park_step(self, d, now): ...     # A method. A leading '_' just means "internal helper".
                                          # 'self' is always the first parameter.

    @property
    def direction(self): ...              # Called WITHOUT brackets: ctl.direction, not ctl.direction()

d["yaw"]                    # Dictionary lookup. Crashes if the key is missing.
d.get("ticks")              # Same, but returns None if missing.
p.get("park_turn_deg", 90)  # ...or returns the default 90 if missing.

x = a if cond else b        # Python's ternary: the same as C's  cond ? a : b

max(lo, min(hi, v))         # Clamp v into [lo, hi]. You will see this constantly.

f"turned {turned:.1f} deg"  # f-string: {expression:format}. :.1f = 1 decimal, :+.0f = sign + 0 dp.

try:
    v = float(p.get(key, 30.0))
except (TypeError, ValueError):   # If the slider value is junk (None, "abc"), do not crash
    v = 30.0                      # the car mid-run. Use the fallback.

self.park_step, self.park_ticks0 = 2, self.ticks
                            # Assign two variables at once.

[b for b in blocks if b["class"] in ("Red", "Green")]
                            # List comprehension: "every b in blocks that is Red or Green".

sorted(blocks, key=lambda b: b["dist_cm"])
                            # Sort by distance. 'lambda b: ...' is a tiny unnamed function.

min(blocks, key=lambda x: x["dist_cm"])      # The nearest block.

(a + 90.0) % 360.0          # Modulo: wraps 400 -> 40. Keeps headings inside 0..359.

log.info(...)  log.warning(...)  log.error(...)
                            # Written to the journal (journalctl -u master.service).
                            # This is how you see what the car decided.
```

## 0.5 The one idea everything is built on: blocking vs tick-based

There are two ways to write "turn until 90°, then drive 30 cm":

**Blocking** (early experimental sketches like `parking_exit_v2.ino`): the function sits in a `while`
loop until it is finished. It is easy to read, but nothing else can run in the
meantime.

```cpp
while (turned < 90) { steer hard; drive; delay(20); }   // stuck here until done
while (cm < 30)     { hold heading; drive; delay(20); }
```

**Tick-based** (both `parking_master.py` and `main.py`): a function is called
about **50 times a second** by the control loop. Each call looks at the telemetry once,
decides once, returns one `(speed, steer)` pair, and exits immediately. It remembers
where it was in state variables such as `self.stage` in `parking_master.py` or
`self.park_step` and `self.pillar_stage` in `main.py`. This is called a **state machine**.

```python
def step(self, d, now):            # called every ~20 ms
    if self.stage == "TURN_OUT":   # "which stage am I in?"
        if self._stage_done(target_cm, now):
            self._begin_stage("STRAIGHT", now)
        else:
            return speed, steer    # one command, then return
```

By using the tick-based state machine, `parking_master.py` shares the exact same
modular architecture as the obstacle round. This is what makes it straightforward to
integrate parking directly into `main.py` (Part 3) so one START press does both
without any firmware changes.

---

# Part 1: parking on its own

In our codebase, parking is configured as an independent operational mode in
[`obstacle_round/pi/flasher.py`](obstacle_round/pi/flasher.py):

```python
    "parking": {
        "label": "Parking test",
        "sketch": "open_slave",
        "build": "build_slave",
        "service": "parking.service",
        "blurb": "test only - the parking exit on its own",
    },
```

Unlike early monolithic sketches, the parking exit system uses our standard
two-tier architecture:
1. **ESP32 Firmware:** [`obstacle_round/esp/open_slave/open_slave.ino`](obstacle_round/esp/open_slave/open_slave.ino) (pure hardware slave: streams sensor telemetry, controls motor and servo, enforces safety watchdog).
2. **Raspberry Pi Controller:** [`parking/parking_master.py`](parking/parking_master.py) with [`parking/config.py`](parking/config.py), managed by `parking.service`.

> **Note on code integration:** While `parking_master.py` runs parking as a standalone
> test, the repository also contains the unified code that **integrates both parking
> and the obstacle round together**: [`obstacle_round/pi/main.py`](obstacle_round/pi/main.py)
> (covered in detail in [Part 3](#part-3-parking--obstacle-round-combined)).

### What it does

When the START button is pressed on the car or web dashboard:
```
1. LOOK: Compare Left vs Right ToF
   - Wall on LEFT  (Left <= Right) → Exit to the RIGHT (exit_dir = +1)
   - Wall on RIGHT (Right < Left)  → Exit to the LEFT  (exit_dir = -1)

2. STAGE 1: TURN OUT (25 cm via encoders)
   - Steer lock away from wall (SERVO_CENTER + exit_dir * 50°)
   - Hold lock for first 40% of distance, then progressively unwind towards center

3. STAGE 2: STRAIGHT ACROSS LANE (55 cm via encoders)
   - Hold perpendicular heading (~80° off start) via IMU P-controller (HOLD_KP = 1.5)

4. STAGE 3: TURN BACK / RECENTER (25 cm via encoders)
   - Opposite lock (SERVO_CENTER - exit_dir * 50°) to swing parallel to lane

5. STAGE 4: FORWARD TO SQUARE UP (20 cm via encoders)
   - Drive forward holding original 0° heading to square up with the track

6. HALT:
   - Motor cut (0 PWM), steering centered (87°). Complete standstill.
```

---

## 1.1 Architecture in flasher.py

The mode switching logic in `flasher.py` automates the entire parking workflow:
1. **Stops any conflicting service:** Halts `master.service` so two processes never fight over the vehicle.
2. **Compiles the firmware:** Invokes `arduino-cli` on `open_slave.ino` targeting the ESP32-C3 (`build_slave` directory).
3. **Flashes over serial:** Pauses the serial interface, flashes the merged binary to the ESP32 via `esptool`, and resumes the port.
4. **Activates the service:** Sets `parking.service` as the active systemd service, starting `parking_master.py` immediately and enabling it on boot.

Because both parking and obstacle rounds run on `open_slave.ino`, **you do not need a custom Arduino sketch on the ESP32 to run parking**. The microcontroller always runs the standard slave firmware, while the Pi determines the driving behavior.

---

## 1.2 ESP32 slave firmware: `open_slave.ino`

`open_slave.ino` runs on the ESP32-C3 Super Mini and has **zero autonomous driving logic**. It serves as a reliable, high-speed hardware interface:

- **Telemetry streaming (50 Hz):**
  Reads the TCA9548A I2C mux (channels 0..3: ToF front, left, right, rear; channel 4: BNO055 IMU) and wheel optical encoder, streaming text frames over USB serial (115200 baud):
  ```
  DATA,front,left,right,rear,yaw,sys,gyro,accel,mag,spd,str,ticks\n
  ```
- **Command execution:**
  Parses incoming drive lines from the Pi:
  ```
  <speed>,<steering>\n    (e.g., "75,137\n" or "0,87\n")
  ```
  Applies soft motor acceleration ramp and sets servo steering angle within calibrated limits (`[30, 140]`).
- **500 ms Safety Watchdog:**
  If the Pi stops sending commands for 500 ms (`WATCHDOG_MS`), the ESP32 immediately cuts motor throttle to 0 and displays a red status LED.

---

## 1.3 Pi controller: `parking_master.py`

`parking_master.py` is the state-machine controller that drives the car during a parking test.

### 1. Connection and telemetry
It connects to the ESP32 either through the dashboard UDP relay (`HubLink` on port 5005) or via direct serial fallback (`/dev/serial/by-id/usb-Espressif*`):
```python
link, is_hub = connect_link()
```

### 2. Idle state and START / STOP control
In the idle state (`ctl is None`), it prints live telemetry once every second:
```python
print(f"IDLE | Hdg={d['yaw']:5.1f}° | F={d['front']:3.0f} cm | "
      f"L={d['left']:3.0f} cm | R={d['right']:3.0f} cm | Ticks={d.get('ticks', 0)}")
```
When START is triggered:
- An instance of `ParkingController` is instantiated with the initial sensor readings.
- If the START/STOP button is pressed mid-run, `parking_master.py` immediately commands `(0, SERVO_CENTER)` to halt the car.

### 3. Exit direction decision
The car measures the side walls with ToF sensors:
```python
self.exit_dir = 1 if left_cm <= right_cm else -1
self.wall_side = "LEFT" if self.exit_dir == 1 else "RIGHT"
self.exit_side = "RIGHT" if self.exit_dir == 1 else "LEFT"
```
- Closer to the **LEFT** wall $\rightarrow$ exit **RIGHT** (`exit_dir = +1`).
- Closer to the **RIGHT** wall $\rightarrow$ exit **LEFT** (`exit_dir = -1`).

### 4. The 4 stages of `ParkingController.step(d, now)`

The state machine runs on every telemetry tick (~50 Hz):

```python
# Stage distance calculated from encoder ticks:
self.stage_cm = abs(self.ticks - self.stage_ticks0) / config.TICKS_PER_CM
```

#### Stage 1: `TURN_OUT` (25 cm)
```python
sharp_steer = config.SERVO_CENTER + self.exit_dir * config.STEER_LOCK_DEG
progress = min(1.0, max(0.0, self.stage_cm / max(1.0, config.TURN_OUT_CM)))
if progress <= 0.40:
    steer = sharp_steer
else:
    unwind = (progress - 0.40) / 0.60
    steer = sharp_steer + (config.SERVO_CENTER - sharp_steer) * (unwind ** 1.5)
```
- Wheels turn to full lock away from the wall (`87 + 50 = 137°` right, or `87 - 50 = 37°` left).
- Full lock is held for the first 40% of the 25 cm travel. Over the remaining 60%, steering unwinds smoothly toward center.
- Transitions to `STRAIGHT` when `self.stage_cm >= config.TURN_OUT_CM` (25 cm).

#### Stage 2: `STRAIGHT` across lane (55 cm)
```python
leg_target_hdg = wrap180(self.start_yaw + self.exit_dir * 80.0)
herr = heading_error(leg_target_hdg, yaw)
steer = config.SERVO_CENTER + clamp(config.HOLD_KP * herr, -25.0, 25.0)
```
- Drives forward across the lane at speed 75 PWM.
- BNO055 heading-hold P-controller maintains heading offset ~80° relative to start wall (`HOLD_KP = 1.5`).
- Transitions to `RECENTER` when `self.stage_cm >= config.STRAIGHT_CM` (55 cm).

#### Stage 3: `RECENTER` (25 cm)
```python
sharp_return = config.SERVO_CENTER - self.exit_dir * config.STEER_LOCK_DEG
progress = min(1.0, max(0.0, self.stage_cm / max(1.0, config.TURN_IN_CM)))
if progress <= 0.40:
    steer = sharp_return
else:
    unwind = (progress - 0.40) / 0.60
    steer = sharp_return + (config.SERVO_CENTER - sharp_return) * (unwind ** 1.5)
```
- Wheels counter-steer sharply in the opposite direction (`SERVO_CENTER - exit_dir * 50°`) to swing the chassis parallel with the lane.
- Holds lock for 40% of travel, unwinding over the last 60%.
- Transitions to `FORWARD_20CM` when `self.stage_cm >= config.TURN_IN_CM` (25 cm).

#### Stage 4: `FORWARD_20CM` & Align (20 cm)
```python
herr = heading_error(self.start_yaw, yaw)
steer = config.SERVO_CENTER + clamp(config.HOLD_KP * herr, -25.0, 25.0)
```
- Squares up with the track by holding the initial heading (`self.start_yaw`).
- Runs forward strictly 20.0 cm measured by the wheel encoder.
- Upon completion, returns `(0, config.SERVO_CENTER, True)`. Motor is cut (`0 PWM`), wheels centered (`87°`), and car halts.

---

## 1.4 Tunable parameters: `parking/config.py`

Constants for the standalone parking exit are configured in [`parking/config.py`](parking/config.py):

| Parameter | Value | Meaning |
|---|---|---|
| `EXIT_SPEED` | 75 | Motor PWM during parking exit |
| `SERVO_CENTER` | 87 | Calibrated straight wheel angle (-3° trim) |
| `SERVO_MIN` / `SERVO_MAX` | 30 / 140 | Software steering travel limits |
| `STEER_LOCK_DEG` | 50 | Angle offset for turning arcs (137° right / 37° left) |
| `TURN_OUT_CM` | 25.0 | Distance for Stage 1 turn-out arc |
| `STRAIGHT_CM` | 55.0 | Distance for Stage 2 cross-lane drive |
| `TURN_IN_CM` | 25.0 | Distance for Stage 3 turn-back arc |
| `FORWARD_CM` | 20.0 | Distance for Stage 4 squaring-up drive |
| `HOLD_KP` | 1.5 | Gain for heading-hold P-controller |
| `TICKS_PER_CM` | 20.78 | Wheel encoder calibration factor (2078 ticks/m) |
| `STAGE_TIMEOUT_S` | 14.0 | Failsafe timeout per stage in case encoder stalls |

---

## 1.5 How to run and test parking

### A. From the web dashboard
1. Connect to dashboard at `http://error404.local:8080`.
2. Under the Round switcher, select **Parking test**.
3. `flasher.py` compiles and flashes `open_slave.ino` (if needed) and activates `parking.service`.
4. Press START on the car or web dashboard.

### B. From the command line
Switch modes using `flasher.py`:
```bash
python3 /home/pi/obstacle_round/flasher.py parking
```

Or run interactively in the terminal:
```bash
# Stop competing services so they don't fight over the serial port
sudo systemctl stop master.service parking.service

# Run parking master script
python3 /home/pi/parking/parking_master.py
```

### C. Viewing live logs
To monitor state transitions and sensor data:
```bash
journalctl -u parking.service -f
```

---

## 1.6 Historical standalone sketches

For reference, the repository also contains earlier standalone parking experiments:
- `parking_exit/esp/parking_exit_v2/parking_exit_v2.ino`: An earlier standalone C++ sketch running
  entirely on the ESP32 using blocking `while` loops.
- `parking/esp/parking_exit_standalone/`: A 5-stage distance-based ESP32 standalone sketch.

While useful as early hardware prototypes, they required repeatedly reflashing the ESP32 between
standalone sketches and `open_slave`. The current architecture in `flasher.py` unifies everything
around `open_slave.ino` on the ESP32 and `parking_master.py` on the Pi.

---

# Part 2: the obstacle round on its own

## 2.1 Who does what

```
 ESP32: open_slave.ino
   reads sensors, streams DATA at 50 Hz, applies speed/steer, watchdog
          │ USB serial
          ▼
 Pi: dashboard.py  (dashboard.service, ALWAYS running from boot)
   - the ONLY program that opens the serial port and the camera
   - runs YOLO on each camera frame (vision.py)
   - serves the web page on :8080 with the live sliders (params.json)
   - relays everything to main.py over UDP on localhost:5005
          │ UDP  (HubLink in serial_link.py)
          ▼
 Pi: main.py  (master.service)
   - ObstacleRound.step(data, blocks, params, now) → (speed, steer)
   - ~50 times a second
```

| File | Job |
|---|---|
| `obstacle_round/esp/open_slave/open_slave.ino` | ESP32 firmware. **No driving logic at all.** |
| `obstacle_round/pi/main.py` | **All the driving decisions.** This is the file you edit most. |
| `obstacle_round/pi/config.py` | Fixed constants, plus `DEFAULT_PARAMS` (the defaults for every slider). |
| `obstacle_round/pi/params.json` | The **live tuned values**, written by the dashboard sliders. |
| `obstacle_round/pi/serial_link.py` | Message formats and the UDP link (`HubLink`). |
| `obstacle_round/pi/vision.py` | YOLO pillar detection. Turns a frame into a list of blocks with distances. |
| `obstacle_round/pi/pixel_distance_method.py` | Sizes the dodge from how far off-centre the pillar is. |
| `obstacle_round/pi/dashboard.py` | Web page, camera, relay. |
| `obstacle_round/pi/tools/*.py` | Tests that need no hardware. |

---

## 2.2 The ESP32 slave: `open_slave.ino`

### The message format

```
Pi → ESP32:   "<speed>,<steering>\n"         e.g. "90,87\n"   "-90,120\n"   "0,90\n"
              "STOP\n" / "START\n"

ESP32 → Pi:   "DATA,F,L,R,B,yaw,sys,gyro,accel,mag,spd,str,ticks\n"   every 20 ms
               field 0 1 2 3  4   5    6    7     8   9   10  11   12
              "START\n"   (button pressed; repeated every 1 s until the Pi answers)
              "STOP\n"    (button pressed during a run)
              "INFO,..." / "ERR,..."
```

### The important parts

**The encoder ISR** is the same idea as the parking sketch:
```cpp
void IRAM_ATTR encISR() { if (digitalRead(ENC_B_PIN)) encTicks++; else encTicks--; }
```

**The motor** has a soft start that ramps 30 PWM every 8 ms, so the wheels do not spin:
```cpp
if (appliedSpeed < target) appliedSpeed = min(appliedSpeed + RAMP_STEP, target);
```

**Parsing a command** (`handleLine`):
```cpp
long speed = strtol(line, &end, 10);         // "90,87" → 90, end points at ','
if (end == line || *end != ',') return;      // not a number, or no comma → ignore
long steer = strtol(end + 1, &end, 10);      // → 87
lastCmdMs = now;                             // feeds the watchdog
if (!started) return;                        // rule: no driving before START
targetSpeed = constrain(speed, -MAX_PWM, MAX_PWM);
steerCmd   = constrain(steer, SERVO_MIN, SERVO_MAX);
```
The ESP32 **clamps again**, independently of the Pi. This is why raising
`MAX_PWM` on the Pi alone does nothing: **it must be raised in both places.**

**The watchdog**, in `loop()`:
```cpp
watchdogTripped = started && (now - lastCmdMs > WATCHDOG_MS);   // 500 ms
updateMotor(now, (started && !watchdogTripped) ? targetSpeed : 0);
```
If the Pi crashes or goes quiet for half a second, the car stops by itself.

**`loop()` never uses `delay()`.** Every job runs on its own `millis()` timer:
```cpp
if (now - lastSensorMs >= SENSOR_PERIOD_MS) { lastSensorMs = now; ...read sensors... }
if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) { lastTelemetryMs = now; sendTelemetry(); }
```
This is the standard Arduino pattern for "do X every N ms without blocking".

**The LED** shows the state: blue = waiting for START, green = running, red = watchdog tripped, orange = a sensor is missing.

> ⚠️ Any change to this file means you must **reflash the ESP32**.

---

## 2.3 The link: `serial_link.py`

```python
def parse_data(line):
    p = line.split(",")                       # "DATA,52,40,..." → ["DATA","52","40",...]
    if len(p) not in (12, 13): return None
    return {"front": int(p[1]), "left": int(p[2]), "right": int(p[3]), "rear": int(p[4]),
            "yaw": float(p[5]), ..., "ticks": int(p[12]) if len(p) == 13 else None, ...}
```
This turns one DATA line into the dictionary `d` that the whole of `main.py`
reads: `d["front"]`, `d["yaw"]`, `d["ticks"]`, and so on.

```python
def command_line(speed, steering):
    speed = int(max(-config.MAX_PWM, min(config.MAX_PWM, round(speed))))
    steering = int(max(config.SERVO_MIN, min(config.SERVO_MAX, round(steering))))
    return f"{speed},{steering}\n"
```
Every command passes through here, **so the Pi can never send an unsafe value.**

**`HubLink`** is what `main.py` uses. The methods you will see:

| Call | Returns / does |
|---|---|
| `link.wait_new(seq, timeout)` | Waits for the next DATA line. Returns `(d, seq)`. `d["age"]` = seconds since it arrived. |
| `link.latest_vision()` | List of pillar blocks from the newest camera frame (`[]` if older than 0.5 s). |
| `link.vision_seq` | Counter that goes up once per **camera frame**. Used to count frames, not ticks. |
| `link.latest_params()` | Dict of all slider values. |
| `link.start_received` | `True` after START is pressed, `False` after STOP. |
| `link.send(speed, steer)` | Sends a drive command. |
| `link.publish({...})` | Sends status to the dashboard page. |

---

## 2.4 Vision: `vision.py`

Every detected pillar ("block") is a dict:
```python
{'class': 'Red', 'confidence': 0.93, 'x1':..,'y1':..,'x2':..,'y2':.., 'cx': 335, 'cy':..,
 'w':.., 'h':.., 'area':.., 'dist_cm': 48.2, 'clipped': False}
```

The distance comes from the pinhole camera formula:
```python
return focal_px * config.PILLAR_HEIGHT_CM / max(1, h_px)     # 443.8 * 10 / box height
```
A pillar is 10 cm tall, so a box that is twice as tall means the pillar is
half as far away. When the box is **cut off at the top or bottom of the frame**
(`clipped`), the height is wrong, so the 5 cm **width** is used instead.

Regions of interest ("ROI" bands) are in `config.py`:
```python
def block_roi(block, frame_w=None):  # → "LEFT" / "CENTER" / "RIGHT"
```
The frame is split into three vertical bands, and a block belongs to whichever
band holds **most of its box**. The dashboard draws the bands with the same
function, so what you see matches what the car decides.

---

## 2.5 Settings: `config.py`, `params.json` and the sliders

There are **three layers**. This is the most common source of "I changed it
and nothing happened".

| Layer | Where | Changed how | Needs restart? |
|---|---|---|---|
| **Hard constants** | `config.py`, `UPPER_CASE = value` | Edit the file, deploy | Yes (restart `master.service`) |
| **Slider defaults** | `config.py`, `DEFAULT_PARAMS = {...}` | Edit the file, deploy | Yes |
| **Live values** | `params.json` on the Pi | Dashboard sliders at `http://error404.local:8080` | **No, applies instantly** |

How they combine (`config.load_params`):
```python
def load_params():
    params = dict(DEFAULT_PARAMS)              # 1. start from the defaults
    if os.path.exists(PARAMS_FILE):
        data = json.load(f)
        for k, v in data.items():
            if k in params:                     # 2. only keys that EXIST in DEFAULT_PARAMS
                params[k] = type(params[k])(v)  #    converted to the default's type (int/float/bool)
    return mirror_green(params)                 # 3. green settings overwritten from red
```

Three things follow from this:
- **A new key added only to `params.json` is ignored.** It must also be in `DEFAULT_PARAMS`.
- **Green settings are ignored.** `mirror_green` copies every red value onto its green counterpart. **To tune the dodge, tune the RED sliders.** Green follows automatically as an exact mirror.
- `deploy.sh` **never overwrites** the Pi's `params.json`. It copies the Pi's copy back to your laptop first, so the tuning done at the track survives a deploy.

How `main.py` reads a slider (the pattern is everywhere):
```python
try:
    v = float((self._params or {}).get("pre_turn_reverse_cm", 30.0))
except (TypeError, ValueError):
    return 30.0
return max(0.0, min(150.0, v))     # clamp, so a typo cannot send the car 3 m
```
`self._params or {}` means "the params dict, or an empty dict if it is None".

### The sliders you will actually touch (the full list is in [Part 4](#part-4-every-parameter-explained))

| Key | Current | Meaning |
|---|---|---|
| `base_speed` / `turn_speed` / `reverse_speed` | 90 | PWM (capped at 110) |
| `front_turn_cm` | 37 | Front distance at which a corner is taken |
| `corner_steer_deg` | 28 | Maximum degrees off straight during a corner turn |
| `pre_turn_reverse_cm` | 35 | Reverse-arc before each corner, for room |
| `reverse_arc_deg` / `reverse_arc_leave_deg` | 40 / 35 | Wheel angle in the reverse arc / degrees left for the forward part |
| `post_turn_reverse_cm` | 40 | Straight reverse after each corner |
| `cruise_wall_k` / `cruise_target_cm` | 1.6 / 42 | Wall-centring strength / distance to hold off the outer wall |
| `pillar_trigger_dist_cm` | 30 | Camera distance at which the dodge starts |
| `pillar_steer_right` | 135 | Red turn-out angle (Green is mirrored from it) |
| `dodge_short_cm` / `dodge_long_cm` | 29 / 35 | Turn-out leg lengths (normal / wide). **Unused while `use_pixel_dodge` is on.** |
| `dodge_past_cm` | 50 | Straight run past the pillar |
| `dodge_red_extra_cm`, `dodge_past_red_extra_cm`, ... | | Per-colour trims (red only, see above) |
| `use_pixel_dodge` | true | Size the turn-out from the pillar's pixel offset instead |
| `recenter_k` / `recenter_target_cm` | 2.0 / 45 | Wall centring during ALIGN after a dodge |
| `servo_trim_deg` | -3 | 90 + this = true straight |
| `park_*` | | Parking exit, see Part 3 |

---

## 2.6 `main.py` part A: the main loop (`main()`, line ~2005)

This is the outer program. Simplified, it looks like this:

```python
link = HubLink()                                   # connect to dashboard.py
ctl = None                                         # no run in progress

while not stop["flag"]:
    d, seq = link.wait_new(seq, timeout=0.1)       # 1. newest sensor data
    now = time.monotonic()                         #    a clock that never jumps backwards
    fresh = d is not None and d["age"] <= config.TELEMETRY_MAX_AGE_S

    if ctl is not None and not link.start_received:    # 2. STOP pressed during a run
        ctl = None; link.send(0, config.SERVO_CENTER)

    if ctl is None and link.start_received and fresh:  # 3. START pressed → new run
        link.clear_vision()
        ctl = ObstacleRound(d["yaw"], now)              #    the current heading = "straight"
        ctl.lock_direction_from_parking(d)              #    CW or CCW, decided ONCE
        ctl._params = link.latest_params() or {}
        ctl.begin_parking(d, now)                       #    arm the parking exit (Part 3)
        ...open a CSV log file...

    if ctl is None:            speed, steer = 0, config.SERVO_CENTER
    elif not fresh:            speed, steer = 0, ...; link.send(...)   # 4. stale data → stop
    else:
        blocks = link.latest_vision()
        params = link.latest_params()
        speed, steer = ctl.step(d, blocks, params, now)   # 5. THE DECISION
        link.send(speed, steer)                           # 6. to the ESP32
        writer.writerow([...])                            # 7. one CSV row per tick
```

- **`ObstacleRound` is created fresh on every START**, so every run starts clean.
- Every run writes `obstacle_round/logs/run_YYYYMMDD_HHMMSS.csv`, with one row per tick: sensors, target heading, stage, speed and steer. **This is the first thing to read after a bad run.**
- The part that checks for missing `ticks` near the top of the loop only matters if the ESP32 is running the stand-alone open-round sketch. In that case `main.py` just watches and sends nothing.

---

## 2.7 `main.py` part B: `step()`, the brain (line ~1182)

`step()` is called every tick and **returns as soon as one part claims the
car**. The **order of the `if` blocks is the priority order**:

```
step(d, blocks, params, now):
  0. housekeeping: de-glitch ToF, heading error, note close pillars, read encoder
  1. state == DONE                 → stop
  2. park_step != 0                → parking exit owns the car             (Part 3)
  3. state == FINISHING            → drive 3 s past turn 12, then DONE
  4. is_turning                    → corner turn in progress
  5. pre_turn_reverse              → reverse-arc before a corner
  6. post_turn_reverse             → straight reverse after a corner
  7. should turn? (not mid-dodge)  → start a corner
  8. no pillar tracked?            → look for one in `blocks`
  9. pillar_stage == BACKOFF       → reverse away from a too-close pillar
 10. pillar_stage == APPROACH      → cruise, wait for the trigger distance
 11. pillar_stage == TURN_OUT      → dodge leg 1
 12.                 STRAIGHTEN    → dodge leg 2
 13.                 RECENTER      → dodge leg 3 (turn back in)
 14.                 ALIGN         → dodge leg 4 (square up)
 15. otherwise                     → normal cruising with wall centring
```

The rules that fall out of this order:
- A **dodge, once started, is never interrupted** (`dodge_running` blocks corners in step 7).
- A **corner outranks a dodge that has not started yet** (APPROACH is dropped if a corner comes).
- Each block ends with `self.last_t = now; return speed, steer`. The `return` is what gives it ownership of the car for this tick.

### Housekeeping at the top of `step()`

```python
self._params = params or {}
d = self._deglitch_tof(d, now)                   # see 2.8
self.herr = heading_error(self.target, d["yaw"]) # + = must turn right
...
t = d.get("ticks")
if t is None: self.ticks_ok = False
else:
    self.ticks_ok = True
    self.ticks = t
    self.stage_cm = abs(self.ticks - self.stage_ticks0) / config.TICKS_PER_CM
```
`self.stage_cm` is **"cm driven since the current stage began"**. Every
distance-based stage ends when it reaches its target. `_begin_stage()` resets
it to 0:

```python
def _begin_stage(self, stage, now):
    self.pillar_stage = stage
    self.stage_t0 = now
    self.stage_ticks0 = self.ticks       # ← new reference point
    self.stage_cm = 0.0

def _stage_done(self, target_cm, now):
    if self.ticks_ok and self.stage_cm >= target_cm: return True
    return (now - self.stage_t0) >= config.STAGE_TIMEOUT_S   # 6 s backstop for a dead encoder
```

---

## 2.8 `main.py` part C: driving straight

### `heading_error()` (line 38)
```python
def heading_error(target, current):
    d = target - current
    while d > 180:  d -= 360
    while d < -180: d += 360
    return d
```
This is the shortest signed angle from `current` to `target`. With target 10
and current 350, it gives +20 (turn right 20°), not -340.

### `_straight()` (line 255): the cruise controller
```python
self.nudge = open_round_wall_centering(d["left"], d["right"], self.lock, kp=k_center, target=target_dist)
steer = self._straight_servo() + config.KP_HEADING_CRUISE * self.herr + self.nudge
return self.base_speed(), clamp_servo(steer)
```
This is the whole straight-line controller:
**servo = true straight (87) + 1.2 × heading error + wall correction**.

`open_round_wall_centering()` (line 82) gives the wall correction in **servo
degrees**:
- **Both walls are seen**: `(right - left) * kp * 0.5`. If the car is nearer the left wall, right > left, the result is positive, and the car steers right.
- **One wall is seen**: hold `target` (42 cm) off it. It prefers the **outer** wall, which depends on the lap direction (`lock`).
- **No wall**: 0, so the car holds heading only.

Set `cruise_wall_k` to 0 for pure-gyro straights.

### `_deglitch_tof()` (line 302)
The ESP32 sends **999 both for "no wall" and for "sensor glitched"**. A single
fake 999 on a side can look like an open corner, and on corner 1 that can set
the **wrong lap direction for the whole race**. So:
- **Low readings** (a real wall) pass straight through.
- A **999** only passes if most readings in the last 0.25 s were also 999. Otherwise it is replaced with the median of the recent real readings.

---

## 2.9 `main.py` part D: corners

### Lap direction: `lock_direction_from_parking()` (line 390)
```python
left, right, ambiguous = self._lock_sides(d)
if ambiguous: self.lock = self._default_lock()           # both open → params "default_lap_dir"
else:         self.lock = 1 if left < right else -1       # nearer LEFT wall → CW → right turns
```
This is decided **once, standing still, at START**, from where the car is
parked. It is the same wall reading that sets the parking exit direction, so
the two can never disagree. The long `_should_turn` code for "lock == 0" only
runs if `LOCK_DIRECTION_AT_START = False` in `config.py`.

### When to turn: `_should_turn()` (line 565)
In order, these checks can say **no**:
1. **A pillar is dead ahead** (seen CLOSE, in the CENTER band, within `block_no_corner_s` seconds): the front ToF is looking at the pillar, not a wall, so this is not a corner.
2. **The car has not driven `MIN_TRAVEL_BETWEEN_TURNS_CM` (20 cm) since the last corner**: this stops it re-triggering on the wall it just turned away from. The exception is when front ≤ 25 cm (critical).
3. **`front <= front_turn_cm` → TURN.** With the direction already locked, that is all it needs.

### The corner sequence
```
_should_turn → True
   │
   ├─ pre_turn_reverse_cm > 0:  corner_target = target + 90*lock
   │     REVERSE ARC: reverse with the wheels turned the OPPOSITE way
   │     (_reverse_arc_steer). The nose swings towards the new heading while
   │     backing off the wall. Stops when only reverse_arc_leave_deg of the
   │     turn is left (or after the set distance or time).
   │
   ├─ _begin_turn():  self.target = corner_target
   │
   ├─ _turn_step() every tick:
   │     steer = _corner_steer(): off = min(cap, max(7, 0.55*|herr|))
   │       → hard while far from the target, easing off as the car comes round
   │     if front < 12 cm: WEDGED → reverse with opposite lock until front > 20
   │     done when |herr| < TURN_DONE_ERR_DEG (13°) or after 6.5 s
   │     turns += 1; if turns >= 12 → FINISHING
   │
   └─ post_turn_reverse_cm > 0:  straight reverse (heading held), then cruise
```

The heading target is **absolute**: `self.target = (self.target + 90.0 * self.lock) % 360.0`.
Each corner adds exactly 90° to the previous target, not to wherever the car
happens to point, so small errors do not add up over 12 corners.

```python
def _reverse_hold_steer(self, herr):
    return clamp_servo(self._straight_servo() - config.KP_REVERSE_HEADING * herr)
#                                            ^ MINUS: reversing flips the steering
```

---

## 2.10 `main.py` part E: pillars

### Picking a pillar (step 8, line ~1386)
Only when **no pillar is being handled** and the car has driven
`pillar_clear_cm` (60 cm) **and** waited `pillar_clear_delay_s` (1.2 s) since
the last dodge. Without that, the camera re-adopts the pillar it has just
passed.

```python
valid = sorted((b for b in blocks if b["class"] in ("Red", "Green")), key=lambda b: b["dist_cm"])
```
**The nearest pillar comes first.** Blocks closer than the floor distance are
dropped, because they are "almost certainly the pillar just passed". Then:
```python
self._begin_stage("APPROACH", now)
```

### APPROACH (step 10)
- Every frame it re-checks: **is a different-coloured pillar now 20 cm nearer?** If so, it switches to that one.
- It updates `pillar_dist` from the nearest block of the tracked colour. A **clipped** box that suddenly reads farther away is ignored, because the pillar cannot be moving away from the car.
- If the pillar is not seen for 1.5 s, it is dropped and the car goes back to cruising.
- **Trigger**: `pillar_dist <= pillar_trigger_dist_cm` for **3 separate camera frames** (`DODGE_TRIGGER_FRAMES`). It counts using `vision_seq`, so three control ticks on the same frame do not count as three.
- **ToF shortcut**: if the pillar is CENTER-band and the front ToF reads ≤ 40 cm **and** agrees with the camera within 25 cm, it commits immediately.
- Until the trigger fires, it just drives with `_straight()`.

When it fires, three decisions are **latched once** (the pillar will leave the
frame as the car turns):
```python
self.pillar_roi = config.block_roi(self.pillar_box ...)          # LEFT / CENTER / RIGHT
self.dodge_wide = (Green and roi == "LEFT") or (Red and roi == "RIGHT")
self.pixel_out_cm = pxd.sideways_cm(...)  if params["use_pixel_dodge"]
self._begin_stage("TURN_OUT", now)
```

### BACKOFF (step 9)
If the pillar is **already inside** the trigger distance when it is picked up
(or stays inside for 0.6 s without committing), the car reverses until the
pillar reads ≥ 35 cm (`BACKOFF_TARGET_CM`), then approaches again. This allows
up to 2 tries. The point is that **every dodge starts from the same distance.**

### The dodge: 4 legs, all measured on the encoder

| Stage | Steering | Ends when |
|---|---|---|
| **TURN_OUT** | Sharp angle for `turn_hold_pct` (30%) of the leg, then unwinds to straight | `stage_cm >= _out_cm()` |
| **STRAIGHTEN** | `_straight(ignore_tof=True)`, gyro only (the inner ToF is looking at the pillar) | `stage_cm >= _past_cm()` |
| **RECENTER** | The mirror angle back in, same hold-then-unwind shape | `stage_cm >= _in_cm()` |
| **ALIGN** | `87 + 2.4*herr` + outer-wall centring | `|herr| < 5°` and a minimum distance, or `ALIGN_MAX_CM` |

The hold-then-unwind curve (the same shape as in parking):
```python
progress = self._stage_progress(out_cm)                # 0.0 … 1.0 through the leg
if progress <= hold_pct:
    steer = sharp_steer                                # hold full angle
else:
    unwind_p = (progress - hold_pct) / max(0.01, 1.0 - hold_pct)   # 0…1 over the rest
    steer = sharp_steer + (config.SERVO_CENTER - sharp_steer) * (unwind_p ** 1.5)
#                                                         ** is "to the power of"
```

### Red is the master, Green is the mirror
Every dodge angle is **written once, for Red**, and passed through `_as_red()`:
```python
def _as_red(self, red_angle):
    tc = self._straight_servo()                                 # 87, the TRUE straight
    limit = min(tc - config.SERVO_MIN, config.SERVO_MAX - tc)   # same max lock both ways
    delta = max(-limit, min(limit, red_angle - tc))             # signed offset from straight
    return tc + (-delta if self.pillar_color == "Green" else delta)
```
Red 135 is 48° right of 87, so Green is 48° left of 87, which is 39. It is
**not** `180 - 135 = 45`, because mirroring about 90 made Green 6° softer and
it clipped pillars. **If you add a new dodge angle, write it for Red and wrap
it in `_as_red()`.**

### Leg lengths
```python
_out_cm()  = _leg_cm()                     # dodge_short_cm or dodge_long_cm (wide) or pixel method,
                                           #   + dodge_red_extra_cm, clamped 5..100
_past_cm() = dodge_past_cm + dodge_past_red_extra_cm
_in_cm()   = _leg_cm() + dodge_return_extra_cm + dodge_return_red_extra_cm
```

> **Check your copy:** in commit `5a1de4d`, `_past_cm()` reads
> `cm = config.DODGE_PAST_CM`, which means **the `dodge_past_cm` slider does
> nothing**. The fix (not yet committed when this was written) is
> `cm = float(p.get("dodge_past_cm", config.DODGE_PAST_CM))`.

### Pixel method (`pixel_distance_method.py`)
```python
sideways_cm = BASE_CM + toward_px * CM_PER_PIXEL       # 31.15 + px * 0.08, clamped 8..55
```
`toward_px` is how far the pillar sits **towards the side the car dodges to**.
A Green that is far LEFT needs a bigger swing left. A Green already on the
right needs less. When `use_pixel_dodge` is on, this replaces the wide/normal
split for the turn-out leg only.

---

# Part 3: parking + obstacle round combined

### What changed & how both codes are integrated

In `flasher.py`, two modes drive the car autonomously with the Pi:
- `"parking"` runs [`parking/parking_master.py`](parking/parking_master.py) with `open_slave.ino` on the ESP32 to test the parking exit on its own.
- `"obstacle"` runs [`obstacle_round/pi/main.py`](obstacle_round/pi/main.py) with `open_slave.ino` on the ESP32.

In this repository, we have the code that **integrates both of these codes together**:
[`obstacle_round/pi/main.py`](obstacle_round/pi/main.py). Instead of stopping after the exit,
the parking maneuver is incorporated directly as the **first phase of the obstacle round**,
sharing the exact same `open_slave.ino` ESP32 slave firmware. One START press does this:

```
START → lock lap direction from the walls → PARKING EXIT (4 steps) → hand over → 3 laps with pillars
```

**Nothing needs reflashing.** Both standalone parking and the integrated obstacle
round use the standard `open_slave.ino` firmware. The ESP32 streams sensor telemetry
and executes the speed/steering commands calculated by `main.py`.

### 3.1 Standalone Parking → Integrated `main.py` translation table

| Standalone (`parking_master.py` / concepts) | Integrated (`main.py`) | Note |
|---|---|---|
| `exit_dir` (+1 right, -1 left) | `self.lock` (+1 CW, -1 CCW) | **The same variable.** Wall on the left → exit right → lap is clockwise → corners are right turns. One reading decides both. |
| `start_yaw` | `self.park_hdg0 = d["yaw"]` | Starting reference heading |
| `start_ticks` / `self.stage_ticks0` | `self.park_ticks0 = self.ticks` | Encoder reference for stage travel |
| `wrap180(yaw - start_yaw)` | `_park_turned(d)` = `self.lock * heading_error(d["yaw"], self.park_hdg0)` | Positive in the exit direction |
| `abs(ticks - stage_ticks0) / TICKS_PER_CM` | `_park_gone()` = `abs(self.ticks - self.park_ticks0) / TICKS_PER_CM` | Distance traveled in the current step |
| `config.SERVO_CENTER` (87) | `self._straight_servo()` (90 + trim) | True straight steering angle |
| `config.STEER_LOCK_DEG` (50) | `park_steer_deg` (50 on the car per PARAMS.html; fallback 53) | Sharp steering angle offset |
| 90° target across lane | `park_turn_deg` 90 | Exit swing angle |
| `config.FORWARD_CM` / `STRAIGHT_CM` | `park_forward_cm` (**80** on the car per PARAMS.html; fallback 30) | Straight forward leg |
| Reverse leg | `park_back_cm` 20 | Reverse straight before straightening up |
| `config.EXIT_SPEED` (75) | `park_speed` (90 on dashboard) | Motor PWM speed during parking |
| `ParkingController.step()` stages | `self.park_step` going 1 → 2 → 3 → 4 → 0 | Tick-based state machine (~50 Hz) |
| STOP button handling | Handled by `main()` for the entire run | Cuts throttle and resets state |
| `STAGE_TIMEOUT_S` (14 s) | `park_max_cm` (400 cm): failsafe | If IMU stalls, ends parking after 4 m and races anyway |
| (always enabled) | `park_enabled`: 0 = skip parking | Dashboard toggle to test laps without parking |

`tools/park_exit_test.py` **verifies that these parameters and step sequences work correctly.**
If you change one side, update the test to match.

### 3.2 The new state variables (`__init__`, line 170)

```python
# Parking exit, run once before the round proper. park_step 0 = not in it.
self.park_step = 0          # 0 = off, 1 swing out, 2 forward, 3 back, 4 swing back
self.park_hdg0 = 0.0        # heading when parking began
self.park_ticks0 = 0        # encoder count when the CURRENT step began
```

### 3.3 Arming it: `begin_parking()` (line 425)

It is called once from `main()`, **after** `lock_direction_from_parking()`
(because it needs `self.lock`) and **after** `ctl._params` is set (because it
reads the park sliders):

```python
ctl = ObstacleRound(d["yaw"], now)
ctl.lock_direction_from_parking(d)          # sets self.lock from the walls
ctl._params = link.latest_params() or {}    # sliders available BEFORE the first step()
ctl.begin_parking(d, now)
```

```python
def begin_parking(self, d, now):
    if not self._park("park_enabled", 1.0):     # 0 → skip
        return
    self.park_step = 1
    self.park_hdg0 = d["yaw"]
    self.park_ticks0 = d.get("ticks") or self.ticks
```
`d.get("ticks") or self.ticks` means "the ticks from this DATA line, or if
that is None or 0, what we already had".

`_park()` is the safe slider reader:
```python
def _park(self, key, fallback):
    try:    return float((self._params or {}).get(key, fallback))
    except (TypeError, ValueError): return fallback
```

### 3.4 The four steps: `_park_step()` (line 457)

It runs once per tick. It returns `(speed, steer)` while parking, or `None` when finished.

```python
turn_deg = self._park("park_turn_deg", 90.0)
lock_deg = self._park("park_steer_deg", 53.0)
speed    = int(self._park("park_speed", self.turn_speed()))
turned   = self._park_turned(d)

if self._park_gone() > self._park("park_max_cm", 400.0):   # failsafe
    self.park_step = 0
    return None
```

**Step 1: swing out** (this is `swingTo(TURN_DEG, true, turnDir)`):
```python
if self.park_step == 1:
    if turned >= turn_deg:                               # threshold CROSSING
        self.park_step, self.park_ticks0 = 2, self.ticks # next step, new distance reference
    else:
        return speed, clamp_servo(self._straight_servo() + self.lock * lock_deg)
```
Notice there is **no `return` after moving to step 2**. The code falls through
into the `if self.park_step == 2:` block **in the same tick**, so no tick is
wasted sending nothing.

**Step 2: forward, holding the swung heading** (this is `straight(FORWARD_CM, TURN_DEG, true)`):
```python
if self.park_step == 2:
    want = self._park("park_forward_cm", 30.0)
    if self._park_gone() >= want:
        self.park_step, self.park_ticks0 = 3, self.ticks
    else:
        err = turn_deg - turned
        return speed, clamp_servo(self._straight_servo()
                                  + self.lock * max(-20.0, min(20.0, 1.2 * err)))
```
`max(-20, min(20, 1.2*err))` is Python's version of `constrain(HOLD_KP*err, -20, 20)`.

**Step 3: back up, holding the same heading** (this is `straight(BACK_CM, TURN_DEG, false)`):
```python
        return -abs(speed), clamp_servo(self._straight_servo()
                                        - self.lock * max(-20.0, min(20.0, 1.2 * err)))
#              ^^^^^^^^^ negative = reverse               ^ MINUS: reversing flips steering
```

**Step 4: swing back to square** (this is `swingTo(0, false, -turnDir)`):
```python
if self.park_step == 4:
    if turned <= 0:
        self.park_step = 0
        return None                        # ← finished
    return speed, clamp_servo(self._straight_servo() - self.lock * lock_deg)
```

> **Why `_park_gone()` is a function and not a variable:** when a step finishes,
> it resets `park_ticks0`, and the next step runs **in the same tick**. A
> distance worked out once at the top of the tick would still hold the old
> step's distance, and the next leg would "finish" instantly.

### 3.5 The handover inside `step()` (line 1219)

```python
if self.park_step:                      # non-zero = parking in progress
    out = self._park_step(d, now)
    if out is not None:
        self.last_t = now
        return out                      # parking owns the car: no corners, no pillars
    # --- parking just finished, on THIS tick ---
    self.target = d["yaw"]              # re-zero the round's heading to where the car points NOW
    self.i_term = 0.0
    self.pillar_stage = None
    self.pillar_clear_t = now           # ignore pillars for pillar_clear_delay_s
```

This block sits **above** everything else in `step()` (only `DONE` comes
before it), which is what gives parking absolute priority. When
`_park_step()` returns `None`, the code falls through into normal driving **in
the same tick**.

`self.target = d["yaw"]` matters because the round was started with
`ObstacleRound(d["yaw"], ...)` while the car sat in the bay. After the exit,
the car is square again but might be a few degrees off, so the straight-line
heading is reset to where it actually points.

### 3.6 Testing the merge without the car

```bash
cd obstacle_round/pi
python3 tools/park_exit_test.py
```
This simulates the car with a simple model and checks all of the following:
both mirror images, that the steps run 1→2→3→4 in order, that it ends square,
that overshooting 90° still ends the swing, that the 400 cm failsafe works,
that `park_enabled = 0` skips it, and **that the numbers match the `.ino`**.
The last line should say `ALL PASS`.

---

# Part 4: every parameter, explained

This part covers **every value you can tune**: what it does, the formula that
uses it, which way to move it, and what it interacts with. It is split into:

- **[4.1 Quick answers: "I want the car to …"](#41-quick-answers-i-want-the-car-to-)**: start here at the track
- **[4.2 How parameters work](#42-how-parameters-work-read-once)**: read this once
- **[4.3–4.11 The sliders, group by group](#43-group-0--leaving-the-parking-bay)**: every slider, in the same order as the dashboard
- **[4.12 Constants in `config.py`](#412-constants-in-configpy-no-slider-edit--deploy)**: settings with no slider, which need a code edit
- **[4.13 Parameters that do nothing](#413-parameters-that-do-nothing-right-now-traps)**: traps to avoid
- **[4.14 The reference docs win](#414-the-reference-docs-win)**: PARAMS.html and SPEC are authoritative

"Current" values are from `obstacle_round/pi/params.json` in the repo. **Where
[`docs/PARAMS.html`](docs/PARAMS.html) or [`SPEC.md`](SPEC.md) give a different value, theirs is the one to use (see 4.14).**
`deploy.sh` copies the Pi's file back into the repo each time it runs, so this
is the last tuning that was saved. Where `params.json` has no entry, the
default from `config.DEFAULT_PARAMS` applies, and it is marked *(default)*.

The group names (**0 · Leaving the parking bay** and so on) and the slider
ranges come from the new dashboard layout in `param_ui.py`. An older dashboard
may arrange things differently, but **the key names are the same.**

---

## 4.1 Quick answers: "I want the car to …"

Go through the list **in order** and change **one thing at a time**. Each
arrow shows which way to move the slider.

### Parking exit
| I want… | Change | Direction |
|---|---|---|
| Skip parking (testing laps) | `park_enabled` | **OFF** |
| Get further out into the lane before the round starts | `park_forward_cm` | ↑ (5–10 cm steps) |
| Stop hitting the bay wall when swinging out | `park_steer_deg` ↑ (tighter), then `park_turn_deg` ↓ | |
| Stop hitting the back wall when reversing | `park_back_cm` | ↓ |
| End more exactly parallel | Nothing in parking. The round re-zeroes the heading on handover. If it looks skewed, check `servo_trim_deg`. | |
| Park slower / more carefully | `park_speed` | ↓ (not below ~80, or the motor stalls) |

### Corners
| I want… | Change | Direction |
|---|---|---|
| Turn **earlier** (hits the wall / turns too late) | `front_turn_cm` | ↑ |
| Turn **later** (turns before reaching the corner) | `front_turn_cm` | ↓ |
| Finish the corner more tightly | `corner_steer_deg` ↑. But above ~35 the tyres scrub and it turns *worse*. | |
| Stop the tyres scrubbing / car not rotating in the turn | `corner_steer_deg` | ↓ |
| More room before turning (car is too close to the wall) | `pre_turn_reverse_cm` ↑, or `front_turn_cm` ↑ | |
| Let the reverse do more of the turn | `reverse_arc_leave_deg` ↓ | ↓ = the reverse does more |
| Reverse arc swings too little / too much | `reverse_arc_deg` | ↑ more / ↓ less |
| Car backs into the outer wall after a corner | `post_turn_reverse_cm` ↓, or `reverse_speed` ↓ | |
| More run-up on the new straight | `post_turn_reverse_cm` | ↑ |
| **Wrong lap direction** | Check where it's parked first. `default_lap_dir` is only a fallback. | |
| Car turns 90° **into a pillar** | `block_no_corner_s` ↑ and/or `block_near_cm` ↑ | |
| Car **misses a real corner** just after passing a pillar | `block_no_corner_s` | ↓ |

### Pillars
| I want… | Change | Direction |
|---|---|---|
| Start dodging **earlier** (it gets too close) | `pillar_trigger_dist_cm` ↑. **Above 35**, also raise `BACKOFF_TARGET_CM` (see 4.4). | |
| Start dodging later | `pillar_trigger_dist_cm` | ↓ |
| Clips the pillar on the **way out** | `pixel_base_cm` ↑ (if picture mode is ON, which it is), or `pillar_steer_right` ↑ (sharper) | |
| Swings out **too far** (hits the outer wall) | `pixel_base_cm` ↓, or `pillar_steer_right` ↓ | |
| Clips it only when the pillar is **off to one side** | `pixel_cm_per_px` ↑ | |
| Clips the pillar **alongside** / turns back in too early | `dodge_past_cm` ↑ (see the trap in 4.13), or `dodge_past_red_extra_cm` ↑ | |
| Clips it on the **way back in** | `dodge_return_extra_cm` ↓ (shorter swing back), or `dodge_align_min_red_cm` ↑ | |
| Ends up **across the lane** after a dodge (over-rotated) | `dodge_return_reduce_deg` ↑, or `dodge_return_extra_cm` ↓ | |
| Doesn't get back to the middle after a dodge | `dodge_return_extra_cm` ↑, or `recenter_k` ↑ | |
| Dodge is too violent / sudden | `turn_hold_pct` ↓ | |
| Doesn't get out sideways fast enough | `turn_hold_pct` ↑ | |
| **Dodges the same pillar twice** | `pillar_clear_cm` ↑ | |
| Ignores a pillar right after a dodge | `pillar_clear_cm` ↓, `pillar_clear_delay_s` ↓ | |
| Distance on the dashboard is wrong vs a tape measure | `pillar_focal_px` (see 4.4 for the formula) | |
| **Green behaves differently from Red** | Tune the **red** value. Green is mirrored from it automatically. | |

### Straights
| I want… | Change | Direction |
|---|---|---|
| Stop weaving / zig-zagging on straights | `cruise_wall_k` | ↓ (try 1.0, then 0.6) |
| Pure gyro, ignore the walls on straights | `cruise_wall_k` | **0** |
| Stay more in the middle | `cruise_wall_k` | ↑ |
| Car drifts to one side with the walls ignored | `servo_trim_deg` (see 4.10) | |
| Hold a different distance off the wall | `recenter_target_cm` (**not** `cruise_target_cm`, see 4.10) | |
| Go faster / slower | `base_speed` (max 110) | |

---

## 4.2 How parameters work (read once)

**Where they live.** Every tunable is a key in `config.DEFAULT_PARAMS`, which
holds the default. `params.json` on the Pi holds the tuned value. The dashboard
sliders write to it.

**Live vs saved.** Moving a slider sends the new value to `main.py`
**immediately**, even in the middle of a run. It is only written to
`params.json` when you click **Save to Disk**. If you don't save, the change is
lost when the Pi reboots.

**When a value takes effect.** Most are read **every tick**, so a change applies
on the next tick. Some are **latched** when a manoeuvre starts:
- `park_*` are read every tick, but changing them mid-exit is asking for trouble.
- `use_pixel_dodge`, `pixel_*`: read **once, when a dodge commits**.
- `pillar_trigger_dist_cm`: every tick of APPROACH.

**Green mirrors red.** When params are loaded or saved,
`config.mirror_green()` overwrites every green value from its red one:

| Green key (ignored) | Copied from |
|---|---|
| `pillar_steer_left` | mirror of `pillar_steer_right` about true straight (87) |
| `dodge_green_extra_cm` | `dodge_red_extra_cm` |
| `dodge_past_green_extra_cm` | `dodge_past_red_extra_cm` |
| `dodge_return_green_extra_cm` | `dodge_return_red_extra_cm` |
| `dodge_align_min_green_cm` | `dodge_align_min_red_cm` |

That is why `params.json` can say `dodge_green_extra_cm: 2.0` while the car
actually uses red's `6.0`. **The red names really mean "both colours".**

**Clamps.** `main.py` limits every value, so a typo cannot do anything wild.
Speeds are limited to 0..`MAX_PWM` (110), dodge legs to 5..100 cm, reverses
to 0..150 cm, and steering to the servo range 30..140. If a slider "stops
working" past a certain value, it has hit one of these limits.

**Two different "centres".** `config.SERVO_CENTER` is **90**. True straight is
**90 + `servo_trim_deg`**, which is **87**. Parking, cruising, reversing and
the dodge angles use 87. **Corner steering and the reverse arc use 90.** Keep
this in mind when you do the arithmetic.

---

## 4.3 Group 0 · Leaving the parking bay

Runs once at START, before the first lap (see Part 3). The car turns away from
the nearer wall. `lock` = +1 (turning right) or -1 (turning left).

### `park_enabled`: ON (default 1)
`begin_parking()` returns without doing anything when this is 0, so the round
starts driving from where the car stands. The lap direction is **still** read
from the side walls, so for testing, place the car nearer the correct wall.

### `park_turn_deg`: 90° (default) · range 30–120
How far to swing out (step 1 ends when `turned >= park_turn_deg`). The same
number is also the heading held during steps 2–3. Step 4 always swings back to
**0**, so the car ends on its starting heading whatever this is set to.
- ↓ to 60–70 for a shallower, longer exit. It needs more `park_forward_cm` to get clear.
- The swing ends on a **threshold crossing**, so it will overshoot by a few degrees at speed. That is expected.

### `park_steer_deg`: **50 on the car** (PARAMS.html) · fallback 53 · range 10–55
Wheel lock for both swings: `servo = 87 ± 50` = **137 / 37**. 53 is the
maximum (87 + 53 = 140); anything larger gets clamped at 140 on the right.
- ↓ = a wider circle, which needs more room in the bay.

### `park_forward_cm`: **80 cm on the car** (PARAMS.html / SPEC) · fallback 30 · range 0–200
Step 2: drive forward this far (encoder), holding `park_turn_deg` with
`steer = 87 + lock × clamp(1.2 × error, ±20)`.
This is the leg that carries the car clear of the bay into the lane. Raise it if
the car is still too close to the wall when the round starts.

### `park_back_cm`: 20 cm (default) · range 0–100
Step 3: reverse this far, holding the same heading. The correction sign is
flipped (Part 0.2). **This is blind**, because there is no rear sensor.

### `park_speed`: 90 PWM (default) · range 40–100
Used for all four steps. Below ~80 the motor may not move at all, and the
car sits there until `park_max_cm`… which never counts up, because it isn't
moving.

### `park_max_cm`: 400 cm (default) · range 100–800
**This is a failsafe, not a control.** If any single step has covered more
than this distance without finishing, the IMU is probably dead, so the car
gives up on parking and starts racing. A normal step is well under 1 m.

---

## 4.4 Group 1 · Seeing a pillar

### `pillar_focal_px`: 443.8 · range 200–1500
The camera focal length in pixels. Every pillar distance is
**`dist_cm = pillar_focal_px × 10 / box_height_px`** (or `× 5 / box_width` for a clipped box).
- **Calibrate it**: put a pillar at exactly 40 cm, read the distance on the dashboard, then
  `new = old × 40 / shown`. If it shows 50 at a real 40 cm: `443.8 × 40/50 = 355`.
- It is tied to the frame width. If `CAMERA_SIZE` changes (it is 640×360 now), this **must** be rescaled, along with `pixel_cm_per_px` and `ROI_CENTER_SHRINK_PX`.
- Every distance-based pillar setting below assumes this is correct. **Check this first** if the pillar behaviour seems off.

### `pillar_trigger_dist_cm`: 30 cm · range 5–120
**The activation distance.** When the camera distance is ≤ this for **3
separate camera frames** (`DODGE_TRIGGER_FRAMES`), the dodge commits. There is
also a shortcut: if the pillar is in the CENTER band, the front ToF reads
≤ 40 cm, and the ToF and camera agree within 25 cm, it commits immediately.
- ↑ = dodges start further away, for a gentler dodge with more room, but the pixel measurement is less precise.
- **Interaction with BACKOFF**: if a pillar is first seen closer than `trigger − 8` cm (`BACKOFF_IMMEDIATE_CM`), the car **reverses until it reads 35 cm** (`BACKOFF_TARGET_CM`), then approaches again. If you raise the trigger above ~35, the back-off stops *inside* the trigger and the car shuffles back and forth (twice, up to `BACKOFF_MAX_TRIES`). **When you raise the trigger, set `BACKOFF_TARGET_CM` in `config.py` to about trigger + 5.**
- The pixel calibration (4.6) was done at this distance. Changing it a lot makes `pixel_cm_per_px` less accurate.

---

## 4.5 Group 2 · The dodge: how hard

All angles are **written for Red** (dodging right) and mirrored for Green
about 87 by `_as_red()`. The largest offset either way is 53°
(`min(87−30, 140−87)`), so both colours run out of lock at the same point.

### `pillar_steer_right`: 135 · range 90–140
The **sharp turn-out angle**. Red: 135 is **48° right** of 87. Green: 87 − 48 = **39**.
- ↑ = sharper swing out. It gets sideways faster over the same distance.
- It also sets the **return angle**. The swing back in uses the mirror, `180 − 135 = 45` (red), i.e. 42° the other way.

### `pillar_steer_left`: *derived*, read-only
Shown for information. Changing it does nothing.

### `turn_hold_pct`: 30% · range 0–80
The shape of both the turn-out and the swing back in. For the first 30% of the
leg the wheels **hold** the sharp angle, then they unwind to straight over the
remaining 70% along a curve (`unwind^1.5`):
```
steer = sharp + (90 − sharp) × ((progress − 0.30) / 0.70) ^ 1.5
```
- ↑ = more time at full lock, so the car moves sideways faster and more abruptly.
- ↓ = starts easing off sooner, so the arc is smoother but gains less sideways.

### `dodge_wide_steer_deg`: 60 (default) · range 20–80
Only used for a **"wide" dodge**, which is a pillar already sitting on the side
the car is moving towards (Green in the LEFT band, Red in the RIGHT band). It
replaces `pillar_steer_right` with a **gentler** angle. It is written as the
green-side angle: red = 180 − 60 = **120** (33° right of 87), green = **54**.
- ↑ (towards 90) = gentler; ↓ = sharper.
- ⚠️ With picture mode ON (as it is now), the wide dodge uses this **gentle angle with the pixel-sized length**. The gentle angle was designed to go with the longer `dodge_long_cm` leg, so wide dodges may not get out far enough. If wide dodges clip, lower this towards 50.

### `block_offset_sens`: 0.8 · range 0–1.5
Fine-tunes the turn-out **angle** by where the pillar sits in the frame:
```
red_angle = pillar_steer_right + (±1) × norm_x × block_offset_sens × 15
norm_x = (pillar_cx − 320) / 320     # −1 at the left edge, +1 at the right edge
```
At 0.8 the most it can change is **±12°**. A Red pillar right of centre → sharper right.
- 0 = off.
- This adjusts the **angle**. The pixel method (4.6) adjusts the **distance**. Both are active at once, so if dodges get extreme on off-centre pillars, lower this first.

---

## 4.6 Group 3 · The dodge: leg lengths

A dodge has four legs, all measured on the encoder:
```
TURN_OUT (_out_cm) → STRAIGHTEN (_past_cm) → RECENTER (_in_cm) → ALIGN (until square)
```

### How the turn-out length is chosen
```python
if use_pixel_dodge and the pillar box is known:        # ← ON right now
    out = pixel_base_cm + toward_px × pixel_cm_per_px   (clamped 8..55, then 5..100)
    # dodge_short_cm, dodge_long_cm and dodge_red_extra_cm are NOT used
else:
    out = (dodge_long_cm if wide else dodge_short_cm) + dodge_red_extra_cm
```

### `use_pixel_dodge`: ON · switch
The master switch for **picture mode**. ON = the turn-out length comes from
where the pillar is in the frame. OFF = the fixed short/long lengths below.
Turn it OFF if the pixel sizing seems random. That usually means
`pillar_focal_px` or the trigger distance has changed since calibration.

### `pixel_base_cm`: 31.15 · range 0–40
The turn-out length for a pillar **dead centre** in the frame. It is **not** a
minimum: a centred pillar needs the most careful clearance.
- ↑ = every dodge swings out further, especially centred ones.

### `pixel_cm_per_px`: 0.08 · range 0–0.25
Extra turn-out per pixel that the pillar sits **towards the side the car is
dodging to**:
```
toward_px = (pillar_cx − 320)   for Red   (pillar further RIGHT → more)
toward_px = (320 − pillar_cx)   for Green (pillar further LEFT  → more)
out = 31.15 + toward_px × 0.08
```
Examples: a red pillar at cx = 420 → 31.15 + 100×0.08 = **39 cm**. At cx = 220
(already on the left, correct side) → 31.15 − 8 = **23 cm**.
- ↑ = more difference between centred and off-centre pillars.
- It is only valid for a **640-wide frame at the trigger distance** it was measured at.

### `dodge_short_cm`: 29 · `dodge_long_cm`: 35 · `dodge_red_extra_cm`: 6
Fixed-mode turn-out lengths: normal / wide, plus a trim. **These do nothing
while `use_pixel_dodge` is ON** (the code returns before reaching them).

### `dodge_past_cm`: 50 · range 5–90 · plus `dodge_past_red_extra_cm`: 5
STRAIGHTEN: how far to drive straight past the pillar, holding heading on the
gyro **with the ToF ignored** (the inner sensor is looking at the pillar).
`past = dodge_past_cm + per-colour extra`. On the car (SPEC): **Green 62 cm, Red 57 cm**.
- ↑ if the car clips the pillar as it turns back in (it turns in too early).
- ⚠️ **In committed `main.py` (`5a1de4d`) the `dodge_past_cm` slider is ignored.** The code uses `config.DODGE_PAST_CM` (52). Only the `_extra` slider works until the fix is committed (see Part 2.10).

### `dodge_return_extra_cm`: 3 · plus `dodge_return_red_extra_cm`: 0
RECENTER (swing back in): `in = turn-out length + 3 + 0`. The extra is added
**on top of whatever the turn-out was**, pixel mode included, so these always
work.
- ↑ = swings back in for longer, ending closer to the middle (or across it).
- ↓ = comes back in less.
- The two sliders do exactly the same thing. `_red_extra` is just a second slider for it.

### `dodge_return_reduce_deg`: 0 · range 0–30
Makes the swing-back-in angle **shallower**. The return angle is the mirror of
the turn-out (red: 45), moved towards straight by this many degrees (never past 90).
- ↑ if the car over-rotates across the lane on the way back in.

The swing back in also doesn't unwind all the way to straight. It ends at
`DODGE_RETURN_END_DEG` (80 green-side → **100 red / 74 green**), leaning
slightly into the direction ALIGN will need.

### `dodge_align_min_red_cm`: 2 · range 0–20
ALIGN won't finish until the car has driven at least this far, even if it is
already square. This stops the dodge ending with the pillar still alongside.
- ↑ if the car clips the pillar right at the end of the dodge.
- ALIGN itself: `steer = 87 + 2.4 × heading_error + clamp(wall_error × recenter_k, ±40)`. It finishes when the heading is within 5° (`ALIGN_DONE_ERR_DEG`) **and** this minimum is met, or after `ALIGN_MAX_CM` (45).

---

## 4.7 Group 4 · After a dodge

### `pillar_clear_cm`: 60 cm · range 0–150
After a dodge finishes, the car must drive this far before it may pick **any**
new pillar. Without it, the camera picks up the pillar just passed (sitting
beside the car at ~20 cm) and dodges it again.
It also controls the **minimum pickup distance**: for `2 × pillar_clear_cm`
(120 cm) after a dodge, pillars closer than 45 cm are ignored
(`PILLAR_MIN_ACQUIRE_CM`). After that, only pillars under 15 cm are ignored.
Turning a corner resets this gate, since the old pillar is behind the corner.
- ↑ if it dodges the same pillar twice.
- ↓ if two pillars are close together and it misses the second one.

### `pillar_clear_delay_s`: 1.2 s · range 0–6
A timer on top of `pillar_clear_cm`, and **both** must pass. Mostly matters
if the encoder stops reporting.

---

## 4.8 Group 5 · Corners

The sequence: **drive → reverse arc → forward turn → straight reverse** (see Part 2.9).

### `front_turn_cm`: 37 cm · range 15–80
A corner starts when the front ToF reads ≤ this (with the lap direction
already locked from parking). This is **the** corner distance.
- ↑ = turns earlier, further from the wall.
- ↓ = turns later, closer to the wall.
- Also blocked when: a pillar is dead ahead (group 6), the car hasn't driven 20 cm since the last corner, or a dodge is in progress.

### `corner_steer_deg`: 28 · range 10–50
The **maximum** steering during the forward turn. The actual angle is:
```
off = min(corner_steer_deg, max(7, 0.55 × |heading_error|))
servo = 90 + off (right turn)  /  90 − off (left turn)
```
So with 90° to go it uses the full 28 (servo 118 / 62). As the car comes round
it eases off, down to 7° near the end. The turn ends when the heading is within
13° of the target (`TURN_DONE_ERR_DEG`), or after 6.5 s.
- ↑ = tighter corners, until the tyres start scrubbing (above ~35). **More is not always tighter.**
- `CORNER_KP` (0.55) and `CORNER_MIN_OFF_DEG` (7) are constants.

### `pre_turn_reverse_cm`: 35 cm · range 0–60
The **reverse arc** before the turn. The car reverses with the wheels turned
**the opposite way** (turning right → wheels left), so the nose swings towards
the new heading while it backs off the wall. It stops at whichever comes first:
the heading is within `reverse_arc_leave_deg` of the target, **or** it has
reversed this far, **or** 8 s have passed.
- 0 = no reverse arc. The car turns forward from wherever it is.
- ↑ = more room, but more blind reversing.

### `reverse_arc_deg`: 40 · range 0–55 (capped at 50)
How far the wheels turn during the reverse arc: servo **90 ∓ 40 = 50 / 130**.
- ↑ = the arc rotates the car more per cm reversed.

### `reverse_arc_leave_deg`: 35 · range 0–90
How many degrees of the 90° are **left for the forward turn**. The arc stops
once `|heading_error| ≤ 35`, so it does about 55° and the forward turn does
the last 35°.
- ↓ = the reverse does more of the corner. At 0 it would try to do the whole corner in reverse.
- ↑ = the forward turn does more.

### `post_turn_reverse_cm`: 40 cm · range 0–150
After the turn finishes, reverse **straight** this far (holding heading, with
the correction flipped), to get run-up on the new straight.
- 0 = off.
- ↓ if it backs into the wall behind. ↑ if it starts the new straight too close to the next pillar.

---

## 4.9 Group 6 · Block or corner?

The front ToF can't tell a pillar from a wall. These two settings stop the car
turning 90° into a pillar.

### `block_near_cm`: 60 cm · range 20–120
A Red or Green pillar **in the CENTER band** that the camera puts closer than
this counts as "the thing in front". Pillars in the LEFT or RIGHT bands never
count, because the front ToF can't be looking at them.

### `block_no_corner_s`: 0.8 s · range 0–6
For this long after such a sighting, **corners are blocked** completely. The
memory matters because up close the pillar fills or leaves the frame and
detection drops out, which is exactly when the wall logic would take over.
- ↑ if it turns into pillars.
- ↓ if it misses a real corner right after a pillar that sits near one.
- 0 = off.

---

## 4.10 Group 7 · Driving straight

The straight-line formula (from the working open round):
```
servo = (90 + servo_trim_deg) + 1.2 × heading_error + wall_term
wall_term:  both walls < 80 cm  → (right − left) × k × 0.5
            one wall             → hold `target` cm off it (the OUTER wall preferred)
            none                 → 0
```

### `cruise_wall_k`: 1.6 · range 0–5
`k` on the straights. At 1.6, with the car at L30 / R70 the wall term alone is
(70−30)×1.6×0.5 = **32°** of steering. That is strong.
- **The first setting to lower if the car weaves or scrubs.** Try 1.0.
- **0 = pure gyro**: holds heading only and ignores the walls.

### `recenter_target_cm`: 45 cm · range 30–60
⚠️ **Despite the name, this is the one-wall target for normal cruising**, and
for ALIGN after a dodge. The final cruise call in `step()` passes
`recenter_target_cm` as the target.
- It only applies when **one** wall is visible. With both visible, the car balances them, and this is ignored.

### `cruise_target_cm`: 42 cm · range 20–70
The one-wall target **only during APPROACH** (driving towards a tracked pillar)
and the **finish push**. For everyday cruising, use `recenter_target_cm`.

### `recenter_k`: 2.0 · range 0.1–5
Wall strength during **ALIGN** only (after the swing back in). It uses the
**outer** wall only: after a Green dodge it trusts the left, after a Red the
right. The ToF on the pillar side can still see the pillar. It is clamped to
±40°.
- ↑ = gets back to the middle faster after a dodge. Too high and ALIGN wobbles and runs out of distance.

### `servo_trim_deg`: −3 · range −15 to 15
Where "straight" really is: 90 + (−3) = **87**. Measured: the heading controller
settles at 87.1, and a reverse at 90 curves left.
- To re-measure: set `cruise_wall_k = 0`, drive a long straight, and read the average `steer` column in the CSV log. Trim = that − 90.
- It changes parking, cruising, reversing and every dodge angle. **It does not change corners** (those use 90).

### `default_lap_dir`: CW · CW / CCW
Used **only** when neither side ToF sees a wall at START (both > 80 cm), so the
parking position can't tell the direction. Normally the direction comes from
the parking position: **nearer the LEFT wall = CW = right turns**.

---

## 4.11 Group 8 · Speed

All three are clamped to `MAX_PWM` = 110. 90 PWM ≈ **26 cm/s**. Below ~80 the
geared motor moves in jerks or doesn't move at all.

| Key | Current | Used for |
|---|---|---|
| `base_speed` | 90 | Straights, APPROACH, every dodge leg, finish push |
| `turn_speed` | 90 | The forward part of a corner. Also `park_speed`'s fallback. |
| `reverse_speed` | 90 | Reverse arc, post-corner reverse, BACKOFF, the wedged reverse. **All blind.** |

Distances are measured on the encoder, so **changing speed does not change
the shape** of any manoeuvre. It changes timing: a faster car covers more
ground between camera frames (so it triggers later and closer) and overshoots
turns further. When you raise speed, expect to raise `pillar_trigger_dist_cm`
and `front_turn_cm` a little too.

---

## 4.12 Constants in `config.py` (no slider: edit + deploy)

These are `UPPER_CASE` names in `obstacle_round/pi/config.py`. Changing one
means: edit, `deploy.sh`, and the service restarts. **Only the ones worth
knowing are listed here.**

### Safety and hardware
| Constant | Value | Meaning / when to change |
|---|---|---|
| `MAX_PWM` | 110 | Speed ceiling. **Also change it in `open_slave.ino` and reflash** (recipe E). Team limit is 170. |
| `SERVO_MIN` / `SERVO_MAX` | 30 / 140 | Steering limits. The mechanical stops are 28/142. Don't widen. |
| `SERVO_CENTER` | 90 | The nominal centre. Use `servo_trim_deg` instead. |
| `TICKS_PER_METER` | 2078 | Encoder calibration. Re-measure with `esp/tick_m_calculate` after any wheel or gear change. **Every distance depends on it.** |
| `TELEMETRY_MAX_AGE_S` | 0.3 | Sensor data older than this → car stops. |
| `CAMERA_SIZE` | (640, 360) | Frame size. Changing it means rescaling `pillar_focal_px`, `pixel_cm_per_px`, `ROI_CENTER_SHRINK_PX`. |

### Race
| Constant | Value | Meaning |
|---|---|---|
| `TOTAL_TURNS` | 12 | 3 laps × 4 corners. |
| `FINISH_TIME_S` | 3.0 | Drive on for this long after corner 12, then stop. **Tune this so the car stops in the start section.** |
| `LOCK_DIRECTION_AT_START` | True | Lap direction from the parking position. False = decide at the first corner (old behaviour). |

### Corners
| Constant | Value | Meaning |
|---|---|---|
| `TURN_DONE_ERR_DEG` | 13 | Turn ends when within this many degrees of the target. ↓ = more exact, but slower to finish. |
| `TURN_TIMEOUT_S` | 6.5 | Turn gives up after this long. |
| `CORNER_KP` / `CORNER_MIN_OFF_DEG` | 0.55 / 7 | Corner steering = 0.55 × error, at least 7°, at most `corner_steer_deg`. |
| `MIN_TRAVEL_BETWEEN_TURNS_CM` | 20 | Must drive this far between corners (stops double-counting). |
| `CORNER_CRITICAL_FRONT_CM` | 25 | Below this, the 20 cm rule is skipped. The car must turn. |
| `REVERSE_START_CM` / `REVERSE_STOP_CM` | 12 / 20 | Mid-turn: if front < 12, reverse with opposite lock until front > 20. |
| `POST_TURN_REVERSE_TIMEOUT_S` | 8 | Backstop for both corner reverses. |
| `KP_REVERSE_HEADING` | 1.2 | Heading-hold strength while reversing. |

### Straights and sensors
| Constant | Value | Meaning |
|---|---|---|
| `KP_HEADING_CRUISE` | 1.2 | Servo degrees per degree of heading error on straights. ↑ = snappier heading hold, but it may oscillate. |
| `WALL_VALID_CM` | 80 | A side reading ≥ this = "no wall there". |
| `TOF_VOTE_S` / `TOF_VOTE_MIN_SAMPLES` | 0.25 / 6 | A 999 reading has to be confirmed over this window before it is believed. |

### Pillars
| Constant | Value | Meaning |
|---|---|---|
| `DODGE_TRIGGER_FRAMES` | 3 | Camera frames that must agree before a dodge commits. ↑ = fewer false triggers, but later. |
| `BACKOFF_TARGET_CM` | 35 | Back off until the pillar reads this far. **Keep it about 5 cm above `pillar_trigger_dist_cm`.** |
| `BACKOFF_IMMEDIATE_CM` | 8 | Back off at once if the pillar is this far inside the trigger. |
| `BACKOFF_GRACE_S` / `BACKOFF_MAX_TRIES` / `BACKOFF_MAX_CM` | 0.6 / 2 / 45 | Grace period, retries, and furthest reverse. |
| `APPROACH_LOST_S` | 1.5 | Give up on a pillar that hasn't been seen for this long. |
| `PILLAR_SWITCH_MARGIN_CM` | 20 | Switch to a different pillar if it's this much nearer. |
| `PILLAR_TOF_COMMIT_CM` / `BLOCK_TOF_AGREE_CM` | 40 / 25 | The ToF commit shortcut, and how closely camera and ToF must agree. |
| `ALIGN_DONE_ERR_DEG` / `ALIGN_MAX_CM` | 5 / 45 | ALIGN ends within 5°. It gives up (with a warning) after 45 cm. |
| `STAGE_TIMEOUT_S` | 6 | Every dodge stage gives up after this long, in case the encoder dies. |
| `DODGE_RETURN_END_DEG` | 80 | Where the swing back in unwinds to (green-side; red = 100). |
| `ROI_CENTER_SHRINK_PX` | −75 | Negative = a **wider** CENTER band. Bands are thirds of 640 = 213 px, so the centre is ~288 px wide. |
| `BLOCK_CONF` | 0.5 | Minimum YOLO confidence. ↑ if it sees phantom pillars, ↓ if it misses real ones. |
| `PILLAR_SKIP_WHEN_ALREADY_CLEAR` | False | True = don't dodge a pillar already on its correct side. Off: every pillar is dodged. |
| (in `main.py`) `kp_align = 2.4` | 2.4 | ALIGN's heading gain. Written as a literal inside the ALIGN block. |

---

## 4.13 Parameters that do nothing right now (traps)

| Parameter | Why it does nothing |
|---|---|
| Every **green** key (`pillar_steer_left`, `dodge_*_green_*`) | Overwritten from red by `mirror_green()`. |
| `dodge_short_cm`, `dodge_long_cm`, `dodge_red_extra_cm` | `use_pixel_dodge` is ON, and `_leg_cm()` returns before reaching them. Turn picture mode OFF to use them. |
| `dodge_past_cm` | In committed `main.py`, `_past_cm()` reads `config.DODGE_PAST_CM` (52) instead. Fixed in the uncommitted working copy. Once that is committed and deployed, it works. |
| `cruise_target_cm` during normal cruising | Normal cruising uses `recenter_target_cm`. This one only affects APPROACH and the finish push. |
| `recenter_target_cm` / `cruise_wall_k` when both walls are visible | With both walls in range, the car balances them, and the target is ignored. |
| Any key in `params.json` that is **not** in `DEFAULT_PARAMS` | `load_params()` drops unknown keys. |

---

## 4.14 The reference docs win

[`docs/PARAMS.html`](docs/PARAMS.html) and [`SPEC.md`](SPEC.md) /
[`docs/SPEC.html`](docs/SPEC.html) are **the authoritative reference** for
values and behaviour on the car. If this guide disagrees with them, **they
win**. This guide explains *how the code uses* each value. Those pages say
*what the value is and should be*.

Values where they differ from the `config.py` fallbacks quoted elsewhere in this guide:

| Parameter | PARAMS / SPEC (use this) | `config.py` fallback |
|---|---|---|
| `park_forward_cm` | **80 cm** | 30 |
| `park_steer_deg` | **50°** | 53 |
| Run past the pillar (`dodge_past_cm` + extra) | **Green 62 / Red 57 cm** | — |
| Swing back in | **Green turn-out + 6 / Red turn-out + 3** | — |

When you change a parameter, update PARAMS.html and republish it, as
[`docs/README.md`](docs/README.md) says.

---

# Part 5: how to change things safely (recipes)

Every recipe ends with the same three steps: **test → deploy → commit.**
```bash
python3 obstacle_round/pi/tools/park_exit_test.py     # and any other relevant test
pi_setup/deploy.sh                                     # copies code, restarts services; keeps params.json
git add -A && git commit -m "what and why" && git push
```

### A. Change a number that has a slider
1. Open `http://error404.local:8080`, move the slider, and try it. It applies instantly.
2. Click **Save to Disk** to write it to `params.json` on the Pi.
3. No code change is needed. `deploy.sh` copies the Pi's `params.json` back into the repo next time.

### B. Change the parking exit
1. In the combined round: use the `park_*` sliders on the dashboard (A), or edit `DEFAULT_PARAMS` in `obstacle_round/pi/config.py`.
2. To turn it off in the combined round: set `park_enabled` to 0.
3. For standalone parking testing: edit parameters in `parking/config.py` and run via `flasher.py parking` or `parking.service`.
4. If you change a default in `config.py`, verify that `python3 obstacle_round/pi/tools/park_exit_test.py` passes.

### C. Add a brand-new slider
1. Add the key and default to `DEFAULT_PARAMS` in `config.py`. **Without this, `load_params` drops it.**
2. Read it in `main.py` with the safe pattern:
   ```python
   try:
       v = float((self._params or {}).get("my_new_key", 12.0))
   except (TypeError, ValueError):
       v = 12.0
   v = max(0.0, min(50.0, v))       # always clamp
   ```
3. To make it appear on the dashboard, copy how an existing key such as `pre_turn_reverse_cm` is listed in `param_ui.py` (a list of `{"key": ..., "label": ..., "unit": ...}` entries).

### D. Change a constant (no slider)
1. Edit `config.py`.
2. Deploy. `master.service` restarts and picks it up.

### E. Change the maximum speed
1. Change `MAX_PWM` in `obstacle_round/pi/config.py`.
2. Change `MAX_PWM` in `obstacle_round/esp/open_slave/open_slave.ino`.
3. **Reflash the ESP32.** If you do only step 1, the ESP silently clips the speed back down.
4. The team limit is 170. Do not go over it.

### F. Corners are taken too early or too late
- Use the `front_turn_cm` slider. Higher = turn earlier.
- If the car turns but ends crooked, look at `TURN_DONE_ERR_DEG` (13°) in `config.py`. Smaller = finishes closer to the target.
- If it scrubs or does not rotate, use the `corner_steer_deg` slider. **Smaller** can actually turn better, because at full lock the tyres slide instead of gripping.

### G. The dodge clips the pillar
- On the way out: raise `pixel_base_cm` (picture mode, which is on), or `pillar_steer_right` (sharper). `dodge_red_extra_cm` only works with picture mode off.
- Passing: raise `dodge_past_red_extra_cm`.
- Coming back in: raise `dodge_return_extra_cm` (the return swing) or `dodge_align_min_red_cm`.
- **Tune the red values only.** Green copies them.

### H. Add a new stage to a state machine
1. Choose a name, e.g. `"WAIT"`.
2. Enter it from the previous stage with `self._begin_stage("WAIT", now)`. That resets `stage_cm`.
3. Add `if self.pillar_stage == "WAIT": ... return speed, steer` **in the right place in `step()`**. The position is its priority.
4. End it with `_stage_done(cm, now)` (distance, with a timeout backstop), then `_begin_stage("NEXT", now)`.
5. Add a line for it in `_stage_display_raw`, so the dashboard and CSV show it.

---

# Part 6: when it misbehaves, where to look

| Symptom | First place to look |
|---|---|
| Car does nothing after START | `journalctl -u master.service -f`. Is `dashboard.service` running? Is `DATA` arriving? Is the ESP LED blue or green? |
| Car stops after half a second, LED red | ESP watchdog: `main.py` crashed or stopped sending. Check the journal. |
| Wrong lap direction | The `PARKED DIRECTION:` log line. Was it parked nearer the correct wall? Is the ToF left/right swap right (ch1 = LEFT)? |
| Parking swings forever | IMU is not reading (BNO on mux ch4?). The 400 cm failsafe will end it. |
| A slider does nothing | Is the key in `DEFAULT_PARAMS`? Is it a green key (overwritten by red)? Does the code read the slider or a constant (see the `_past_cm` note)? |
| Speed change does nothing | `MAX_PWM` is capped on the ESP too (recipe E). |
| Car turns into a pillar | `CORNER BLOCKED` in the log. Check `block_near_cm` and `block_no_corner_s`. |
| Car dodges the same pillar twice | Raise `pillar_clear_cm`. |
| Any bad run | Open `logs/run_*.csv`. Every tick has the stage, decision, sensors, target, speed and steer. |

**Log lines worth searching for:** `RUN STARTED`, `PARKED DIRECTION`,
`PARKING 1/4` … `4/4`, `CORNER n: TURNING`, `CORNER n COMPLETE`,
`PILLAR DETECTED`, `STAGE 2: TURNING OUT`, `DODGE COMPLETE`, `BACKING OFF`,
`FINISH LINE CROSSED`.

**Before trusting any change:** put the car on a stand (wheels off the ground),
press START, and watch the dashboard. The steering should go the way you
expect before the car ever touches the mat.
