<title>V2 Bot Movement Spec</title>

# V2 bot — complete movement specification

Team Yo_Error_404, WRO 2026 Future Engineers. Written 2026‑09‑25.
Every number below was read from the live car, not from memory.

---

## 1. The machine

| | |
|---|---|
| **Master** | Raspberry Pi 5, `pi@error404.local`, user `pi`, password `pi` |
| **Slave** | ESP32‑**C3** Super Mini over USB (`/dev/ttyACM0`) |
| **Drive** | MD10112 brushed motor driver |
| **Steering** | DS3218 servo |
| **Sensing** | 3× VL53L0X ToF, BNO055 IMU, wheel encoder, Logitech C920 |

### Pins — ESP32‑C3

| Function | GPIO | Notes |
|---|---|---|
| Motor **PWM** (IN1) | **1** | |
| Motor **DIR** (IN2) | **0** | **forward = LOW** |
| Steering servo | 6 | 50 Hz, 500–2400 µs |
| NeoPixel ×16 | 7 | |
| START button | 10 | `INPUT_PULLUP`, active LOW |
| Encoder A | 4 | interrupt, RISING |
| Encoder B | 5 | direction |
| I²C SDA / SCL | 8 / 9 | 100 kHz |

### TCA9548A mux at `0x70`

| Channel | Device |
|---|---|
| 0 | ToF **front** |
| 1 | ToF **LEFT** |
| 2 | ToF **RIGHT** |
| 3 | ToF rear — **not fitted** |
| 4 | BNO055 @ `0x28` |

> Left and right were physically swapped on the car; the firmware compensates.
> **All reversing is blind** — there is no rear sensor.

### Calibrated constants

| Constant | Value | Meaning |
|---|---|---|
| `TICKS_PER_METER` | **2078** | measured; 20.78 ticks per cm |
| `SERVO_CENTER` | 90 | nominal |
| `servo_trim_deg` | **−3** | **true straight ahead is 87**, not 90 |
| `SERVO_MIN` / `MAX` | 30 / 140 | mechanical stops are 28/142 |
| `MAX_PWM` | 110 | enforced in **both** Pi and firmware |
| `BASE / TURN / REVERSE_SPEED` | 90 / 90 / 90 | |
| `CAMERA_SIZE` | 640 × 360 | |
| `PILLAR_FOCAL_PX` | 443.8 | scales with frame width |

---

## 2. Things that will bite you

These each cost hours. They are not theoretical.

1. **Never leave GPIO 0 or 1 as an input.** IN1 is the driver's PWM input; a pull‑up reads as 100 % duty and the car drives off at full speed with no code commanding it. Set both `OUTPUT` and `LOW` as the *first* statements in `setup()`.
2. **Attach the servo before the first `analogWrite`.** On ESP32 core 3.x `analogWrite` claims an LEDC timer at ~1 kHz; if it runs first the servo shares that timer and spins continuously.
3. **Forward counts DOWN on the encoder.** The Pi takes `abs()` on every comparison, so the sign has been invisible for the whole project. Any new code comparing a *signed* distance must flip it.
4. **90 is not straight.** Measured two ways: the heading controller settles at 87.1 holding a straight line, and a reverse commanded at 90 curves left at 0.45 °/cm over 486 cm.
5. **NDOF gives a dead heading on this car.** The BNO055 in NDOF fuses the magnetometer, which never calibrates beside a motor and a LiPo — `mag` stays 0 and `orientation.x` reads a permanent `0.0`. Use **`OPERATION_MODE_IMUPLUS`** (accel + gyro, no mag) for relative heading. The gyro reaches `cal 3` about two seconds after you start reading it.
6. **Read the IMU while waiting for START.** A BNO055 read moments after `begin()` has not settled.
7. **999 means two different things** — "nothing in range" *and* "sensor errored" — and the driving logic reads it as an OPEN side, which is what triggers a corner. A flaky sensor invents corners.
8. **Stop `master.service` and `dashboard.service` before flashing**, or esptool dies mid‑write.
9. **`dashboard.py` owns the serial port.** `main.py` talks to the ESP32 *through it* over UDP 127.0.0.1:5005. **If dashboard.py is not running, the car will not move.**
10. **The ESP32 only sends `DATA` lines under `open_slave`.** Any other sketch → the dashboard correctly reports the ESP32 as silent. That is not a fault.
11. **The Pi throttles.** It sits at ~85 °C with no active cooling; `vcgencmd get_throttled` shows the ARM clock capped. No under‑voltage.

---

## 3. Parking exit

The car starts parked against one wall. **This now runs as the first phase of
the obstacle round itself** — it is no longer a separate sketch you flash and
run by hand. Press START once and the car parks out, squares up, and goes
straight into the race.

### Sequence

| Step | Action | **Ends on** |
|---|---|---|
| 0 | **LOOK** — the same wall reading that locks the lap direction | — |
| 1 | **TURN OUT** — full lock *away from the nearer wall*, forward | **IMU**: turned 90° |
| 2 | **STRAIGHT** — hold that heading with a P term | **ENCODER**: 80 cm |
| 3 | **BACK** — reverse, holding heading | **ENCODER**: 20 cm |
| 4 | **SQUARE UP** — opposite full lock | **IMU**: back to 0° |

**Direction rule.** Nearer the **left** wall → exit **right** *and* lap
**clockwise**. Nearer the **right** → exit left, lap anticlockwise. One reading
drives both, so the exit and the lap can never disagree.

### Parameters

All seven are live on the dashboard, so the exit can be retuned or switched off
at the track without a reflash.

| | Value |
|---|---|
| `park_enabled` | 1 |
| `park_turn_deg` | 90 |
| `park_forward_cm` | 80 |
| `park_back_cm` | 20 |
| `park_steer_deg` | 50 |
| `park_speed` | 90 PWM |
| `park_max_cm` (failsafe) | 400 |

### Rules that matter

- **Every step ends on a sensor, never a clock.** Angles on the IMU, distances
  on the encoder.
- **The swings stop on a threshold CROSSING, not inside a window.** The IMU is
  read about every 20 ms while the car turns, so it steps straight over any
  window — 86 then 97 — and once past it the error only grows while the
  steering stays hard over. That is what made the ESP sketch spin in circles.
- **Both swings are bounded by distance.** Past `park_max_cm` with the heading
  not arriving, the car abandons the exit and races anyway rather than turning
  on the spot.
- **Parking owns the car outright.** No corners, no pillars, no centring until
  it finishes.
- **Handover re-zeroes the heading target** to where the car actually points —
  it is square again, but displaced sideways from where it was parked.

## 4. Obstacle round — overall movement

Three behaviours, in strict priority order.

### Priority

1. **Going around a block** — once it starts, **nothing** interrupts it
2. **Corner turns**
3. **Lane centring** — only when no pillar stage is active

### 4.1 Lane centring (straights)

Copied verbatim from `final_open_round_working.ino`:

```
correction = headingError × 1.2 + wallCentering
```

One proportional term on heading, one on the walls, **no integral**. Both walls seen → balance them; one wall → hold `cruise_target_cm` off the outer one, chosen from the locked lap direction.

| | Value |
|---|---|
| `cruise_wall_k` | 1.6 |
| `cruise_target_cm` | 42 |
| `KP_HEADING_CRUISE` | 1.2 |
| valid wall reading | < 80 cm |

> The servo has 50° of authority either side of centre. At gain 1.6 an L10/R70 lane position asks 48° of it from the wall term alone. If the car weaves or scrubs, drop this gain first.

### 4.2 Corners

**Lap direction is decided once, before the car moves**, from the parking bay:
nearer the **left** wall → **clockwise**, right turns only. Nearer the **right** → anti‑clockwise, left turns only. It never changes afterwards. Deciding it per corner from whichever side looked open produced left turns on a clockwise track whenever a sensor dropped out.

| Step | Action | Ends on |
|---|---|---|
| 1 | drive forward | front ToF ≤ **37 cm** |
| 2 | **reverse arc** — wheels **40° the opposite way**, reversing | **IMU**: all but 35° of the turn done |
| 3 | **forward turn** onto the exact heading target | **IMU** |
| 4 | **reverse 40 cm** straight, holding heading on the gyro | **ENCODER** |

The reverse arc swings the nose toward the new heading *while* backing off the wall, so one manoeuvre buys clearance and does part of the turn. The heading target is absolute (previous + 90°), so rotation gained in the arc is banked, not wasted.

> That is **75 cm of reversing per corner**, all of it blind.

### 4.3 Going around a block

Green is passed on the car's **LEFT**, Red on its **RIGHT**.

| Stage | Action | Ends on |
|---|---|---|
| APPROACH | drive on, tracking the nearest pillar | trigger distance |
| BACKOFF | if closer than 35 cm, **reverse** to 35 cm | ENCODER |
| TURN_OUT | swing out | ENCODER |
| STRAIGHTEN | run past the pillar on heading hold, ToF ignored | ENCODER |
| RECENTER | swing back in | ENCODER |
| ALIGN | square up on the original heading | IMU, min forward run |

**The turn‑out length is computed from the pillar's pixel offset**, not a two‑value arc split:

```
sideways_cm = 31.15 + toward_px × 0.08
```

`toward_px` is signed *by dodge direction* — a Green further **left** needs more, one already drifting right needs less; Red mirrors. Latched **once**, at commit, because the pillar leaves frame as the car turns.

| | Value |
|---|---|
| trigger distance | 30 cm |
| back‑off target / max | 35 cm / 45 cm |
| run past pillar | Green 62 cm, Red 57 cm |
| swing back in | turn‑out + 3 cm (+3 more for Green) |

### Rules that matter

- **Nothing interrupts a dodge** — no corner, and no front reading however short. The front sensor sweeps across the pillar being dodged and reads short every time; aborting on it killed good dodges and left the car angled across the lane.
- **Only a pillar dead ahead may claim a short front reading.** The front ToF looks straight forward, so a pillar in the LEFT or RIGHT band cannot be what it is measuring — that reading belongs to a wall, and the corner takes it.
- **Camera and ToF must agree** within 25 cm before the ToF commits a dodge. Disagreement means they are looking at different objects.
- **The nearest pillar wins**, re‑evaluated every frame with a 20 cm margin so two at similar range cannot make it flip.
- **A pillar just passed is not a new one** — the car must travel 60 cm clear before adopting another.

---

## 5. Testing without the car

Fourteen suites in `obstacle_round/pi/tools/`, no hardware needed:

```bash
python3 tools/smoke_test.py          # every dodge stage + both corner paths
python3 tools/corner_test.py         # corners land on 90°
python3 tools/parking_test.py        # direction from the parking bay
python3 tools/priority_test.py       # nothing interrupts a dodge
python3 tools/backoff_test.py        # back off to a fixed distance
```

**Run them before every deploy.** They have caught real bugs that would otherwise have reached the car — including one where a fix silently disabled dodging entirely.
