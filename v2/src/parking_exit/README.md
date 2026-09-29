# Parking exit

Stand-alone test: pull the car out of a parking space and square it up again.

| Phase | What it does | Ends on |
|---|---|---|
| 0. LOOK | Read both side ToFs and pick a direction | — |
| 1. TURN OUT | Swing 70° off the starting heading | **BNO055 angle** |
| 2. STRAIGHT | Drive 60 cm holding that new heading | **Encoder distance** |
| 3. CORRECT | Swing back to the original heading | **BNO055 angle** |
| 4. REVERSE | Back up 20 cm holding that heading | **Encoder distance** |

The car ends at **0° relative to where it started** — parallel to its original
attitude, displaced out of the space.

## Which way it goes

Automatic: the car turns **away from whichever wall is nearer**.

| Reading | Meaning | Exit |
|---|---|---|
| left ToF smaller | wall on the left | **RIGHT** |
| right ToF smaller | wall on the right | **LEFT** |
| neither side sees a wall | nothing to go on | refuses, asks for an explicit direction |

Pass `left` or `right` as the first argument to override it.

Angles come from the IMU and the distance from the wheel encoder, so the 40 cm
is real ground covered rather than a timer — the same on a full battery or a
flat one.

## Running it

`dashboard.py` owns the serial port, so stop the services first:

```bash
ssh pi@192.168.1.53
sudo systemctl stop master.service dashboard.service
cd ~/obstacle_round
python3 parking_exit.py                 # decide from the side sensors
python3 parking_exit.py right           # force a direction
python3 parking_exit.py left 70 60 20   # direction, degrees, forward, reverse
sudo systemctl start dashboard.service master.service
```

It drives the normal `open_slave` firmware over USB, so **nothing needs
reflashing**.

## Separate on purpose

This does not import `main.py`, and nothing in the obstacle round imports this.
Changing one cannot break the other. It does read `obstacle_round/pi/config.py`
for the shared hardware facts — servo limits, the 170 PWM cap, and
`TICKS_PER_METER` — so a recalibration is picked up automatically rather than
drifting out of step.

## Tuning

At the top of `parking_exit.py`:

| Constant | Default | Meaning |
|---|---|---|
| `PARK_TURN_DEG` | 70° | how far round to swing coming out |
| `PARK_FORWARD_CM` | 60 cm | how far to drive once out |
| `PARK_REVERSE_CM` | 20 cm | how far to back up once square again |
| `PARK_STEER_DEG` | 30° | servo degrees off centre for the swings |
| `TURN_SPEED` / `DRIVE_SPEED` | 120 | PWM (capped at `config.MAX_PWM`) |
| `REVERSE_SPEED` | 100 | PWM for the blind reverse |
| `ANGLE_DONE_DEG` | 5° | how close counts as "reached" |

Speed is capped at the team limit of 170 PWM, every phase has an 8 s timeout,
and the run aborts if anything comes within 15 cm ahead.

## Testing it without the car

`test_parking_exit.py` drives the real phase logic against a kinematic
stand-in — no serial port, no ESP32, no power:

```bash
python3 parking_exit/test_parking_exit.py
```

It checks the direction rule in both mirror images, that the car ends square
with its starting heading, that the run finishes with a reverse, and that speed
and steering stay inside the limits.
