#!/usr/bin/env python3
"""parking_exit.py - stand-alone parking exit test.

Pulls the car out of a parking space, squares it up, and backs into position:

    0. LOOK        read both side ToFs and decide which way to go
    1. TURN OUT    swing PARK_TURN_DEG (70 deg) off the starting heading
    2. STRAIGHT    drive PARK_FORWARD_CM (60 cm) holding that new heading
    3. CORRECT     swing back until the heading is the ORIGINAL one again
    4. REVERSE     back up PARK_REVERSE_CM (20 cm) holding that heading

The car finishes at 0 deg relative to where it started - parallel to its
original attitude, just displaced out of the space.

DIRECTION IS AUTOMATIC. The car turns AWAY from the wall it is closest to:

    left ToF smaller  -> wall on the left  -> turn RIGHT
    right ToF smaller -> wall on the right -> turn LEFT

Angles come from the BNO055 and distances from the wheel encoder, so every
distance is real ground covered rather than a timer, and is the same on a full
battery or a flat one.

Completely separate from the obstacle round: it does not import main.py and
nothing in the round imports this. It talks straight to the ESP32 over USB, so
no reflashing is needed - the normal open_slave firmware is what it drives.

    sudo systemctl stop master.service dashboard.service   # they own the port
    python3 parking_exit/parking_exit.py                   # decide from the ToFs
    python3 parking_exit/parking_exit.py right             # force a direction
    python3 parking_exit/parking_exit.py left 70 60 20     # dir, deg, fwd, back

Safety: speed is capped at the team's MAX_PWM, every phase has a timeout, and
the forward phases abort if anything comes within ABORT_FRONT_CM ahead.

PATCH NOTES
  PATCH 1  First version: 70 deg out, 40 cm forward, correct back to square.
  PATCH 2  Direction is now chosen from the side ToFs instead of argv - the car
           turns away from whichever wall is nearer. Forward leg 40 -> 60 cm.
           Added phase 4, a 20 cm reverse, so the car ends square AND back in
           position. The reverse flips the steering sign: the same servo angle
           rotates the car the other way when the wheels are going backwards,
           so holding a heading in reverse needs -KP, not +KP.
"""
import glob
import os
import sys
import time

# config.py lives with the obstacle round. Prefer the copy next to this repo so
# the file is runnable (and testable) off the Pi, and fall back to the deployed
# path on the car.
_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(_HERE, "..", "obstacle_round", "pi"), "/home/pi/obstacle_round"):
    if os.path.isfile(os.path.join(_p, "config.py")):
        sys.path.insert(0, os.path.abspath(_p))
        break
import config

# ---------------------------------------------------------------- parameters
PARK_TURN_DEG = 70.0          # how far round to swing coming out
PARK_FORWARD_CM = 60.0        # how far to drive once out
PARK_REVERSE_CM = 20.0        # how far to back up once square again
PARK_STEER_DEG = 30.0         # servo degrees off centre for the swings
TURN_SPEED = 120              # PWM for the swings
DRIVE_SPEED = 120             # PWM for the straight
REVERSE_SPEED = 100           # PWM for the reverse - gentler, it is blind
KP_HEADING = 1.2              # servo deg per deg of heading error, holding straight

ANGLE_DONE_DEG = 5.0          # a swing is finished within this of its target
PHASE_TIMEOUT_S = 8.0
ABORT_FRONT_CM = 15.0
SIDE_SAMPLES = 12             # frames averaged before choosing a direction


def heading_error(target, current):
    """Signed smallest angle from current to target, -180..180."""
    d = target - current
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


def choose_side(left, right):
    """Which way to exit, from the two side readings.

    The car turns AWAY from the nearer wall: a wall close on the left means
    exiting to the right. Returns "left"/"right", or None when the readings
    cannot decide - both sides open, both blocked equally, or no data - in
    which case the caller must be told rather than sent somewhere arbitrary.
    """
    lv = left if (left is not None and 0 < left < config.WALL_VALID_CM) else None
    rv = right if (right is not None and 0 < right < config.WALL_VALID_CM) else None
    if lv is None and rv is None:
        return None
    if rv is None:                      # only the left sees a wall
        return "right"
    if lv is None:                      # only the right sees a wall
        return "left"
    if abs(lv - rv) < 1.0:              # equally boxed in - no answer
        return None
    return "right" if lv < rv else "left"


class Car:
    """Minimal ESP32 link: read telemetry, send speed/steer."""

    def __init__(self):
        import serial
        ports = glob.glob("/dev/serial/by-id/usb-Espressif*")
        if not ports:
            raise SystemExit("ESP32 not found - is it plugged in?")
        self.s = serial.Serial(ports[0], config.SERIAL_BAUD, timeout=0.05)
        self.s.reset_input_buffer()
        time.sleep(0.2)
        self.s.write(b"START\n")          # rule 9.14: nothing moves before START
        self.s.flush()
        time.sleep(0.4)

    def read(self):
        """Newest complete DATA line -> dict, or None values when nothing came.

        DATA,<F>,<L>,<R>,<B>,<yaw>,<sys>,<gyro>,<accel>,<mag>,<spd>,<str>,<ticks>
        """
        best = None
        for _ in range(60):
            line = self.s.readline().decode(errors="replace").strip()
            if line.startswith("DATA,"):
                f = line.split(",")
                if len(f) == 13:
                    try:
                        best = {"front": int(f[1]), "left": int(f[2]),
                                "right": int(f[3]), "yaw": float(f[5]),
                                "ticks": int(f[12])}
                    except ValueError:
                        pass
            if self.s.in_waiting == 0 and best is not None:
                break
        return best or {"front": None, "left": None, "right": None,
                        "yaw": None, "ticks": None}

    def drive(self, speed, steer):
        speed = max(-config.MAX_PWM, min(config.MAX_PWM, int(speed)))
        steer = max(config.SERVO_MIN, min(config.SERVO_MAX, int(steer)))
        self.s.write(f"{speed},{steer}\n".encode())
        self.s.flush()

    def stop(self):
        for _ in range(15):
            self.drive(0, config.SERVO_CENTER)
            time.sleep(0.03)

    def close(self):
        self.stop()
        self.s.write(b"STOP\n")
        self.s.flush()
        self.s.close()


def look(car):
    """Average a few frames of both side ToFs, ignoring dropouts."""
    lefts, rights = [], []
    for _ in range(SIDE_SAMPLES):
        d = car.read()
        if d["left"] is not None and 0 < d["left"] < config.WALL_VALID_CM:
            lefts.append(d["left"])
        if d["right"] is not None and 0 < d["right"] < config.WALL_VALID_CM:
            rights.append(d["right"])
        time.sleep(0.02)
    left = sum(lefts) / len(lefts) if lefts else None
    right = sum(rights) / len(rights) if rights else None
    return left, right


def swing(car, label, target_heading, steer_angle):
    """Turn until the heading reaches target_heading."""
    t0 = time.monotonic()
    while True:
        d = car.read()
        yaw, front = d["yaw"], d["front"]
        if yaw is None:
            continue
        err = heading_error(target_heading, yaw)
        if abs(err) <= ANGLE_DONE_DEG:
            print(f"  {label}: reached {yaw:.0f} deg (target {target_heading:.0f}, "
                  f"err {err:+.0f})")
            return True
        if front is not None and 0 < front <= ABORT_FRONT_CM:
            print(f"  {label}: ABORT - something {front} cm ahead")
            return False
        if time.monotonic() - t0 > PHASE_TIMEOUT_S:
            print(f"  {label}: timeout at {yaw:.0f} deg (err {err:+.0f})")
            return False
        car.drive(TURN_SPEED, steer_angle)
        time.sleep(0.02)


def straight(car, label, hold_heading, distance_cm, reverse=False):
    """Drive distance_cm holding hold_heading, forwards or backwards.

    Reversing flips the steering sign: the same servo angle rotates the car the
    other way once the wheels are turning backwards, so a +KP correction that
    holds a heading going forward drives it away going back.
    """
    start = car.read()
    if start["ticks"] is None:
        print(f"  {label}: no encoder data")
        return False
    start_ticks = start["ticks"]
    speed = -REVERSE_SPEED if reverse else DRIVE_SPEED
    kp = -KP_HEADING if reverse else KP_HEADING
    t0 = time.monotonic()
    while True:
        d = car.read()
        yaw, ticks, front = d["yaw"], d["ticks"], d["front"]
        if yaw is None or ticks is None:
            continue
        gone = abs(ticks - start_ticks) / config.TICKS_PER_CM
        if gone >= distance_cm:
            print(f"  {label}: {gone:.1f} cm covered, heading {yaw:.0f} deg")
            return True
        # Nothing looks backwards - the rear ToF is not fitted - so the reverse
        # is blind by necessity and only the forward legs can abort on an
        # obstacle.
        if not reverse and front is not None and 0 < front <= ABORT_FRONT_CM:
            print(f"  {label}: ABORT at {gone:.1f} cm - something {front} cm ahead")
            return False
        if time.monotonic() - t0 > PHASE_TIMEOUT_S:
            print(f"  {label}: timeout at {gone:.1f} cm")
            return False
        err = heading_error(hold_heading, yaw)
        car.drive(speed, config.SERVO_CENTER + kp * err)
        time.sleep(0.02)


def run(car, side, turn_deg, fwd_cm, back_cm):
    """The four phases. Returns 0 on success."""
    start = car.read()
    start_yaw = start["yaw"]
    if start_yaw is None:
        print("no telemetry from the ESP32")
        return 1

    if side is None:
        left, right = look(car)
        side = choose_side(left, right)
        ltxt = "-" if left is None else f"{left:.0f} cm"
        rtxt = "-" if right is None else f"{right:.0f} cm"
        if side is None:
            print(f"cannot tell which way to exit (left {ltxt}, right {rtxt}) - "
                  f"pass 'left' or 'right' explicitly")
            return 1
        print(f"  sides: left {ltxt}, right {rtxt} -> nearer wall on the "
              f"{'LEFT' if side == 'right' else 'RIGHT'}, exiting {side.upper()}")

    sign = 1 if side == "right" else -1          # +ve yaw is clockwise
    out_steer = config.SERVO_CENTER + sign * PARK_STEER_DEG
    back_steer = config.SERVO_CENTER - sign * PARK_STEER_DEG
    out_heading = (start_yaw + sign * turn_deg) % 360.0

    print(f"parking exit to the {side.upper()}: {turn_deg:.0f} deg out, "
          f"{fwd_cm:.0f} cm forward, square up, then {back_cm:.0f} cm back")
    print(f"  start heading {start_yaw:.0f} deg -> out heading {out_heading:.0f} deg")

    if not swing(car, "1 TURN OUT", out_heading, out_steer):
        return 1
    if not straight(car, "2 STRAIGHT", out_heading, fwd_cm):
        return 1
    if not swing(car, "3 CORRECT ", start_yaw, back_steer):
        return 1
    if back_cm > 0 and not straight(car, "4 REVERSE ", start_yaw, back_cm, reverse=True):
        return 1

    yaw = car.read()["yaw"]
    off = heading_error(start_yaw, yaw) if yaw is not None else float("nan")
    print(f"done - final heading {yaw:.0f} deg, {off:+.0f} deg off where it started")
    return 0


def main():
    args = [a for a in sys.argv[1:]]
    side = None
    if args and args[0].lower() in ("left", "right"):
        side = args.pop(0).lower()
    elif args and not args[0].replace(".", "").isdigit():
        raise SystemExit("direction must be 'left' or 'right', or omitted to "
                         "decide from the side sensors")
    turn_deg = float(args[0]) if len(args) > 0 else PARK_TURN_DEG
    fwd_cm = float(args[1]) if len(args) > 1 else PARK_FORWARD_CM
    back_cm = float(args[2]) if len(args) > 2 else PARK_REVERSE_CM

    car = Car()
    try:
        return run(car, side, turn_deg, fwd_cm, back_cm)
    finally:
        car.close()


if __name__ == "__main__":
    sys.exit(main())
