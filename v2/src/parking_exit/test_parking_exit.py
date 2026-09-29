#!/usr/bin/env python3
"""Offline test for parking_exit.py - no car, no serial port, no ESP32.

Drives the real phase logic against a simple kinematic stand-in so the
sequence, the direction choice and the reverse steering sign can be checked
without the robot powered up.

    python3 parking_exit/test_parking_exit.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import parking_exit as P
import config

time.sleep = lambda *_: None          # no real-time waits
time.monotonic = lambda: FakeCar.now  # phase timeouts follow simulated time


class FakeCar:
    """Bicycle-ish model: steering off centre rotates, speed moves."""
    now = 0.0

    def __init__(self, left, right, yaw=10.0):
        self.left, self.right = left, right
        self.yaw, self.ticks = yaw, 0
        self.cmds = []
        FakeCar.now = 0.0

    def read(self):
        return {"front": 200, "left": self.left, "right": self.right,
                "yaw": self.yaw % 360.0, "ticks": self.ticks}

    def drive(self, speed, steer):
        speed = max(-config.MAX_PWM, min(config.MAX_PWM, int(speed)))
        steer = max(config.SERVO_MIN, min(config.SERVO_MAX, int(steer)))
        self.cmds.append((speed, steer))
        dt = 0.02
        FakeCar.now += dt
        cm = speed * 0.30 * dt                       # ~36 cm/s at 120 PWM
        self.ticks += int(cm * config.TICKS_PER_CM)
        # wheels turned + moving -> rotate; reversing rotates the other way
        self.yaw += (steer - config.SERVO_CENTER) * 0.22 * (1 if cm >= 0 else -1) * (abs(cm) / 0.72)

    def stop(self):
        pass


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (("  " + detail) if detail else ""))
    return cond


ok = True

# --- direction choice, the rule the user stated -----------------------------
print("choose_side - turn AWAY from the nearer wall")
for left, right, want in ((15, 60, "right"), (60, 15, "left"), (15, 999, "right"),
                          (999, 15, "left"), (999, 999, None), (30, 30, None)):
    got = P.choose_side(left, right)
    ok &= check(f"left={left:<4} right={right:<4} -> {got}", got == want, f"(expected {want})")

# --- full sequence, left wall -> exits right --------------------------------
print("\nfull run with a wall on the LEFT (15 cm) - should exit RIGHT")
car = FakeCar(left=15, right=60, yaw=10.0)
rc = P.run(car, None, P.PARK_TURN_DEG, P.PARK_FORWARD_CM, P.PARK_REVERSE_CM)
ok &= check("completed", rc == 0)
ok &= check("ends square with the start heading",
            abs(P.heading_error(10.0, car.yaw)) <= P.ANGLE_DONE_DEG + 1,
            f"(final {car.yaw % 360:.1f} deg)")
ok &= check("turned RIGHT out of the space (steer > centre first)",
            car.cmds[0][1] > config.SERVO_CENTER, f"(first steer {car.cmds[0][1]})")
rev = [c for c in car.cmds if c[0] < 0]
ok &= check("finished with a reverse phase", len(rev) > 0, f"({len(rev)} reverse commands)")
ok &= check("reverse never exceeded the PWM cap",
            all(abs(s) <= config.MAX_PWM for s, _ in car.cmds))
ok &= check("steering stayed inside the servo stops",
            all(config.SERVO_MIN <= st <= config.SERVO_MAX for _, st in car.cmds))

# --- mirror image ------------------------------------------------------------
print("\nfull run with a wall on the RIGHT (15 cm) - should exit LEFT")
car = FakeCar(left=60, right=15, yaw=350.0)
rc = P.run(car, None, P.PARK_TURN_DEG, P.PARK_FORWARD_CM, P.PARK_REVERSE_CM)
ok &= check("completed", rc == 0)
ok &= check("turned LEFT out of the space (steer < centre first)",
            car.cmds[0][1] < config.SERVO_CENTER, f"(first steer {car.cmds[0][1]})")
ok &= check("ends square with the start heading",
            abs(P.heading_error(350.0, car.yaw)) <= P.ANGLE_DONE_DEG + 1,
            f"(final {car.yaw % 360:.1f} deg)")

# --- refuses to guess --------------------------------------------------------
print("\nno usable side readings - must refuse rather than pick one")
car = FakeCar(left=999, right=999)
ok &= check("returns an error", P.run(car, None, 70, 60, 20) == 1)

print("\n" + ("ALL PARKING TESTS PASSED" if ok else "SOME TESTS FAILED"))
sys.exit(0 if ok else 1)
