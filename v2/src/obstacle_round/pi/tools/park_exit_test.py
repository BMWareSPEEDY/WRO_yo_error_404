#!/usr/bin/env python3
"""The parking exit inside the obstacle round. No hardware needed.

Drives the controller with a simple kinematic model: steering off straight turns
the car, speed moves it. Checks the four steps run in order, end on the right
sensor, and hand over to the round squared up and facing the start heading.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config, main                                            # noqa: E402

fails = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not ok:
        fails.append(name)


def run(left, right, turn_deg=90.0, forward_cm=30.0, back_cm=20.0, ticks_per_deg=None):
    """Simulate a whole parking exit. Returns (controller, step log)."""
    now, yaw, ticks = 0.0, 100.0, -12345      # free-running encoder, forward counts DOWN
    d = {"left": left, "right": right, "front": 999.0, "rear": 999.0,
         "yaw": yaw, "ticks": ticks}
    ctl = main.ObstacleRound(yaw, now)
    ctl.lock_direction_from_parking(d)
    ctl._params = dict(config.DEFAULT_PARAMS,
                       park_turn_deg=turn_deg, park_forward_cm=forward_cm,
                       park_back_cm=back_cm)
    ctl.begin_parking(d, now)

    seen, guard = [], 0
    while ctl.park_step and guard < 4000:
        guard += 1
        seen.append(ctl.park_step)
        speed, steer = ctl.step(d, [], ctl._params, now)
        # Model: distance from speed, rotation from how far the servo is off straight.
        off = steer - (config.SERVO_CENTER + ctl._params["servo_trim_deg"])
        cm = abs(speed) * 0.02
        ticks -= int(round(cm * config.TICKS_PER_CM)) * (1 if speed >= 0 else -1)
        yaw = (yaw + off * cm * 0.030 * (1 if speed >= 0 else -1)) % 360.0
        d = dict(d, yaw=yaw, ticks=ticks)
        now += 0.02
    return ctl, seen, d, guard


print("\n=== parking exit, wall on the LEFT -> lap clockwise, exit RIGHT ===")
ctl, seen, d, guard = run(22.0, 70.0)
check("lock is clockwise", ctl.lock == 1, f"lock={ctl.lock}")
check("all four steps ran in order", sorted(set(seen)) == [1, 2, 3, 4], f"{sorted(set(seen))}")
check("the exit finished", ctl.park_step == 0 and guard < 4000, f"{guard} ticks")
turned = ctl.lock * main.heading_error(d["yaw"], ctl.park_hdg0)
check("squared up within 5 deg", abs(turned) < 5.0, f"{turned:.1f} deg off")
check("round target re-zeroed to where the car points",
      abs(main.heading_error(ctl.target, d["yaw"])) < 3.0,
      f"{main.heading_error(ctl.target, d['yaw']):.1f} deg")
check("and that is the heading it was parked on",
      abs(main.heading_error(ctl.target, ctl.park_hdg0)) < 5.0,
      f"{main.heading_error(ctl.target, ctl.park_hdg0):.1f} deg")
check("handed over to the round, no stage active", ctl.pillar_stage is None)

print("\n=== parking exit, wall on the RIGHT -> lap anticlockwise, exit LEFT ===")
ctl2, seen2, d2, _ = run(70.0, 22.0)
check("lock is anticlockwise", ctl2.lock == -1, f"lock={ctl2.lock}")
check("all four steps ran", sorted(set(seen2)) == [1, 2, 3, 4])
turned2 = ctl2.lock * main.heading_error(d2["yaw"], ctl2.park_hdg0)
check("squared up within 5 deg", abs(turned2) < 5.0, f"{turned2:.1f} deg off")
check("mirrored: the two exits turn opposite ways", ctl.lock == -ctl2.lock)

print("\n=== the swing stops on a CROSSING, not inside a window ===")
# The bug this guards: with a coarse IMU the heading jumps straight over the
# target (86 -> 97). A window never matches and the car circles forever; a
# threshold, once crossed, stays crossed.
ctl3 = main.ObstacleRound(0.0, 0.0)
ctl3.lock = 1
ctl3._params = dict(config.DEFAULT_PARAMS)
ctl3.park_step, ctl3.park_hdg0, ctl3.park_ticks0 = 1, 0.0, 0
ctl3.ticks = 0
out = ctl3._park_step({"yaw": 97.0, "ticks": 0}, 0.0)   # overshot the 90 target
check("overshooting the target still ends the swing", ctl3.park_step == 2,
      f"step={ctl3.park_step}")

print("\n=== failsafe: a dead IMU cannot trap the car ===")
ctl4 = main.ObstacleRound(0.0, 0.0)
ctl4.lock = 1
ctl4._params = dict(config.DEFAULT_PARAMS)
ctl4.park_step, ctl4.park_hdg0, ctl4.park_ticks0 = 1, 0.0, 0
ctl4.ticks = int(500 * config.TICKS_PER_CM)             # 500 cm with no rotation
out = ctl4._park_step({"yaw": 0.0, "ticks": ctl4.ticks}, 0.0)
check("gives up and races anyway", out is None and ctl4.park_step == 0)

print("\n=== the defaults still match parking_exit_v2.ino ===")
import re
# tools -> pi -> obstacle_round -> repo root. Not present on the Pi, where only
# obstacle_round/pi is deployed, so this section skips there rather than failing.
ino = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))), "parking_exit", "esp", "parking_exit_v2", "parking_exit_v2.ino")
if os.path.exists(ino):
    src = open(ino).read()
    def define(name):
        return float(re.search(r"#define\s+%s\s+([0-9.]+)" % name, src).group(1))
    # forward_cm is deliberately shorter than the sketch's: the Pi version only
    # needs to clear the bay, not cross the lane.
    for cfg, dfn in (("park_turn_deg", "TURN_DEG"), ("park_back_cm", "BACK_CM"),
                     ("park_speed", "SPEED"), ("park_steer_deg", "STEER_LOCK")):
        check(f"{cfg} matches {dfn}",
              float(config.DEFAULT_PARAMS[cfg]) == define(dfn),
              f"{config.DEFAULT_PARAMS[cfg]} vs {define(dfn):.0f}")
    check("park_forward_cm is the deliberate 30 cm, not the sketch's 80",
          float(config.DEFAULT_PARAMS["park_forward_cm"]) == 30.0,
          f"{config.DEFAULT_PARAMS['park_forward_cm']}")
else:
    print("  SKIP  sketch not found")

print("\n=== park_enabled = 0 skips it ===")
ctl5 = main.ObstacleRound(0.0, 0.0)
ctl5.lock = 1
ctl5._params = dict(config.DEFAULT_PARAMS, park_enabled=0)
ctl5.begin_parking({"left": 22.0, "right": 70.0, "yaw": 0.0, "ticks": 0}, 0.0)
check("never enters the exit", ctl5.park_step == 0)

print("\n" + ("ALL PASS" if not fails else f"{len(fails)} FAILED: {', '.join(fails)}"))
sys.exit(1 if fails else 0)
