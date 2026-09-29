"""A block picked up too close is reversed away from before being dodged."""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M
P = config.load_params()


def blk(dist, cx=400):
    return {"class": "Green", "dist_cm": float(dist), "confidence": 0.95, "cx": cx,
            "x1": cx - 45, "x2": cx + 45, "y1": 200, "y2": 300, "clipped": False}


def run(start_dist):
    """Pick the block up at `start_dist`; reversing increases its distance."""
    ctl = M.ObstacleRound(0.0, 0.0); ctl.state = "RUNNING"
    ctl.pillar_clear_t = -1e9; ctl.lock = 1; ctl.turns = 1
    ctl.turn_end_ticks = -999999
    t = 0.0; ticks = 0; seq = 0
    dist = float(start_dist)
    stages, reversed_cm = [], 0.0
    for i in range(4000):
        t += 0.02
        if i % 3 == 0:
            seq += 1
        ctl.vision_seq = seq
        d = {"front": 999.0, "left": 45.0, "right": 45.0, "rear": 999.0,
             "yaw": 0.0, "ticks": ticks}
        speed, steer = ctl.step(d, [blk(dist)], P, t)
        if ctl.pillar_stage and (not stages or stages[-1] != ctl.pillar_stage):
            stages.append(ctl.pillar_stage)
        cm = speed * 0.30 * 0.02
        ticks += int(cm * config.TICKS_PER_CM)
        # the block's apparent distance tracks the car: reversing opens the gap
        dist = max(4.0, dist - cm)
        if speed < 0:
            reversed_cm += -cm
        if ctl.pillar_stage == "BACKOFF":
            run.at_backoff_end = dist
        if ctl.pillar_stage in ("TURN_OUT", "STRAIGHTEN"):
            return stages, reversed_cm, dist
    return stages, reversed_cm, dist


print("  target %.0f cm, max reverse %.0f cm\n"
      % (config.BACKOFF_TARGET_CM, config.BACKOFF_MAX_CM))
ok = True

run.at_backoff_end = 0.0
stages, rev, at = run(18)
# what matters is that the BACK-OFF reached the target, not where the dodge
# then fired - the car drives forward again afterwards.
good = ("BACKOFF" in stages and rev > 5
        and run.at_backoff_end >= config.BACKOFF_TARGET_CM - 2)
ok &= good
print("  picked up at 18 cm -> %s" % " -> ".join(stages))
print("     reversed %.0f cm, reached %.0f cm before approaching again   %s"
      % (rev, run.at_backoff_end, "correct" if good else "WRONG"))

# Picked up well outside the activation distance. It still backs off, because
# the trigger needs several confirming frames and the car keeps closing during
# them - so it crosses the activation line before committing. That is exactly
# what the fixed starting distance is for.
stages, rev, at = run(60)
good = stages.count("BACKOFF") <= config.BACKOFF_MAX_TRIES and "TURN_OUT" in stages
ok &= good
print("  picked up at 60 cm -> %s" % " -> ".join(stages))
print("     reversed %.0f cm, %d back-off(s), max allowed %d   %s"
      % (rev, stages.count("BACKOFF"), config.BACKOFF_MAX_TRIES,
         "correct - bounded, and it still dodges" if good else "WRONG"))

assert ok
print("  BACK OFF TO A FIXED DISTANCE BEFORE DODGING - correct")
