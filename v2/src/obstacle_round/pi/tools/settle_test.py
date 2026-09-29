"""A corner arriving right after a dodge must not read the sides for direction."""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M
P = config.load_params()
print("  LOCK_AFTER_DODGE_CM =", config.LOCK_AFTER_DODGE_CM,
      " default_lap_dir =", P.get("default_lap_dir"))

def block(colour, dist, cx=512):
    return {"class": colour, "dist_cm": dist, "confidence": 0.9, "cx": cx,
            "x1": cx-40, "x2": cx+40, "y1": 200, "y2": 300}

def scenario(cm_since_dodge):
    """Replays 09:24:35 - a Red dodge just ended, L999 R40, corner at 37 cm."""
    ctl = M.ObstacleRound(0.0, 0.0); ctl.state = "RUNNING"
    ctl.lock = 0; ctl.turns = 0
    ctl.ticks_ok = True
    ctl.ticks = int(cm_since_dodge * config.TICKS_PER_CM)
    ctl.dodge_end_ticks = 0
    ctl.turn_end_ticks = -999999
    ctl._params = P
    d = {"front": 37.0, "left": 999.0, "right": 40.0, "rear": 999.0,
         "yaw": 0.0, "ticks": ctl.ticks}
    fired = ctl._should_turn(d, 100.0)
    return fired, ctl.lock

ok = True
fired, lock = scenario(42.0)          # what actually happened: ~1.4 s after the dodge
name = {0: "none", 1: "CW (right)", -1: "CCW (left)"}[lock]
good = lock == 1
ok &= good
print("  42 cm after a Red dodge, L999 R40 -> %s  %s"
      % (name, "correct: used the declared direction" if good else "WRONG: read the dodge displacement"))

fired, lock = scenario(120.0)         # well clear of the dodge, sides are real
name = {0: "none", 1: "CW (right)", -1: "CCW (left)"}[lock]
good = lock == -1
ok &= good
print("  120 cm after the dodge, L999 R40 -> %s  %s"
      % (name, "correct: a genuinely open left wins" if good else "WRONG"))

assert ok, "the settle gate is not protecting the forced override"
print("  POST-DODGE READINGS NO LONGER CHOOSE THE DIRECTION - correct")
