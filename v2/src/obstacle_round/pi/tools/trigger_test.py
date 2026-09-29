"""Corners must all fire at the same front distance."""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M
P = config.load_params()
trig = P.get("front_turn_cm")
print("  front_turn_cm =", trig)
fired = []
for openside in ("right", "left", "none"):
    ctl = M.ObstacleRound(0.0, 0.0); ctl.state = "RUNNING"
    ctl.lock = 1; ctl.turns = 1; ctl.turn_end_ticks = -99999
    t = [0.0]; ticks = [0]
    f = 90.0
    for i in range(3000):
        t[0] += 0.02
        left  = 999.0 if openside == "left"  else 40.0
        right = 999.0 if openside == "right" else 40.0
        d = {"front": f, "left": left, "right": right, "rear": 999.0,
             "yaw": 0.0, "ticks": ticks[0]}
        ctl._params = P
        ctl.ticks = ticks[0]; ctl.ticks_ok = True
        if ctl._should_turn(d, t[0]):
            fired.append((openside, f)); break
        ticks[0] += int(0.6 * config.TICKS_PER_CM)
        f = max(5.0, f - 0.6)
for side, f in fired:
    print("   open %-6s -> corner fired at front %.0f cm" % (side, f))
ds = [f for _, f in fired]
assert ds and max(ds) - min(ds) <= 2.0, "corners still fire at different distances: %s" % ds
print("  ALL CORNERS FIRE AT THE SAME DISTANCE - correct")
