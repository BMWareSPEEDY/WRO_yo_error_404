"""Direction comes from the parking wall, and a dodge is one whole manoeuvre."""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M
P = config.load_params()

def blk(colour, dist, cx=512):
    return {"class": colour, "dist_cm": dist, "confidence": 0.9, "cx": cx,
            "x1": cx-45, "x2": cx+45, "y1": 200, "y2": 300}

print("  --- lap direction from where the car is parked ---")
ok = True
for left, right, want, why in ((22, 70, 1, "against the LEFT wall -> clockwise, RIGHT turns"),
                               (70, 22, -1, "against the RIGHT wall -> anti-clockwise, LEFT turns"),
                               (30, 999, 1, "left wall seen, right open -> clockwise"),
                               (999, 30, -1, "right wall seen, left open -> anti-clockwise")):
    ctl = M.ObstacleRound(0.0, 0.0); ctl._params = P
    ctl.lock_direction_from_parking({"left": float(left), "right": float(right),
                                     "front": 999.0, "rear": 999.0, "yaw": 0.0})
    good = ctl.lock == want
    ok &= good
    print("   L%-4s R%-4s -> %-4s %s (%s)"
          % (left, right, {1: "CW", -1: "CCW", 0: "none"}[ctl.lock],
             "correct" if good else "WRONG", why))

print("\n  --- the direction never changes afterwards ---")
ctl = M.ObstacleRound(0.0, 0.0); ctl._params = P
ctl.lock_direction_from_parking({"left": 22.0, "right": 70.0, "front": 999.0,
                                 "rear": 999.0, "yaw": 0.0})
start = ctl.lock
t = 0.0; ticks = 0
for i in range(600):                    # drive past an "open left" that would have flipped it
    t += 0.02
    ctl.ticks = ticks; ctl.ticks_ok = True
    d = {"front": 40.0, "left": 999.0, "right": 30.0, "rear": 999.0, "yaw": 0.0, "ticks": ticks}
    ctl._params = P
    ctl._should_turn(d, t)
    ticks += int(60 * config.TICKS_PER_CM / 50)
same = ctl.lock == start
ok &= same
print("   locked %s, after 600 ticks of an open LEFT it is still %s  %s"
      % ({1: "CW", -1: "CCW"}[start], {1: "CW", -1: "CCW", 0: "none"}[ctl.lock],
         "correct" if same else "WRONG - it flipped"))

print("\n  --- a dodge runs to completion, a corner does not cut in ---")
ctl = M.ObstacleRound(0.0, 0.0); ctl.state = "RUNNING"
ctl.pillar_clear_t = -1e9
ctl.lock_direction_from_parking({"left": 22.0, "right": 70.0, "front": 999.0,
                                 "rear": 999.0, "yaw": 0.0})
ctl.turns = 1; ctl.turn_end_ticks = -999999
t = 0.0; ticks = 0; seq = 0; dist = 80.0; stages = []
interrupted = False
for i in range(4000):
    t += 0.02
    if i % 3 == 0:
        seq += 1
        if ctl.pillar_stage in (None, "APPROACH"):
            dist = max(8.0, dist - 1.2)
    ctl.vision_seq = seq
    started = ctl.pillar_stage in ("TURN_OUT", "STRAIGHTEN", "RECENTER", "ALIGN")
    # a wall sits at 30 cm the whole time the dodge is running - enough to fire
    # a corner, not enough to be a genuine crash risk
    d = {"front": 30.0 if started else 999.0, "left": 45.0, "right": 45.0,
         "rear": 999.0, "yaw": 0.0, "ticks": ticks}
    ctl.step(d, [blk("Green", dist, 380)], P, t)
    if ctl.pillar_stage and (not stages or stages[-1] != ctl.pillar_stage):
        stages.append(ctl.pillar_stage)
    if started and (ctl.is_turning or ctl.pre_turn_reverse):
        interrupted = True; break
    if stages and ctl.pillar_stage is None and "ALIGN" in stages:
        break
    ticks += int(110 * 0.30 * 0.02 * config.TICKS_PER_CM)
print("   stages reached:", " -> ".join(stages))
done = "ALIGN" in stages and not interrupted
ok &= done
print("   %s" % ("correct - ran through to squaring up without being cut off" if done
                else "WRONG - a corner interrupted the dodge"))

assert ok
print("\n  PARKING DIRECTION + ATOMIC DODGE - correct")
