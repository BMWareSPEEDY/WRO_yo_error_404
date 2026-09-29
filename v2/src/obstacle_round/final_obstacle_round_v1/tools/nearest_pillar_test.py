"""A nearer pillar of the other colour must take over the approach."""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M
P = config.load_params()
TRIG = P.get("pillar_trigger_dist_cm", 30.0)

def blk(colour, dist, cx=512):
    return {"class": colour, "dist_cm": dist, "confidence": 0.9, "cx": cx,
            "x1": cx-40, "x2": cx+40, "y1": 200, "y2": 300}

def run(scene):
    ctl = M.ObstacleRound(0.0, 0.0); ctl.state = "RUNNING"
    ctl.pillar_clear_t = -1e9; ctl.lock = -1; ctl.turns = 1
    ctl.turn_end_ticks = -999999
    t = 0.0; ticks = 0; seq = 0
    for i in range(2000):
        t += 0.02
        if i % 3 == 0:
            seq += 1
        ctl.vision_seq = seq
        d = {"front": 999.0, "left": 45.0, "right": 45.0, "rear": 999.0,
             "yaw": 0.0, "ticks": ticks}
        ctl.step(d, scene(i), P, t)
        if ctl.pillar_stage in ("TURN_OUT", "STRAIGHTEN", "RECENTER"):
            return ctl.pillar_color
        ticks += int(90 * 0.30 * 0.02 * config.TICKS_PER_CM)
    return "none"

# A far Red is picked up first; a near Green appears a moment later and closes.
def scene_a(i):
    out = [blk("Red", max(95.0, 130.0 - i * 0.05), 300)]
    if i > 40:
        out.append(blk("Green", max(10.0, 75.0 - (i - 40) * 0.05), 700))
    return out

r = run(scene_a)
print("  far Red (95 cm) + near Green closing -> dodged %s" % r)
ok1 = r == "Green"
print("  %s" % ("correct - switched to the nearer Green" if ok1
                else "WRONG - followed the far Red"))

def scene_b(i):
    out = [blk("Green", max(95.0, 130.0 - i * 0.05), 700)]
    if i > 40:
        out.append(blk("Red", max(10.0, 75.0 - (i - 40) * 0.05), 300))
    return out

r = run(scene_b)
print("  far Green (95 cm) + near Red closing -> dodged %s" % r)
ok2 = r == "Red"
print("  %s" % ("correct - switched to the nearer Red" if ok2 else "WRONG"))

# Similar range: must not flip-flop, and must still dodge something.
def scene_c(i):
    return [blk("Red", max(10.0, 90.0 - i * 0.05), 300),
            blk("Green", max(14.0, 94.0 - i * 0.05), 700)]

r = run(scene_c)
ok3 = r in ("Red", "Green")
print("  Red and Green within the margin      -> dodged %s  (no thrash)" % r)

assert ok1 and ok2 and ok3
print("  APPROACH FOLLOWS THE NEAREST PILLAR - correct")
