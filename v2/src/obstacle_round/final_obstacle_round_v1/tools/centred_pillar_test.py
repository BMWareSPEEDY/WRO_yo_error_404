"""An off-centre pillar must not take a corner's front reading."""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M
P = config.load_params()
W = config.CAMERA_SIZE[0]

def blk(colour, dist, cx):
    return {"class": colour, "dist_cm": dist, "confidence": 0.9, "cx": cx,
            "x1": cx-45, "x2": cx+45, "y1": 200, "y2": 300}

def run(cx):
    """Track a Green at `cx` while the front ToF closes on a wall."""
    ctl = M.ObstacleRound(0.0, 0.0); ctl.state = "RUNNING"
    ctl.pillar_clear_t = -1e9; ctl.lock = 1; ctl.turns = 1
    ctl.turn_end_ticks = -999999
    t = 0.0; ticks = 0; seq = 0; tof = 90.0; optical = 90.0
    for i in range(1500):
        t += 0.02
        if i % 3 == 0:
            seq += 1
            optical = max(48.0, optical - 0.55)
        ctl.vision_seq = seq
        d = {"front": tof, "left": 22.0, "right": 33.0, "rear": 999.0,
             "yaw": 0.0, "ticks": ticks}
        ctl.step(d, [blk("Green", optical, cx)], P, t)
        if ctl.is_turning or ctl.pre_turn_reverse:
            return "corner at %.0f" % tof
        if ctl.pillar_stage in ("TURN_OUT", "STRAIGHTEN", "RECENTER"):
            return "dodge at %.0f" % tof
        ticks += int(90 * 0.30 * 0.02 * config.TICKS_PER_CM)
        tof = max(10.0, tof - 0.55)
    return "nothing"

ok = True
r = run(cx=180)                      # hard LEFT band - the 10:17 case
print("  Green in the LEFT band, wall closing  -> %s" % r)
g1 = r.startswith("corner"); ok &= g1
print("  %s" % ("correct - the turn goes first" if g1 else "WRONG - the pillar stole the corner"))

r = run(cx=W - 180)                  # hard RIGHT band
print("  Green in the RIGHT band, wall closing -> %s" % r)
g2 = r.startswith("corner"); ok &= g2
print("  %s" % ("correct - the turn goes first" if g2 else "WRONG"))

r = run(cx=W // 2)                   # dead ahead - this one IS the pillar
print("  Green dead ahead (CENTER)             -> %s" % r)
g3 = r.startswith("dodge"); ok &= g3
print("  %s" % ("correct - dodged, the ToF really is seeing it" if g3 else "WRONG"))

assert ok
print("  OFF-CENTRE PILLARS DO NOT STEAL THE CORNER - correct")
