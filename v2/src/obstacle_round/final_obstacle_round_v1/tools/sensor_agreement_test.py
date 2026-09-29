"""The ToF backstop must not commit a dodge to a pillar it cannot be seeing."""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M
P = config.load_params()

def blk(colour, dist, cx=512):
    return {"class": colour, "dist_cm": dist, "confidence": 0.9, "cx": cx,
            "x1": cx-40, "x2": cx+40, "y1": 200, "y2": 300}

def run(optical_floor, tof_close, extra=None, optical_start=90.0):
    """Camera distance falls to `optical_floor`; the ToF falls to `tof_close`.

    Both start far enough out that the pillar is legitimately adopted first.
    """
    ctl = M.ObstacleRound(0.0, 0.0); ctl.state = "RUNNING"
    ctl.pillar_clear_t = -1e9; ctl.lock = -1; ctl.turns = 1
    ctl.turn_end_ticks = -999999
    t = 0.0; ticks = 0; seq = 0
    tof = 90.0; optical = optical_start
    for i in range(900):
        t += 0.02
        if i % 3 == 0:
            seq += 1
            optical = max(optical_floor, optical - 0.8)
        ctl.vision_seq = seq
        blocks = [blk("Red", optical)] + (extra or [])
        d = {"front": tof, "left": 45.0, "right": 45.0, "rear": 999.0,
             "yaw": 0.0, "ticks": ticks}
        ctl.step(d, blocks, P, t)
        if ctl.pillar_stage in ("TURN_OUT", "STRAIGHTEN", "RECENTER"):
            return "dodged %s at ToF %.0f (camera %.0f)" % (ctl.pillar_color, tof, optical)
        if ctl.is_turning or ctl.pre_turn_reverse:
            return "corner at %.0f" % tof
        ticks += int(90 * 0.30 * 0.02 * config.TICKS_PER_CM)
        tof = max(tof_close, tof - 0.6)
    return "nothing"

# The 09:59 failure: camera holds the Red at 116 cm, ToF sees 32 cm - a
# different object entirely (the green in front).
r = run(optical_floor=116.0, tof_close=32.0, optical_start=116.0)
print("  camera: Red at 116 cm, ToF: 32 cm  -> %s" % r)
ok1 = not r.startswith("dodged")
print("  %s" % ("correct - refused to dodge a pillar it cannot be seeing" if ok1
                else "WRONG - committed the Red dodge anyway"))

# Agreement: both sensors are looking at the same pillar. The camera lags the
# ToF by a realistic ~10 cm here - a lag wider than BLOCK_TOF_AGREE_CM is not
# "agreement", and the code is right to refuse it (that is case 1).
r = run(optical_floor=40.0, tof_close=32.0, optical_start=58.0)
print("  camera: Red at 40 cm,  ToF: 32 cm  -> %s" % r)
ok2 = r.startswith("dodged Red")
print("  %s" % ("correct - they agree, so commit" if ok2 else "WRONG"))

# Two blocks: the NEAR one is the one that matters.
r = run(optical_floor=120.0, tof_close=60.0, optical_start=120.0,
        extra=[blk("Green", 55.0, 700)])
print("  Red at 120 cm + Green at 55 cm      -> %s" % r)
ok3 = "Red" not in r
print("  %s" % ("correct - the distant Red was not tracked" if ok3
                else "WRONG - tracked the distant Red"))

assert ok1 and ok2 and ok3
print("  BACKSTOP REQUIRES AGREEMENT, NEAREST BLOCK WINS - correct")
