"""One bad camera frame must not be able to fire a dodge."""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M
P = config.load_params()
print("  DODGE_TRIGGER_FRAMES =", config.DODGE_TRIGGER_FRAMES,
      " trigger dist =", P.get("pillar_trigger_dist_cm"), "cm")

def block(colour, dist, cx=512):
    return {"class": colour, "dist_cm": dist, "confidence": 0.9, "cx": cx,
            "x1": cx-40, "x2": cx+40, "y1": 200, "y2": 300}

TRIG = float(P.get("pillar_trigger_dist_cm", 35.0))


def run(bad_frames, far=114.0, close=None):
    """Pillar really sits at `far`. `bad_frames` distinct frames misreport `close`.

    Control loop 50 Hz, camera ~15 fps, so a new frame every third tick - the
    exact ratio that let three control ticks pass inside one camera frame.
    """
    # Just inside the live activation distance, whatever it is set to.
    close = (TRIG - 1.0) if close is None else close
    ctl = M.ObstacleRound(0.0, 0.0); ctl.state = "RUNNING"
    ctl.pillar_clear_t = -1e9
    t = [0.0]; ticks = 0; seq = 0; dist = far
    adopt_frames, bad_left = 6, bad_frames
    for i in range(600):
        t[0] += 0.02
        if i % 3 == 0:                       # a NEW camera frame
            seq += 1
            if adopt_frames > 0:
                adopt_frames -= 1
                dist = far
            elif bad_left > 0:
                bad_left -= 1
                dist = close
            else:
                dist = far
        ctl.vision_seq = seq
        d = {"front": 999.0, "left": 45.0, "right": 45.0, "rear": 999.0,
             "yaw": 0.0, "ticks": ticks}
        ctl.step(d, [block("Red", dist)], P, t[0])
        if ctl.pillar_stage not in (None, "APPROACH"):
            return True
        ticks += int(90 * 0.30 * 0.02 * config.TICKS_PER_CM)
    return False

ok = True
for n, expect in ((1, False), (2, False), (3, True), (6, True)):
    fired = run(n)
    good = fired == expect
    ok &= good
    print("  %d bad frame(s) just inside the trigger (pillar at 114): %-8s %s"
          % (n, "dodged" if fired else "ignored",
             "correct" if good else "WRONG - expected %s" % ("dodge" if expect else "ignore")))
assert ok, "the dodge trigger is not counting distinct camera frames"
print("  TRIGGER NEEDS %d DISTINCT CAMERA FRAMES - correct" % config.DODGE_TRIGGER_FRAMES)
