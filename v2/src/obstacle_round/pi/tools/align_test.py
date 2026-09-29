"""The dodge must not finish while the car is still beside the pillar."""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M
P = config.load_params()
print("  dodge_align_min_red_cm =", P.get("dodge_align_min_red_cm"),
      " green =", P.get("dodge_align_min_green_cm"))

def block(colour, dist, cx):
    return {"class": colour, "dist_cm": dist, "confidence": 0.9, "cx": cx,
            "x1": cx-40, "x2": cx+40, "y1": 200, "y2": 300}

ok = True
for colour in ("Red", "Green"):
    ctl = M.ObstacleRound(0.0, 0.0); ctl.state = "RUNNING"
    ctl.pillar_clear_t = -1e9          # no post-dodge cooldown pending
    t=[0.0]; ticks=[0]; dist=90.0
    exit_cm = None
    for i in range(6000):
        t[0] += 0.02
        b = [block(colour, dist, 512)] if dist > 4 else []
        d = {"front":999.0,"left":45.0,"right":45.0,"rear":999.0,"yaw":0.0,"ticks":ticks[0]}
        prev = ctl.pillar_stage
        speed, steer = ctl.step(d, b, P, t[0])
        if prev == "ALIGN" and ctl.pillar_stage is None:
            exit_cm = ctl.stage_cm
            break
        if ctl.pillar_stage in (None, "APPROACH"):
            dist = max(3.0, dist - 1.0)
        ticks[0] += int(speed * 0.30 * 0.02 * config.TICKS_PER_CM)
    want = P.get("dodge_align_min_%s_cm" % colour.lower(), 0.0)
    got = -1 if exit_cm is None else exit_cm
    good = exit_cm is not None and exit_cm + 0.7 >= want
    print("  %-6s squared up after %5.1f cm (minimum %.1f)  %s"
          % (colour, got, want, "ok" if good else "TOO EARLY"))
    ok &= good
assert ok, "a dodge ended before clearing the pillar"
print("  MINIMUM RUN BEFORE THE DODGE ENDS - correct")
