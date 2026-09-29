"""The correct-side skip, and the ToF vote on high readings."""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M
P = config.load_params()
W = config.CAMERA_SIZE[0]

def blk(colour, dist, cx, half=40):
    return {"class": colour, "dist_cm": dist, "confidence": 0.9, "cx": cx,
            "x1": cx-half, "x2": cx+half, "y1": 200, "y2": 300}

print("  --- correct-side skip (PILLAR_SKIP_WHEN_ALREADY_CLEAR=%s) ---"
      % config.PILLAR_SKIP_WHEN_ALREADY_CLEAR)
ok = True
cases = [
    ("Red",   "hard LEFT, close",   40.0, 90,     True,  "passes on the left already"),
    ("Green", "hard RIGHT, close",  40.0, W-90,   True,  "passes on the right already"),
    ("Red",   "hard RIGHT, close",  40.0, W-90,   False, "wrong side - must dodge"),
    ("Green", "hard LEFT, close",   40.0, 90,     False, "wrong side - must dodge"),
    ("Red",   "LEFT but far",      100.0, 90,     False, "too far to judge yet"),
    ("Red",   "LEFT band, central", 40.0, W//2-60,False, "drifted into band, still ahead"),
]
for colour, what, dist, cx, want, why in cases:
    got = M.already_clear(blk(colour, dist, cx), colour, P)
    good = got == want
    ok &= good
    print("   %-5s %-18s -> %-5s %s (%s)" % (colour, what, "skip" if got else "dodge",
                                             "correct" if good else "WRONG", why))
assert ok, "correct-side skip misjudged a case"

print("  --- ToF vote: a lone high reading must not read as an open side ---")
ctl = M.ObstacleRound(0.0, 0.0)
t = 0.0
for i in range(40):                       # steady wall at 40 cm
    t += 0.02
    ctl._deglitch_tof({"front":200.0,"left":40.0,"right":40.0,"rear":999.0}, t)
t += 0.02
d = ctl._deglitch_tof({"front":200.0,"left":999.0,"right":40.0,"rear":999.0}, t)
print("   one 999 after a steady 40 cm wall -> left reads %s  %s"
      % (d["left"], "correct" if d["left"] < 999 else "WRONG - passed straight through"))
assert d["left"] < 999, "a single spurious 999 reached the driving logic"

for i in range(20):                       # sustained: a real gap
    t += 0.02
    d = ctl._deglitch_tof({"front":200.0,"left":999.0,"right":40.0,"rear":999.0}, t)
print("   sustained 999 (a real corner)   -> left reads %s  %s"
      % (d["left"], "correct" if d["left"] >= 999 else "WRONG - real gap suppressed"))
assert d["left"] >= 999, "a genuine open side never got through"
print("  NEW BEHAVIOUR CORRECT")
