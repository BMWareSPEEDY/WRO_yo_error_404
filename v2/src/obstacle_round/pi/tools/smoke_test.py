"""Dry exercise of every dodge stage + corner path. No hardware touched."""
import sys, types
sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M

ctl = M.ObstacleRound(0.0, 0.0)
ctl.state = "RUNNING"
P = config.load_params()
TPC = config.TICKS_PER_CM

def tick(front=200.0, left=45.0, right=45.0, yaw=0.0, blocks=None, dcm=1.0, t=[0.0], ticks=[0]):
    t[0] += 0.02
    ticks[0] += int(dcm * TPC)
    d = {"front": front, "left": left, "right": right, "rear": 999.0,
         "yaw": yaw, "ticks": ticks[0]}
    out = ctl.step(d, blocks or [], P, t[0])
    s = ctl.stage_display                      # the line that crashed the run
    assert "display error" not in s, s
    return out, s

def block(colour, dist, cx):
    return {"class": colour, "dist_cm": dist, "confidence": 0.9, "cx": cx,
            "x1": cx - 40, "x2": cx + 40, "y1": 200, "y2": 300}

seen = set()
for colour, cx in (("Green", 100), ("Green", 512), ("Green", 900),
                   ("Red", 100), ("Red", 512), ("Red", 900)):
    ctl.pillar_stage = None; ctl.pillar_color = None; ctl.pillar_clear_t = -1e9
    ctl.lock = 0; ctl.turns = 0
    dist = 90.0
    for i in range(1200):
        b = [block(colour, dist, cx)] if dist > 4 else []
        _, s = tick(front=200.0, blocks=b)
        if ctl.pillar_stage:
            seen.add(ctl.pillar_stage)
        if ctl.pillar_stage in (None, "APPROACH"):
            dist = max(3.0, dist - 1.0)
        if ctl.pillar_stage is None and i > 20:
            break
print("  dodge stages exercised:", sorted(seen))

# corner path: front closes in, sides tight (the L37/R42 case), then a real corner
ctl.pillar_stage = None; ctl.lock = 0; ctl.turns = 0
f = 120.0
for i in range(400):
    tick(front=f, left=37.0, right=42.0)
    f = max(30.0, f - 1.0)
print("  tight-sides lock (was CW on 5 cm):", {0: "still unlocked - correct", 1: "CW", -1: "CCW"}[ctl.lock])

ctl.lock = 0; ctl.turns = 0; ctl.is_turning = False; ctl.pre_turn_reverse = False
f = 120.0
for i in range(600):
    tick(front=f, left=30.0, right=999.0)
    f = max(20.0, f - 1.0)
print("  genuine open RIGHT locks:", {0: "NONE", 1: "CW - correct", -1: "CCW"}[ctl.lock])
print("  SMOKE TEST PASSED")

# --- regression: both side sensors at 999 must NOT silently mean left --------
ctl.lock = 0; ctl.turns = 0; ctl.is_turning = False; ctl.pre_turn_reverse = False
ctl.lock_vote = 0; ctl.lock_votes = 0; ctl._tof_last = {}
f = 120.0
for i in range(400):
    tick(front=f, left=999.0, right=999.0)
    f = max(20.0, f - 1.0)
got = {0: "NONE", 1: "CW", -1: "CCW"}[ctl.lock]
print("  both sides 999 (was always CCW):", got, "- correct" if got == "CW" else "- WRONG")
assert ctl.lock == 1, "999/999 must follow default_lap_dir (CW), got %s" % got

# --- and one real side reading still wins over the default -------------------
ctl.lock = 0; ctl.turns = 0; ctl.is_turning = False; ctl.pre_turn_reverse = False
ctl.lock_vote = 0; ctl.lock_votes = 0; ctl._tof_last = {}
f = 120.0
for i in range(400):
    tick(front=f, left=999.0, right=30.0)   # left is the open side -> CCW
    f = max(20.0, f - 1.0)
got = {0: "NONE", 1: "CW", -1: "CCW"}[ctl.lock]
print("  open LEFT beats the default:", got, "- correct" if got == -1 or got == "CCW" else "- WRONG")
assert ctl.lock == -1, "a real open side must beat default_lap_dir, got %s" % got
print("  DIRECTION TESTS PASSED")

# --- regression: a dodge leaves the car against a wall; that must not lock ---
# Red dodge ends hugging the RIGHT wall -> left reads long -> looks "open left".
# The track runs CW, so locking CCW off those readings is the bug.
ctl.pillar_stage = None; ctl.pillar_color = None; ctl.pillar_clear_t = -1e9
ctl.lock = 0; ctl.turns = 0; ctl.is_turning = False; ctl.pre_turn_reverse = False
ctl.lock_vote = 0; ctl.lock_votes = 0; ctl._tof_last = {}
dist = 90.0
for i in range(1200):                      # run a full Red dodge
    b = [block("Red", dist, 512)] if dist > 4 else []
    tick(front=200.0, left=70.0, right=20.0, blocks=b)
    if ctl.pillar_stage in (None, "APPROACH"):
        dist = max(3.0, dist - 1.0)
    if ctl.pillar_stage is None and i > 20:
        break
assert ctl.dodge_end_ticks is not None, "dodge end was not recorded"
f = 60.0
for i in range(30):                        # corner arrives immediately after
    tick(front=f, left=95.0, right=20.0)   # left looks wide open - it is not
    f = max(38.0, f - 1.0)
got = {0: "NONE", 1: "CW", -1: "CCW"}[ctl.lock]
print("  lock right after a Red dodge (was CCW):", got, "- correct" if got != "CCW" else "- WRONG")
assert ctl.lock != -1, "post-dodge side readings must not lock CCW, got %s" % got
print("  POST-DODGE LOCK TEST PASSED")

# --- regression: the exact corner from the 06:10 run, L54 R999 --------------
# Right side sees NO wall -> the corner opens right -> CW. The old code
# replaced that 999 with a stale 46 from the deglitch cache and turned CCW.
ctl.pillar_stage = None; ctl.pillar_color = None; ctl.pillar_clear_t = -1e9
ctl.lock = 0; ctl.turns = 0; ctl.is_turning = False; ctl.pre_turn_reverse = False
ctl.lock_vote = 0; ctl.lock_votes = 0; ctl.dodge_end_ticks = None
ctl._tof_last = {"right": (46.0, 0.0), "left": (54.0, 0.0)}   # stale cache, as it was
f = 70.0
for i in range(200):
    tick(front=f, left=54.0, right=999.0)
    f = max(30.0, f - 1.0)
got = {0: "NONE", 1: "CW", -1: "CCW"}[ctl.lock]
print("  L54 R999 open-right corner (was CCW):", got, "- correct" if got == "CW" else "- WRONG")
assert ctl.lock == 1, "an open RIGHT side must give CW, got %s" % got

# mirror image
ctl.lock = 0; ctl.turns = 0; ctl.is_turning = False; ctl.pre_turn_reverse = False
ctl.lock_vote = 0; ctl.lock_votes = 0; ctl.dodge_end_ticks = None; ctl._tof_last = {}
f = 70.0
for i in range(200):
    tick(front=f, left=999.0, right=54.0)
    f = max(30.0, f - 1.0)
got = {0: "NONE", 1: "CW", -1: "CCW"}[ctl.lock]
print("  L999 R54 open-left corner:", got, "- correct" if got == "CCW" else "- WRONG")
assert ctl.lock == -1, "an open LEFT side must give CCW, got %s" % got
print("  OPEN-SIDE TESTS PASSED")

# --- regression: the pillar just dodged must not be re-adopted --------------
# 06:14 run: a Red first seen at 21 cm, 1.2 s after a dodge ended, got dodged a
# second time. Every corner that then needed 120-148 deg instead of 90 followed
# one of those. Both guards below must reject it.
ctl.pillar_stage = None; ctl.pillar_color = None; ctl.pillar_clear_t = -1e9
ctl.lock = 1; ctl.turns = 1; ctl.is_turning = False; ctl.pre_turn_reverse = False
ctl.dodge_end_ticks = None; ctl._tof_last = {}
dist = 90.0
for i in range(1200):                                   # a full clean dodge
    b = [block("Red", dist, 512)] if dist > 4 else []
    tick(front=200.0, blocks=b)
    if ctl.pillar_stage in (None, "APPROACH"):
        dist = max(3.0, dist - 1.0)
    if ctl.pillar_stage is None and i > 20:
        break
assert ctl.dodge_end_ticks is not None

for _ in range(60):                                     # ~1.2 s later, 21 cm away
    tick(front=999.0, blocks=[block("Red", 21.0, 300)], dcm=0.5)
print("  re-adopting the pillar just passed (21 cm):",
      ctl.pillar_stage or "ignored", "- correct" if ctl.pillar_stage is None else "- WRONG")
assert ctl.pillar_stage is None, "a 21 cm pillar right after a dodge must be ignored"

for _ in range(400):                                    # drive on, then a real one
    tick(front=999.0, dcm=0.5)
for _ in range(20):
    tick(front=999.0, blocks=[block("Green", 85.0, 512)])
print("  a genuine pillar at 85 cm further on:", ctl.pillar_stage or "ignored",
      "- correct" if ctl.pillar_stage == "APPROACH" else "- WRONG")
assert ctl.pillar_stage == "APPROACH", "a real pillar must still be adopted"
print("  RE-ACQUISITION TESTS PASSED")
