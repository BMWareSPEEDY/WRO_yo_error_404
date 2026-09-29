"""A pillar must be dodged, never taken as a corner."""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M
P = config.load_params()

CENTRE = config.CAMERA_SIZE[0] // 2     # never hard-code 512: the frame moved


def blk(colour, dist, cx=None):
    cx = CENTRE if cx is None else cx
    return {"class": colour, "dist_cm": dist, "confidence": 0.9, "cx": cx,
            "x1": cx-40, "x2": cx+40, "y1": 200, "y2": 300}

# Replays corner 12 of the 09:52 run: a Green tracked from 64 cm while the
# camera's distance stays optimistic and the front ToF closes to 15 cm.
ctl = M.ObstacleRound(0.0, 0.0); ctl.state = "RUNNING"
ctl.pillar_clear_t = -1e9; ctl.lock = -1; ctl.turns = 1
ctl.turn_end_ticks = -999999
t = 0.0; ticks = 0; seq = 0
tof = 64.0
optical = 64.0          # camera under-reports the closing rate - reads LONG
outcome = None
for i in range(1200):
    t += 0.02
    if i % 3 == 0:
        seq += 1
        optical = max(38.0, optical - 0.35)      # never reaches the 30 cm trigger
    ctl.vision_seq = seq
    d = {"front": tof, "left": 45.0, "right": 45.0, "rear": 999.0,
         "yaw": 0.0, "ticks": ticks}
    ctl.step(d, [blk("Green", optical)], P, t)
    if ctl.is_turning or ctl.pre_turn_reverse:
        outcome = "CORNER at %.0f cm" % tof; break
    if ctl.pillar_stage in ("TURN_OUT", "STRAIGHTEN", "RECENTER"):
        outcome = "dodged at ToF %.0f cm (camera said %.0f)" % (tof, optical); break
    ticks += int(90 * 0.30 * 0.02 * config.TICKS_PER_CM)
    tof = max(8.0, tof - 0.6)

print("  camera reads long, ToF closes in -> %s" % outcome)
print("  %s" % ("correct - the pillar was dodged" if outcome and outcome.startswith("dodged")
                else "WRONG - the pillar was taken as a corner"))
assert outcome and outcome.startswith("dodged"), "a tracked pillar became a corner"
print("  PILLAR IS DODGED, NOT TURNED AT - correct")
