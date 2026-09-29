"""Corner > block. A real corner mid-dodge must win; a pillar ahead must not
be mistaken for one."""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M
P = config.load_params()

def blk(colour, dist, cx=512):
    return {"class": colour, "dist_cm": dist, "confidence": 0.9, "cx": cx,
            "x1": cx-40, "x2": cx+40, "y1": 200, "y2": 300}

def dodge_then(front_after, keep_seeing_block):
    """Get a dodge under way, then present `front_after` on the front ToF."""
    ctl = M.ObstacleRound(0.0, 0.0); ctl.state = "RUNNING"
    ctl.pillar_clear_t = -1e9; ctl.lock = 1; ctl.turns = 1
    ctl.turn_end_ticks = -999999
    t = 0.0; ticks = 0; seq = 0; dist = 90.0
    for i in range(3000):
        t += 0.02
        if i % 3 == 0:
            seq += 1
            if ctl.pillar_stage in (None, "APPROACH"):
                dist = max(5.0, dist - 1.5)
        ctl.vision_seq = seq
        started = ctl.pillar_stage in ("TURN_OUT", "STRAIGHTEN", "RECENTER")
        front = front_after if started else 999.0
        blocks = [blk("Red", dist)] if (not started or keep_seeing_block) else []
        d = {"front": front, "left": 40.0, "right": 999.0, "rear": 999.0,
             "yaw": 0.0, "ticks": ticks}
        ctl.step(d, blocks, P, t)
        if started:
            for _ in range(200):                 # hold that reading a while
                t += 0.02
                ctl.vision_seq = seq
                d = {"front": front, "left": 40.0, "right": 999.0, "rear": 999.0,
                     "yaw": 0.0, "ticks": ticks}
                ctl.step(d, blocks, P, t)
                ticks += int(90 * 0.30 * 0.02 * config.TICKS_PER_CM)
                if ctl.is_turning or ctl.pre_turn_reverse:
                    return "corner"
            return "dodge kept"
        ticks += int(90 * 0.30 * 0.02 * config.TICKS_PER_CM)
    return "no dodge started"

r = dodge_then(30.0, keep_seeing_block=False)
print("  real WALL at 30 cm mid-dodge (camera sees nothing): %s  %s"
      % (r, "correct - the turn wins" if r == "corner" else "WRONG"))
assert r == "corner", "a genuine corner mid-dodge must take priority"

r = dodge_then(30.0, keep_seeing_block=True)
print("  the PILLAR itself at 30 cm mid-dodge:              %s  %s"
      % (r, "correct - not a corner" if r != "corner" else "WRONG - false corner"))
assert r != "corner", "the pillar being dodged must not trigger a corner"
print("  CORNER OUTRANKS BLOCK, WITHOUT FALSE CORNERS - correct")
