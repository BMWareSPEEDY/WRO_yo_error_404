"""NOTHING interrupts a dodge. This is the highest-priority rule in the round.

From the moment the trigger distance is reached and the car starts going around
a block, it goes all the way around and squares up again. No corner interrupts
it, and no front-sensor reading does either - not even one short enough to look
like a wall.

Why the wall abort is gone: the front sensor sweeps across the pillar being
dodged and reads short mid-manoeuvre, so the abort kept killing perfectly good
dodges and leaving the car half-displaced and angled across the lane, which
then ruined the corner that followed as well.

What still holds: a pillar never manufactures a corner out of its own
reflection, and the stage machine still ends every leg on measured distance, so
a dodge cannot run forever.
"""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M
P = config.load_params()

DODGE_STAGES = ("TURN_OUT", "STRAIGHTEN", "RECENTER", "ALIGN")


def blk(colour, dist, cx=512):
    return {"class": colour, "dist_cm": dist, "confidence": 0.9, "cx": cx,
            "x1": cx - 45, "x2": cx + 45, "y1": 200, "y2": 300}


def dodge_with_front(front_during, keep_seeing_block=True):
    """Start a dodge, then hold `front_during` on the front sensor throughout."""
    ctl = M.ObstacleRound(0.0, 0.0); ctl.state = "RUNNING"
    ctl.pillar_clear_t = -1e9; ctl.lock = 1; ctl.turns = 1
    ctl.turn_end_ticks = -999999
    t = 0.0; ticks = 0; seq = 0; dist = 80.0
    reached = []
    for i in range(5000):
        t += 0.02
        if i % 3 == 0:
            seq += 1
            if ctl.pillar_stage in (None, "APPROACH"):
                dist = max(8.0, dist - 1.2)
        ctl.vision_seq = seq
        started = ctl.pillar_stage in DODGE_STAGES
        blocks = [blk("Green", dist, 380)] if (not started or keep_seeing_block) else []
        d = {"front": front_during if started else 999.0, "left": 45.0, "right": 45.0,
             "rear": 999.0, "yaw": 0.0, "ticks": ticks}
        ctl.step(d, blocks, P, t)
        if ctl.pillar_stage and (not reached or reached[-1] != ctl.pillar_stage):
            reached.append(ctl.pillar_stage)
        if started and (ctl.is_turning or ctl.pre_turn_reverse):
            return "INTERRUPTED by a corner", reached
        if "ALIGN" in reached and ctl.pillar_stage is None:
            return "completed", reached
        ticks += int(110 * 0.30 * 0.02 * config.TICKS_PER_CM)
    return "unfinished", reached


ok = True
# Every front reading from open space down to nearly touching. All must finish.
for front in (999.0, 60.0, 30.0, 16.0, 10.0, 4.0):
    r, reached = dodge_with_front(front)
    good = r == "completed" and "ALIGN" in reached
    ok &= good
    print("  front held at %5.0f cm through the dodge -> %-22s %s"
          % (front, r, "correct" if good else "WRONG"))

r, reached = dodge_with_front(30.0)
print("  stages:", " -> ".join(reached))

assert ok, "something interrupted a dodge"
print("  NOTHING INTERRUPTS A DODGE - correct")
