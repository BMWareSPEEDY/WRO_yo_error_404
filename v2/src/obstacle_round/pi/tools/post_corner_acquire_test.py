"""A pillar just past a corner must be adopted, even straight after a dodge.

The scenario that ended runs: dodge a pillar near the end of a straight, take
the corner (reverse-arc, turn, reverse again), and a Green sits 55 cm up the
new straight, a little left of centre. The encoder is SIGNED, so the two
reverses wind the count back towards where the dodge ended, and the post-dodge
gate read "only 5 cm since the dodge" - so the Green was refused until it was
under the 45 cm floor, then ignored as "the pillar just passed". It was never
dodged. Run on the Pi:  python3 tools/post_corner_acquire_test.py
"""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import logging
import config, main as M

logging.basicConfig(level=logging.INFO, format="      log: %(message)s")
P = config.load_params()
W = config.CAMERA_SIZE[0]
TPC = config.TICKS_PER_CM


def data(ticks, yaw=0.0):
    return {"front": 200.0, "left": 40.0, "right": 40.0, "rear": 999.0,
            "yaw": yaw, "ticks": int(ticks)}


def green(dist, cx):
    return [{"class": "Green", "dist_cm": dist, "confidence": 0.9, "cx": cx,
             "x1": cx - 15, "x2": cx + 15, "y1": 150, "y2": 250}]


def run(colour_cx, label):
    ctl = M.ObstacleRound(0.0, 0.0)
    ctl.lock = 1
    t = 0.0
    for _ in range(40):                       # settle the ToF filter
        t += 0.02
        ctl._deglitch_tof(data(0), t)

    # A dodge ended at T. The car then ran on, reverse-arced and turned:
    # net +45 cm by the time the corner completes.
    T = 100000
    ctl.dodge_end_ticks = T
    ctl.pillar_clear_t = t
    tk = T + 45 * TPC
    ctl.is_turning = True
    ctl.turn_t0 = t
    ctl.target = 0.0
    t += 0.02
    ctl.step(data(tk), [], P, t)              # heading is on target -> corner completes

    # Post-corner reverse, 1 cm a tick, until the car lets go of it.
    for _ in range(300):
        if not ctl.post_turn_reverse:
            break
        t += 0.02
        tk -= TPC
        ctl.step(data(tk), [], P, t)
    since = (tk - T) / TPC
    print(f"   {label}: reverse done, encoder says {since:.0f} cm since the dodge ended")

    # Drive up the new straight. Green starts 55 cm ahead, closes 1 cm a tick.
    dist = 55.0
    while dist > 10:
        t += 0.05
        tk += TPC
        dist -= 1.0
        ctl.step(data(tk), green(dist, colour_cx), P, t)
        if ctl.pillar_stage is not None:
            print(f"   {label}: ADOPTED at {dist:.0f} cm -> {ctl.pillar_stage}")
            return True
    print(f"   {label}: *** NEVER ADOPTED - drove past it ***")
    return False


print("  --- pillar on the new straight, right after dodge + corner + reverse ---")
ok = run(W // 2 - 60, "Green slightly LEFT ")
ok &= run(W // 2,      "Green dead ahead    ")
assert ok, "a pillar on the new straight was skipped"
print("  NEW BEHAVIOUR CORRECT")
