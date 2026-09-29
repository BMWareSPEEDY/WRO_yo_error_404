"""Does a corner now finish on exactly 90 degrees, arc + forward turn?"""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M

P = config.load_params()
print("  reverse_arc_deg=%s  leave=%s  pre_reverse=%s cm  post_reverse=%s cm"
      % (P.get("reverse_arc_deg"), P.get("reverse_arc_leave_deg"),
         P.get("pre_turn_reverse_cm"), P.get("post_turn_reverse_cm")))

for lock, name in ((1, "CW (right)"), (-1, "CCW (left)")):
    ctl = M.ObstacleRound(0.0, 0.0); ctl.state = "RUNNING"
    ctl.lock = lock; ctl.turns = 0
    t = [0.0]; ticks = [0]; yaw = [0.0]
    start_target = ctl.target
    arc_cm = fwd_ticks = None
    phase_seen = []
    for i in range(6000):
        t[0] += 0.02
        front = 35.0 if ctl.turns == 0 and not ctl.is_turning and not ctl.pre_turn_reverse else 999.0
        d = {"front": front, "left": 45.0, "right": 999.0 if lock == 1 else 45.0,
             "rear": 999.0, "yaw": yaw[0] % 360.0, "ticks": ticks[0]}
        if lock == -1:
            d["left"], d["right"] = 999.0, 45.0
        speed, steer = ctl.step(d, [], P, t[0])
        ph = ("ARC" if ctl.pre_turn_reverse else
              "TURN" if ctl.is_turning else
              "POSTREV" if ctl.post_turn_reverse else "CRUISE")
        if not phase_seen or phase_seen[-1] != ph:
            phase_seen.append(ph)
        cm = speed * 0.30 * 0.02
        ticks[0] += int(cm * config.TICKS_PER_CM)
        # steering rotates the car; reversing rotates it the other way
        yaw[0] += (steer - config.SERVO_CENTER) * 0.22 * (abs(cm) / 0.72) * (1 if cm >= 0 else -1)
        if ctl.turns >= 1 and ph == "CRUISE":
            break
    turned = (yaw[0] - 0.0) * (1 if lock == 1 else -1)
    print("  %-11s phases %s" % (name, " -> ".join(phase_seen[:5])))
    print("              turned %+.1f deg (target %+d), target heading %.0f"
          % (yaw[0], 90 * lock, ctl.target))
    err = abs(abs(yaw[0]) - 90)
    print("              %s off 90 deg" % ("%.1f deg" % err))
    assert err < 15, "corner did not land near 90: %+.1f" % yaw[0]
    assert "ARC" in phase_seen and "TURN" in phase_seen, "arc or forward turn missing"
print("  CORNER LANDS ON 90 DEG, ARC THEN FORWARD TURN - correct")
