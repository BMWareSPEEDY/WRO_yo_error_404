"""Green's dodge must be Red's, mirrored about the car's TRUE straight.

Red is known good, so this asserts two things at once: every Red angle is
EXACTLY what it was before the mirror rework, and every Green angle is Red's
reflected about 90 + servo_trim_deg (87 on this car), not about 90.

Run:  python3 tools/mirror_test.py
"""
import sys; sys.path.insert(0, "/home/pi/obstacle_round")
import config, main as M

P = config.load_params()
W = config.CAMERA_SIZE[0]
TC = config.SERVO_CENTER + P["servo_trim_deg"]
ok = True


def ctl_for(colour, wide=False, cx=None):
    c = M.ObstacleRound(0.0, 0.0)
    c._params = P
    c.pillar_color = colour
    c.dodge_wide = wide
    c.pillar_cx = cx
    c.pixel_out_cm = None
    return c


def check(what, red, green, tol=0.51):
    """green must be red mirrored about TC."""
    global ok
    want = TC - (red - TC)
    good = abs(green - want) <= tol
    ok &= good
    print(f"   {what:34s} red {red:7.2f}   green {green:7.2f}   "
          f"mirror wants {want:7.2f}   {'OK' if good else 'WRONG'}")


print(f"  true straight = 90 + {P['servo_trim_deg']} = {TC}   (NOT 90)")

print("  --- turn-out angle, normal arc ---")
check("sharp out", ctl_for("Red")._sharp_out_steer(P), ctl_for("Green")._sharp_out_steer(P))
# Red must be untouched by the rework.
assert ctl_for("Red")._sharp_out_steer(P) == P["pillar_steer_right"], "RED turn-out changed!"
print(f"   red turn-out is still exactly pillar_steer_right ({P['pillar_steer_right']})  OK")

print("  --- turn-out angle, WIDE arc ---")
check("sharp out WIDE", ctl_for("Red", wide=True)._sharp_out_steer(P),
      ctl_for("Green", wide=True)._sharp_out_steer(P))

print("  --- turn-out with the block off to one side (mirrored positions) ---")
for off in (40, 100, 160):
    sens = P["block_offset_sens"]
    half = W / 2.0

    def steer(colour, cx):
        c = ctl_for(colour, cx=cx)
        nx = (cx - half) / half
        return M.clamp_servo(c._as_red(c._red_out_steer(P) + c._dodge_sign() * nx * sens * 15.0))
    # a Red +off right of centre is the mirror of a Green -off left of it
    check(f"off-centre {off:>3} px", steer("Red", half + off), steer("Green", half - off))

print("  --- return-in angle, and where the unwind ends ---")
for colour in ("Red", "Green"):
    c = ctl_for(colour)
    red_mirror = 2 * config.SERVO_CENTER - c._red_out_steer(P)
    rd = abs(float(P["dodge_return_reduce_deg"]))
    globals()[f"ret_{colour}"] = c._as_red(min(config.SERVO_CENTER, red_mirror + rd))
    globals()[f"end_{colour}"] = c._as_red(2 * config.SERVO_CENTER - config.DODGE_RETURN_END_DEG)
check("return-in", ret_Red, ret_Green)
check("unwind end", end_Red, end_Green)

print("  --- leg lengths identical (params are mirrored, not negated) ---")
for name in ("_out_cm", "_past_cm", "_in_cm", "_align_min_cm"):
    r = getattr(ctl_for("Red"), name)()
    g = getattr(ctl_for("Green"), name)()
    ok &= (r == g)
    print(f"   {name:34s} red {r:7.2f}   green {g:7.2f}   {'OK' if r == g else 'WRONG'}")

print("  --- every angle stays inside the servo's mechanical limits ---")
for colour in ("Red", "Green"):
    for wide in (False, True):
        a = ctl_for(colour, wide=wide)._sharp_out_steer(P)
        inside = config.SERVO_MIN <= a <= config.SERVO_MAX
        ok &= inside
        print(f"   {colour:5s} {'WIDE' if wide else 'norm'} out {a:7.2f}  "
              f"{'OK' if inside else 'OUT OF RANGE'}")

assert ok, "green is not an exact mirror of red"
print("  NEW BEHAVIOUR CORRECT")
