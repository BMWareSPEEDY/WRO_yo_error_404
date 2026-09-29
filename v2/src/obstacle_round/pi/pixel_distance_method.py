"""pixel_distance_method.py - size the dodge from the pillar's pixel offset.

THE IDEA
    The wide/normal arc split is a two-value approximation of something
    continuous: how far sideways the car must move to get past a pillar. That
    is visible directly in the image, so measure it there.

        sideways_cm = BASE_CM + toward_px * CM_PER_PIXEL

    Only the SIDEWAYS leg varies. How far the car runs past the pillar and how
    far it swings back in stay constant - those are about the pillar's depth
    and the lane width, not its bearing.

WHICH WAY ROUND
    Green is passed on the car's LEFT, Red on its RIGHT. So what matters is not
    the raw offset but how far the pillar sits TOWARDS the side the car is
    dodging to:

        Green  pillar further LEFT  -> car must swing further left  -> MORE
        Green  pillar to the RIGHT  -> it is already clear          -> LESS
        Red    mirrored

    A pillar dead ahead is the zero case and gets BASE_CM. `toward_px` is
    therefore signed, and negative for a pillar already drifting to the correct
    side, which correctly shrinks the movement.

CALIBRATION
    Measured on the car, 2026-09-24, with a green pillar at the trigger
    distance:

        DET [G34@335/0.96]   ->  34 cm, cx = 335 in a 1024-wide frame
                                 offset  = 335 - 512 = -111 px
                                 toward  = +111 px (left, and Green dodges left)

    25 cm of sideways movement clears it, plus 3 cm margin = 28 cm. With
    BASE_CM for a pillar dead ahead:

        CM_PER_PIXEL = (28 - BASE_CM) / 177

IMPORTANT
    Pixel offset grows as the car closes on a pillar at a fixed lateral
    position, so the factor is only meaningful at the distance it was
    calibrated at. Evaluate this ONCE, when the dodge commits - the same
    moment the region-of-interest band is latched.
"""

FRAME_W = 640      # must match config.CAMERA_SIZE[0]

# A pillar DEAD AHEAD still needs this much sideways movement: roughly half the
# car plus half the pillar. It is the zero-offset case, not the minimum.
BASE_CM = 31.15

# (28 - 12) / 177, from the measurement above.
CM_PER_PIXEL = 0.08

MIN_CM = 8.0                  # a pillar already well to the correct side
MAX_CM = 55.0                 # never swing further than this, whatever is seen


def pixel_offset(block, frame_w=FRAME_W):
    """Signed pixels from frame centre to the box centre. +ve = right of centre."""
    cx = block.get("cx")
    if cx is None:
        x1, x2 = block.get("x1"), block.get("x2")
        if x1 is None or x2 is None:
            return 0.0
        cx = (x1 + x2) / 2.0
    return float(cx) - frame_w / 2.0


def toward_px(block, colour, frame_w=FRAME_W):
    """Pixels the pillar sits TOWARDS the side the car will dodge to.

    Positive means the car has further to go; negative means the pillar is
    already drifting to the side it needs to end up on.
    """
    off = pixel_offset(block, frame_w)
    return -off if colour == "Green" else off


def sideways_cm(block, colour, frame_w=FRAME_W, cm_per_px=CM_PER_PIXEL,
                base_cm=BASE_CM, min_cm=MIN_CM, max_cm=MAX_CM):
    """How far sideways the car must move to clear this pillar, in cm."""
    cm = base_cm + toward_px(block, colour, frame_w) * cm_per_px
    return max(min_cm, min(max_cm, cm))


def describe(block, colour, frame_w=FRAME_W):
    return (f"{colour} offset {pixel_offset(block, frame_w):+.0f} px "
            f"(toward {toward_px(block, colour, frame_w):+.0f}) -> "
            f"{sideways_cm(block, colour, frame_w):.0f} cm sideways")


if __name__ == "__main__":
    CAL_PX, CAL_CM = 110.6, 40.0        # 177 px in the old 1024-wide frame
    print(f"  frame {FRAME_W} px, centre {FRAME_W//2}, base {BASE_CM:.2f} cm, "
          f"{CM_PER_PIXEL:.3f} cm/px")
    print(f"  calibration: {CAL_PX:.0f} px off centre -> {CAL_CM:.0f} cm sideways\n")

    print("  GREEN (passed on the LEFT, so a green further LEFT needs more)")
    for cx in (60, 130, 209, 260, 320, 380, 440, 520, 600):
        b = {"cx": cx}
        mark = "   <-- calibration point" if cx == 209 else ""
        print("   cx=%-4d offset %+5.0f px -> %5.1f cm%s"
              % (cx, pixel_offset(b), sideways_cm(b, "Green"), mark))

    print("\n  RED (mirrored)")
    for cx in (60, 200, 320, 431, 500, 600):
        b = {"cx": cx}
        mark = "   <-- mirror of the calibration point" if cx == 431 else ""
        print("   cx=%-4d offset %+5.0f px -> %5.1f cm%s"
              % (cx, pixel_offset(b), sideways_cm(b, "Red"), mark))

    print("\n  sanity")
    g = sideways_cm({"cx": FRAME_W // 2 - CAL_PX}, "Green")
    r = sideways_cm({"cx": FRAME_W // 2 + CAL_PX}, "Red")
    assert abs(g - CAL_CM) < 0.3, "green calibration point: %.2f" % g
    assert abs(r - CAL_CM) < 0.3, "red calibration point: %.2f" % r
    assert sideways_cm({"cx": 120}, "Green") > sideways_cm({"cx": 320}, "Green"), \
        "a green further left must need MORE"
    assert sideways_cm({"cx": 520}, "Green") < sideways_cm({"cx": 320}, "Green"), \
        "a green already right must need LESS"
    assert sideways_cm({"cx": 320}, "Green") == sideways_cm({"cx": 320}, "Red") == BASE_CM
    print("   both colours hit %.0f cm at the calibration point, dead ahead gives" % CAL_CM)
    print("   the base, and the two directions move opposite ways - OK")
