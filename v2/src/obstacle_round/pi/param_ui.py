"""
param_ui.py - the one list of what the dashboard lets you tune.

The sliders used to be hand-written HTML: a `<div>` per parameter, each with its
own element id, its own `{{PARAM_VAL_X}}` placeholder and its own entry in a
JavaScript lookup table. Four places to keep in step, so they drifted - one
slider (`dodge_past_cm`) moved a value nothing in main.py ever read, and a
dozen parameters the car genuinely uses had no slider at all.

Now there is one list. `GROUPS` below is the whole user interface: the
dashboard builds the card from it, and `check()` fails loudly if it ever falls
out of step with `config.DEFAULT_PARAMS`. Adding a tunable means adding a line
here and nothing else.

Every entry carries an `info` string - the text behind the little (i) next to
each slider. Write it for someone standing at the mat who has not read the
code: what the number does, and which way to move it.

A parameter is listed here ONLY if the running code reads it. Derived values
(everything green, which always mirrors red) are marked `derived` and shown
greyed out, because changing them does nothing - `config.mirror_green()`
overwrites them on every load.
"""

# Each entry: key, label, then the slider's range.
#   lo, hi, step   numeric slider
#   kind           "bool" for a switch, "choice" for a fixed set, else a number
#   derived        True  -> shown, but read-only; the code computes it
#   info           what the (i) button says
GROUPS = [
    {
        "title": "0 · Leaving the parking bay",
        "blurb": "run once at the very start of the obstacle round, before the first lap",
        "params": [
            {
                "key": "park_enabled", "label": "Start from the parking bay", "kind": "bool",
                "info": "ON: the round opens with the exit manoeuvre - swing out of the bay, "
                        "run forward, back up, square up, then start lapping. OFF: skip it "
                        "entirely and start the round from wherever the car is standing. Turn "
                        "it off when testing laps away from the bay.",
            },
            {
                "key": "park_turn_deg", "label": "Swing out of the bay by", "unit": "deg",
                "lo": 30, "hi": 120, "step": 5,
                "info": "How far the car rotates to get its nose out of the bay - and the same "
                        "amount back the other way when squaring up afterwards.",
            },
            {
                "key": "park_steer_deg", "label": "Steering lock for both swings",
                "unit": "deg off centre", "lo": 10, "hi": 55, "step": 1,
                "info": "How hard the wheels are turned during the swing out and the swing back. "
                        "Bigger turns in a tighter circle but scrubs the tyres.",
            },
            {
                "key": "park_forward_cm", "label": "Forward leg after the swing", "unit": "cm",
                "lo": 0, "hi": 200, "step": 5,
                "info": "The straight run once the car is out of the bay, to get clear of it "
                        "before squaring up to the wall.",
            },
            {
                "key": "park_back_cm", "label": "Reverse before squaring up", "unit": "cm",
                "lo": 0, "hi": 100, "step": 5,
                "info": "A short back-up at the end of the exit, to leave room for the "
                        "straightening swing.",
            },
            {
                "key": "park_speed", "label": "Speed for the whole exit", "unit": "PWM",
                "lo": 40, "hi": 100, "step": 1,
                "info": "Motor power for the exit manoeuvre only. Slower is easier to watch and "
                        "easier on the geometry; it costs a second at the start.",
            },
            {
                "key": "park_max_cm", "label": "Failsafe - give up on a step after", "unit": "cm",
                "lo": 100, "hi": 800, "step": 20,
                "info": "If any step of the exit runs this far without finishing, the car "
                        "abandons the exit and starts racing anyway, rather than grinding away "
                        "in the bay while the clock runs.",
            },
        ],
    },
    {
        "title": "1 · Seeing a pillar",
        "blurb": "how far away a block is noticed, and how far away the camera thinks it is",
        "params": [
            {
                "key": "pillar_focal_px", "label": "Camera focal length", "unit": "px",
                "lo": 200, "hi": 1500, "step": 5,
                "info": "Turns the height of a block in the picture into a distance in cm. "
                        "Only this one number sets the camera's distance scale. Check it by "
                        "standing a pillar at a measured 50 cm: if the dashboard reads 60, the "
                        "focal length is too high. Calibrated at 710 on this car.",
            },
            {
                "key": "pillar_trigger_dist_cm", "label": "Start the dodge at", "unit": "cm",
                "lo": 5, "hi": 120, "step": 1,
                "info": "How close the car lets a pillar get before it commits to going round "
                        "it. Bigger = swings out earlier and wider, from further back. Too "
                        "small and the car is on top of the block before it reacts.",
            },
        ],
    },
    {
        "title": "2 · The dodge — how hard, how far",
        "blurb": "red is the one you tune; green is always its mirror image",
        "params": [
            {
                "key": "pillar_steer_right", "label": "Steer angle for RED", "unit": "servo deg",
                "lo": 90, "hi": 140, "step": 1,
                "info": "How hard the wheels turn to go round a RED pillar (red = car passes "
                        "on the pillar's right). 90 is straight-ish, 140 is full lock. This is "
                        "the master setting: green is worked out from it automatically, so the "
                        "two colours can never drift apart.",
            },
            {
                "key": "pillar_steer_left", "label": "Steer angle for GREEN", "unit": "servo deg",
                "lo": 30, "hi": 90, "step": 1, "derived": True,
                "info": "Worked out from the red angle, mirrored about the car's TRUE straight "
                        "(90 plus the servo trim), not about 90. Read-only on purpose - tune "
                        "red and this follows.",
            },
            {
                "key": "turn_hold_pct", "label": "Hold full lock for the first", "unit": "% of the leg",
                "lo": 0, "hi": 80, "step": 5,
                "info": "At the start of a turn-out or turn-back-in the wheels stay at the full "
                        "angle above for this share of the leg, then unwind smoothly. Bigger = "
                        "a sharper, more sudden turn. 0 = unwinds from the very first cm.",
            },
            {
                "key": "dodge_wide_steer_deg", "label": "Wide turn-out - steer angle",
                "unit": "deg", "lo": 20, "hi": 80, "step": 1,
                "info": "The steering angle used for the WIDE turn-out - the one taken when the "
                        "pillar already sits on the side the car has to dodge towards. Written "
                        "green-side and mirrored for red, like every other dodge angle.",
            },
            {
                "key": "block_offset_sens", "label": "Aim off-centre sensitivity", "unit": "",
                "lo": 0.0, "hi": 1.5, "step": 0.05,
                "info": "How much the car leans on WHERE the pillar sits in the picture, rather "
                        "than just its colour. Higher = a block already off to one side gets a "
                        "bigger correction. 0 turns it off and every dodge is the same shape.",
            },
        ],
    },
    {
        "title": "3 · The dodge — leg lengths",
        "blurb": "out, past, back in. All measured on the wheel encoder, so they are real cm.",
        "params": [
            {
                "key": "use_pixel_dodge", "label": "Size the turn-out from the picture", "kind": "bool",
                "info": "ON: how far the car swings out is worked out from how far the pillar "
                        "sits off-centre in the picture - a block dead ahead gets a small swing, "
                        "one off to the side gets a big one. OFF: the two fixed leg lengths "
                        "below are used instead (normal / wide). ON is the newer method.",
            },
            {
                "key": "pixel_base_cm", "label": "Picture mode — swing for a block dead ahead",
                "unit": "cm", "lo": 0, "hi": 40, "step": 0.5,
                "info": "With picture mode ON: the smallest turn-out, used when the pillar is "
                        "exactly in the middle of the frame. Every pillar gets at least this.",
            },
            {
                "key": "pixel_cm_per_px", "label": "Picture mode — extra swing per pixel off-centre",
                "unit": "cm/px", "lo": 0.0, "hi": 0.25, "step": 0.002,
                "info": "With picture mode ON: how many extra cm of turn-out per pixel the "
                        "pillar sits away from the middle. Measured on this car at 0.0904, i.e. "
                        "a pillar 177 px off-centre adds about 16 cm.",
            },
            {
                "key": "dodge_short_cm", "label": "Fixed mode — turn out, normal", "unit": "cm",
                "lo": 5, "hi": 60, "step": 1,
                "info": "Only used with picture mode OFF. The normal turn-out: how far the car "
                        "drives while steering away from the pillar.",
            },
            {
                "key": "dodge_long_cm", "label": "Fixed mode — turn out, wide arc", "unit": "cm",
                "lo": 5, "hi": 80, "step": 1,
                "info": "Only used with picture mode OFF. The wider turn-out, used when the "
                        "pillar already sits on the side the car has to dodge towards, so it "
                        "has further to go to get round it.",
            },
            {
                "key": "dodge_red_extra_cm", "label": "Turn out — extra", "unit": "cm",
                "lo": -10, "hi": 20, "step": 0.5,
                "info": "Added to the turn-out leg whichever method is used above. A trim: nudge "
                        "it up if the car keeps clipping pillars, down if it swings out wastefully "
                        "far. Green gets the same amount.",
            },
            {
                "key": "dodge_past_cm", "label": "Run straight past the pillar", "unit": "cm",
                "lo": 5, "hi": 90, "step": 1,
                "info": "Leg 2: having swung out, how far the car holds that line before turning "
                        "back in. Too short and the back wheels catch the pillar on the way past.",
            },
            {
                "key": "dodge_past_red_extra_cm", "label": "...extra on that", "unit": "cm",
                "lo": -10, "hi": 20, "step": 0.5,
                "info": "Trim on the run-past leg. Green gets the same.",
            },
            {
                "key": "dodge_return_extra_cm", "label": "Swing back in — extra over the turn-out",
                "unit": "cm", "lo": -15, "hi": 20, "step": 0.5,
                "info": "Leg 3 is the turn-out leg plus this, so the car comes back in slightly "
                        "further than it went out and ends up back in the lane. Raise it if the "
                        "car finishes a dodge still wide of the lane.",
            },
            {
                "key": "dodge_return_red_extra_cm", "label": "...and a trim on top", "unit": "cm",
                "lo": -15, "hi": 20, "step": 0.5,
                "info": "A second, per-colour trim on the swing back in. Negative cuts in less. "
                        "Green gets the same.",
            },
            {
                "key": "dodge_return_reduce_deg", "label": "Soften the swing back in", "unit": "deg",
                "lo": 0, "hi": 30, "step": 1,
                "info": "Pulls the turn-back-in angle this many degrees back towards straight, so "
                        "the car returns to the lane on a gentler curve instead of cutting across "
                        "the line it just left.",
            },
            {
                "key": "dodge_align_min_red_cm", "label": "Minimum straight while squaring up",
                "unit": "cm", "lo": 0, "hi": 20, "step": 0.5,
                "info": "The car must run at least this far forward while straightening up, so a "
                        "dodge cannot end with the car still alongside the pillar it just passed. "
                        "Green gets the same.",
            },
        ],
    },
    {
        "title": "4 · After a dodge",
        "blurb": "stop the camera re-adopting the pillar the car has only just squeezed past",
        "params": [
            {
                "key": "pillar_clear_cm", "label": "Run clear for", "unit": "cm",
                "lo": 0, "hi": 150, "step": 1,
                "info": "After a dodge finishes, the car must travel this far before it is "
                        "allowed to start another dodge. Without it the camera picks up the same "
                        "pillar sitting right beside the car and dodges it all over again.",
            },
            {
                "key": "pillar_clear_delay_s", "label": "...and wait at least", "unit": "s",
                "lo": 0, "hi": 6, "step": 0.1,
                "info": "The same block, but as a plain clock. BOTH have to pass. A safety net "
                        "for when the wheels slip and the distance count runs ahead of reality.",
            },
        ],
    },
    {
        "title": "5 · Corners",
        "blurb": "the front sensor sees a wall → a 90° turn, then a reverse for room",
        "params": [
            {
                "key": "front_turn_cm", "label": "Turn when the wall is this close", "unit": "cm",
                "lo": 15, "hi": 80, "step": 1,
                "info": "Raise it to turn EARLIER, from further out. Turning late means arriving "
                        "at the wall before committing, which leaves the car hard against it "
                        "instead of swinging round from the middle of the lane.",
            },
            {
                "key": "corner_steer_deg", "label": "Corner steering cap", "unit": "deg off centre",
                "lo": 10, "hi": 50, "step": 1,
                "info": "The most the wheels turn in a corner. Steering is proportional: this "
                        "much while the car is still far from the new heading, easing off as it "
                        "comes round. Not a full lock on purpose - at full lock the front tyres "
                        "scrub instead of biting and the car carries on roughly straight.",
            },
            {
                "key": "pre_turn_reverse_cm", "label": "Reverse BEFORE the corner", "unit": "cm",
                "lo": 0, "hi": 60, "step": 1,
                "info": "A short back-up before the turn starts, to buy room to swing. 0 turns "
                        "it off.",
            },
            {
                "key": "post_turn_reverse_cm", "label": "Reverse AFTER the corner", "unit": "cm",
                "lo": 0, "hi": 150, "step": 1,
                "info": "The straight reverse once the corner is finished, to get clear of the "
                        "wall before running down the new straight. This is the big one: 100 cm "
                        "is roughly 4 seconds of backing up.",
            },
            {
                "key": "reverse_arc_deg", "label": "Reverse arc — steering", "unit": "deg",
                "lo": 0, "hi": 55, "step": 1,
                "info": "The reverse into a corner is steered, not straight: this is how hard. "
                        "It swings the nose round so the forward part of the turn has less to do.",
            },
            {
                "key": "reverse_arc_leave_deg", "label": "Reverse arc — degrees left to the turn",
                "unit": "deg", "lo": 0, "hi": 90, "step": 1,
                "info": "How much of the 90° the reverse leaves for the forward turn to finish. "
                        "Smaller = the reverse does more of the corner itself.",
            },
        ],
    },
    {
        "title": "6 · Block or corner?",
        "blurb": "the front sensor cannot tell a pillar from a wall — this decides",
        "params": [
            {
                "key": "block_near_cm", "label": "A block this close counts as in front",
                "unit": "cm", "lo": 20, "hi": 120, "step": 1,
                "info": "A pillar seen closer than this is treated as being in the car's way. "
                        "While that is true, corner turns are blocked outright - otherwise the "
                        "car reads the block as a wall and turns 90° straight into it.",
            },
            {
                "key": "block_no_corner_s", "label": "...and blocks corners for", "unit": "s",
                "lo": 0, "hi": 6, "step": 0.1,
                "info": "How long that block lasts after the pillar was last SEEN. The memory "
                        "matters: up close the pillar fills or leaves the frame and the camera "
                        "loses it, which is exactly the moment the wall logic used to take over. "
                        "0 switches the whole protection off.",
            },
        ],
    },
    {
        "title": "7 · Driving straight",
        "blurb": "holding the lane down every straight, and after a dodge",
        "params": [
            {
                "key": "cruise_target_cm", "label": "Straights — hold this far off the wall",
                "unit": "cm", "lo": 20, "hi": 70, "step": 1,
                "info": "The distance from the outer wall the car aims to keep while simply "
                        "driving down a straight.",
            },
            {
                "key": "cruise_wall_k", "label": "Straights — how hard it corrects", "unit": "",
                "lo": 0.0, "hi": 5.0, "step": 0.1,
                "info": "Degrees of steering per cm off the target above. Higher = snappier but "
                        "can weave. 0 = ignore the walls on straights and steer by gyro alone.",
            },
            {
                "key": "recenter_target_cm", "label": "After a dodge — wall distance to return to",
                "unit": "cm", "lo": 30, "hi": 60, "step": 1,
                "info": "Where the car puts itself back to once a dodge is over. Kept separate "
                        "from the straight-line target so the return can be tuned on its own.",
            },
            {
                "key": "recenter_k", "label": "After a dodge — how hard it corrects", "unit": "",
                "lo": 0.1, "hi": 5.0, "step": 0.1,
                "info": "Degrees of steering per cm off the re-centring target. Higher gets back "
                        "to the lane quicker but overshoots.",
            },
            {
                "key": "servo_trim_deg", "label": "Servo trim — where straight really is",
                "unit": "deg", "lo": -15, "hi": 15, "step": 0.5,
                "info": "90 is not actually straight on this car. This is the correction, and it "
                        "matters more than it looks: the green dodge angle is mirrored about "
                        "TRUE straight, so getting this wrong makes green and red different "
                        "manoeuvres. Set it by driving the car straight down the mat.",
            },
            {
                "key": "default_lap_dir", "label": "Lap direction if the sensors cannot tell",
                "kind": "choice", "choices": ["CW", "CCW"],
                "info": "Only consulted when NEITHER side sensor gives a usable reading at the "
                        "first corner. CW = clockwise. If the car has guessed wrong in practice, "
                        "set the one you are actually running.",
            },
        ],
    },
    {
        "title": "8 · Speed",
        "blurb": "motor power. Every one is clamped to MAX_PWM, which the ESP32 enforces too.",
        "params": [
            {
                "key": "base_speed", "label": "Straights", "unit": "PWM",
                "lo": 40, "hi": 100, "step": 1,
                "info": "Motor power down a straight. The single biggest lever on lap time - and "
                        "on how far the car overshoots everything. Raising it past MAX_PWM does "
                        "nothing: the firmware clamps it back.",
            },
            {
                "key": "turn_speed", "label": "Through a corner", "unit": "PWM",
                "lo": 40, "hi": 100, "step": 1,
                "info": "Motor power while a corner is being taken. Lower than the straights "
                        "gives the front tyres a chance to bite instead of scrubbing.",
            },
            {
                "key": "reverse_speed", "label": "Reversing", "unit": "PWM",
                "lo": 40, "hi": 100, "step": 1,
                "info": "Motor power for every reverse. All of it is blind - there is no sensor "
                        "looking backwards - so this is a safety setting as much as a speed one.",
            },
        ],
    },
]


# Deliberately not shown. Every one of these is overwritten by
# config.mirror_green() on each load, so a slider for it would move a number
# and change nothing. The red one of each pair IS in the interface above.
HIDDEN = (
    "dodge_green_extra_cm",
    "dodge_past_green_extra_cm",
    "dodge_return_green_extra_cm",
    "dodge_align_min_green_cm",
)


def entries():
    """Every parameter in the interface, group by group, flat."""
    return [p for g in GROUPS for p in g["params"]]


def keys():
    return [p["key"] for p in entries()]


def check(defaults):
    """Compare this interface against config.DEFAULT_PARAMS.

    Returns (missing, extra): parameters the car reads but nothing here shows,
    and entries here for parameters that no longer exist. Both should be empty -
    dashboard.py logs them at startup and tools/preview_dashboard.py prints them.
    """
    mine = set(keys()) | set(HIDDEN)
    theirs = set(defaults)
    return sorted(theirs - mine), sorted(mine - theirs - set(HIDDEN))
