"""
config.py - every constant the Pi uses. Nothing else hard-codes a value.

The driving numbers are copied from the working open-round sketch
(open_round/Working_Open_Round_V2_FE), so the Pi drives the same way the ESP32
did on its own. Distances are in cm, like the sketch.
"""
import json
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(BASE_DIR, "logs")

# ---------------------------------------------------------------- serial link
SERIAL_PORT = None                    # None = auto-detect (first match below)
SERIAL_PORT_GLOBS = ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*")
SERIAL_BAUD = 115200
SERIAL_RECONNECT_S = 2.0
TELEMETRY_MAX_AGE_S = 0.3             # older than this -> stop the car
VISION_MAX_AGE_S = 0.5                # detections older than this are ignored.
                                      # The camera runs ~18 fps, so a live frame
                                      # is never close to this old; it only bites
                                      # when the stream stops - e.g. between runs,
                                      # where a stale frame would otherwise be
                                      # replayed into the next run's first step.
HUB_PORT = 5005                       # UDP port dashboard.py relays main.py on (localhost only)

# ------------------------------------------------- flashing the ESP32 from the Pi
# Used by flasher.py, which is what the three round buttons on the dashboard
# drive. These are paths on the Pi; on a laptop they simply never get called.
SKETCH_DIR = os.path.expanduser("~/sketches")
ARDUINO_CLI = os.path.expanduser("~/bin/arduino-cli")
ESPTOOL = os.path.expanduser("~/esp_flash/venv/bin/esptool")
ESP_FQBN = "esp32:esp32:esp32c3:CDCOnBoot=cdc"

# ---------------------------------------------------------------- servo / motor
SERVO_CENTER = 90
SERVO_MIN = 30                        # 2 deg inside the V2 linkage's mechanical stops (28..142)
SERVO_MAX = 140                       # so the servo never stalls against them
SERVO_TURN_LEFT = 45                  # SERVO_SHIFT_LEFT in the sketch
SERVO_TURN_RIGHT = 135                # SERVO_SHIFT_RIGHT

# 90 PWM and a 100 cap, set deliberately. 130/140 was tried and reverted on
# 2026-09-24 at the user's call - they know the car. Note for the record: 90 of
# 255 is ~35% duty on the core's ~1 kHz PWM, which is where Rohan found the
# open round cogging (d324d96). If the car ever moves in jerks again, this is
# the first place to look, not the control loop - that was measured clean at
# 48 Hz with commanded speed never once reaching zero.
MAX_PWM = 110                         # hard speed cap, forward AND reverse (ESP32 caps it too)
BASE_SPEED = 90                       # PWM 0..255, straights. 90 measures 25.9 cm/s
TURN_SPEED = 90                       # PWM, forward part of a turn
REVERSE_SPEED = 90                    # PWM, reversing

# ---------------------------------------------------------------- open round logic
TOTAL_TURNS = 12                      # 3 laps x 4 corners
TURN_THRESHOLD_CM = 60.0              # front distance that triggers a corner
SIDE_GAP_THRESHOLD_CM = 50.0          # side farther than this = open corner
EMERGENCY_FRONT_CM = 40.0             # turn even without an open side
# Blind lock (no side is clearly open) is how the FIRST corner can pick the
# wrong lap direction and take the whole race with it. One frame of L37/R42 -
# five centimetres, well inside ToF noise, and a pillar near a wall shortens
# that side anyway - is not evidence. Require a real gap, held steady.
LOCK_MIN_MARGIN_CM = 15.0             # one side must beat the other by this
LOCK_CONFIRM_FRAMES = 12              # ...for this many consecutive frames
TURN_COOLDOWN_S = 1.0                 # straight time after a turn
TURN_DONE_ERR_DEG = 13.0               # turn ends when heading error < this
TURN_TIMEOUT_S = 6.5                  # ...or after this long
FINISH_TIME_S = 3.0                   # drive on after turn 12 to reach the start section

REVERSE_START_CM = 12.0               # wedged: reverse below this...
REVERSE_STOP_CM = 20.0                # ...until the front is clear again

# Hard override: this close to something ahead, TURN. No side-gap check, no
# cooldown, no waiting for a clean read - the car is about to hit a wall and
# nothing else matters. The ONLY thing that suppresses it is an actual dodge
# already in progress, where the front sensor is looking at the pillar rather
# than at a wall.
FRONT_FORCE_TURN_CM = 48.0            # fallback; live value is params["front_turn_cm"]

# Corner turns are PROPORTIONAL, not a fixed lock: the steering follows the
# heading still to be turned, so the car goes in hard at the start and eases
# out as it comes round, instead of holding full lock to the last degree and
# scrubbing. corner_steer_deg is the CAP, reached only while the error is
# large. Starting earlier (front_turn_cm) with a smaller cap is what makes a
# turn smooth: the same 90 degrees spread over more ground.
CORNER_KP = 0.55                      # servo deg off centre per deg of heading error
CORNER_MIN_OFF_DEG = 7.0              # ...never less than this, or the turn stalls

# Dodging a pillar first seen FAR away should be gentler and longer, not the
# same sharp flick used when it is close: the car has room, so it can lean out
# over a longer distance. Scale = trigger distance / distance at commit,
# clamped - a pillar committed at twice the trigger distance gets roughly half
# the steering angle held over a longer leg.
DODGE_FAR_STEER_MIN = 0.55            # gentlest the angle may be scaled to
DODGE_FAR_LEG_MAX = 1.7               # longest the legs may be stretched to

# RECENTRE (after the dodge) is a controller, not a fixed mirrored lock. It
# drives the normal heading hold with the wall error folded in, and ends when
# the car is genuinely back in the lane - centred AND square - rather than
# after a set distance that was right once and wrong everywhere else.
RECENTER_DONE_CM = 7.0                # this close to lane centre...
RECENTER_DONE_DEG = 7.0               # ...and this square...
RECENTER_SETTLE_S = 0.35              # ...held this long = done
RECENTER_MAX_CM = 70.0                # failsafe: give up and cruise after this
RECENTER_MAX_NUDGE_DEG = 22.0         # most the wall error may bend the heading

# A turn may not start until the car has actually DRIVEN this far since the
# last one ended. Without it the forced override re-fires the instant a turn
# completes - the car is still pointing near the wall it just turned away from,
# so front is still under FRONT_FORCE_TURN_CM - and the car spins on the spot
# racking up "corners" without ever moving. Measured in encoder ticks, so it is
# real distance travelled; the time cooldown remains as the fallback when the
# encoder is not reporting.
MIN_TRAVEL_BETWEEN_TURNS_CM = 20.0

# THE CORNER, START TO FINISH:
#   1. a smooth drifting turn - the steering follows the heading still to be
#      turned (CORNER_KP, floor CORNER_MIN_OFF_DEG, cap corner_steer_deg), so
#      it goes in firm and eases out, exactly as the open round does
#   2. then reverse POST_TURN_REVERSE_CM straight back, for clearance on the
#      new straight
# There is no reversing BEFORE a corner any more: it fired only when the turn
# happened to begin close to a wall, so it appeared at some corners and not
# others, and it delayed every turn it did fire on.
POST_TURN_REVERSE_CM = 100.0          # fallback; live value is params["post_turn_reverse_cm"]
POST_TURN_REVERSE_TIMEOUT_S = 8.0     # failsafe if the encoder stops reporting.
                                      # 100 cm at 90 PWM (25.9 cm/s) takes ~3.9 s,
                                      # so this must be well clear of that or the
                                      # timeout would cut the reverse short.

# The travel gate below must never keep the car driving into a wall. Inside
# this distance the turn is allowed even if the car has not covered
# MIN_TRAVEL_BETWEEN_TURNS_CM yet - the back-off above is what stops this from
# becoming the old spin-on-the-spot, because the car physically moves back.
# How hard the wheels are cranked over during the reverse arc into a corner.
# Opposite lock, so the nose swings towards the new heading while the car backs
# away from the wall. Capped to the servo stops by _reverse_arc_steer().
# Which way the track is run, used ONLY when both side sensors read 999 and
# there is genuinely no measurement to decide from. "CW" = right-hand turns.
# A dodge ends with the car hard against one wall, so its side readings say
# nothing about where the corners are. Travel this far before the lap direction
# may be locked from them.
# Stands in for "no wall within range" when comparing the two sides, so an
# open side always beats a measured one however far that measurement is.
OPEN_CM = 999.0

# How far the car must travel after a dodge before it may adopt another
# pillar. Was a 1.2 s clock, which expired while the pillar just dodged was
# still beside the car: it got adopted again at ~21 cm and dodged a second
# time, into the space the corner needed.
PILLAR_CLEAR_CM = 60.0

# A pillar first seen closer than this cannot be dodged - the manoeuvre needs
# turn-out + run-past + turn-back-in, well over a metre of travel.
PILLAR_MIN_ACQUIRE_CM = 45.0
# ...but only within the window after a dodge, where the pillar just passed is
# still beside the car. Outside it a close block is a REAL block: refusing
# everything under 45 cm meant a Green at 33 cm was dropped while a Red at
# 50 cm was chased. Below this floor a detection is unusable, not merely tight.
PILLAR_MIN_ACQUIRE_FLOOR_CM = 15.0

# How much nearer a different-coloured pillar must be before the approach
# switches to it. A margin, not zero, so two blocks at similar range cannot
# make the car flip between them frame by frame.
PILLAR_SWITCH_MARGIN_CM = 20.0

# Decide the lap direction ONCE, from the parking bay, before the car moves:
# parked against the LEFT wall means a clockwise lap and right turns only.
# Deciding it per corner from whichever side looked open produced left turns on
# a clockwise track whenever a sensor dropped out or a dodge had just displaced
# the car.
LOCK_DIRECTION_AT_START = True

LOCK_AFTER_DODGE_CM = 50.0

DEFAULT_LAP_DIR = "CW"

REVERSE_ARC_DEG = 40.0

# How much of the 90 deg corner to leave for the forward turn once the reverse
# arc has done its part. The arc is flown on the IMU and stops here; the
# forward turn then finishes onto the exact heading target.
REVERSE_ARC_LEAVE_DEG = 35.0

CORNER_CRITICAL_FRONT_CM = 25.0

# A corner can finish with the car still nose-to-wall (the heading came good
# while wedged). The bypass above then lets a turn fire again immediately - and
# it MUST, or the car grinds into the wall waiting for travel it cannot make.
# But that second turn is the SAME corner being retried, not a new one: within
# this window, and with less than MIN_TRAVEL_BETWEEN_TURNS_CM covered, the
# heading target is left where it is and the turn is not counted. Without this
# the target gained another 90 deg each retry, so a left corner came out as a
# right and the heading ran away entirely (2026-09-23 19:37 run).
CORNER_RETRY_WINDOW_S = 3.0

# While the camera is LOOKING AT a pillar and closing on it, the corner logic
# must not steal the manoeuvre. The front ToF sees the pillar as if it were a
# wall, so without this the forced corner fires first and the car turns 90 deg
# into the block instead of dodging it (2026-09-23 run log: corners 3 and 7
# both fired mid-approach). Only a CURRENT detection holds the corner off -
# this many seconds since the pillar was last actually seen - and the abort
# distance below still overrides it, so the car can never drive into a wall.
APPROACH_CORNER_HOLD_S = 0.5

# Emergency abort of a dodge. Corners are normally suppressed mid-dodge because
# the front sensor is looking at the pillar, but that must not extend to driving
# into a wall: inside this distance the dodge is abandoned and the car turns.
# A committed dodge is only aborted by a wall the camera cannot explain.
# If a pillar was seen within DODGE_ABORT_PILLAR_MEMORY_S and was itself
# close, a short front reading is that pillar, not a new obstacle.
DODGE_ABORT_PILLAR_NEAR_CM = 45.0
DODGE_ABORT_PILLAR_MEMORY_S = 1.5

DODGE_ABORT_FRONT_CM = 16.0

# A dodge only triggers once the pillar has read closer than the trigger
# distance on this many CONSECUTIVE frames. The optical distance is derived
# from box height, which can spike on a single bad detection - one frame jumped
# 55 cm to 33 cm in 20 ms, firing the dodge far too early.
DODGE_TRIGGER_FRAMES = 3
# Shortest gap that can separate two camera frames. Only used when no frame
# counter is supplied, as a floor so control ticks cannot be mistaken for
# fresh looks at the world - the bug that dodged a pillar a metre early.
VISION_MIN_FRAME_S = 0.04

# APPROACH is just cruising while watching a pillar, so it must not be able to
# latch forever. If the block has not been seen for this long, the approach is
# abandoned and normal driving (including corners) resumes.
APPROACH_LOST_S = 1.5

# Every dodge starts from the same distance. A block picked up closer than this
# is reversed away from until it is this far, then dodged. Backing off also
# brings a pillar that is half out of frame fully back into view, so the
# distance taken from its bounding box is trustworthy again.
BACKOFF_TARGET_CM = 35.0
# Never reverse further than this chasing the target - the rear is blind.
BACKOFF_MAX_CM = 45.0
# How many times one pillar may be backed away from. More than one is allowed
# because a single back-off does not always end with a clean approach, but it
# is bounded so a block the car cannot resolve cannot trap it in a loop.
BACKOFF_MAX_TRIES = 2
# How long a block may sit inside the activation distance while the car is
# still only approaching. DODGE_TRIGGER_FRAMES of camera agreement takes about
# 0.2 s; past this the trigger is clearly not settling and the car reverses to
# a known distance rather than closing further on a number it cannot trust.
BACKOFF_GRACE_S = 0.6
# This far inside the activation distance is not "about to dodge", it is too
# close to dodge from at all - reverse straight away rather than waiting out
# the grace period.
BACKOFF_IMMEDIATE_CM = 8.0

# While closing on a pillar its measured distance can only fall. An increase
# larger than this from a CLIPPED box is the estimate breaking as the block
# leaves frame, not the pillar moving away, so the last good figure is kept.
PILLAR_DIST_GROW_CM = 12.0

# Log every detection the model returns, at most this often. 0 disables it.
# Cheap, and it settles "is it skipping them or not seeing them" with data.
LOG_DETECTIONS_S = 0.5


# Straight driving = BNO055 heading hold, with the ToF only nudging the heading.
#   centre error  = (right - left) / 2          cm, + means the car is left of centre
#   heading nudge = clamp(K_CENTER * error, +-MAX_CENTER_DEG)   (only when BOTH walls are seen)
#   steer = 90 + SERVO_TRIM + KP_HEADING * (heading error incl. nudge) + KI_HEADING * integral
# A side reading >= WALL_VALID_CM (a gap / 999 at a corner) switches the nudge
# off, so the car just holds its heading instead of steering into the gap.
KP_HEADING = 1.8                      # servo deg per deg of heading error (same as the open round)

# Straight-line heading gain, taken from final_open_round_working.ino, which
# uses `headingError * 1.2` alongside its wall term and no integral at all.
# KP_HEADING above still drives the corner turns.
KP_HEADING_CRUISE = 1.2
KI_HEADING = 0.4                      # servo deg per deg*s - removes a steady pull to one side
HEADING_I_LIMIT = 6.0                 # max servo deg from the integral
# 90 is NOT straight ahead on this car. Measured two independent ways from
# run_20260924_083553.csv: the heading controller settles on 87.1 deg while
# tracking a straight line, and a reverse commanded at exactly 90 curved LEFT
# at 0.45 deg/cm over 486 cm. Live value is params["servo_trim_deg"].
SERVO_TRIM = -3                       # servo deg added to straight (+ = right)

# Gain holding a heading while REVERSING. Negated inside _reverse_hold_steer:
# the same servo angle rotates the car the other way with the wheels going
# backwards, so a forward-sign correction would drive it further off.
KP_REVERSE_HEADING = 1.2
K_CENTER = 0.5                        # heading nudge, deg per cm off-centre
MAX_CENTER_DEG = 10.0                 # the ToF can never ask for more than this
WALL_VALID_CM = 80.0                  # side reading at/above this = no wall there (gap)

# A VL53L0X that cannot get a confident return reports an error, and the
# firmware turns any error into 999 - indistinguishable from "no wall there".
# A sensor reading a wall at 42 cm but failing a quarter of its frames would
# therefore look like an open corner on every dropout, which is enough to
# trigger a false corner and lock the wrong lap direction.
#
# So a 999 is only believed once it has PERSISTED. Inside this window the last
# good reading is held instead. A real gap stays 999 well beyond it, so genuine
# corners are unaffected; single bad frames vanish.
TOF_HOLD_S = 0.30

# A HIGH ToF reading is voted on over this window rather than believed at once.
# 999 means "no wall" to the driving logic, which is what commits a corner and
# picks the lap direction, so one spurious high reading could send the car the
# wrong way round the track. Below the sample floor the window is too thin to
# vote and the median of its real measurements is used instead.
TOF_VOTE_S = 0.25
TOF_VOTE_MIN_SAMPLES = 6

# ---------------------------------------------------------------- wheel encoder / distances
# The ESP32 streams a free-running tick count as the 13th DATA field. Every
# manoeuvre below is measured in TICKS, never in seconds, so the geometry is
# identical on a full battery and a flat one and does not change if the PWM is
# changed. This is the whole point of the encoder.
#
# Measured 2026-09-22 with esp/tick_m_calculate: 2494 ticks over a tape-measured
# 1.2 m, two runs agreeing within 1.3%. The old 3041 was 46% too high, so every
# distance ran 46% longer than its label - a 23 cm dodge leg was really 34 cm.
TICKS_PER_METER = 2078.0
TICKS_PER_CM = TICKS_PER_METER / 100.0

def cm_to_ticks(cm):
    """Distance in cm -> encoder ticks."""
    return int(round(cm * TICKS_PER_CM))

# Pillar dodge geometry, as distances travelled. Green = dodge LEFT,
# Red = dodge RIGHT (mirrored); the numbers are identical either way.
#   1. turn out away from the pillar .......... DODGE_OUT_CM
#   2. run straight past it ................... DODGE_PAST_CM
#   3. turn back in toward the lane ........... DODGE_IN_CM
#   4. straighten up on the IMU heading
# Fallbacks only - the live values come from params.json via DEFAULT_PARAMS
# ("dodge_short_cm" / "dodge_long_cm"). These are what get used if a params
# push has not arrived yet.
DODGE_OUT_CM = 23.0
DODGE_PAST_CM = 52.0
DODGE_IN_CM = 23.0

# Where the turn-back-in stops unwinding. It used to unwind all the way to
# straight (90); ending at 80 leaves a slight steer held ON in the straightening
# direction - for Green (which came back to the right) that is a touch of left,
# and it is mirrored to 100 for Red. ALIGN then squares the car up.
DODGE_RETURN_END_DEG = 80.0

# A pillar already sitting on the side it has to end up on needs no dodge at
# all: the car will pass it correctly just by driving on. Red must finish on
# the car's LEFT (the car goes right), so a red already in the LEFT band is
# fine; green must finish on the car's RIGHT, so a green in the RIGHT band is
# fine. Checked against the same ROI bands the dodge uses.

# A pillar counts as "already on the correct side" only if it is genuinely off
# to the side, not merely drifted into the right band by perspective as the car
# closed in. Either far enough away that the call can be made later, or its
# near edge clear of the centre line by this fraction of half the frame.
PILLAR_SKIP_MIN_DIST_CM = 60.0
PILLAR_SKIP_MIN_OFFSET = 0.45

# While a pillar is tracked, a front ToF reading within this of the camera's
# optical distance is the pillar itself, not a wall, so it must not trigger a
# corner.
BLOCK_TOF_AGREE_CM = 25.0

# Commit the dodge once a TRACKED pillar is this close on the front ToF, even
# if the camera's box-height distance still reads long. The camera says WHAT it
# is; the ToF says how far. Without this the car drove to 15 cm and the pillar
# was taken as a corner.
# Commit the dodge once a tracked pillar is this close on the front ToF - but
# ONLY when that pillar is CENTRED, because the front ToF looks straight ahead.
# An off-centre pillar cannot be what it is reading, so a short reading then
# belongs to a wall and the corner takes it. That is the real discriminator;
# the threshold alone is not. Sitting just above front_turn_cm means a pillar
# dead ahead is dodged, while anything off to the side lets the turn go first.
PILLAR_TOF_COMMIT_CM = 40.0

# ...and only for a pillar the front ToF could actually be looking at. Used
# when the detection has no box edges; block_roi's CENTER band is preferred.
PILLAR_CENTRED_FRAC = 0.33
# How recently the camera must have seen a pillar close for a short front
# reading to be attributed to it rather than to a wall.
BLOCK_NEAR_MEMORY_S = 1.5

PILLAR_SKIP_WHEN_ALREADY_CLEAR = False   # off for now - every pillar gets dodged

# ---------------------------------------------------------------- region of interest
# The frame is split into three equal vertical bands. A pillar is assigned to
# whichever band holds the MAJORITY OF ITS BOX AREA - a sliver poking into the
# centre does not count, the bulk does. Because the box height is the same in
# every band, comparing overlapping WIDTH is the same as comparing area.
#
# The band is latched ONCE, at the moment the dodge triggers, and never
# re-evaluated mid-manoeuvre - the pillar leaves the frame as soon as the car
# turns, so a live reading would be worthless.
ROI_LEFT_END_FRAC = 1.0 / 3.0
ROI_RIGHT_START_FRAC = 2.0 / 3.0

# Adjusts the CENTER band width by this many pixels IN TOTAL, taken half off
# each side. POSITIVE narrows the centre, NEGATIVE widens it.
#
# Widened to -120 on 2026-09-23: at +20 the centre band was 331 px of 1024 and
# blocks sitting just outside it were judged "already on the correct side" and
# skipped - but the car drives down the middle, so it went on to clip them. A
# wider centre means a borderline block counts as CENTER, gets dodged rather
# than skipped, and the ones that really are out of the way still fall to
# LEFT/RIGHT.
ROI_CENTER_SHRINK_PX = -75.0          # scaled with the frame: -120 was for 1024 wide


def roi_bounds(frame_w=None):
    """(left_end, right_start) in pixels, centre band shrunk.

    The single source of truth for where the bands sit: block_roi() decides
    with it and the dashboard overlay draws with it, so the picture can never
    disagree with the behaviour.
    """
    if frame_w is None:
        frame_w = CAMERA_SIZE[0]
    half = ROI_CENTER_SHRINK_PX / 2.0
    left_end = frame_w * ROI_LEFT_END_FRAC + half
    right_start = frame_w * ROI_RIGHT_START_FRAC - half
    if right_start < left_end:          # absurd shrink -> degenerate centre
        left_end = right_start = (left_end + right_start) / 2.0
    return left_end, right_start

# A pillar on the far side it has to be dodged AWAY from needs a wider berth:
# Green (dodged left) sitting in the LEFT band, or Red (dodged right) sitting
# in the RIGHT band. Those get a gentler steer held for a longer distance,
# which traces a wider, smoother arc than the normal sharp-and-short dodge.
# Anything in the other two bands uses the normal geometry.
DODGE_WIDE_STEER_DEG = 60.0     # servo angle for Green; mirrored to 120 for Red
DODGE_WIDE_OUT_CM = 35.0        # fallback; live value is params["dodge_long_cm"]
DODGE_WIDE_IN_CM = 35.0         # matching return, so it corrects back the same amount

# Sanity rails for the tunable leg lengths, so a bad params.json edit cannot
# send the car on a 3 metre excursion or collapse a stage to nothing.
DODGE_CM_MIN = 5.0
DODGE_CM_MAX = 100.0


def block_roi(block, frame_w=None):
    """Which third of the frame holds most of this block: 'LEFT'/'CENTER'/'RIGHT'.

    Uses the box edges, so the answer is by area rather than by centre point.
    Falls back to the centre point if the detection has no edges.
    """
    if frame_w is None:
        frame_w = CAMERA_SIZE[0]
    left_end, right_start = roi_bounds(frame_w)

    x1 = block.get("x1")
    x2 = block.get("x2")
    if x1 is None or x2 is None:
        cx = block.get("cx")
        if cx is None:
            return "CENTER"
        return "LEFT" if cx < left_end else ("RIGHT" if cx >= right_start else "CENTER")

    if x2 < x1:
        x1, x2 = x2, x1

    bands = {
        "LEFT":   (0.0, left_end),
        "CENTER": (left_end, right_start),
        "RIGHT":  (right_start, float(frame_w)),
    }
    overlaps = {name: max(0.0, min(x2, hi) - max(x1, lo)) for name, (lo, hi) in bands.items()}
    return max(overlaps, key=overlaps.get)

# Backstop only. If the encoder ever stops reporting mid-stage the car would
# otherwise hold full lock forever, so each stage also gives up after this long.
# It is a failsafe, NOT the normal way a stage ends.
STAGE_TIMEOUT_S = 6.0

# Squaring up after the turn-in. This one ends on ANGLE, not distance - there
# is no distance to cover, only a heading error to kill. ALIGN_MAX_CM caps how
# far the car may travel while doing it.
ALIGN_DONE_ERR_DEG = 5.0
ALIGN_MAX_CM = 45.0

# ---------------------------------------------------------------- camera (dashboard.py owns it)
CAMERA_ENABLE = True
# Which camera to use: "auto" tries the CSI Pi camera and falls back to a USB
# webcam, "csi" and "usb" force one. The USB path goes through OpenCV V4L2 and
# asks for MJPG, because a USB 2.0 bus cannot carry uncompressed 1024x576 at
# 30 fps. CAMERA_USB_DEVICE None = probe /dev/video* and take the first node
# that actually returns a frame (a UVC webcam also claims a metadata node).
CAMERA_BACKEND = "auto"
CAMERA_USB_DEVICE = None
CAMERA_USB_FOURCC = "MJPG"
# 640x360, NOT 1024x576. The detector letterboxes whatever arrives down to
# 320x320, and both resolutions produce an IDENTICAL 320x180 image for it - so
# this costs nothing in detection and captures 39% of the pixels. The camera
# feed was lagging and the Pi running hot at 1024x576.
#
# CHANGING THIS AGAIN means scaling every pixel-measured constant with it:
# PILLAR_FOCAL_PX, ROI_CENTER_SHRINK_PX, params["pixel_cm_per_px"] and
# pixel_distance_method.FRAME_W. The ROI band edges are fractions and do not.
CAMERA_SIZE = (640, 360)

# The C920 drops off the USB bus under load and comes back as a NEW device
# number, which kills the open capture handle for good. Reconnect rather than
# run blind: this many consecutive failed grabs means the device has gone.
CAMERA_REOPEN_AFTER_FAILS = 25
CAMERA_REOPEN_PERIOD_S = 2.0

# No frame for this long means the camera is gone, whatever fps_measured
# still says - it keeps its last value when the device vanishes.
CAMERA_STALE_S = 1.5
CAMERA_FORMAT = "RGB888"              # picamera2 "RGB888" is BGR byte order = OpenCV order
CAMERA_FPS = 30
CAMERA_ROTATE_180 = False
CAMERA_OPEN_RETRIES = 3

# ---------------------------------------------------------------- pillar detection (YOLO, runs in dashboard.py)
VISION_ENABLE = True
MODEL_PATH = os.path.join(BASE_DIR, "models", "wro_det320.onnx")   # YOLOv8n, 320 px, Green/Magenta/Red
BLOCK_CLASSES = ("Red", "Green")      # Magenta is in the model but not used yet
BLOCK_CONF = 0.5                      # minimum confidence
BLOCK_IOU = 0.45                      # NMS overlap

# A pillar is a 5x10 cm BLOCK standing up: in the picture it is a compact,
# TALLER-THAN-WIDE box. The coloured lines painted on the mat are the opposite -
# long, flat and thin - and the model sometimes calls one a red pillar. Two
# cheap shape tests throw those out before anything acts on them:
#
#   too small  - a detection under BLOCK_MIN_AREA_FRAC of the frame is noise;
#                a real pillar at 100 cm still fills about 0.5% of it
#   too flat   - a real pillar is about twice as tall as it is wide; a line is
#                many times wider than it is tall. Anything wider than
#                BLOCK_MAX_ASPECT (width / height) is not a pillar.
#
# Both are deliberately loose: they reject lines and specks, not pillars seen at
# an angle or half out of frame.
BLOCK_MIN_AREA_FRAC = 0.0015          # 0.15% of the frame = ~900 px at 1024x576
BLOCK_MAX_ASPECT = 1.30               # width / height above this = a line, not a pillar

# Shape alone cannot separate RED from ORANGE: the orange line on the mat, seen
# at an angle or partly out of frame, can look compact enough to pass. Colour
# can - they are far apart in hue. Every Red detection is checked against the
# actual pixels in its box: the median hue of the colourful ones must be red
# (wrapping through 0), or it is an orange line and gets thrown away.
#
# OpenCV hue is 0..179, so red sits at 0-10 and again at 170-179, with orange
# immediately above red at roughly 11-25. Widen RED_HUE_MAX if a genuine pillar
# is ever rejected under warm lighting.
RED_HUE_CHECK = True
RED_HUE_MAX = 10                      # 0..RED_HUE_MAX is red...
RED_HUE_MIN_WRAP = 168                # ...and RED_HUE_MIN_WRAP..179 is red too
RED_HUE_MIN_SAT = 70                  # ignore washed-out pixels when judging
RED_HUE_MIN_VAL = 50                  # ...and dark ones
RED_HUE_MIN_PIXELS = 25               # too few colourful pixels to judge = keep it
ONNX_THREADS = 3
PILLAR_HEIGHT_CM = 10.0               # WRO traffic sign: 10 cm tall...
PILLAR_WIDTH_CM = 5.0                 # ...and 5 cm wide. Used when the box is
                                      # cut off by the top/bottom of the frame,
                                      # where the height is no longer the whole
                                      # pillar and would read too far away.
PILLAR_FOCAL_PX = 443.8               # scaled with the frame: 710 was for a 1024-wide frame

# ---------------------------------------------------------------- tunable pillar avoidance params
PARAMS_FILE = os.path.join(BASE_DIR, "params.json")
DEFAULT_PARAMS = {
    "pillar_trigger_dist_cm": 25.0,   # ACTIVATION DISTANCE: dodge starts here
    # RED IS THE MASTER, GREEN MIRRORS IT. The two colours are the same
    # manoeuvre in opposite directions, so tuning them separately only means
    # one side is always the stale one. load_params() derives the green angle
    # as the mirror of the red one about centre (90), and every other dodge
    # setting - trigger distance, leg lengths, hold, timings - is already
    # shared by both colours.
    "pillar_steer_left": 60,          # DERIVED: 180 - pillar_steer_right
    "pillar_steer_right": 120,        # sharp right steer for Red (the one to tune)
    "pillar_focal_px": 710.0,         # camera focal length in pixels (calibrated to 710)
    "turn_hold_pct": 30,              # % of turn time to hold sharp angle before unwinding
    "block_offset_sens": 0.8,         # sensitivity to block off-center position (0=off)
    "recenter_target_cm": 45.0,       # target distance from wall to lane center
    "recenter_k": 1.5,                # recentering sensitivity (deg per cm offset)
    "cruise_wall_k": 1.2,          # 0 = pure gyro on the straights; re-centring still uses recenter_k

    # Dodge leg lengths, in cm of measured travel. SHORT is the normal dodge;
    # LONG is the wider arc used when the pillar already sits on the side the
    # car has to dodge towards (Green in LEFT band, Red in RIGHT). Both the
    # turn-out and the turn-back-in use the same figure, so the car corrects
    # back exactly as far as it went out. Tunable here so they can be changed
    # without a redeploy - the fixed geometry stays fixed per run either way.
    # Leg 2: how far the car runs STRAIGHT past the block between turning out
    # and turning back in. Was a fixed constant with no slider, which is why
    # nothing on the dashboard appeared to change it.
    "dodge_past_cm": 45.0,

    "dodge_short_cm": 23.0,
    "dodge_long_cm": 35.0,
    # Green keeps bumping the pillar while Red clears it, on steering that is
    # an exact mirror - so the difference is mechanical, not logical. This is
    # added to the Green turn-out leg only. Red is untouched.
    # Per-colour dodge trim. Green: turn out, run past, swing back in.
    "dodge_green_extra_cm": 2.0,          # turn-out leg, on top of dodge_short_cm
    "dodge_red_extra_cm": 2.0,
    "dodge_past_green_extra_cm": 5.0,     # run past the pillar, on top of DODGE_PAST_CM
    "dodge_past_red_extra_cm": 0.0,
    # 7.0 put the last inward turn about 3 cm too far into the lane - the car
    # cut back across the line it had just left - so this is 4.0 now.
    "dodge_return_green_extra_cm": 4.0,   # swing back in, on top of dodge_return_extra_cm
    "dodge_return_red_extra_cm": 0.0,
    # Minimum forward run while squaring up, so the dodge cannot finish with
    # the car still alongside the pillar it just passed.
    "dodge_align_min_red_cm": 2.0,
    "dodge_align_min_green_cm": 0.0,
    # Pixel-offset dodge: size the turn-out from where the pillar sits in the
    # frame instead of a wide/normal arc split. See pixel_distance_method.py.
    "use_pixel_dodge": True,
    "pixel_cm_per_px": 0.080,       # scaled with the frame: 0.05 was for 1024 wide
    "pixel_base_cm": 31.15,        # 40 - 177 x 0.05, so 177 px gives 25 + 15 cm
    # Pixel-offset dodge: size the turn-out from where the pillar sits in the
    # frame instead of a wide/normal arc split. See pixel_distance_method.py.
    # Lane centring on the straights, from final_open_round_working.ino:
    # kp 1.2 holding 42 cm off the outer wall. Cruise keeps its own target so
    # the post-dodge re-centre can stay on recenter_target_cm.
    "cruise_target_cm": 42.0,
    # Reverse arc into each corner - see _reverse_arc_steer().
    "reverse_arc_deg": 40.0,
    "reverse_arc_leave_deg": 35.0,   # degrees left for the forward turn
    # Only consulted when neither side sensor gives a usable reading.
    "default_lap_dir": "CW",
    # Servo degrees added to straight. 90 is not straight on this car.
    "servo_trim_deg": -3.0,
    # Return leg = turn-out leg + this extra, and the mirrored return
    # angle pulled back toward centre by reduce_deg. Both were missing
    # here, so load_params() dropped them and the dodge never saw them.
    "dodge_return_extra_cm": 3.0,
    "dodge_return_reduce_deg": 0.0,

    # Corner turn steering, in degrees OFF CENTRE (so 30 -> servo 120 right /
    # 60 left). The open-round sketch uses 45, but at full lock the front tyres
    # scrub instead of biting: the servo swings over and the car carries on
    # roughly straight. A shallower angle actually rotates the car.
    # --- drive power. Promoted from constants so they can be tuned at the
    # venue without editing code and reflashing. Every one is still clamped to
    # MAX_PWM, which stays a constant because the ESP32 enforces its own copy
    # and raising it here alone would be silently clipped back.
    "base_speed": 90,                 # PWM on the straights
    "turn_speed": 90,                 # PWM through a corner
    "reverse_speed": 90,              # PWM reversing - all of it is blind
    # --- the "wide" turn-out angle, for a pillar already on the dodge side.
    # --- parking exit, run once at the start of the round ---
    "park_enabled": 1,          # 0 = skip it and start the round where it stands
    "park_turn_deg": 90,        # how far to swing out of the bay, and back
    "park_forward_cm": 30,      # straight leg after the swing - just clear of the bay
    "park_back_cm": 20,         # reverse before squaring up
    "park_steer_deg": 53,       # full lock: 87+53 = 140, 87-53 = 34 (same as the sketch)
    "park_speed": 90,           # PWM for the whole manoeuvre
    "park_max_cm": 400,         # failsafe: give up on a step past this and race anyway

    "dodge_wide_steer_deg": 60.0,     # written green-side; mirrored for red
    "corner_steer_deg": 28.0,

    # Straight reverse AFTER every corner, for clearance on the new straight.
    "post_turn_reverse_cm": 100.0,
    "pre_turn_reverse_cm": 10.0,   # reverse BEFORE a corner, for room to swing

    # Front distance at which the car turns, come what may - no side-gap check,
    # no direction-lock wait. Turning late means arriving at the wall before
    # committing, which leaves the car hard against it instead of swinging
    # round from the middle of the lane. Raise this to turn earlier.
    "front_turn_cm": 48.0,

    # BLOCK IN FRONT -> NO CORNER. The front ToF cannot tell a pillar from a
    # wall, so a block the car has closed right up to reads like a corner and
    # the car turns 90 deg into it. While a pillar has been SEEN closer than
    # block_near_cm within the last block_no_corner_s seconds, corner turns are
    # blocked outright, whatever the front ToF says. The memory matters: up
    # close the pillar fills or leaves the frame and the detection drops out,
    # which is exactly the moment the wall logic used to take over.
    # Set block_no_corner_s to 0 to switch this off.
    "block_near_cm": 60.0,
    "block_no_corner_s": 2.5,

    # POST-DODGE STRAIGHT. After a dodge finishes the car must run clear of the
    # pillar it just passed before it is allowed to adopt another one, or the
    # camera re-adopts that same pillar sitting right beside it. Two gates, both
    # have to pass: distance travelled since DODGE COMPLETE, and a plain clock
    # on top of it. Was hard-coded (PILLAR_CLEAR_CM and a literal 1.2 s) with no
    # slider, so neither could be tuned track-side.
    "pillar_clear_cm": PILLAR_CLEAR_CM,
    "pillar_clear_delay_s": 1.2,
}

# Green's dodge legs are Red's, always - the same rule the steering angle
# already follows. Tuned separately they drifted: green ran 10 cm past the
# pillar against red's 5, and 2 cm of turn-out trim against red's 6, so the two
# colours traced visibly different paths round the same pillar. Whatever is
# saved for the green side is overwritten here, so they cannot drift again.
GREEN_MIRRORS_RED = (
    ("dodge_green_extra_cm", "dodge_red_extra_cm"),
    ("dodge_past_green_extra_cm", "dodge_past_red_extra_cm"),
    ("dodge_return_green_extra_cm", "dodge_return_red_extra_cm"),
    ("dodge_align_min_green_cm", "dodge_align_min_red_cm"),
)

def mirror_green(params):
    """Make every green dodge setting the mirror of the red one, in place.

    The steer angle mirrors about the car's TRUE straight, not 90: with
    servo_trim_deg = -3 straight is 87, so red 135 (48 deg off straight)
    mirrors to 39, not to 45. Mirroring about 90 made green a 42 deg turn
    against red's 48 - the same manoeuvre on paper, six degrees softer on the
    mat. main.py derives the angle it actually drives the same way; this keeps
    the figure shown on the dashboard honest.
    """
    try:
        trim = float(params.get("servo_trim_deg", SERVO_TRIM))
    except (TypeError, ValueError):
        trim = 0.0
    tc = SERVO_CENTER + trim
    params["pillar_steer_left"] = int(round(max(SERVO_MIN, min(SERVO_MAX,
                                     tc - (params["pillar_steer_right"] - tc)))))
    for green, red in GREEN_MIRRORS_RED:
        params[green] = params[red]
    return params

def load_params():
    params = dict(DEFAULT_PARAMS)
    if os.path.exists(PARAMS_FILE):
        try:
            with open(PARAMS_FILE, "r") as f:
                data = json.load(f)
                for k, v in data.items():
                    if k in params:
                        params[k] = type(params[k])(v)
        except Exception:
            pass
    # Green is the mirror of Red, always. Whatever is saved for the green side
    # is ignored, so the two can never drift apart.
    return mirror_green(params)

def save_params(params, save_to_disk=True):
    p = load_params()
    p.update(params)
    # Again AFTER the update: this dict is what goes live to main.py, so a red
    # slider moved just now has to carry green with it on this same write.
    mirror_green(p)
    if save_to_disk:
        try:
            with open(PARAMS_FILE, "w") as f:
                json.dump(p, f, indent=2)
        except Exception:
            pass
    return p

# ---------------------------------------------------------------- OLED
# SSD1306 on the Pi's own I2C bus - found at 0x3c on i2c-1. It shows whether
# the car is ready, how many turns it has done and how long the run has taken,
# so the track side needs no laptop. Nothing depends on it: if it is unplugged
# the round carries on exactly the same.
OLED_ENABLE = True
OLED_BUS = 1
OLED_ADDR = 0x3C
OLED_WIDTH = 128
OLED_HEIGHT = 64                      # set 32 for the short panels
OLED_ROTATE_180 = False
OLED_FONT_BIG = 22
OLED_FONT_SMALL = 13
OLED_REFRESH_S = 0.5
OLED_NET_REFRESH_S = 5.0              # how often to ask NetworkManager for the SSID

# ---------------------------------------------------------------- dashboard
DASHBOARD_ENABLE = True
DASHBOARD_PORT = 8080
DASHBOARD_STREAM_FPS = 12
DASHBOARD_JPEG_QUALITY = 70
DASHBOARD_LOG_LINES = 40
COMPETITION_FLAG_FILE = "/boot/firmware/COMPETITION"   # rule 11.10: no dashboard

