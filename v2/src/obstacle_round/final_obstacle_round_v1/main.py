"""
main.py - Pi master: 3 laps of the open round (no pillars yet).

The ESP32 (esp/open_slave) only sends sensor data and applies commands. All
decisions are made here, using the same logic and numbers as the working
open-round sketch:

    STRAIGHT   hold the BNO055 heading (PI); the ToF only nudge that heading
               back to the lane centre, and only when both walls are seen
    TURNING    front <= 60 cm and the side is open -> fixed full lock until the
               heading is 90 deg on (or 6.5 s). Wedged (front < 12 cm) ->
               reverse with opposite lock until front > 20 cm.
    FINISHING  after 12 turns: drive on for 3 s to reach the start section
    DONE       stopped

Direction (CW / CCW) is locked at the first corner from which side is open.

It does not open the serial port itself: dashboard.py (dashboard.service,
always running from boot) owns the port and relays for it - see HubLink.

Run:  python3 main.py   (or master.service), then press START on the car.
Dashboard: http://error404.local:8080/
"""
import collections
import csv
import logging
import os
import signal
import time

import config
from serial_link import HubLink

log = logging.getLogger("main")


def heading_error(target, current):
    d = target - current
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


def compute_wall_error_cm(left, right, target_dist=45.0, avoid_color=None):
    """Calculates lateral error from lane center in cm (+ = car is left of center, steer right).
    If avoid_color is specified, relies strictly on the reliable outer wall to prevent being
    fooled by the pillar on the inner side ToF sensor!
    """
    left_valid = 0 < left < config.WALL_VALID_CM
    right_valid = 0 < right < config.WALL_VALID_CM

    # If we just avoided Green (dodged left), outer wall is left; pillar is on right
    if avoid_color == "Green" and left_valid:
        return target_dist - left
    # If we just avoided Red (dodged right), outer wall is right; pillar is on left
    if avoid_color == "Red" and right_valid:
        return -(target_dist - right)

    if left_valid and right_valid:
        return (right - left) / 2.0
    elif left_valid:
        return target_dist - left
    elif right_valid:
        return -(target_dist - right)
    return 0.0


def compute_center_nudge(left, right, target_dist=45.0, k_center=0.5, max_deg=10.0):
    err_cm = compute_wall_error_cm(left, right, target_dist=target_dist)
    nudge = k_center * err_cm
    return max(-max_deg, min(max_deg, nudge))


def center_nudge(left, right):
    return compute_center_nudge(left, right, target_dist=45.0, k_center=config.K_CENTER, max_deg=config.MAX_CENTER_DEG)



def open_round_wall_centering(left, right, lock, kp=1.2, target=42.0, valid=80.0):
    """Wall centring, ported from final_open_round_working.ino.

    Returned in SERVO DEGREES, to be added straight onto 90 - NOT a heading
    nudge. Feeding it through the heading P term (as the nudge version did)
    lets the heading controller cancel it out, and clamping it to +-10 deg
    leaves it too weak to actually pull the car to the middle.

    Both walls seen -> balance them. One wall -> hold `target` off it, choosing
    the outer wall from the locked lap direction. No wall -> hold heading only.
    """
    left_ok = 0 < left < valid
    right_ok = 0 < right < valid

    if left_ok and right_ok:
        return (right - left) * (kp * 0.5)
    if lock == 1:                       # clockwise: right wall is the outer one
        if right_ok:
            return -(target - right) * kp
        if left_ok:
            return (target - left) * kp
    elif lock == -1:                    # anticlockwise: left wall is the outer
        if left_ok:
            return (target - left) * kp
        if right_ok:
            return -(target - right) * kp
    else:
        if right_ok:
            return -(target - right) * kp
        if left_ok:
            return (target - left) * kp
    return 0.0


def clamp_servo(a):
    return int(max(config.SERVO_MIN, min(config.SERVO_MAX, round(a))))


def already_clear(block, colour, params=None, frame_w=None):
    """True when this pillar needs no dodge at all.

    Red has to finish on the car's LEFT (the car goes right), so a red pillar
    already in the LEFT band will be passed correctly by simply driving on.
    Green is the mirror. Uses the same ROI bands the dodge itself uses, so the
    picture on the dashboard and the decision can never disagree.
    """
    if not config.PILLAR_SKIP_WHEN_ALREADY_CLEAR:
        return False
    roi = config.block_roi(block, frame_w)
    on_correct_side = ((colour == "Red" and roi == "LEFT")
                       or (colour == "Green" and roi == "RIGHT"))
    if not on_correct_side:
        return False

    # Band membership alone is not clearance. As the car closes on a pillar,
    # perspective sweeps it towards the edge of frame, so a block dead ahead
    # drifts into the "correct" band while still being right in the car's path.
    # That is what dropped a green at 31 cm and left it re-acquired at 19 cm,
    # far too late to dodge. Require it to be genuinely off to the side: either
    # far enough away that there is still room to act, or visibly clear of the
    # car's centre line.
    if frame_w is None:
        frame_w = config.CAMERA_SIZE[0]
    try:
        dist = float(block.get("dist_cm", 0) or 0)
    except (TypeError, ValueError):
        dist = 0.0
    edge = block.get("x1") if colour == "Green" else block.get("x2")
    if edge is None:
        edge = block.get("cx")
    if edge is None:
        return False        # no box edges: cannot judge clearance, so do not skip

    # How far the near edge of the block sits from the centre of the frame,
    # as a fraction of half the width. Small = still in front of the car.
    offset = abs(float(edge) - frame_w / 2.0) / (frame_w / 2.0)
    if dist >= config.PILLAR_SKIP_MIN_DIST_CM:
        return False        # too far to judge yet - keep approaching and decide later
    return offset >= config.PILLAR_SKIP_MIN_OFFSET


class ObstacleRound:
    """Control step for single-pillar avoidance sequence: step(data, blocks, params, now) -> (speed, steer)."""

    def __init__(self, start_yaw, now):
        self.target = start_yaw
        self.t_start = now
        self.state = "RUNNING"
        self.turns = 0
        self.lock = 0
        self.lock_vote = 0                 # which way the blind lock is leaning
        self.dodge_end_ticks = None        # encoder count when the last dodge ended
        self.lock_wait_logged = False
        self._tof_win = {}                 # rolling window per ToF, for the vote
        self.corner_target = 0.0           # heading the corner is aiming at
        self.corner_target_set = False     # ...fixed before the reverse arc
        self.lock_votes = 0                # consecutive frames it has leaned there
        self.turn_dir = 0
        self.turn_t0 = 0.0
        self.last_turn_end = -1e9
        self.reversing = False
        # Corner turning, ported from Working_Open_Round_V2_FE.ino
        self.is_turning = False
        self.is_reversing = False
        self.turn_steer = config.SERVO_CENTER
        self.turn_end_ticks = 0         # tick count when the last turn finished
        self.finish_t0 = None
        self.post_turn_reverse = False     # backing up after a finished corner
        self.pre_turn_reverse = False      # backing up BEFORE the turn, for room to swing
        self.pre_turn_ticks0 = 0
        self.pre_turn_t0 = 0.0
        self.post_turn_ticks0 = 0
        self.post_turn_t0 = 0.0
        self.skip_logged = None
        self.near_skip_logged = None
        self.tof_disagree_logged = False
        self._det_log_t = -1e9             # rate limit for the detection log
        self.block_near_centred = False    # was that near block dead ahead?
        self.block_near_t = -1e9           # last time a pillar was seen CLOSE
        self.block_corner_blocked = False
        self.turn_is_retry = False
        self.commit_dist = None            # pillar distance when the dodge committed
        self.recenter_ok_t = None          # when the car first looked centred
        self.backoff_ticks0 = 0
        self.backoff_t0 = 0.0
        self.herr = 0.0
        self.nudge = 0.0
        self.i_term = 0.0
        self.last_t = now

        # Single pillar avoidance stage state:
        # None -> "APPROACH" -> "TURN_OUT" -> "STRAIGHTEN" -> "RECENTER" -> "HALT"
        self.pillar_stage = None
        self.pillar_color = None
        self.pillar_conf = 0.0
        self.pillar_dist = 0.0
        self.pillar_cx = None
        self.pillar_box = None          # last seen detection, for the ROI decision
        self.pillar_roi = None          # band latched when the dodge triggered
        self.dodge_wide = False         # True = wider, gentler arc for this dodge
        self._params = {}               # newest params push, for the tunable leg lengths
        self.stage_t0 = 0.0
        # Distance bookkeeping. Every dodge stage ends on ticks travelled, not
        # on a timer, so the shape of the manoeuvre does not change with
        # battery charge or PWM. stage_ticks0 is the tick count when the
        # current stage began; ticks_ok is False if the ESP32 is old firmware
        # that does not report ticks, in which case we fall back to timers.
        self.stage_ticks0 = 0
        self.ticks = 0
        self.ticks_ok = False
        self.stage_cm = 0.0
        self.pillar_clear_t = 0.0
        self.pillar_seen_t = 0.0        # when the tracked pillar was last actually seen
        self.trigger_frames = 0         # consecutive CAMERA frames inside the trigger
        self.vision_seq = None          # set by the main loop; None = unknown
        self.trigger_seq = None         # the frame trigger_frames last counted
        self.trigger_frame_t = -1e9     # fallback clock when there is no seq
        self._tof_last = {}             # sensor -> (last good cm, when), for de-glitching
        self.tof_held = 0               # how many sensors are currently being held
        self.decision_text = "No pillar → driving straight / centring"

    @property
    def direction(self):
        return {1: "CW", -1: "CCW"}.get(self.lock)

    def _straight(self, d, now, ignore_tof=False, target_dist=None, k_center=None, max_nudge=None):
        """Cruise: hold the heading, with the ToF bending it back to lane centre.

        With BOTH walls seen it balances them. With only ONE - the other timed
        out, or there is a gap there - it holds target_dist off the wall it can
        see, which walks the car towards the empty side until the two read
        alike again. That is the case that used to be ignored entirely, so the
        car simply held its heading and stayed wherever it was in the lane.

        The gains are the live "lane centre" sliders, so they now govern
        cruising as well as the re-centre after a dodge, as their labels say.
        """
        p = self._params or {}
        dt = min(0.1, max(0.0, now - self.last_t))
        if ignore_tof:
            self.nudge = 0.0
        else:
            if target_dist is None:
                try:
                    target_dist = float(p.get("cruise_target_cm", 42.0))
                except (TypeError, ValueError):
                    target_dist = 42.0
            if k_center is None:
                # Cruising and post-dodge RE-CENTRING are different jobs and now
                # have different gains. cruise_wall_k = 0 gives pure gyro on the
                # straights WITHOUT also disabling the re-centre, which shares
                # recenter_k and needs the ToF to find the middle of the lane.
                try:
                    k_center = float(p.get("cruise_wall_k", 0.0))
                except (TypeError, ValueError):
                    k_center = 0.0
            # final_open_round_working's centring: the wall term goes STRAIGHT
            # onto the servo angle, not through the heading controller, and is
            # not clamped to +-10 deg. That is what makes it actually centre.
            self.nudge = open_round_wall_centering(
                d["left"], d["right"], self.lock, kp=k_center, target=target_dist)
        # final_open_round_working.ino, verbatim:
        #     correction = (headingError * 1.2) + wallCentering
        #     servo = SERVO_CENTER_DEG + correction, constrained to the stops
        # One proportional term on heading, one on the walls, no integral. The
        # integral here was ours, not the sketch's, and on a controller that
        # already has a wall term it mostly winds up fighting it.
        self.i_term = 0.0
        steer = (self._straight_servo()
                 + config.KP_HEADING_CRUISE * self.herr + self.nudge)
        return config.BASE_SPEED, clamp_servo(steer)

    def _deglitch_tof(self, d, now):
        """Vote on each ToF reading over a short window instead of trusting one.

        The firmware reports 999 both for "nothing in range" and for "the
        sensor errored", and those mean opposite things here: 999 reads as an
        OPEN side, which is exactly what commits a corner and, on the first
        corner, the lap direction for the whole race. One spurious high reading
        could therefore send the car the wrong way round the track.

        So a HIGH reading is not acted on the moment it appears. Every reading
        goes into a TOF_VOTE_S window, and when the current one is high it is
        only passed through if most of that window agrees. Otherwise the median
        of the window's real measurements stands in - the most representative
        number available rather than the noisiest one.

        Low readings pass straight through: a wall appearing close is not the
        failure mode, and delaying it would delay braking.
        """
        held = 0
        for k in ("front", "left", "right", "rear"):
            v = d.get(k)
            if v is None:
                continue
            win = self._tof_win.setdefault(k, collections.deque())
            win.append((now, v))
            while win and now - win[0][0] > config.TOF_VOTE_S:
                win.popleft()

            if 0 < v < 999:
                self._tof_last[k] = (v, now)
                continue

            # A high reading. Does the window agree?
            highs = sum(1 for _, s in win if not (0 < s < 999))
            if len(win) >= config.TOF_VOTE_MIN_SAMPLES and highs * 2 > len(win):
                continue                      # genuinely open - let 999 through

            reals = sorted(s for _, s in win if 0 < s < 999)
            if reals:
                d[k] = reals[len(reals) // 2]        # median of the window
                held += 1
            else:
                last = self._tof_last.get(k)
                if last is not None and (now - last[1]) <= config.TOF_HOLD_S:
                    d[k] = last[0]
                    held += 1
        self.tof_held = held
        return d

    # ------------------------------------------------------------------ corners
    # Ported from Working_Open_Round_V2_FE.ino. Lane centring is deliberately
    # NOT ported - the straights still use _straight() exactly as before.

    def _default_lock(self):
        """Lap direction to assume when the sensors genuinely cannot say.

        Both sides reading 999 means no wall within range on either side, so
        there is no measurement to reason from. The track is run in a known
        direction, so state it (`default_lap_dir`) instead of letting an
        arbitrary comparison decide - the old code compared 999 with 999 and
        took the False branch, which meant it always turned left.
        """
        v = str((self._params or {}).get("default_lap_dir", config.DEFAULT_LAP_DIR)).upper()
        return -1 if v.startswith("CCW") or v.startswith("ANTI") else 1

    def _lock_sides(self, d):
        """Side readings for a direction decision: (left, right, ambiguous).

        A reading at or past WALL_VALID_CM means NO WALL THERE, and that is the
        strongest corner signal there is - not missing data. The previous
        version substituted the deglitcher's last good value for it, which
        turned a genuinely open side back into a closed one: the corner logged
        as "L54 R999" got decided as "L54 R46" and turned left into the wall
        when the open right side was screaming the answer.

        An open side is therefore reported as open, and beats any measured
        distance. Only when BOTH sides are open is there nothing to choose
        between, and the caller falls back on the declared lap direction.
        """
        left, right = d.get("left"), d.get("right")
        left_open = left is None or not (0 < left < config.WALL_VALID_CM)
        right_open = right is None or not (0 < right < config.WALL_VALID_CM)
        if left_open and right_open:
            return None, None, True
        l = config.OPEN_CM if left_open else left
        r = config.OPEN_CM if right_open else right
        return l, r, False

    def _settled_after_dodge(self, d):
        """True once the car is far enough past a dodge to trust its sides.

        A dodge ends with the car hard against one wall - a Red dodge leaves it
        hugging the right, so the LEFT reads open and looks exactly like a
        corner. This gate existed but only guarded the normal open-side rule;
        the forced override runs BEFORE that and had no such check, so it could
        still lock the lap direction off post-dodge readings. That is what took
        a left turn 1.4 s after DODGE COMPLETE (Red) on "L999 R40".
        """
        if self.dodge_end_ticks is None or not self.ticks_ok:
            return True
        since = abs(self.ticks - self.dodge_end_ticks) / config.TICKS_PER_CM
        if since >= config.LOCK_AFTER_DODGE_CM:
            return True
        if not self.lock_wait_logged:
            self.lock_wait_logged = True
            log.info(f"NOT LOCKING DIRECTION yet - only {since:.0f} cm since the dodge "
                     f"ended, the car is still off-centre "
                     f"(L{d['left']:.0f} R{d['right']:.0f})")
        return False

    def _blind_lock_dir(self, d):
        """Direction for a blind lock, or 0 if the sides do not clearly say.

        Neither side is open, so all we have is which is roomier. That has to
        win by LOCK_MIN_MARGIN_CM and keep winning for LOCK_CONFIRM_FRAMES in a
        row before it counts - a single noisy frame, or a pillar sitting near
        one wall, must not be able to set the lap direction for the whole race.
        """
        left, right, ambiguous = self._lock_sides(d)
        if ambiguous:
            vote = 0
        elif right - left >= config.LOCK_MIN_MARGIN_CM:
            vote = 1
        elif left - right >= config.LOCK_MIN_MARGIN_CM:
            vote = -1
        else:
            vote = 0
        if vote == 0 or vote != self.lock_vote:
            self.lock_vote = vote
            self.lock_votes = 1 if vote else 0
        else:
            self.lock_votes += 1
        return vote if self.lock_votes >= config.LOCK_CONFIRM_FRAMES else 0

    def _should_turn(self, d, now):
        """Corner decision, same thresholds and order as the sketch.

        Plus a hard override: inside FRONT_FORCE_TURN_CM the car turns
        regardless of side gaps, cooldown or anything else. Being about to hit
        a wall outranks every other consideration.
        """
        front = d["front"]

        # ---- a block right in front is NOT a corner --------------------------
        # The front ToF cannot tell a pillar from a wall, so a block the car has
        # closed right up to reads exactly like a corner and the car turns 90
        # deg into it. Corners are blocked for block_no_corner_s after the last
        # CLOSE sighting - the memory is the point, because up close the pillar
        # fills or leaves the frame and the detection drops out, which is the
        # very moment the wall logic used to take over.
        pr = self._params or {}
        try:
            hold_s = float(pr.get("block_no_corner_s", 2.5))
        except (TypeError, ValueError):
            hold_s = 2.5
        # While a pillar is actually being tracked, the camera's optical
        # distance is the reliable number, not the front ToF - the ToF cannot
        # tell a pillar from a wall and reads whichever is nearer. If the
        # camera says the thing ahead is a pillar and the ToF roughly agrees,
        # the ToF is looking at the pillar, so it must not drive corner logic.
        if (self.pillar_stage in ("APPROACH", "TURN_OUT", "STRAIGHTEN")
                and self.pillar_dist and 0 < front < 999):
            if abs(front - self.pillar_dist) <= config.BLOCK_TOF_AGREE_CM:
                front = 999.0                 # that reading is the pillar, not a wall

        # ...and only for a block DEAD AHEAD. The front ToF looks straight
        # forward, so a pillar off in the LEFT or RIGHT band cannot be what it
        # is reading - suppressing the corner for one of those left the car
        # doing neither: no corner, because a block was "in front", and no
        # dodge, because the ToF backstop also knows it is not centred.
        if hold_s > 0 and self.block_near_centred and (now - self.block_near_t) <= hold_s:
            # This used to release below DODGE_ABORT_FRONT_CM - "never drive
            # into anything" - which meant the closer the car got to a pillar,
            # the more certain it was to take it as a CORNER. In the 09:52 run
            # corners 10 and 12 were both fired at pillars, at 24 cm and 15 cm,
            # with "CORNER BLOCKED - a block is right in front" logged moments
            # before. A block right in front is a block at any distance; the
            # answer is to dodge it, which the ToF backstop in APPROACH now
            # guarantees, not to turn ninety degrees into it.
            if not self.block_corner_blocked:
                self.block_corner_blocked = True
                log.info(f"CORNER BLOCKED - a block is right in front (front reads "
                         f"{front:.0f} cm, that is the block, not a wall)")
            return False
        self.block_corner_blocked = False

        # ---- must have actually MOVED since the last turn -------------------
        # This gates the forced override too, deliberately. After a turn the car
        # is still pointing near the wall it just turned away from, so front is
        # still short; without this the override re-fires immediately and the
        # car spins on the spot, counting corners it never drove to.
        # ...but never while driving into a wall. Inside CORNER_CRITICAL_FRONT_CM
        # the gate is bypassed; _begin_turn then backs the car off before it
        # turns, so this cannot become the old spin-on-the-spot.
        critical = 0 < front <= config.CORNER_CRITICAL_FRONT_CM
        if self.turns > 0 and not critical:
            if self.ticks_ok:
                moved_cm = abs(self.ticks - self.turn_end_ticks) / config.TICKS_PER_CM
                if moved_cm < config.MIN_TRAVEL_BETWEEN_TURNS_CM:
                    return False
            elif now - self.last_turn_end <= config.TURN_COOLDOWN_S:
                return False

        # ---- hard override -------------------------------------------------
        try:
            force_cm = float((self._params or {}).get("front_turn_cm", config.FRONT_FORCE_TURN_CM))
        except (TypeError, ValueError):
            force_cm = config.FRONT_FORCE_TURN_CM
        if 0 < front <= force_cm:
            if self.lock == 0:
                # The lap direction is NOT the forced override's to decide. It
                # fires at front_turn_cm, a normal turn distance with room to
                # spare, and locking there threw the whole race away on a 5 cm
                # side difference. Let the open-side rule below have its say;
                # if it cannot decide, the emergency lock takes over at 40 cm.
                # Still displaced from a dodge? Then the side readings describe
                # the dodge, not the corner, and must not choose the direction
                # for the whole race.
                if not self._settled_after_dodge(d):
                    vote = self._default_lock()
                    log.error(f"CORNER at {front:.0f} cm while still off-centre from a "
                              f"dodge (L{d['left']:.0f} R{d['right']:.0f}) - using the "
                              f"declared direction {'CW' if vote == 1 else 'CCW'}")
                    self.lock = vote
                    return True
                vote = self._blind_lock_dir(d)
                if vote == 0:
                    if front > config.EMERGENCY_FRONT_CM:
                        return False        # still room - keep looking
                    # Out of room. The normal decision below never runs from
                    # here, so the emergency guess has to be made HERE or the
                    # car just drives into the wall waiting for a clean read.
                    el, er, amb = self._lock_sides(d)
                    if amb or el == er:
                        # No usable side reading on one or both sides - nothing
                        # to choose between, and this is exactly where the old
                        # `right > left` tie-break silently meant LEFT every
                        # time. Fall back on the track's declared direction
                        # rather than on the sign of a failed comparison.
                        vote = self._default_lock()
                        log.error(f"BOTH SIDES OPEN at {front:.0f} cm "
                                  f"(L{d['left']:.0f} R{d['right']:.0f}) - falling back "
                                  f"to the declared lap direction "
                                  f"{'CW' if vote == 1 else 'CCW'}")
                        self.lock = vote
                        return True
                    vote = 1 if er > el else -1
                    log.info(f"EMERGENCY BLIND LOCK (UNCONFIRMED) at {front:.0f} cm: "
                             f"{'CW' if vote == 1 else 'CCW'} "
                             f"(L{el:.0f} R{er:.0f})")
                self.lock = vote
                log.info(f"FORCED TURN at {front:.0f} cm - lap direction "
                         f"{'CW' if self.lock == 1 else 'CCW'} "
                         f"(raw L{d['left']:.0f} R{d['right']:.0f})")
            else:
                log.info(f"FORCED TURN at {front:.0f} cm (front < "
                         f"{force_cm:.0f} cm overrides everything)")
            return True

        # ---- normal open-round decision ------------------------------------
        # The corner commits at ONE distance: force_cm. This used to fire as
        # soon as front <= TURN_THRESHOLD_CM (60) with a side open, while the
        # override above fired at front_turn_cm (40), so the same corner was
        # taken anywhere between 60 and 40 cm depending on when the side wall
        # happened to drop out. Measured over a run: 60, 59, 58, 56, 55, 50,
        # 40, 39, 34... The two rules now agree on WHEN and differ only in how
        # they pick the direction. TURN_THRESHOLD_CM stays as the window in
        # which the side readings are worth looking at.
        if not (0 < front <= force_cm and front > 2.0):
            return False
        if now - self.last_turn_end <= config.TURN_COOLDOWN_S:
            return False

        right_open = d["right"] > config.SIDE_GAP_THRESHOLD_CM
        left_open = d["left"] > config.SIDE_GAP_THRESHOLD_CM

        if self.lock == 0:
            # Lap 1: whichever side is open decides the direction for the race.
            #
            # Two things have to be true before that is believable.
            #
            # 1. The car must be back in the middle of the lane. A dodge leaves
            #    it hard against one wall - a Red dodge ends it hugging the
            #    right, so the LEFT reads long and looks exactly like an open
            #    corner. That is how a corner locked ANTI-CLOCKWISE 0.9 s after
            #    DODGE COMPLETE (Red) when the track runs clockwise.
            # 2. The open side must be genuinely open - no wall in range at
            #    all - not merely past SIDE_GAP_THRESHOLD_CM. In a ~100 cm
            #    corridor a centred car reads about 50 on both sides, so that
            #    threshold sits right on the noise.
            settled = self._settled_after_dodge(d)
            really_right = d["right"] >= config.WALL_VALID_CM
            really_left = d["left"] >= config.WALL_VALID_CM

            if settled and really_right and not left_open:
                self.lock = 1
                log.info(f"LOCKED LAP DIRECTION: CLOCKWISE "
                         f"(L{d['left']:.0f} R{d['right']:.0f})")
                return True
            if settled and really_left and not right_open:
                self.lock = -1
                log.info(f"LOCKED LAP DIRECTION: ANTI-CLOCKWISE "
                         f"(L{d['left']:.0f} R{d['right']:.0f})")
                return True
            if front <= config.EMERGENCY_FRONT_CM:
                # Too close to keep waiting for a clean read - commit to the
                # roomier side rather than drive into the wall. Still off-centre
                # from a dodge? Then the side readings are about the dodge, not
                # the corner, and the declared direction beats reading them.
                if not settled:
                    vote = self._default_lock()
                    log.error(f"CORNER at {front:.0f} cm while still off-centre from a "
                              f"dodge - sides unusable (L{d['left']:.0f} R{d['right']:.0f}), "
                              f"using the declared direction "
                              f"{'CW' if vote == 1 else 'CCW'}")
                    self.lock = vote
                    return True
                vote = self._blind_lock_dir(d)
                if vote == 0:
                    # Still no clear side. Take the roomier one rather than
                    # drive into the wall, but say so - this is a guess.
                    el, er, amb = self._lock_sides(d)
                    if amb or el == er:
                        vote = self._default_lock()
                        log.error(f"BOTH SIDES OPEN at {front:.0f} cm "
                                  f"(L{d['left']:.0f} R{d['right']:.0f}) - using the "
                                  f"declared direction {'CW' if vote == 1 else 'CCW'}")
                        self.lock = vote
                        return True
                    vote = 1 if er > el else -1
                    log.info(f"EMERGENCY BLIND LOCK (UNCONFIRMED): "
                             f"{'CW' if vote == 1 else 'CCW'} "
                             f"(L{el:.0f} R{er:.0f})")
                else:
                    log.info(f"EMERGENCY BLIND LOCK: {'CW' if vote == 1 else 'CCW'} "
                             f"(L{d['left']:.0f} R{d['right']:.0f})")
                self.lock = vote
                return True
            return False

        # Later corners: direction is settled, the side check only guards
        # against turning on a false ToF reading.
        if self.lock == 1 and (right_open or front <= config.EMERGENCY_FRONT_CM):
            return True
        if self.lock == -1 and (left_open or front <= config.EMERGENCY_FRONT_CM):
            return True
        return False

    def _trim(self):
        """Servo degrees to add so the wheels actually point straight ahead.

        90 is not straight on this car. Measured two independent ways from
        run_20260924_083553.csv: the heading controller settles at 87.1 while
        holding a straight line, and a reverse commanded at exactly 90 curves
        LEFT at 0.45 deg/cm, consistently, over 486 cm of reversing.
        """
        try:
            return float((self._params or {}).get("servo_trim_deg", config.SERVO_TRIM))
        except (TypeError, ValueError):
            return config.SERVO_TRIM

    def _straight_servo(self):
        """The servo angle that drives straight."""
        return config.SERVO_CENTER + self._trim()

    def _reverse_hold_steer(self, herr):
        """Steer to hold a heading while REVERSING.

        Two things bite here. The wheels are not straight at 90, and reversing
        inverts the steering: the same angle rotates the car the other way once
        the wheels are going backwards, so the correction gain is negated. The
        old code commanded a bare SERVO_CENTER open-loop and every clearance
        reverse rotated the car 25-29 deg before it had driven anywhere.
        """
        return clamp_servo(self._straight_servo() - config.KP_REVERSE_HEADING * herr)

    def _reverse_arc_steer(self):
        """Servo angle for the reverse arc into a corner.

        Reversing with the wheels turned the OPPOSITE way to the corner swings
        the nose round towards the new heading while the car backs off the
        wall - so the manoeuvre buys clearance and does part of the turn at the
        same time, instead of reversing straight and then turning from a stop.

        Turning right (lock +1) means wheels LEFT here, and vice versa. The
        rotation is not wasted: _begin_turn sets the target as previous target
        + 90 deg absolute, so whatever the arc gained is already banked.
        """
        try:
            off = abs(float((self._params or {}).get("reverse_arc_deg",
                                                     config.REVERSE_ARC_DEG)))
        except (TypeError, ValueError):
            off = config.REVERSE_ARC_DEG
        off = min(off, config.SERVO_MAX - config.SERVO_CENTER)
        # lock +1 = clockwise = turning right -> wheels left -> below centre
        return config.SERVO_CENTER - off if self.lock == 1 else config.SERVO_CENTER + off

    def _arc_leave_deg(self):
        """How much of the corner to leave for the forward turn.

        The reverse arc is efficient but coarse - it is also backing towards
        whatever is behind the car. Handing the last stretch to the forward
        turn keeps the corner finishing on a controlled heading rather than on
        wherever the arc happened to stop.
        """
        try:
            v = float((self._params or {}).get("reverse_arc_leave_deg",
                                               config.REVERSE_ARC_LEAVE_DEG))
        except (TypeError, ValueError):
            return config.REVERSE_ARC_LEAVE_DEG
        return max(5.0, min(85.0, v))

    def _pre_reverse_cm(self):
        """How far the car reverses BEFORE a corner, to buy room to swing round.

        The corner fires at whatever distance the block-hold lets it - often
        15-16 cm, which is far too close to swing 90 deg. Backing up first
        creates the clearance instead of trying to predict it.
        """
        try:
            v = float((self._params or {}).get("pre_turn_reverse_cm", 30.0))
        except (TypeError, ValueError):
            return 30.0
        return max(0.0, min(150.0, v))

    def _post_reverse_cm(self):
        """How far the car reverses AFTER a corner, for clearance."""
        try:
            v = float((self._params or {}).get("post_turn_reverse_cm",
                                               config.POST_TURN_REVERSE_CM))
        except (TypeError, ValueError):
            return config.POST_TURN_REVERSE_CM
        return max(0.0, min(150.0, v))     # cap raised for the 100 cm reverse

    def _corner_steer(self, to_right, herr=None):
        """Servo angle for a corner turn - PROPORTIONAL to the heading left to turn.

        A fixed lock held to the last degree makes the tyres scrub and the car
        finish the corner still cranked over, which is what made the turns feel
        so sharp. Now the angle follows the remaining error: hard while there
        is 90 deg to go, easing to almost straight as the car comes round.
        corner_steer_deg is the CAP, not the constant.
        """
        p = self._params or {}
        try:
            cap = abs(float(p.get("corner_steer_deg", 30.0)))
        except (TypeError, ValueError):
            cap = 30.0
        cap = min(cap, config.SERVO_MAX - config.SERVO_CENTER)   # stay inside the stops
        if herr is None:
            off = cap
        else:
            off = min(cap, max(config.CORNER_MIN_OFF_DEG, config.CORNER_KP * abs(herr)))
        return config.SERVO_CENTER + off if to_right else config.SERVO_CENTER - off

    def _begin_turn(self, d, now):
        # Is this the same corner again, rather than the next one? A turn that
        # ended wedged fires again at once (see CORNER_CRITICAL_FRONT_CM), and
        # adding another 90 deg to the target for it would send the car the
        # wrong way round.
        soon = (now - self.last_turn_end) <= config.CORNER_RETRY_WINDOW_S
        if self.ticks_ok:
            moved_cm = abs(self.ticks - self.turn_end_ticks) / config.TICKS_PER_CM
            self.turn_is_retry = (self.turns > 0 and soon
                                  and moved_cm < config.MIN_TRAVEL_BETWEEN_TURNS_CM)
        else:
            # No encoder: go on time alone. Erring towards "retry" is the safe
            # way round - a missed corner count beats a heading 90 deg wrong.
            moved_cm = float("nan")
            self.turn_is_retry = (self.turns > 0 and soon)
        self.is_turning = True
        self.is_reversing = False
        front = d["front"]
        self.turn_t0 = now
        self.turn_dir = self.lock
        if self.corner_target_set:
            # The reverse arc already fixed it and has been flying to it.
            self.target = self.corner_target
            self.corner_target_set = False
        elif not self.turn_is_retry:
            self.target = (self.target + 90.0 * self.lock) % 360.0
        # self.herr was computed against the OLD target at the top of step().
        # Without recomputing it the turn sees ~0 error and finishes instantly.
        self.herr = heading_error(self.target, d["yaw"])
        self.turn_steer = self._corner_steer(self.lock == 1, self.herr)
        self.i_term = 0.0
        if self.turn_is_retry:
            moved_txt = "no encoder" if moved_cm != moved_cm else f"only {moved_cm:.0f} cm"
            log.info(f"CORNER {self.turns}: RETRYING ({moved_txt} since it ended, front "
                     f"{front:.0f} cm) - same heading target {self.target:.0f}, not counted again")
        else:
            log.info(f"CORNER {self.turns + 1}: TURNING {self.direction}, target heading {self.target:.0f}")

    def _turn_step(self, d, now):
        """Fixed-lock turn with the sketch's 3-point wedge recovery.

        Returns (speed, steer), or None when the turn has just finished and the
        caller should fall through to normal driving on this same tick.
        """
        front = d["front"]

        # Hysteresis, as in the sketch: start reversing when very close, stop
        # only once genuinely clear again.
        if 0 < front < config.REVERSE_START_CM:
            self.is_reversing = True
        elif front > config.REVERSE_STOP_CM:
            self.is_reversing = False

        if abs(self.herr) < config.TURN_DONE_ERR_DEG or (now - self.turn_t0) > config.TURN_TIMEOUT_S:
            timed_out = abs(self.herr) >= config.TURN_DONE_ERR_DEG
            self.is_turning = False
            self.is_reversing = False
            if not self.turn_is_retry:            # a retry is the same corner
                self.turns += 1
            self.last_turn_end = now
            self.turn_end_ticks = self.ticks      # for MIN_TRAVEL_BETWEEN_TURNS_CM
            self.i_term = 0.0
            self.pillar_clear_t = 0.0          # vision live again on the new straight
            log.info(f"CORNER {self.turns} COMPLETE{' (timed out)' if timed_out else ''} "
                     f"- heading {self.target:.0f}, scanning new straight")
            # Now back straight up for clearance on the new straight.
            back_cm = self._post_reverse_cm()
            if back_cm > 0:
                self.post_turn_reverse = True
                self.post_turn_ticks0 = self.ticks
                self.post_turn_t0 = now
                log.info(f"reversing {back_cm:.0f} cm after corner {self.turns}")
            if self.turns >= config.TOTAL_TURNS:
                self.state = "FINISHING"
                self.finish_t0 = now
                log.info(f"{config.TOTAL_TURNS} TURNS DONE - {config.FINISH_TIME_S}s finish line push")
            return None

        if self.is_reversing:
            # Wedged: invert the lock and back out. The sketch uses 255 here;
            # clamped to the team's reverse cap.
            opposite = self._corner_steer(self.turn_dir != 1, self.herr)
            self.decision_text = f"CORNER {self.turns + 1} → WEDGED, REVERSING"
            self.nudge = 0.0
            return -config.REVERSE_SPEED, clamp_servo(opposite)

        # Recomputed every tick, so the lock eases off as the heading comes good.
        self.turn_steer = self._corner_steer(self.turn_dir == 1, self.herr)
        self.decision_text = f"CORNER {self.turns + 1} → TURNING {self.direction} ({self.herr:+.0f}°)"
        self.nudge = 0.0
        return config.TURN_SPEED, clamp_servo(self.turn_steer)

    def _begin_stage(self, stage, now):
        """Enter a dodge stage, zeroing the distance measured for it."""
        self.pillar_stage = stage
        self.stage_t0 = now
        self.stage_ticks0 = self.ticks
        self.stage_cm = 0.0

    def _stage_done(self, target_cm, now):
        """True once the car has travelled target_cm in the current stage.

        Distance is the real test. The elapsed-time check is only a backstop
        for a dead or unplugged encoder - without it a stage would hold full
        steering lock forever if ticks stopped arriving.
        """
        if self.ticks_ok and self.stage_cm >= target_cm:
            return True
        return (now - self.stage_t0) >= config.STAGE_TIMEOUT_S

    def _leg_cm(self):
        """Length of each turn leg, from the live tunables.

        dodge_long_cm for the wide arc (pillar on the side we dodge towards),
        dodge_short_cm otherwise. Clamped, so a bad params.json edit cannot
        send the car on an excursion or collapse the stage to nothing.
        """
        p = self._params or {}
        if self.dodge_wide:
            cm = p.get("dodge_long_cm", config.DODGE_WIDE_OUT_CM)
        else:
            cm = p.get("dodge_short_cm", config.DODGE_OUT_CM)
        try:
            cm = float(cm)
        except (TypeError, ValueError):
            cm = config.DODGE_OUT_CM
        # Per-colour trim on the turn-out. The steering is an exact mirror in
        # software, so these absorb the mechanical difference between the two
        # directions rather than any logical one.
        key = ("dodge_green_extra_cm" if self.pillar_color == "Green"
               else "dodge_red_extra_cm")
        try:
            cm += float(p.get(key, 0.0))
        except (TypeError, ValueError):
            pass
        return max(config.DODGE_CM_MIN, min(config.DODGE_CM_MAX, cm))

    def _align_min_cm(self):
        """Minimum forward run while squaring up, before the dodge may end.

        Nothing forced the car to travel at all here: reach the heading and the
        stage was over, even on the first tick, with the pillar still beside
        the car. This guarantees it drives clear.
        """
        p = self._params or {}
        key = ("dodge_align_min_green_cm" if self.pillar_color == "Green"
               else "dodge_align_min_red_cm")
        try:
            return max(0.0, min(config.ALIGN_MAX_CM, float(p.get(key, 0.0))))
        except (TypeError, ValueError):
            return 0.0

    def _past_cm(self):
        """How far to run straight past the pillar, per colour.

        Was a single config constant. Green needs longer than Red to clear the
        pillar before it starts swinging back in.
        """
        p = self._params or {}
        cm = config.DODGE_PAST_CM
        key = ("dodge_past_green_extra_cm" if self.pillar_color == "Green"
               else "dodge_past_red_extra_cm")
        try:
            cm += float(p.get(key, 0.0))
        except (TypeError, ValueError):
            pass
        return max(config.DODGE_CM_MIN, min(config.DODGE_CM_MAX, cm))

    def _out_cm(self):
        """Turn-out leg length."""
        return self._leg_cm()

    def _in_cm(self):
        """Turn-back-in leg: the turn-out length plus dodge_return_extra_cm.

        The two were locked together; the extra lets the car swing back in for
        longer than it turned out without changing the turn-out at all.
        """
        p = self._params or {}
        try:
            extra = float(p.get("dodge_return_extra_cm", 0.0))
        except (TypeError, ValueError):
            extra = 0.0
        # ...plus a per-colour one, so Green can swing back in for longer than
        # Red without touching either turn-out.
        key = ("dodge_return_green_extra_cm" if self.pillar_color == "Green"
               else "dodge_return_red_extra_cm")
        try:
            extra += float(p.get(key, 0.0))
        except (TypeError, ValueError):
            pass
        cm = self._leg_cm() + extra
        return max(config.DODGE_CM_MIN, min(config.DODGE_CM_MAX, cm))

    def _sharp_out_steer(self, params):
        """Servo angle for the turn-out, mirrored per colour.

        The wide case uses a GENTLER angle held over a LONGER distance, which
        traces a broader arc than the normal sharp-and-short dodge.
        """
        if self.dodge_wide:
            wide = config.DODGE_WIDE_STEER_DEG
            return wide if self.pillar_color == "Green" else (2 * config.SERVO_CENTER - wide)
        if self.pillar_color == "Green":
            return params.get("pillar_steer_left", 45)
        return params.get("pillar_steer_right", 135)

    def _stage_progress(self, target_cm):
        """0..1 through the current stage, by distance."""
        if target_cm <= 0:
            return 1.0
        return min(1.0, max(0.0, self.stage_cm / target_cm))

    def step(self, d, blocks, params, now):
        self._params = params or {}
        d = self._deglitch_tof(d, now)      # before anything reads the ToF values
        self.herr = heading_error(self.target, d["yaw"])

        # A pillar seen CLOSE stamps the clock that blocks corners above. Done
        # at the very top so it already counts for this tick's corner decision.
        try:
            near_cm = float(self._params.get("block_near_cm", 60.0))
        except (TypeError, ValueError):
            near_cm = 60.0
        for b in (blocks or []):
            if b["class"] in ("Red", "Green") and 0 < b["dist_cm"] <= near_cm:
                # Record WHERE it was, not just that it was there. Only a block
                # the front ToF could actually be looking at may suppress a
                # corner - see _should_turn.
                self.block_near_t = now
                self.block_near_centred = config.block_roi(b) == "CENTER"
                break

        # Distance travelled, from the ESP32's free-running encoder count.
        t = d.get("ticks")
        if t is None:
            self.ticks_ok = False
        else:
            self.ticks_ok = True
            self.ticks = t
            self.stage_cm = abs(self.ticks - self.stage_ticks0) / config.TICKS_PER_CM

        # ---------------------------------------------------------- run lifecycle
        if self.state == "DONE":
            self.decision_text = f"RACE COMPLETE - {self.turns} turns"
            self.last_t = now
            return 0, config.SERVO_CENTER

        if self.state == "FINISHING":
            # Past turn 12: keep driving for FINISH_TIME_S to cross the start
            # section, then stop for good.
            if now - self.finish_t0 >= config.FINISH_TIME_S:
                self.state = "DONE"
                log.info("FINISH LINE CROSSED - 3 laps complete, halting")
                self.last_t = now
                return 0, config.SERVO_CENTER
            self.decision_text = "FINISH LINE PUSH"
            out = self._straight(d, now, ignore_tof=False)
            self.last_t = now
            return out

        # ---------------------------------------------------------- corners
        # A turn in progress owns the car outright.
        if self.is_turning:
            out = self._turn_step(d, now)
            if out is not None:
                self.last_t = now
                return out

        # ---- clearance reverse BEFORE the corner -----------------------------
        if self.pre_turn_reverse:
            back_cm = self._pre_reverse_cm()
            moved = abs(self.ticks - self.pre_turn_ticks0) / config.TICKS_PER_CM
            # The arc is flown on the IMU: it stops once it has turned all but
            # reverse_arc_leave_deg of the corner, and the forward turn finishes
            # onto the exact target. Distance and time are backstops only.
            # Before this the arc ran purely on distance and 35 cm at 40 deg
            # happened to be the whole 90, so corners completed in 20 ms or
            # overshot and timed out.
            arc_herr = heading_error(self.corner_target, d["yaw"])
            leave = self._arc_leave_deg()
            done = (abs(arc_herr) <= leave
                    or (self.ticks_ok and moved >= back_cm)
                    or (now - self.pre_turn_t0) >= config.POST_TURN_REVERSE_TIMEOUT_S)
            if not done:
                # Hold the ARC for the whole reverse. This returned a bare
                # SERVO_CENTER, so the arc lasted exactly one tick and the rest
                # of the 35 cm was an (uncorrected, curving) straight reverse.
                self.decision_text = (f"CORNER {self.turns + 1} → REVERSE ARC "
                                      f"{abs(arc_herr):.0f}° to go ({moved:.0f} cm)")
                self.nudge = 0.0
                self.last_t = now
                return -config.REVERSE_SPEED, clamp_servo(self._reverse_arc_steer())
            self.pre_turn_reverse = False
            log.info(f"CORNER {self.turns + 1}: arc turned {90 - abs(arc_herr):.0f} deg over "
                     f"{moved:.0f} cm - {abs(arc_herr):.0f} deg left for the forward turn")
            self._begin_turn(d, now)          # target set HERE, after the reverse
            out = self._turn_step(d, now)
            if out is not None:
                self.last_t = now
                return out

        # ---- clearance reverse after the corner ------------------------------
        # Straight back, wheels centred, measured on the encoder. Part of the
        # corner: the turn finishes, then the car gives itself room on the new
        # straight before driving on.
        if self.post_turn_reverse:
            back_cm = self._post_reverse_cm()
            moved = abs(self.ticks - self.post_turn_ticks0) / config.TICKS_PER_CM
            done = (back_cm <= 0
                    or (self.ticks_ok and moved >= back_cm)
                    or (now - self.post_turn_t0) >= config.POST_TURN_REVERSE_TIMEOUT_S)
            if not done:
                self.decision_text = (f"AFTER CORNER {self.turns} → REVERSING "
                                      f"{moved:.0f}/{back_cm:.0f} cm for clearance")
                self.nudge = 0.0
                self.last_t = now
                return -config.REVERSE_SPEED, self._reverse_hold_steer(self.herr)
            self.post_turn_reverse = False
            log.info(f"reversed {moved:.0f} cm after corner {self.turns} - driving on")

        # Corners are suppressed only while a dodge is actually being executed,
        # where the front sensor is looking at the pillar rather than a wall.
        # APPROACH is NOT one of those: it is just cruising while watching a
        # pillar, and blocking corners during it let a single distant detection
        # latch the car out of ever turning again.
        # PRIORITY: the corner outranks the dodge.
        #
        # A dodge used to suppress corners outright until it finished, so a
        # corner arriving mid-manoeuvre was simply missed and the car ran on
        # into the wall. The turn is the thing that must not be missed - a
        # skipped pillar costs points, a missed corner ends the run.
        #
        # This is only safe because _should_turn already refuses to call a
        # corner when the front reading is explained by a BLOCK: block_no_corner_s
        # holds corners off for that long after a pillar was last seen close.
        # So "front is short" during a dodge does not fire a corner - only a
        # front that the camera cannot account for does.
        if not self.is_turning and self._should_turn(d, now):
            if self.pillar_stage is not None:
                log.info(f"CORNER DURING {self.pillar_stage} - the turn takes priority, "
                         f"dropping the {self.pillar_color} pillar (front "
                         f"{d['front']:.0f} cm)")
                self.pillar_stage = None
                self.pillar_color = None
                self.pillar_box = None
                self.pillar_roi = None
                self.dodge_wide = False
                self.pillar_clear_t = now
            back = self._pre_reverse_cm()
            if back > 0:
                # Fix the heading target NOW, before the arc, so the reverse can
                # be flown on the IMU instead of guessed at from distance. The
                # target is absolute - previous target + 90 - so setting it here
                # is identical to setting it later, except that the arc can now
                # measure how much of the corner it has actually turned.
                self.corner_target = (self.target + 90.0 * self.lock) % 360.0
                self.corner_target_set = True
                self.pre_turn_reverse = True
                self.pre_turn_ticks0 = self.ticks
                self.pre_turn_t0 = now
                log.info(f"CORNER {self.turns + 1}: reverse-arc {back:.0f} cm, wheels "
                         f"{'LEFT' if self.lock == 1 else 'RIGHT'} (front {d['front']:.0f} cm)")
                self.nudge = 0.0
                self.last_t = now
                return -config.REVERSE_SPEED, clamp_servo(self._reverse_arc_steer())
            self._begin_turn(d, now)
            out = self._turn_step(d, now)
            if out is not None:
                self.last_t = now
                return out
        
        
        # Look for target block if not already handling a pillar (with cooldown after previous pillar)
        # A dodge ends with the pillar it just passed sitting right beside the
        # car. The old guard was a 1.2 s clock, and the moment it expired the
        # camera re-adopted that very pillar at ~21 cm and dodged it a second
        # time - in the wrong direction, in the space the corner needed. In the
        # 06:14 run every corner that needed 120-148 deg instead of 90 followed
        # one of those; the two corners that did not were +89 and +93. So the
        # gate is DISTANCE now: the car must physically leave the pillar behind
        # rather than merely wait out a clock.
        gap_ok = True
        if self.pillar_stage is None and self.dodge_end_ticks is not None and self.ticks_ok:
            gone = abs(self.ticks - self.dodge_end_ticks) / config.TICKS_PER_CM
            gap_ok = gone >= config.PILLAR_CLEAR_CM
        if self.pillar_stage is None and blocks and gap_ok and (now - self.pillar_clear_t > 1.2):
            # NEAREST first. The one about to be hit is the one that matters,
            # and tracking a distant pillar while a nearer one sits in front is
            # how a Red dodge got committed with a Green right ahead.
            valid = sorted((b for b in blocks if b["class"] in ("Red", "Green")),
                           key=lambda b: b["dist_cm"])
            # The manoeuvre is turn-out + run-past + turn-back-in, over a metre
            # of travel. A pillar first seen closer than this cannot be dodged -
            # the car is past it before the turn-out ends - so adopting one only
            # burns the run-up the corner needs.
            # The acquire gate exists to stop the car re-adopting the pillar it
            # has just driven past, which sits right beside it at ~20 cm. It
            # was applied unconditionally, so ANY block first seen closer than
            # 45 cm was ignored - including genuinely new ones. With a Green at
            # 33 cm and a Red at 50 cm in frame, the Green was dropped and the
            # car went for the Red. That is the "it skips greens" behaviour.
            #
            # A just-passed pillar can only be near a dodge that just ended, and
            # PILLAR_CLEAR_CM already covers that window by distance travelled.
            # Outside it, only an unusably close detection is refused.
            floor = config.PILLAR_MIN_ACQUIRE_FLOOR_CM
            if self.dodge_end_ticks is not None and self.ticks_ok:
                gone = abs(self.ticks - self.dodge_end_ticks) / config.TICKS_PER_CM
                if gone < config.PILLAR_CLEAR_CM * 2:
                    floor = config.PILLAR_MIN_ACQUIRE_CM
            too_close = [b for b in valid if b["dist_cm"] < floor]
            valid = [b for b in valid if b["dist_cm"] >= floor]
            # A pillar that already sits on the side it has to be passed on -
            # Red in the LEFT band, Green in the RIGHT - is cleared by simply
            # driving on, so it must not start an approach either. already_clear
            # also demands real lateral clearance, not just band membership,
            # because perspective sweeps a block dead ahead into the "correct"
            # band as the car closes on it.
            clear_now = [b for b in valid if already_clear(b, b["class"], params)]
            valid = [b for b in valid if b not in clear_now]
            if clear_now and not valid:
                b = min(clear_now, key=lambda x: x["dist_cm"])
                tag = (b["class"], config.block_roi(b))
                if self.skip_logged != tag:
                    self.skip_logged = tag
                    log.info(f"{b['class']} pillar already in the {tag[1]} band at "
                             f"{b['dist_cm']:.0f} cm - correct side, no dodge needed")
            if valid:
                self.skip_logged = None
            if too_close and not valid:
                b = min(too_close, key=lambda x: x["dist_cm"])
                if self.near_skip_logged != b["class"]:
                    self.near_skip_logged = b["class"]
                    log.info(f"IGNORING {b['class']} at {b['dist_cm']:.0f} cm - under the "
                             f"{floor:.0f} cm floor; almost certainly the pillar just passed")
            if valid:
                self.near_skip_logged = None
            if valid:
                b = valid[0]
                self.pillar_color = b["class"]
                self.pillar_conf = b["confidence"]
                self.pillar_dist = b["dist_cm"]
                self.pillar_cx = b.get("cx")
                self.pillar_box = b
                self.pillar_seen_t = now
                self.trigger_frames = 0
                self._begin_stage("APPROACH", now)
                log.info(f"PILLAR DETECTED: {self.pillar_color} at {self.pillar_dist:.1f} cm (cx={self.pillar_cx}) - STAGE 1: APPROACHING")
                self.tof_disagree_logged = False

        # Update distance if seen during APPROACH
        if self.pillar_stage == "APPROACH":
            # Re-evaluate WHICH pillar is being approached, every frame.
            # The approach only ever looked for more of the colour it first
            # locked on to, so a Red picked up at 120 cm held the approach all
            # the way in while a Green sat 40 cm in front - the car followed the
            # far Red and drove straight at the near Green. The pillar that
            # matters is the nearest one, and that can change as the car moves.
            nearer = [b for b in (blocks or [])
                      if b["class"] in ("Red", "Green")
                      and b["class"] != self.pillar_color
                      and b["dist_cm"] >= config.PILLAR_MIN_ACQUIRE_CM
                      and self.pillar_dist - b["dist_cm"] >= config.PILLAR_SWITCH_MARGIN_CM
                      and not already_clear(b, b["class"], params)]
            if nearer:
                b = min(nearer, key=lambda x: x["dist_cm"])
                log.info(f"SWITCHING to the nearer {b['class']} at {b['dist_cm']:.0f} cm "
                         f"- was approaching a {self.pillar_color} at "
                         f"{self.pillar_dist:.0f} cm")
                self.pillar_color = b["class"]
                self.pillar_dist = b["dist_cm"]
                self.pillar_cx = b.get("cx")
                self.pillar_box = b
                self.pillar_conf = b["confidence"]
                self.pillar_seen_t = now
                self.pillar_roi = None
                self.dodge_wide = False
                self.trigger_frames = 0
                self.trigger_seq = None
                self.tof_disagree_logged = False

            # NEAREST of that colour. This was unsorted, so match[0] was
            # whichever the detector happened to list first. In the 10:11 run a
            # Red was adopted at 129 cm and on the very same tick already_clear
            # judged "LEFT band at 33 cm" - a different Red entirely. Both the
            # skip decision and the tracked distance were being taken from an
            # arbitrary block.
            match = sorted((b for b in (blocks or []) if b["class"] == self.pillar_color),
                           key=lambda b: b["dist_cm"])
            if match and already_clear(match[0], self.pillar_color, params):
                log.info(f"{self.pillar_color} pillar is on the correct side already "
                         f"({config.block_roi(match[0])} band at "
                         f"{match[0]['dist_cm']:.0f} cm) - dropping the approach")
                self.pillar_stage = None
                self.pillar_color = None
                self.pillar_box = None
                self.pillar_roi = None
                self.dodge_wide = False
                self.pillar_clear_t = now
                match = []
            if match:
                self.pillar_dist = match[0]["dist_cm"]
                self.pillar_conf = match[0]["confidence"]
                self.pillar_cx = match[0].get("cx")
                self.pillar_box = match[0]
                self.pillar_seen_t = now
            elif now - self.pillar_seen_t > config.APPROACH_LOST_S:
                # The pillar is gone - a false detection, or it passed out of
                # frame. Without this the approach latches forever and, with it,
                # so does everything it suppresses.
                log.info(f"PILLAR LOST ({self.pillar_color}, last seen "
                         f"{now - self.pillar_seen_t:.1f}s ago) - resuming normal driving")
                self.pillar_stage = None
                self.pillar_color = None
                self.pillar_box = None
                self.pillar_roi = None
                self.dodge_wide = False
                self.pillar_clear_t = now

        # Pillar Avoidance State Machine
        if self.pillar_stage == "APPROACH":
            trig = params.get("pillar_trigger_dist_cm", 35.0)
            self.decision_text = f"{self.pillar_color.upper()} pillar → TAKING {'RIGHT' if self.pillar_color=='Red' else 'LEFT'}"
            # One bad frame must not fire the dodge: the optical distance comes
            # from box height and can spike badly (55 cm -> 33 cm in 20 ms was
            # logged). Require several consecutive frames inside the trigger.
            # Count CAMERA frames, not control ticks. The loop runs at ~50 Hz and
            # the camera at ~15, so this used to tick three times inside a
            # single frame: DODGE_TRIGGER_FRAMES was satisfied in 60 ms by one
            # detection, and the whole point of it - requiring several
            # independent looks - never applied. In the 09:18 run a Red was
            # picked up at 114.5 cm and the dodge fired 40 ms later on a frame
            # reading 34 cm. The car dodged a pillar more than a metre away and
            # turned back in straight onto it.
            # A frame counter is the reliable signal, but falling back on a
            # clock matters: if this ever runs without one being supplied,
            # counting every tick would silently restore the original bug, and
            # counting none would mean the car never dodges at all. Neither
            # failure should be possible by omission.
            if self.vision_seq is not None:
                new_frame = self.vision_seq != self.trigger_seq
                if new_frame:
                    self.trigger_seq = self.vision_seq
            else:
                new_frame = (now - self.trigger_frame_t) >= config.VISION_MIN_FRAME_S
                if new_frame:
                    self.trigger_frame_t = now
            if new_frame:
                if self.pillar_dist <= trig:
                    self.trigger_frames += 1
                else:
                    self.trigger_frames = 0

            # The camera's distance comes from box height and reads LONG when
            # the box is clipped or poorly lit, so the approach can run the car
            # right up to a pillar while still believing it is 60 cm away. The
            # front ToF is the accurate number at close range - it just cannot
            # say WHAT it is looking at. The camera has already answered that,
            # so once a tracked pillar is this close on the ToF, commit.
            # Without this the car closed to 15 cm, the block guard released at
            # DODGE_ABORT_FRONT_CM, and the pillar was taken as a CORNER.
            # ...but ONLY when the two are plausibly looking at the same thing.
            # Without that check this committed a Red dodge on a Red the camera
            # put at 116 cm because the ToF read 32 cm - and the 32 cm object
            # was a GREEN block in front, so the car dodged the wrong way for a
            # pillar it had not even identified. A gap that large between the
            # two sensors means they are looking at DIFFERENT objects, and an
            # unidentified thing close ahead is not something to dodge blind.
            tof = d["front"]
            # The front ToF looks STRAIGHT AHEAD. A pillar off in the LEFT or
            # RIGHT band is not what it is measuring, so a short reading then
            # belongs to a wall and the corner should have it. Attributing one
            # to an off-centre pillar is how a Green in the LEFT band committed
            # a dodge at 45 cm and the corner had to interrupt it two seconds
            # later - the turn should simply have gone first.
            centred = False
            if self.pillar_box is not None:
                centred = config.block_roi(self.pillar_box) == "CENTER"
            elif self.pillar_cx is not None:
                half = config.CAMERA_SIZE[0] / 2.0
                centred = abs(self.pillar_cx - half) / half <= config.PILLAR_CENTRED_FRAC
            close_enough = centred and 0 < tof <= config.PILLAR_TOF_COMMIT_CM
            # "Seen recently" must mean the camera is still tracking THIS pillar,
            # not that it once judged it CLOSE - block_near_t keys off the same
            # box-height distance that is unreliable here, so it stayed unset
            # while the car drove in. Inside APPROACH a pillar is tracked by
            # definition; all that matters is that the sighting is current.
            seen_recently = (now - self.pillar_seen_t) <= config.BLOCK_NEAR_MEMORY_S
            same_object = abs(self.pillar_dist - tof) <= config.BLOCK_TOF_AGREE_CM
            if centred and 0 < tof <= config.PILLAR_TOF_COMMIT_CM \
                    and seen_recently and not same_object:
                if not self.tof_disagree_logged:
                    self.tof_disagree_logged = True
                    log.warning(f"SOMETHING AT {tof:.0f} cm the camera cannot account for "
                                f"- it puts the {self.pillar_color} at {self.pillar_dist:.0f} cm. "
                                f"Not committing a {self.pillar_color} dodge to it.")
            if close_enough and seen_recently and same_object \
                    and self.trigger_frames < config.DODGE_TRIGGER_FRAMES:
                log.info(f"{self.pillar_color} pillar at {tof:.0f} cm on the front "
                         f"ToF (camera says {self.pillar_dist:.0f}) - committing now")
                self.trigger_frames = config.DODGE_TRIGGER_FRAMES
            if self.trigger_frames >= config.DODGE_TRIGGER_FRAMES:
                # Latch the region of interest NOW, from the last good look at
                # the pillar. Once the car starts turning the pillar slides out
                # of frame, so this decision can only be made here, and it is
                # never revisited for the rest of the dodge.
                self.pillar_roi = config.block_roi(self.pillar_box or {"cx": self.pillar_cx})

                # A pillar already sitting on the side we are dodging towards
                # needs a wider berth: Green in the LEFT band, Red in the RIGHT.
                self.dodge_wide = ((self.pillar_color == "Green" and self.pillar_roi == "LEFT") or
                                   (self.pillar_color == "Red" and self.pillar_roi == "RIGHT"))

                self._begin_stage("TURN_OUT", now)
                log.info(f"STAGE 2: TURNING OUT ({self.pillar_color}) at {self.pillar_dist:.1f} cm "
                         f"- ROI={self.pillar_roi}, {'WIDE' if self.dodge_wide else 'normal'} arc, "
                         f"{self._out_cm():.0f} cm of turn")
            else:
                out = self._straight(d, now, ignore_tof=False)
                self.last_t = now
                return out

        if self.pillar_stage == "TURN_OUT":
            base_sharp = self._sharp_out_steer(params)
            out_cm = self._out_cm()
            hold_pct = min(0.8, max(0.0, params.get("turn_hold_pct", 30) / 100.0))
            offset_sens = max(0.0, params.get("block_offset_sens", 0.5))

            # Block off-center sensitivity: adjust turn angle if block is not in middle of lane
            sharp_steer = base_sharp
            if self.pillar_cx is not None and offset_sens > 0:
                norm_x = (self.pillar_cx - 512.0) / 512.0
                sharp_steer = clamp_servo(base_sharp + norm_x * offset_sens * 15.0)

            # Progress is measured in DISTANCE TRAVELLED, not seconds.
            progress = self._stage_progress(out_cm)

            # Phase 1: hold sharp lock for hold_pct of the distance, so the car
            #          displaces far enough out to clear the pillar
            # Phase 2: unwind smoothly back to centre over the rest of it
            if progress <= hold_pct:
                steer = sharp_steer
            else:
                unwind_p = (progress - hold_pct) / max(0.01, 1.0 - hold_pct)
                steer = sharp_steer + (config.SERVO_CENTER - sharp_steer) * (unwind_p ** 1.5)

            self.decision_text = (f"{self.pillar_color.upper()} [{self.pillar_roi}]"
                                  f"{' WIDE' if self.dodge_wide else ''} → TURNING OUT "
                                  f"{self.stage_cm:.0f}/{out_cm:.0f} cm")

            if self._stage_done(out_cm, now):
                log.info(f"STAGE 3: STRAIGHTENING PAST PILLAR ({self.stage_cm:.1f} cm turned out) "
                         f"- {self._past_cm():.0f} cm to run")
                self._begin_stage("STRAIGHTEN", now)
            self.last_t = now
            return config.BASE_SPEED, clamp_servo(steer)

        if self.pillar_stage == "STRAIGHTEN":
            # Drive straight past the pillar on IMU heading hold, ignoring the
            # ToF sensors (the inner one is staring at the pillar). Ends on
            # distance travelled, not on a clock.
            self.decision_text = (f"{self.pillar_color.upper()} → PASSING "
                                  f"{self.stage_cm:.0f}/{self._past_cm():.0f} cm")
            if self._stage_done(self._past_cm(), now):
                log.info(f"STAGE 4: TURNING BACK IN ({self.stage_cm:.1f} cm run past pillar) "
                         f"- {self._in_cm():.0f} cm of turn")
                self._begin_stage("RECENTER", now)
                self.i_term = 0.0
            out = self._straight(d, now, ignore_tof=True)
            self.last_t = now
            return out

        if self.pillar_stage == "RECENTER":
            hold_pct = min(0.8, max(0.0, params.get("turn_hold_pct", 30) / 100.0))
            side_from = "LEFT" if self.pillar_color == "Green" else "RIGHT"

            # Mirror of the turn-out, and the same distance: Green dodged left
            # so it comes back right; Red dodged right so it comes back left.
            # Purely distance-driven - no ToF early exit, so the manoeuvre is
            # the same shape every single time.
            # Mirror of the turn-out angle, then pulled back toward centre by
            # dodge_return_reduce_deg. A pure mirror over-rotated the car across
            # the lane on the way back in, so the return is deliberately
            # shallower than the turn-out. Green 45 out -> 135 mirrored -> 125.
            mirrored = 2 * config.SERVO_CENTER - self._sharp_out_steer(params)
            reduce_deg = abs(float(params.get("dodge_return_reduce_deg", 0.0)))
            if mirrored > config.SERVO_CENTER:
                sharp_return = max(config.SERVO_CENTER, mirrored - reduce_deg)
            else:
                sharp_return = min(config.SERVO_CENTER, mirrored + reduce_deg)

            # The unwind no longer ends at straight-90: it ends at
            # DODGE_RETURN_END_DEG, mirrored per colour, so the car finishes
            # the return already leaning into the straighten-up direction.
            # Green came back to the RIGHT, so it ends left of centre (80);
            # Red came back to the LEFT, so it ends right of centre (100).
            if self.pillar_color == "Green":
                return_end = config.DODGE_RETURN_END_DEG
            else:
                return_end = 2 * config.SERVO_CENTER - config.DODGE_RETURN_END_DEG

            in_cm = self._in_cm()
            progress = self._stage_progress(in_cm)
            if progress <= hold_pct:
                steer = sharp_return
            else:
                unwind_p = (progress - hold_pct) / max(0.01, 1.0 - hold_pct)
                steer = sharp_return + (return_end - sharp_return) * (unwind_p ** 1.5)

            self.decision_text = (f"TURNING BACK IN ({side_from} → CENTER) "
                                  f"{self.stage_cm:.0f}/{in_cm:.0f} cm")
            self.nudge = steer - config.SERVO_CENTER

            if self._stage_done(in_cm, now):
                log.info(f"STAGE 5: ALIGNING ({self.stage_cm:.1f} cm turned back in)")
                self._begin_stage("ALIGN", now)
                self.i_term = 0.0

            self.last_t = now
            return config.BASE_SPEED, clamp_servo(steer)

        if self.pillar_stage == "ALIGN":
            # The turn-in leaves the car angled across the lane, so square it
            # up: snap hard onto the locked IMU heading and blend in ToF
            # balancing off the reliable outer wall. Ends on heading, not on a
            # clock - there is no distance to travel here, only an angle to
            # kill. ALIGN_MAX_CM caps how far it may run while doing it.
            target_dist = params.get("recenter_target_cm", 45.0)
            kp_wall = params.get("recenter_k", 1.5)
            wall_err = compute_wall_error_cm(d["left"], d["right"],
                                             target_dist=target_dist, avoid_color=self.pillar_color)

            kp_align = 2.4
            align_steer = self._straight_servo() + kp_align * self.herr
            wall_steer = max(-40.0, min(40.0, wall_err * kp_wall))
            steer = align_steer + wall_steer
            self.nudge = steer - config.SERVO_CENTER

            self.decision_text = f"SQUARING UP ({self.herr:+.0f}°)"

            # The car clips the pillar coming back in, because if it happens to
            # be square already the moment the turn-back-in ends, this stage
            # exits on its very first tick and hands straight back to cruise -
            # while the car is still alongside the block. Make it drive clear
            # first: a minimum forward run before the dodge may be declared
            # done. Per colour, since only Red was catching.
            squared = (abs(self.herr) < config.ALIGN_DONE_ERR_DEG
                       and self.stage_cm >= self._align_min_cm())
            ran_out = self._stage_done(config.ALIGN_MAX_CM, now)
            if squared or ran_out:
                if not squared:
                    # Leaving ALIGN still badly skewed is how the NEXT pillar
                    # gets dodged into a wall - it starts the manoeuvre pointing
                    # 40-odd degrees across the lane. Worth shouting about.
                    log.warning(f"ALIGN RAN OUT of {config.ALIGN_MAX_CM:.0f} cm still "
                                f"{self.herr:+.1f}° off - the next dodge starts skewed")
                log.info(f"DODGE COMPLETE ({self.pillar_color}, herr {self.herr:+.1f}°) "
                         f"→ RESUMING CRUISE & ToF CENTERING")
                # The car is against one wall here. Remember where, so the lap
                # direction cannot be locked off these lopsided side readings.
                self.dodge_end_ticks = self.ticks
                self.lock_wait_logged = False
                self.pillar_stage = None
                self.pillar_color = None
                self.pillar_dist = None
                self.pillar_cx = None
                self.pillar_box = None
                self.pillar_roi = None
                self.dodge_wide = False
                self.pillar_clear_t = now
                self.i_term = 0.0

            self.last_t = now
            return config.BASE_SPEED, clamp_servo(steer)

        if self.pillar_stage == "HALT":
            self.decision_text = f"PASSED {self.pillar_color.upper() if self.pillar_color else ''} PILLAR → HALTED"
            self.last_t = now
            return 0, config.SERVO_CENTER

        # Default normal driving when no pillar sequence is active
        target_dist = params.get("recenter_target_cm", 45.0)
        # Cruise uses its OWN gain. recenter_k drives the post-dodge squaring-up
        # and is deliberately not reused here, so the straights can run on pure
        # gyro (cruise_wall_k = 0) without also killing the dodge's re-centre.
        kp_wall = params.get("cruise_wall_k", params.get("recenter_k", 1.5))
        self.decision_text = "No pillar → cruising & centering in lane"
        out = self._straight(d, now, ignore_tof=False, target_dist=target_dist, k_center=kp_wall, max_nudge=20.0)
        self.last_t = now
        return out

    @property
    def stage_display(self):
        """Human-readable stage. Cosmetic only - see _stage_display_raw.

        This is called once per tick by the CSV writer, so an error in here used
        to kill the whole run: main.py exited, systemd restarted it, and the car
        came back sitting in WAITING mid-lap. Nothing about a display string is
        worth that, so it can never raise.
        """
        try:
            return self._stage_display_raw
        except Exception as e:                      # noqa: BLE001 - cosmetic only
            return f"(stage display error: {e})"

    @property
    def _stage_display_raw(self):
        if self.state == "FINISHING":
            return f"FINISH LINE PUSH ({config.TOTAL_TURNS} turns done)"
        if self.state == "DONE":
            return f"DONE - {self.turns} turns, 3 laps"
        if self.is_turning:
            what = "WEDGED, REVERSING" if self.is_reversing else f"TURNING {self.direction}"
            return f"Corner {self.turns + 1}: {what} ({self.herr:+.0f}°)"
        if self.post_turn_reverse:
            return f"After corner {self.turns}: reversing for clearance"
        if self.pillar_stage is None:
            return (f"Normal driving (turn {self.turns}/{config.TOTAL_TURNS}, "
                    f"{self.direction or 'dir unlocked'})")
        if self.pillar_stage == "APPROACH":
            return f"Stage 1: APPROACHING ({self.pillar_color} at {self.pillar_dist:.0f} cm)"
        if self.pillar_stage == "TURN_OUT":
            return (f"Stage 2: TURN OUT ({self.pillar_color} [{self.pillar_roi}]"
                    f"{' WIDE' if self.dodge_wide else ''}) "
                    f"{self.stage_cm:.0f}/{self._out_cm():.0f} cm")
        if self.pillar_stage == "STRAIGHTEN":
            return f"Stage 3: PASSING PILLAR {self.stage_cm:.0f}/{self._past_cm():.0f} cm"
        if self.pillar_stage == "RECENTER":
            return f"Stage 4: TURN BACK IN {self.stage_cm:.0f}/{self._in_cm():.0f} cm"
        if self.pillar_stage == "ALIGN":
            return f"Stage 5: SQUARING UP ({self.herr:+.0f}°)"
        if self.pillar_stage == "HALT":
            return f"Stage 5: HALTED (Centered)"
        return self.pillar_stage


def dash_publish(link, ctl, d, now):
    """Keep the dashboard fed while the master is not driving."""
    link.publish({
        "state": "STANDALONE (ESP32 is driving)",
        "stage": "open round on the ESP32 - master standing down",
        "decision": "the ESP32 drives itself; nothing sent from here",
        "turns": 0,
        "speed": None, "steer": None,
    })
    return True


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    os.makedirs(config.LOG_DIR, exist_ok=True)

    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *a: stop.update(flag=True))

    link = HubLink()
    time.sleep(0.5)
    if not link.hub_alive:
        log.error("dashboard.py is not running - it owns the ESP32 port "
                  "(sudo systemctl start dashboard.service). Waiting for it...")

    ctl = None
    speed, steer = 0, config.SERVO_CENTER
    seq = 0
    csv_file = writer = None
    last_pub = 0.0
    det_log_t = -1e9          # rate limit for the detection log
    log.info("waiting for the START button...")

    try:
        standalone_since = None
        standalone_logged = False

        while not stop["flag"]:
            d, seq = link.wait_new(seq, timeout=0.1)
            now = time.monotonic()
            fresh = d is not None and d["age"] <= config.TELEMETRY_MAX_AGE_S

            # STANDALONE OPEN ROUND: that firmware sends a 12-field DATA line
            # with no encoder ticks, drives the whole round itself and never
            # reads a byte from us. Sending it commands is pointless, so the
            # master stands down and just watches - the dashboard still shows
            # everything. The slave firmware always reports ticks, so this
            # cannot trip during an obstacle run.
            if fresh and d.get("ticks") is None:
                if standalone_since is None:
                    standalone_since = now
                elif now - standalone_since > 2.0:
                    if not standalone_logged:
                        standalone_logged = True
                        log.info("STANDALONE OPEN-ROUND FIRMWARE on the ESP32 (no encoder "
                                 "ticks): it drives itself and ignores us - not sending commands")
                    if dash_publish(link, None, d, now):
                        pass
                    continue
            else:
                if standalone_logged:
                    log.info("slave firmware is back - the master is driving again")
                standalone_since, standalone_logged = None, False

            # If stopped while running (via button or dashboard), reset cleanly
            if ctl is not None and not link.start_received:
                log.info("RUN STOPPED - stopping motor and returning to WAITING")
                if csv_file:
                    csv_file.close()
                    csv_file = None
                ctl = None
                speed, steer = 0, config.SERVO_CENTER
                link.send(0, config.SERVO_CENTER)
                log.info("waiting for the START button...")

            if ctl is None and link.start_received and fresh:
                # Forget anything the camera saw before this run. Otherwise the
                # last frame of the previous run is still cached and gets picked
                # up in the very first step, so the car starts dodging a pillar
                # that may since have been moved or taken away.
                link.clear_vision()
                ctl = ObstacleRound(d["yaw"], now)
                log.info(f"RUN STARTED - heading locked at {d['yaw']:.1f}")
                path = os.path.join(config.LOG_DIR, time.strftime("run_%Y%m%d_%H%M%S.csv"))
                csv_file = open(path, "w", newline="")
                writer = csv.writer(csv_file)
                writer.writerow(["t", "state", "stage", "decision", "turns", "front", "left", "right", "rear",
                                 "yaw", "target", "herr", "nudge", "speed", "steer", "ticks", "stage_cm"])

            # What the detector actually returned, rate-limited, and logged
            # whether or not a run is under way - so "is it skipping greens or
            # never seeing them?" is answered by holding a block in front of
            # the camera, with no run and no guessing.
            if config.LOG_DETECTIONS_S > 0 and (now - det_log_t) >= config.LOG_DETECTIONS_S:
                det_log_t = now
                seen_blocks = link.latest_vision()
                if seen_blocks:
                    seen = " ".join(
                        f"{b['class'][0]}{b.get('dist_cm', 0):.0f}"
                        f"@{b.get('cx', -1):.0f}/{b.get('confidence', 0):.2f}"
                        for b in seen_blocks)
                else:
                    seen = "-"
                log.info(f"DET [{seen}]" + (f" tracking={ctl.pillar_color or '-'} "
                                            f"stage={ctl.pillar_stage or '-'}"
                                            if ctl else " (waiting)"))

            if ctl is None:
                speed, steer = 0, config.SERVO_CENTER
                if link.start_received:
                    link.send(0, config.SERVO_CENTER)      # stops the ESP repeating START
            elif not fresh:
                speed, steer = 0, config.SERVO_CENTER
                link.send(speed, steer)
            else:
                blocks = link.latest_vision()
                if ctl is not None:
                    ctl.vision_seq = link.vision_seq

                params = link.latest_params()
                speed, steer = ctl.step(d, blocks, params, now)
                link.send(speed, steer)
                writer.writerow([f"{now - ctl.t_start:.3f}", ctl.state, ctl.stage_display, ctl.decision_text, ctl.turns,
                                 d["front"], d["left"], d["right"], d["rear"], f"{d['yaw']:.1f}",
                                 f"{ctl.target:.1f}", f"{ctl.herr:.1f}", f"{ctl.nudge:.1f}", speed, steer,
                                 ctl.ticks, f"{ctl.stage_cm:.1f}"])

                # After halting for 2 seconds, auto-reset so next button press starts a fresh run
                if ctl.pillar_stage == "HALT" and (now - ctl.stage_t0 >= 2.0):
                    log.info("PILLAR PASS COMPLETE (HALTED 2s) - resetting for next run")
                    if csv_file:
                        csv_file.close()
                        csv_file = None
                    ctl = None
                    speed, steer = 0, config.SERVO_CENTER
                    link.send(0, config.SERVO_CENTER)
                    link.send_line("STOP")
                    link.clear_start()
                    log.info("waiting for the START button...")

            if now - last_pub >= 0.1:
                last_pub = now
                link.publish({
                    "state": "WAITING" if ctl is None else ctl.state,
                    "stage": ctl.stage_display if ctl else "Normal driving",
                    "decision": ctl.decision_text if ctl else "No pillar → driving straight / centring",
                    "pillar_color": ctl.pillar_color if ctl else None,
                    "pillar_conf": ctl.pillar_conf if ctl else None,
                    "pillar_dist": ctl.pillar_dist if ctl else None,
                    "pillar_cx": ctl.pillar_cx if ctl else None,
                    "ticks": ctl.ticks if ctl else None,
                    "stage_cm": round(ctl.stage_cm, 1) if ctl else None,
                    "encoder_ok": ctl.ticks_ok if ctl else None,
                    "turns": ctl.turns if ctl else 0,
                    "direction": ctl.direction if ctl else None,
                    "runtime": (now - ctl.t_start) if ctl else None,
                    "target": ctl.target if ctl else None,
                    "herr": ctl.herr if ctl else None,
                    "nudge": ctl.nudge if ctl else None,
                    "speed": speed, "steer": steer,
                })
    except KeyboardInterrupt:
        pass
    finally:
        log.info("stopping")
        link.close()
        if csv_file:
            csv_file.close()


if __name__ == "__main__":
    main()

