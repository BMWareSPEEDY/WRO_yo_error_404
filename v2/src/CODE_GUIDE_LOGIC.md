# WRO 2026 Vehicle Logic Specification & Control Flow

This document provides the **exhaustive control logic, mathematical formulas, state machine transitions, and branching conditions** implemented in `obstacle_round/pi/main.py`. Use this specification to diagnose logical flaws, edge-case race conditions, and parameter mismatches without reading code line-by-line.

---

## 1. System Coordinates, Sign Conventions & Constants

All calculations in `main.py` follow these rigid mathematical definitions:

| Variable | Definition / Formula | Behavior & Range |
|---|---|---|
| **True Straight** | `SERVO_CENTER (87) = 90 + servo_trim_deg (-3)` | Neutral center steering angle. |
| **Steering Angle** | Clamped to `[SERVO_MIN, SERVO_MAX] = [30, 140]` | **$> 87$ = Wheels RIGHT**, **$< 87$ = Wheels LEFT**. |
| **Yaw / Heading** | BNO055 IMU in `[0.0, 359.9]` | **Clockwise = increasing yaw**. |
| **Heading Error** | `herr = (target - current + 180) % 360 - 180` | Normalizes to `[-180.0, +180.0]`.<br>**Positive ($>0$) = Must steer RIGHT**.<br>**Negative ($<0$) = Must steer LEFT**. |
| **Distance (cm)** | `abs(ticks - ref_ticks) / 20.78` | Encoder ticks: `TICKS_PER_CM = 20.78` ($2078\text{ ticks/m}$). |
| **Lap Lock (`self.lock`)** | `+1` (Clockwise) or `-1` (Anti-Clockwise) | **`+1` (CW)**: All corners are **RIGHT** turns.<br>**`-1` (CCW)**: All corners are **LEFT** turns. |
| **Dodge Direction** | `_dodge_sign() = +1` (Red) or `-1` (Green) | **Red**: Steers **RIGHT** first.<br>**Green**: Steers **LEFT** first. |
| **Reverse Polarity** | Inverts yaw rotation relative to wheel direction | When reversing, wheels angled right swing the nose left. **All reverse heading-hold formulas invert the correction sign (`-herr`).** |

---

## 2. Master Execution Hierarchy (`step()` Preemption Order)

`ObstacleRound.step(d, blocks, params, now)` runs at **50 Hz**. It evaluates actions strictly from top to bottom. **The first branch that matches takes total control of the car for that tick and returns `(speed, steer)`.**

```
[1. state == "DONE"] ──▶ Halt (0 PWM, 87°)
       │
[2. self.park_step != 0] ──▶ Execute Parking Exit (_park_step)
       │
[3. state == "FINISHING"] ──▶ Finish Line Push (3.0s past turn 12)
       │
[4. self.is_turning == True] ──▶ Execute Forward Turn (_turn_step)
       │
[5. self.pre_turn_reverse == True] ──▶ Reverse Arc Before Corner
       │
[6. self.post_turn_reverse == True] ──▶ Straight Reverse After Corner
       │
[7. Corner Check: _should_turn() == True] ──▶ Trigger Corner (if NOT mid-dodge)
       │
[8. Pillar Acquisition & Tracking] ──▶ Scan blocks, evaluate gates
       │
[9. self.pillar_stage == "BACKOFF"] ──▶ Reverse to restore trigger distance
       │
[10. self.pillar_stage == "APPROACH"] ──▶ Cruise straight, check commit gates
       │
[11. self.pillar_stage == "TURN_OUT"] ──▶ Dodge Leg 1: Steer lock & unwind
       │
[12. self.pillar_stage == "STRAIGHTEN"] ──▶ Dodge Leg 2: Drive past pillar
       │
[13. self.pillar_stage == "RECENTER"] ──▶ Dodge Leg 3: Return arc into corridor
       │
[14. self.pillar_stage == "ALIGN"] ──▶ Dodge Leg 4: Heading + wall squaring
       │
[15. Default] ──▶ Normal cruising with wall-centering P-controller
```

---

## 3. ToF Sensor De-Glitching Logic (`_deglitch_tof`)

The raw hardware streams `999` for both "out of range" and "hardware I2C timeout". Because an open side (`999`) can instantly lock lap direction or commit a corner, high readings are filtered through a temporal voting window:

- **Window Parameters:** Duration `TOF_VOTE_S = 0.2s`, minimum samples `TOF_VOTE_MIN_SAMPLES = 3`.
- **Low Reading ($0 < \text{dist} < 999$):** Passed through immediately without delay (ensures instant emergency braking).
- **High Reading ($\text{dist} \ge 999$ or $\le 0$):**
  - If `high_count * 2 > total_samples_in_window`: Passed through as genuine open space (`999`).
  - Otherwise: Replaced with the **median of real valid readings** in the window.
  - If no valid readings exist in window: Replaced with the last valid reading if younger than `TOF_HOLD_S = 0.35s`.

---

## 4. Lap Direction Determination (`lock_direction_from_parking`)

Determined **once** at the start of the round while parked in the bay.

### Direction Logic:
```python
left, right, ambiguous = self._lock_sides(d)
if ambiguous or left is None or right is None:
    self.lock = self._default_lock() # Read from param 'default_lap_dir'
elif left < right:
    self.lock = 1   # Wall on LEFT  -> Exits RIGHT -> Lap is CLOCKWISE (Right corners)
else:
    self.lock = -1  # Wall on RIGHT -> Exits LEFT  -> Lap is ANTI-CLOCKWISE (Left corners)
```
- **Side validity floor:** `WALL_VALID_CM = 130 cm`. Any reading $\ge 130\text{ cm}$ or $999$ is considered an open side.
- **Ambiguity Condition:** If `left >= 130` AND `right >= 130` (both sides read open), direction falls back to `default_lap_dir` (default: CW / `+1`).

---

## 5. Parking Exit State Machine (`_park_step`)

Active when `self.park_step in (1, 2, 3, 4)`.

- **Failsafe Limit:** If `_park_gone() > park_max_cm (400 cm)`, parking aborts immediately (`self.park_step = 0`), hands over to lap driving.
- **Exit Turn Measurement:** `turned = self.lock * heading_error(current_yaw, park_hdg0)`. (Always positive in the exit direction).

### Stage 1: Swing Out of Bay
- **Steering:** Full lock in exit direction: `87 + self.lock * park_steer_deg` (default $87 \pm 50^\circ$).
- **Exit Condition:** `turned >= park_turn_deg` (default $90^\circ$).
- **On Exit:** `self.park_step = 2`, `self.park_ticks0 = self.ticks` (resets distance counter; falls through to Step 2 in the same tick).

### Stage 2: Straight Forward Across Lane
- **Steering:** P-controller holds the $90^\circ$ exit heading:
  $$\text{steer} = 87 + \text{self.lock} \times \text{clamp}(1.2 \times (90 - \text{turned}), -20^\circ, +20^\circ)$$
- **Exit Condition:** `_park_gone() >= park_forward_cm` (car uses 80 cm per SPEC; default fallback 30 cm).
- **On Exit:** `self.park_step = 3`, `self.park_ticks0 = self.ticks`.

### Stage 3: Back Up Straight
- **Steering:** Inverted P-controller (reverse flips yaw rotation):
  $$\text{steer} = 87 - \text{self.lock} \times \text{clamp}(1.2 \times (90 - \text{turned}), -20^\circ, +20^\circ)$$
- **Speed:** `-abs(park_speed)` (negative PWM = reverse).
- **Exit Condition:** `_park_gone() >= park_back_cm` (default 20 cm).
- **On Exit:** `self.park_step = 4`, `self.park_ticks0 = self.ticks`.

### Stage 4: Swing Back Square to Corridor
- **Steering:** Opposite full lock: `87 - self.lock * park_steer_deg`.
- **Exit Condition:** `turned <= 0.0^\circ` (threshold crossing, not a window).
- **On Exit:** `self.park_step = 0`, returns `None`. Handover:
  - Re-zeros master target heading: `self.target = d["yaw"]`.
  - Sets `self.pillar_clear_t = now` (briefly suppresses pillar acquisition).
  - Control falls through to normal lap cruising in the same tick.

---

## 6. Corner Detection & Decision Logic (`_should_turn`)

Called every tick when `self.is_turning == False` and `dodge_running == False`.

### Gate 1: Close Pillar Suppression (Pillars are NOT corners!)
The front ToF cannot distinguish a pillar from a wall. To stop the car turning $90^\circ$ into a pillar:
1. If a pillar was detected in the `CENTER` ROI band with `dist <= block_near_cm` (default 60 cm):
   - Corners are **completely suppressed** for `block_no_corner_s` (default 2.5s) after the last close sighting:
     ```python
     if hold_s > 0 and self.block_near_centred and (now - self.block_near_t) <= hold_s:
         return False # Suppress corner!
     ```
2. If currently in `APPROACH` or `TURN_OUT` with `abs(front_tof - pillar_dist) <= BLOCK_TOF_AGREE_CM (25 cm)`:
   - Front ToF is overridden to `999.0` (treated as the pillar, not a wall).

### Gate 2: Inter-Turn Movement Floor
To prevent the car spinning in circles at a single corner:
- If `turns > 0` and front is NOT critically close (`front > CORNER_CRITICAL_FRONT_CM = 15 cm`):
  - Must have traveled `moved_cm >= MIN_TRAVEL_BETWEEN_TURNS_CM (100 cm)` since previous turn.
  - If encoder is dead: `now - last_turn_end > TURN_COOLDOWN_S (1.0s)`.
  - If traveled $< 100\text{ cm}$, returns `False`.

### Gate 3: Turn Trigger Distance (`front_turn_cm`)
Corner is triggered when:
$$0 < \text{front} \le \text{front\_turn\_cm (default 37 cm)}$$

### Gate 4: Direction Verification (If `self.lock` already fixed)
- If `self.lock == 1` (CW): Triggers if `d["right"] > SIDE_GAP_THRESHOLD_CM (55 cm)` OR `front <= EMERGENCY_FRONT_CM (40 cm)`.
- If `self.lock == -1` (CCW): Triggers if `d["left"] > SIDE_GAP_THRESHOLD_CM (55 cm)` OR `front <= EMERGENCY_FRONT_CM (40 cm)`.

### Gate 5: Unlocked Direction Handling (Lap 1 fallback)
If `self.lock == 0`:
1. Checks `_settled_after_dodge(d)`: requires `abs(ticks - dodge_end_ticks) >= LOCK_AFTER_DODGE_CM (45 cm)`.
2. Genuinely open side test: `really_right = (d["right"] >= 130)` vs `really_left = (d["left"] >= 130)`.
3. If both open or both closed: `_blind_lock_dir(d)` requires `abs(left - right) >= LOCK_MIN_MARGIN_CM (15 cm)` for `LOCK_CONFIRM_FRAMES (3)` consecutive frames.
4. Emergency fallback at `front <= 40 cm`: defaults to `default_lap_dir`.

---

## 7. Corner Execution Maneuver

Once `_should_turn()` returns `True`, execution branches into three phases:

### Phase A: Pre-Turn Reverse Arc (`pre_turn_reverse`)
Active if `pre_turn_reverse_cm > 0` (default 35 cm):
- **Calculates absolute target heading immediately:**
  $$\text{self.corner\_target} = (\text{self.target} + 90.0 \times \text{self.lock}) \pmod{360}$$
- **Steering:** Wheels turned **opposite** to turn direction:
  - If `self.lock == 1` (CW / Right turn): Steers **LEFT** (`87 - reverse_arc_deg = 87 - 40 = 47°`).
  - If `self.lock == -1` (CCW / Left turn): Steers **RIGHT** (`87 + reverse_arc_deg = 87 + 40 = 127°`).
  - This swings the front of the car toward the new heading while backing away from the wall.
- **Exit Conditions (Whichever happens first):**
  1. IMU error threshold met: `abs(heading_error(corner_target, yaw)) <= reverse_arc_leave_deg` (default 35° left for forward turn).
  2. Traveled distance: `moved >= pre_turn_reverse_cm` (35 cm).
  3. Failsafe timeout: `elapsed >= POST_TURN_REVERSE_TIMEOUT_S (3.0s)`.

### Phase B: Forward Corner Turn (`_turn_step`)
- **Target Heading:** `self.target = self.corner_target` (from arc) or $(\text{target} \pm 90^\circ)$.
- **Dynamic Proportional Steering (`_corner_steer`):**
  $$\text{error} = \text{abs}(herr)$$
  $$\text{offset} = \min(\text{corner\_steer\_deg (28°)}, \max(\text{CORNER\_MIN\_OFF\_DEG (12°)}, \text{CORNER\_KP (0.8)} \times \text{error}))$$
  $$\text{steer} = 87 + \text{offset} \quad (\text{if Right turn}), \quad 87 - \text{offset} \quad (\text{if Left turn})$$
  *(Angle eases off as heading approaches target, preventing tire scrub).*
- **Wedge Detection & 3-Point Recovery:**
  - If `0 < front < REVERSE_START_CM (15 cm)`: Sets `self.is_reversing = True`.
  - While reversing: `speed = -reverse_speed`, steers opposite lock to back away.
  - Clears reversing state once `front > REVERSE_STOP_CM (22 cm)`.
- **Exit Conditions:**
  $$\text{abs}(herr) < \text{TURN\_DONE\_ERR\_DEG (13°)} \quad \text{OR} \quad \text{elapsed} > \text{TURN\_TIMEOUT\_S (2.5s)}$$
- **On Completion:**
  - `self.turns += 1`
  - `self.dodge_end_ticks = None` (clears post-dodge suppression on new straight).
  - Arms `post_turn_reverse = True`.
  - If `self.turns >= TOTAL_TURNS (12)`: transitions to `state = "FINISHING"`.

### Phase C: Post-Turn Straight Reverse (`post_turn_reverse`)
- **Steering:** Closed-loop heading hold in reverse:
  $$\text{steer} = 87 - \text{KP\_REVERSE\_HEADING (1.0)} \times herr$$
- **Exit Condition:** `moved >= post_turn_reverse_cm` (default 40 cm) or timeout (3.0s).

---

## 8. Pillar Acquisition & Tracking Logic

Scanning occurs when `self.pillar_stage is None`, `not self.is_turning`, and `not self.pre_turn_reverse`.

### Acquisition Suppression Gates:
1. **Cooldown Gate:**
   - Traveled distance: `abs(ticks - dodge_end_ticks) >= pillar_clear_cm (50 cm)`.
   - Elapsed time: `now - pillar_clear_t > pillar_clear_delay_s (1.2s)`.
2. **Close-Floor Gate (Prevent re-adopting the pillar just passed):**
   - If `traveled < clear_cm * 2 (100 cm)`: Minimum acquire distance is `PILLAR_MIN_ACQUIRE_CM = 45 cm`.
   - Otherwise: Floor is `PILLAR_MIN_ACQUIRE_FLOOR_CM = 20 cm`.
   - Any pillar closer than floor is discarded.
3. **"Already Clear" Gate (`already_clear()`):**
   - Red pillar already in `LEFT` band (`cx < 320 - 75 = 245 px`).
   - Green pillar already in `RIGHT` band (`cx > 320 + 75 = 395 px`).
   - If pillar is already on its correct passing side, it is **ignored** (no dodge executed).

### Candidate Selection:
- Filter valid blocks (`Red` or `Green`), sort by `dist_cm` ascending: `valid[0]` is nearest.
- Adopt: Sets `pillar_stage = "APPROACH"`, records `pillar_color`, `pillar_dist`, `pillar_cx`.

### Dynamic Target Switching in `APPROACH`:
If another pillar of the **opposite** color is detected:
- Condition: `other_dist >= 45 cm` AND `self.pillar_dist - other_dist >= PILLAR_SWITCH_MARGIN_CM (20 cm)`.
- **Switches target immediately to the nearer pillar.**

---

## 9. Pillar Approach & Backoff State Machine

### 9.1 Backoff Logic (`BACKOFF`)
If a pillar is detected too close, dodging without room causes the car to sideswipe it or clip it while turning.
- **Trigger Conditions (Enters `BACKOFF`):**
  1. `pillar_dist < pillar_trigger_dist_cm (30) - BACKOFF_IMMEDIATE_CM (8) = 22 cm` (Immediately too close).
  2. `pillar_dist < 30 cm` continuously for `> BACKOFF_GRACE_S (0.6s)` without commit frames confirming.
- **Execution:** Reverses straight holding heading (`-reverse_speed`, `_reverse_hold_steer`).
- **Exit Conditions:**
  - `pillar_dist >= BACKOFF_TARGET_CM (35 cm)` $\rightarrow$ Returns to `APPROACH`.
  - Traveled `stage_cm >= BACKOFF_MAX_CM (45 cm)` $\rightarrow$ Stops reversing, forces `APPROACH`.
  - Max tries: `BACKOFF_MAX_TRIES = 2`.

### 9.2 Approach Logic & Commit Conditions
In `APPROACH`, the car cruises straight (`_straight`) waiting for the trigger:
- **Clipped-Box Protection:** If bounding box is clipped at frame edge and distance reports an increase `> PILLAR_DIST_GROW_CM (15 cm)`, the increase is rejected and the last valid distance is held.
- **Pillar Lost Timeout:** If pillar is not seen for `APPROACH_LOST_S (1.5s)`, drops approach and resumes cruising.
- **Trigger Condition (Commits to `TURN_OUT`):**
  - **Normal Path:** `trigger_frames >= DODGE_TRIGGER_FRAMES (3)` consecutive camera frames with `pillar_dist <= pillar_trigger_dist_cm (30 cm)`.
  - **Front ToF Fast-Commit Shortcut:**
    If pillar is centered, `0 < front_tof <= PILLAR_TOF_COMMIT_CM (40 cm)`, seen within `0.4s`, and `abs(pillar_dist - front_tof) <= BLOCK_TOF_AGREE_CM (25 cm)`:
    - **Instantly sets `trigger_frames = 3` and commits to `TURN_OUT` immediately.**

---

## 10. The 4 Dodge Stages

Once committed, `dodge_running == True`. **No corner can interrupt these stages.**

```
(Commit) ──▶ Stage 1: TURN_OUT ──▶ Stage 2: STRAIGHTEN ──▶ Stage 3: RECENTER ──▶ Stage 4: ALIGN ──▶ Cruise
```

### Stage 1: `TURN_OUT`
- **Lateral Distance Calculation (`_out_cm()`):**
  - If `use_pixel_dodge == True` (Default):
    $$\text{offset\_px} = \text{abs}(cx - 320) \quad \text{towards dodge side}$$
    $$\text{out\_cm} = \text{clamp}(\text{pixel\_base\_cm (31.15)} + \text{offset\_px} \times \text{pixel\_cm\_per\_px (0.08)}, 8.0, 55.0)$$
  - Else: `dodge_short_cm (29 cm)` or `dodge_long_cm (35 cm)` + `dodge_red_extra_cm`.
- **Steering Angle Calculation:**
  - Red Base: `pillar_steer_right (135°)`.
  - Off-center adjustment: `red_out = 135 + sign * norm_x * offset_sens * 15.0`.
  - Mirrored for Green via `_as_red()`:
    $$\delta = \text{clamp}(red\_angle - 87, -53, +53)$$
    $$\text{steer} = 87 - \delta \quad (\text{for Green}), \quad 87 + \delta \quad (\text{for Red})$$
- **Unwind Profile:**
  - For progress $0.0 \to \text{turn\_hold\_pct (0.30)}$: Holds full `sharp_steer`.
  - For progress $0.30 \to 1.0$:
    $$\text{unwind} = \frac{\text{progress} - 0.30}{0.70}, \quad \text{steer} = \text{sharp} + (87 - \text{sharp}) \times \text{unwind}^{1.5}$$
- **Exit Condition:** `self.stage_cm >= out_cm`.

### Stage 2: `STRAIGHTEN` (Running Past Pillar)
- **Target Distance:** `_past_cm() = dodge_past_cm (50 cm) + dodge_past_red_extra_cm`.
- **Steering:** Calls `_straight(d, now, ignore_tof=True)`:
  - **Ignores ToF entirely** because the inner side sensor is pointing directly into the pillar body.
  - Steers on pure IMU heading error: `steer = 87 + KP_HEADING_CRUISE (1.2) * herr`.
- **Exit Condition:** `self.stage_cm >= _past_cm()`.

### Stage 3: `RECENTER` (Turning Back into Corridor)
- **Target Distance:** `_in_cm() = _leg_cm() + dodge_return_extra_cm (6.0 cm) + dodge_return_red_extra_cm`.
- **Return Angle:** Mirrored from turn-out and softened by `dodge_return_reduce_deg` (so the car does not cross the lane):
  $$\text{red\_mirror} = 2 \times 87 - \text{red\_out}$$
  $$\text{red\_return} = \min(87, \text{red\_mirror} + \text{dodge\_return\_reduce\_deg (10°)})$$
  $$\text{sharp\_return} = \text{\_as\_red}(\text{red\_return})$$
- **Pre-rotated Unwind Target:**
  Unwinds not to 87°, but to `return_end = _as_red(2*87 - DODGE_RETURN_END_DEG)` ($80^\circ$ for Green, $100^\circ$ for Red), pre-leaning the wheels to square up.
- **Exit Condition:** `self.stage_cm >= _in_cm()`.

### Stage 4: `ALIGN` (Squaring Up)
- **Dual P-Controller:**
  $$\text{align\_steer} = 87 + \text{kp\_align (2.4)} \times herr$$
  $$\text{wall\_steer} = \text{clamp}(\text{wall\_err} \times \text{recenter\_k (1.5)}, -40^\circ, +40^\circ)$$
  $$\text{steer} = \text{align\_steer} + \text{wall\_steer}$$
- **Exit Conditions (Must satisfy BOTH or timeout):**
  1. `abs(herr) < ALIGN_DONE_ERR_DEG (5.0°)`
  2. `self.stage_cm >= _align_min_cm()` (Guarantees forward travel so rear bumper clears pillar before ending dodge).
  3. Failsafe timeout: `self.stage_cm >= ALIGN_MAX_CM (45 cm)`.
- **On Completion:**
  - `self.dodge_end_ticks = self.ticks` (Arms distance suppression for corners and new acquisitions).
  - Resets all pillar tracking variables to `None`.
  - Resumes normal cruise.

---

## 11. Normal Cruising & Wall Centering (`_straight`)

Active when no dodge and no corner is running:

$$\text{steer} = 87 + \text{KP\_HEADING\_CRUISE (1.2)} \times herr + \text{self.nudge}$$

### `self.nudge` Calculation:
- **Both Walls in Range ($< 130\text{ cm}$):**
  $$\text{nudge} = \text{cruise\_wall\_k} \times \frac{\text{left} - \text{right}}{2.0}$$
- **Only Left Wall in Range:**
  $$\text{nudge} = \text{cruise\_wall\_k} \times (\text{cruise\_target\_cm (42)} - \text{left})$$
- **Only Right Wall in Range:**
  $$\text{nudge} = -\text{cruise\_wall\_k} \times (\text{cruise\_target\_cm (42)} - \text{right})$$
- **Neither Wall in Range:** `nudge = 0.0` (pure IMU gyro heading hold).

---

## 12. Systematic Bug-Hunter Diagnostic Matrix

Use this index to instantly identify which line of logic is responsible for a physical malfunction:

| Symptom on Track | Logical Cause in Code | Exact Parameter / Variable to Fix |
|---|---|---|
| **Car turns 90° into a pillar** | Front ToF saw pillar and fired corner before vision could suppress. | Raise `block_near_cm` (60 $\to$ 75) or raise `block_no_corner_s` (2.5 $\to$ 3.5). |
| **Car skips corner right after a pillar** | `block_no_corner_s` timer still active when car reached real wall. | Lower `block_no_corner_s` (2.5 $\to$ 1.5). |
| **Car turns in circles after a corner** | `moved_cm < MIN_TRAVEL_BETWEEN_TURNS_CM` bypassed by `CORNER_CRITICAL_FRONT_CM`. | Check if car is wedged against wall; raise `pre_turn_reverse_cm` (35 $\to$ 45). |
| **Car reverses too far before corner** | Pre-turn reverse arc didn't meet angle criteria and hit distance limit. | Raise `reverse_arc_deg` (40 $\to$ 45) or raise `reverse_arc_leave_deg` (35 $\to$ 40). |
| **Car hits inner wall during corner** | Corner turn radius too wide (front triggered too late). | Raise `front_turn_cm` (37 $\to$ 42). |
| **Car scrubs front tires during corner** | Corner steering angle too aggressive, losing lateral grip. | Lower `corner_steer_deg` (28 $\to$ 24). |
| **Car clips pillar on way out (Leg 1)** | Lateral displacement too small or trigger fired too late. | Raise `pixel_base_cm` (31 $\to$ 36) or raise `pillar_trigger_dist_cm` (30 $\to$ 35). |
| **Car clips pillar on straight (Leg 2)** | Began turning back in before rear wheels cleared pillar. | Raise `dodge_past_cm` (50 $\to$ 60). |
| **Car over-rotates on return (Leg 3)** | Return arc too sharp, throwing car across corridor. | Raise `dodge_return_reduce_deg` (0 $\to$ 8) or lower `dodge_return_extra_cm`. |
| **Car dodges same pillar twice** | Post-dodge clearance gate expired before passing pillar. | Raise `pillar_clear_cm` (50 $\to$ 65). |
| **Green pillar behaviors differ from Red** | Mechanical servo asymmetry. | Adjust `dodge_green_extra_cm` or `dodge_past_green_extra_cm` independently. |
| **Car weaves/oscillates on straight** | Wall centering feedback gain too high for current chassis speed. | Lower `cruise_wall_k` (1.6 $\to$ 0.8) or set to `0` for pure gyro. |
