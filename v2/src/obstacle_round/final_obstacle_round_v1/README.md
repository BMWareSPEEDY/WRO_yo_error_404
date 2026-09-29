# final_obstacle_round_v1

A frozen snapshot of the working obstacle round, taken 2026-09-24.
**Do not edit this directory.** It is the known-good fallback: if an experiment
in `obstacle_round/pi/` goes wrong, copy these files back over it.

## What it does

| Phase | Behaviour |
|---|---|
| Straights | Lane centring copied verbatim from `final_open_round_working.ino` — `correction = headingError * 1.2 + wallCentering`, Kp_Wall 1.4, target 42 cm, no integral |
| Corners | Drive to 37 cm, reverse-arc on the IMU until 35° remain, forward turn onto the exact heading, then 35 cm straight back |
| Pillars | APPROACH → TURN_OUT → STRAIGHTEN → RECENTER → ALIGN, every stage ending on measured encoder distance |

## Dodge legs

| colour | turn-out | run-past | back-in |
|---|---|---|---|
| Green | 31 cm | 52 cm | 37 cm |
| Red | 35 cm | 47 cm | 38 cm |

Green and Red use a wide arc when the pillar already sits on the side the car
must dodge towards, and a normal arc otherwise. **This is what the pixel-offset
method is intended to replace.**

## Priorities, in order

1. **Corner turns** — a missed corner ends the run
2. **Pillar dodging** — only a pillar *dead ahead* may take the front sensor's
   reading or suppress a corner; anything off to the side yields to the turn
3. **Lane centring** — runs only when no pillar stage is active

## Tests

`tools/` holds twelve suites that run with no hardware:

```bash
python3 tools/smoke_test.py            # every dodge stage + both corner paths
python3 tools/corner_test.py           # corners land on 90 deg
python3 tools/centred_pillar_test.py   # off-centre pillars yield to the corner
python3 tools/nearest_pillar_test.py   # the nearest pillar is followed
...
```
