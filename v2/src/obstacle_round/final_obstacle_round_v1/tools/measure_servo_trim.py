import csv, math, sys, re
sys.path.insert(0, "/home/pi/obstacle_round")
import config
f = sys.argv[1] if len(sys.argv) > 1 else "/home/pi/obstacle_round/logs/run_20260924_083553.csv"
rows = list(csv.DictReader(open(f)))
print("  %s  -  %d ticks, %.1f s" % (f.split("/")[-1], len(rows), float(rows[-1]["t"])))

def fl(r, k):
    try: return float(r[k])
    except (TypeError, ValueError): return None

def unwrap(a, b):
    d = b - a
    while d > 180: d -= 360
    while d < -180: d += 360
    return d

# ---- 1. REVERSE at commanded centre: any yaw change is mechanical bias ----
segs, cur = [], None
for r in rows:
    sp, st = fl(r, "speed"), fl(r, "steer")
    straight_back = sp is not None and sp < 0 and st is not None and abs(st - config.SERVO_CENTER) < 0.6
    if straight_back:
        if cur is None: cur = []
        cur.append(r)
    elif cur:
        if len(cur) > 25: segs.append(cur)
        cur = None
if cur and len(cur) > 25: segs.append(cur)

print()
print("  STRAIGHT REVERSE segments (steer commanded exactly %d - wheels should be straight)" % config.SERVO_CENTER)
tot_yaw = tot_cm = 0.0
for s in segs:
    y0, y1 = fl(s[0], "yaw"), fl(s[-1], "yaw")
    t0, t1 = fl(s[0], "ticks"), fl(s[-1], "ticks")
    if None in (y0, y1, t0, t1): continue
    dy, dcm = unwrap(y0, y1), abs(t1 - t0) / config.TICKS_PER_CM
    if dcm < 10: continue
    tot_yaw += dy; tot_cm += dcm
    print("   %6.2fs  %5.1f cm back  yaw %+6.1f deg  (%+5.2f deg/cm)"
          % (float(s[0]["t"]), dcm, dy, dy / dcm))
if tot_cm > 0:
    rate = tot_yaw / tot_cm
    print("   ---- overall: %+.1f deg over %.0f cm = %+.3f deg/cm" % (tot_yaw, tot_cm, rate))
    L = 15.0    # wheelbase, cm (approx)
    delta = math.degrees(math.atan(L * math.radians(rate)))
    print("   a straight reverse should hold yaw. It drifts %s,"
          % ("RIGHT (clockwise)" if rate > 0 else "LEFT (anticlockwise)"))
    print("   implying the wheels sit ~%.1f deg off centre with the servo at %d."
          % (abs(delta), config.SERVO_CENTER))

# ---- 2. FORWARD cruise: what angle does the controller settle on? ---------
vals = []
for r in rows:
    if not r["stage"].startswith("Normal driving"): continue
    sp, st, he, nu = fl(r, "speed"), fl(r, "steer"), fl(r, "herr"), fl(r, "nudge")
    if None in (sp, st, he, nu) or sp <= 0: continue
    if abs(he) > 6: continue                    # only while it is actually tracking
    vals.append(st - config.KP_HEADING * he - nu)
print()
if len(vals) > 40:
    vals.sort()
    med = vals[len(vals)//2]
    print("  FORWARD CRUISE - servo angle the controller settles on while holding heading")
    print("   %d samples, median %.1f  (nominal centre %d)" % (len(vals), med, config.SERVO_CENTER))
    print("   => straight-ahead is about %.0f, i.e. SERVO_TRIM %+.0f" % (med, med - config.SERVO_CENTER))
else:
    print("  FORWARD CRUISE - not enough clean samples (%d)" % len(vals))
