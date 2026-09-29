"""steer_check.py - which way does the car ACTUALLY turn?

    python3 tools/steer_check.py            # both directions
    python3 tools/steer_check.py right      # one of them

Drives a short arc at the corner steering angle and reports what the IMU did.
Watch the car while it runs: the point is to compare what you SEE with what the
code believes.

It answers the one question the run logs cannot. In a log, commanded steering,
IMU yaw and heading error are all consistent with each other - the loop closes
and the corner "completes" - whether or not the servo and IMU are both mirrored
relative to the real world. Only a human looking at the car can break the tie.

Nothing is changed by running this. It drives for about a second in each
direction at the normal corner angle, through the same relay main.py uses, so
master.service must be stopped first:

    sudo systemctl stop master.service
    cd ~/obstacle_round && python3 tools/steer_check.py
    sudo systemctl start master.service
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config                                       # noqa: E402
from serial_link import HubLink                     # noqa: E402

DRIVE_S = 1.2
SPEED = config.TURN_SPEED


def arc(link, name, steer):
    d, _ = link.latest()
    if d is None:
        print("no telemetry - is dashboard.service running?")
        return
    start_yaw = d["yaw"]
    print(f"\n{name}: servo {steer} for {DRIVE_S:.1f} s - WATCH THE CAR")
    t_end = time.monotonic() + DRIVE_S
    while time.monotonic() < t_end:
        link.send(SPEED, steer)
        time.sleep(0.02)
    link.send(0, config.SERVO_CENTER)
    time.sleep(0.6)
    d, _ = link.latest()
    end_yaw = d["yaw"] if d else start_yaw
    delta = (end_yaw - start_yaw + 540) % 360 - 180
    imu_says = "CLOCKWISE (right)" if delta > 0 else "ANTI-CLOCKWISE (left)"
    print(f"  yaw {start_yaw:.0f} -> {end_yaw:.0f}  ({delta:+.0f} deg)")
    print(f"  the IMU says the car turned {imu_says}")
    print(f"  the code believes servo {steer} steers "
          f"{'RIGHT' if steer > config.SERVO_CENTER else 'LEFT'}")
    return delta


def main():
    which = sys.argv[1].lower() if len(sys.argv) > 1 else "both"
    off = float(config.load_params().get("corner_steer_deg", 28))
    link = HubLink()
    time.sleep(0.8)
    if not link.hub_alive:
        print("the relay is not answering - start dashboard.service first")
        return 1
    link.trigger_start()            # the ESP32 ignores commands before START
    time.sleep(0.2)

    print("Starting in 3 s - make sure the car has room in front of it.")
    time.sleep(3)
    try:
        if which in ("both", "right"):
            arc(link, "RIGHT arc", int(config.SERVO_CENTER + off))
        if which == "both":
            time.sleep(1.5)
        if which in ("both", "left"):
            arc(link, "LEFT arc", int(config.SERVO_CENTER - off))
    finally:
        link.send(0, config.SERVO_CENTER)
        link.close()
    print("\nIf what you SAW disagrees with what the IMU said, the servo and the "
          "IMU are both mirrored on this car:\n"
          "tell me and I will invert them in config, which fixes corner direction "
          "everywhere at once.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
