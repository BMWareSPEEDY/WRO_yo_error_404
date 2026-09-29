#!/usr/bin/env python3
"""reverse_cm.py - drive the car BACKWARDS a measured distance, then stop.

Talks straight to the ESP32 over USB, so nothing needs reflashing. Distance is
measured on the wheel encoder (the <ticks> field of the DATA line), not timed,
so it is the same on a full battery and a flat one.

    sudo systemctl stop master.service dashboard.service   # they own the port
    python3 tools/reverse_cm.py [cm] [pwm]

Defaults: 50 cm at 150 PWM. Speed is capped at the team limit of 170.

Stops early and reports if the car is not actually moving - reverse has failed
on this car before with the ESP reporting -150 applied and the encoder frozen,
which is a driver/wiring fault rather than anything in software.
"""
import glob
import sys
import time

import serial

sys.path.insert(0, "/home/pi/obstacle_round")
import config

TARGET_CM = float(sys.argv[1]) if len(sys.argv) > 1 else 50.0
PWM = min(int(sys.argv[2]) if len(sys.argv) > 2 else 150, config.MAX_PWM)
TARGET_TICKS = abs(TARGET_CM) * config.TICKS_PER_CM

TIMEOUT_S = 15.0
STUCK_S = 1.0          # if it has not moved by now, it is not going to
STUCK_CM = 1.0


def newest(s):
    """Most recent complete DATA line: (ticks, applied_speed)."""
    best = (None, None)
    for _ in range(60):
        line = s.readline().decode(errors="replace").strip()
        if line.startswith("DATA,"):
            f = line.split(",")
            if len(f) == 13:
                try:
                    best = (int(f[12]), int(f[10]))
                except ValueError:
                    pass
        if s.in_waiting == 0 and best[0] is not None:
            break
    return best


def main():
    port = glob.glob("/dev/serial/by-id/usb-Espressif*")
    if not port:
        print("ESP32 not found on /dev/serial/by-id/")
        return 1
    s = serial.Serial(port[0], config.SERIAL_BAUD, timeout=0.05)
    s.reset_input_buffer()
    time.sleep(0.2)

    s.write(b"START\n")
    s.flush()
    time.sleep(0.4)
    s.reset_input_buffer()
    time.sleep(0.1)

    start_ticks, _ = newest(s)
    if start_ticks is None:
        print("no telemetry from the ESP32")
        return 1
    print(f"target {TARGET_CM:.0f} cm backwards = {TARGET_TICKS:.0f} ticks at {PWM} PWM")
    print(f"start ticks {start_ticks}")

    cmd = f"{-PWM},{config.SERVO_CENTER}\n".encode()
    t0 = time.monotonic()
    moved = 0.0
    applied = None
    try:
        while True:
            s.write(cmd)
            s.flush()
            time.sleep(0.04)
            ticks, applied = newest(s)
            if ticks is not None:
                moved = abs(ticks - start_ticks) / config.TICKS_PER_CM
                print(f"\r  {moved:6.1f} / {TARGET_CM:.0f} cm   (esp applied {applied})   ",
                      end="", flush=True)
                if moved >= abs(TARGET_CM):
                    print("\nreached target")
                    break
            elapsed = time.monotonic() - t0
            if elapsed > STUCK_S and moved < STUCK_CM:
                print(f"\nNOT MOVING after {elapsed:.1f}s - ESP reports applying {applied}, "
                      f"encoder has not changed.\nReverse is not driving the motor: "
                      f"check the MD10112 direction input and its wiring.")
                break
            if elapsed > TIMEOUT_S:
                print(f"\ntimeout after {elapsed:.1f}s at {moved:.1f} cm")
                break
    finally:
        for _ in range(15):
            s.write(f"0,{config.SERVO_CENTER}\n".encode())
            s.flush()
            time.sleep(0.03)
        time.sleep(0.4)
        end_ticks, _ = newest(s)
        if end_ticks is not None and start_ticks is not None:
            total = abs(end_ticks - start_ticks) / config.TICKS_PER_CM
            print(f"final: {total:.1f} cm travelled ({abs(end_ticks - start_ticks)} ticks)")
        s.write(b"STOP\n")
        s.flush()
        s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
