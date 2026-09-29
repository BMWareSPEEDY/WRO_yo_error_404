#!/usr/bin/env python3
"""
parking_master.py - Standalone parking exit master controller.
Team Yo_Error_404 - WRO 2026 Future Engineers

Runs on the Raspberry Pi. Purely encoder-driven distance tracking
matching the obstacle round architecture.

Flow:
  1. WAITING: Prints live telemetry every 1s.
  2. Press START (car button or dashboard UI):
     - LOOK: Check Left vs Right ToF. Wall on Left -> Exit Right; Wall on Right -> Exit Left.
     - STAGE 1: TURN OUT away from wall (25 cm arc via encoders at 50° steering lock).
     - STAGE 2: STRAIGHT forward across lane (55 cm via encoders on ~80° heading hold).
     - STAGE 3: TURN BACK / RECENTER towards center (25 cm arc via encoders at opposite lock).
     - STAGE 4: FORWARD 20 CM & ALIGN (20 cm via encoders on 0° heading hold to square up).
     - HALT: Stop motor (0 PWM), center wheels (87°). Complete halt!
  3. Press button again:
     - If pressed during a run: immediately STOPS and returns to WAITING.
     - If pressed after completion: restarts a fresh run from scratch!
"""

import math
import os
import sys
import time

# Local config
import config


def wrap180(deg):
    """Normalize angle to [-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


def heading_error(target, current):
    """Positive = need to turn RIGHT, Negative = need to turn LEFT."""
    return wrap180(target - current)


def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))


class ParkingController:
    def __init__(self, start_yaw, start_ticks, left_cm, right_cm, now):
        self.start_yaw = start_yaw
        self.start_ticks = start_ticks
        self.t0 = now

        # Decide exit direction based on closer wall
        # Left < Right -> Exit RIGHT (+1)
        # Right < Left -> Exit LEFT (-1)
        self.exit_dir = 1 if left_cm <= right_cm else -1
        self.wall_side = "LEFT" if self.exit_dir == 1 else "RIGHT"
        self.exit_side = "RIGHT" if self.exit_dir == 1 else "LEFT"

        self.ticks = start_ticks
        self.stage = "TURN_OUT"
        self.stage_t0 = now
        self.stage_ticks0 = start_ticks
        self.stage_cm = 0.0

        print("\n" + "=" * 60)
        print(">>> STARTING PARKING EXIT (ENCODER-DRIVEN) <<<")
        print(f"Initial Sensors: Left = {left_cm:.1f} cm, Right = {right_cm:.1f} cm")
        print(f"Wall detected on {self.wall_side} -> Exiting to the {self.exit_side}")
        print(f"Start Heading: {self.start_yaw:.1f}° | Start Ticks: {self.start_ticks}")
        print(f"Lock: {config.STEER_LOCK_DEG}° (Steer {config.SERVO_CENTER + self.exit_dir * config.STEER_LOCK_DEG}°)")
        print(f"Planned: Turn Out {config.TURN_OUT_CM} cm -> Straight Across {config.STRAIGHT_CM} cm -> "
              f"Turn Back {config.TURN_IN_CM} cm -> Forward {config.FORWARD_CM} cm -> HALT")
        print("=" * 60 + "\n")

    def _begin_stage(self, stage, now):
        self.stage = stage
        self.stage_t0 = now
        self.stage_ticks0 = self.ticks
        self.stage_cm = 0.0

    def _stage_done(self, target_cm, now):
        if self.stage_cm >= target_cm:
            return True
        return (now - self.stage_t0) >= config.STAGE_TIMEOUT_S

    def total_dist_cm(self):
        return abs(self.ticks - self.start_ticks) / config.TICKS_PER_CM

    def step(self, d, now):
        """
        Takes current telemetry dict `d` and timestamp `now`.
        Returns (speed, steer, is_complete).
        """
        yaw = d["yaw"]
        t = d.get("ticks")
        if t is not None:
            self.ticks = t
            self.stage_cm = abs(self.ticks - self.stage_ticks0) / config.TICKS_PER_CM

        # -------------------------------------------------------------
        # STAGE 1: TURN OUT (25 cm via encoders, increased angle)
        # -------------------------------------------------------------
        if self.stage == "TURN_OUT":
            sharp_steer = config.SERVO_CENTER + self.exit_dir * config.STEER_LOCK_DEG
            progress = min(1.0, max(0.0, self.stage_cm / max(1.0, config.TURN_OUT_CM)))
            # Hold lock for 40% of the distance before unwinding
            if progress <= 0.40:
                steer = sharp_steer
            else:
                unwind = (progress - 0.40) / 0.60
                steer = sharp_steer + (config.SERVO_CENTER - sharp_steer) * (unwind ** 1.5)

            speed = config.EXIT_SPEED

            if self._stage_done(config.TURN_OUT_CM, now):
                print(f"[Stage 1: TURN OUT] Done: traveled {self.stage_cm:.1f} cm (Heading: {yaw:.1f}°)")
                self._begin_stage("STRAIGHT", now)
                print(f"[Stage 2: STRAIGHT ACROSS] Driving {config.STRAIGHT_CM:.1f} cm forward...")
            return speed, clamp(steer, config.SERVO_MIN, config.SERVO_MAX), False

        # -------------------------------------------------------------
        # STAGE 2: STRAIGHT ACROSS LANE (55 cm via encoders on heading hold)
        # -------------------------------------------------------------
        if self.stage == "STRAIGHT":
            # Hold angle towards perpendicular (~80 deg) across the lane
            leg_target_hdg = wrap180(self.start_yaw + self.exit_dir * 80.0)
            herr = heading_error(leg_target_hdg, yaw)
            steer = config.SERVO_CENTER + clamp(config.HOLD_KP * herr, -25.0, 25.0)
            speed = config.EXIT_SPEED

            if self._stage_done(config.STRAIGHT_CM, now):
                print(f"[Stage 2: STRAIGHT ACROSS] Done: traveled {self.stage_cm:.1f} cm (Heading: {yaw:.1f}°)")
                self._begin_stage("RECENTER", now)
                print(f"[Stage 3: RECENTER] Turning back towards center ({config.TURN_IN_CM:.1f} cm)...")
            return speed, clamp(steer, config.SERVO_MIN, config.SERVO_MAX), False

        # -------------------------------------------------------------
        # STAGE 3: TURN BACK / RECENTER (25 cm via encoders, opposite lock)
        # -------------------------------------------------------------
        if self.stage == "RECENTER":
            sharp_return = config.SERVO_CENTER - self.exit_dir * config.STEER_LOCK_DEG
            progress = min(1.0, max(0.0, self.stage_cm / max(1.0, config.TURN_IN_CM)))
            if progress <= 0.40:
                steer = sharp_return
            else:
                unwind = (progress - 0.40) / 0.60
                steer = sharp_return + (config.SERVO_CENTER - sharp_return) * (unwind ** 1.5)

            speed = config.EXIT_SPEED

            if self._stage_done(config.TURN_IN_CM, now):
                print(f"[Stage 3: RECENTER] Done: traveled {self.stage_cm:.1f} cm (Heading: {yaw:.1f}°)")
                self._begin_stage("FORWARD_20CM", now)
                print(f"[Stage 4: FORWARD] Driving {config.FORWARD_CM:.1f} cm forward to square up...")
            return speed, clamp(steer, config.SERVO_MIN, config.SERVO_MAX), False

        # -------------------------------------------------------------
        # STAGE 4: FORWARD 20 CM & SQUARE UP (Strictly 20 cm encoder-driven)
        # -------------------------------------------------------------
        if self.stage == "FORWARD_20CM":
            herr = heading_error(self.start_yaw, yaw)
            steer = config.SERVO_CENTER + clamp(config.HOLD_KP * herr, -25.0, 25.0)
            speed = config.EXIT_SPEED

            # Completes ONLY when stage_cm reaches 20.0 cm as measured by the encoder!
            if self._stage_done(config.FORWARD_CM, now):
                print(f"[Stage 4: FORWARD 20 CM] Done! Traveled {self.stage_cm:.1f} cm (Heading error: {herr:+.1f}°)")
                self.stage = "COMPLETE"
                print("\n" + "=" * 60)
                print(">>> PARKING EXIT FULLY COMPLETE - HALTING <<<")
                print(f"Total distance: {self.total_dist_cm():.1f} cm | Total Time: {now - self.t0:.2f} s")
                print(f"Final Heading: {yaw:.1f}° (error: {wrap180(yaw - self.start_yaw):+.1f}°)")
                print("Status: Motor STOPPED, steering CENTERED (87°). Car is HALTED.")
                print("=" * 60 + "\n")
                return 0, config.SERVO_CENTER, True
            return speed, clamp(steer, config.SERVO_MIN, config.SERVO_MAX), False

        return 0, config.SERVO_CENTER, True


def connect_link():
    """Tries to connect to dashboard relay (HubLink), falls back to direct serial."""
    try:
        if "/home/pi/obstacle_round" not in sys.path:
            sys.path.insert(0, "/home/pi/obstacle_round")
        import serial_link
        hub = serial_link.HubLink()
        time.sleep(0.5)
        if hub.hub_alive:
            print("Connected via Dashboard relay (UDP localhost:5005)")
            return hub, True
        hub.close()
    except Exception as e:
        print(f"HubLink not available ({e}), trying direct serial...")

    # Direct serial fallback
    import glob
    import serial
    ports = glob.glob("/dev/serial/by-id/usb-Espressif*") or glob.glob("/dev/ttyACM*")
    if not ports:
        raise RuntimeError("No ESP32 serial port found!")

    port = ports[0]
    print(f"Connecting directly to ESP32 on {port}...")
    ser = serial.Serial(port, 115200, timeout=0.05)
    return ser, False


def main():
    print("=" * 60)
    print("PARKING EXIT RUNNER - Team Yo_Error_404")
    print("=" * 60)

    try:
        link, is_hub = connect_link()
    except Exception as e:
        print(f"Error connecting: {e}")
        return 1

    ctl = None
    last_print = 0.0
    seq = 0

    print("\nWaiting for START button (press physical button on car or web dashboard)...")

    try:
        while True:
            # 1. Fetch latest telemetry
            if is_hub:
                d, seq = link.wait_new(seq, timeout=0.05)
                start_triggered = link.start_received
            else:
                line = link.readline().decode("utf-8", errors="ignore").strip()
                d = None
                start_triggered = False
                if "DATA," in line:
                    idx = line.rfind("DATA,")
                    chunk = line[idx:].split(",")
                    if len(chunk) >= 11:
                        try:
                            d = {
                                "front": float(chunk[1]),
                                "left": float(chunk[2]),
                                "right": float(chunk[3]),
                                "rear": float(chunk[4]),
                                "yaw": float(chunk[5]),
                                "ticks": int(chunk[12]) if len(chunk) > 12 else 0,
                            }
                        except (ValueError, IndexError):
                            d = None
                elif "START" in line:
                    start_triggered = True

            now = time.monotonic()

            if d is None:
                time.sleep(0.02)
                continue

            # 2. Check if stopped by user during run
            if ctl is not None and is_hub and not start_triggered:
                print("\n[STOP TRIGGERED] Button pressed during run. Halting car!")
                link.send(0, config.SERVO_CENTER)
                ctl = None
                print("Waiting for START button to run again...\n")
                continue

            # 3. Idle waiting
            if ctl is None:
                if now - last_print >= 1.0:
                    last_print = now
                    print(f"IDLE | Hdg={d['yaw']:5.1f}° | F={d['front']:3.0f} cm | "
                          f"L={d['left']:3.0f} cm | R={d['right']:3.0f} cm | Ticks={d.get('ticks', 0)}")

                if start_triggered:
                    # START pressed: Begin parking exit
                    ctl = ParkingController(d["yaw"], d.get("ticks", 0), d["left"], d["right"], now)
                continue

            # 4. Active parking run
            speed, steer, is_complete = ctl.step(d, now)
            if is_hub:
                link.send(speed, steer)
            else:
                link.write(f"{speed},{steer}\n".encode())

            if is_complete:
                print("Run complete. Resetting to WAITING state.")
                if is_hub:
                    link.send(0, config.SERVO_CENTER)
                    link.clear_start()
                else:
                    link.write(f"0,{config.SERVO_CENTER}\n".encode())
                ctl = None
                print("\n--- Press START button to run parking exit again! ---\n")

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nExiting on Ctrl+C...")
    finally:
        if is_hub:
            link.send(0, config.SERVO_CENTER)
            link.close()
        else:
            link.write(f"0,{config.SERVO_CENTER}\n".encode())
            link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
