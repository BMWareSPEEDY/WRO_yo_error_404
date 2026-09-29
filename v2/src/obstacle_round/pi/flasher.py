"""
flasher.py - switch the car between rounds from the dashboard.

Three modes, one button each:

    open        the ESP32 drives the open round by itself, the Pi just watches
    obstacle    open_slave on the ESP32, main.py on the Pi drives
    parking     open_slave on the ESP32, parking_master.py on the Pi drives

Open round is the default the car boots into: no parking, no main.py, the
ESP32 just waits for its START button. Obstacle round is the same car with
main.py driving - also no parking. Parking is a test button on its own; a real
round never runs it.

Pressing a button does the whole job: compile the sketch with arduino-cli,
release the serial port, flash it with esptool, take the port back, then start
the right Pi service and stop the others - and enable the right one at boot, so
the car comes back up in the mode it was left in.

Everything runs in a background thread and reports progress into `status()`,
which the page polls. A flash takes about 15 s; nothing else on the dashboard
blocks while it happens.

Stand-alone:
    python3 flasher.py obstacle
"""
import json
import logging
import os
import subprocess
import threading
import time

import config

log = logging.getLogger("flash")

MODE_FILE = os.path.join(config.BASE_DIR, "mode.json")

# What each mode needs. `sketch` is a directory under ~/sketches and `build` its
# existing build directory - reused on purpose, since a warm one compiles in
# about 8 s where a fresh one re-downloads the toolchain. `service` is
# the systemd unit that drives the car in that mode (None = the ESP32 drives
# itself and nothing on the Pi should be sending commands).
MODES = {
    "open": {
        "label": "Open round",
        "sketch": "final_open_round_working",
        "build": "build_fowr",
        "service": None,
        "blurb": "ESP32 drives alone - press START on the car",
    },
    "obstacle": {
        "label": "Obstacle round",
        "sketch": "open_slave",
        "build": "build_slave",
        "service": "master.service",
        "blurb": "Pi drives, pillars and corners - no parking",
    },
    "parking": {
        "label": "Parking test",
        "sketch": "open_slave",
        "build": "build_slave",
        "service": "parking.service",
        "blurb": "test only - the parking exit on its own",
    },
}
ALL_SERVICES = ("master.service", "parking.service")


def read_mode():
    try:
        with open(MODE_FILE) as f:
            m = json.load(f).get("mode")
            return m if m in MODES else None
    except Exception:
        return None


def _write_mode(mode):
    try:
        with open(MODE_FILE, "w") as f:
            json.dump({"mode": mode, "set_at": time.strftime("%Y-%m-%d %H:%M:%S")}, f, indent=2)
    except Exception as exc:
        log.warning(f"could not save the mode: {exc}")


def active_service():
    """Which driving service is running right now, if any."""
    for svc in ALL_SERVICES:
        out = subprocess.run(["systemctl", "is-active", svc], capture_output=True, text=True)
        if out.stdout.strip() == "active":
            return svc
    return None


class Flasher:
    """Runs one mode switch at a time, with a log the page can read."""

    def __init__(self, serial_link):
        self.serial = serial_link
        self._lock = threading.Lock()
        self._busy = False
        self._mode = read_mode()
        self._step = "idle"
        self._log = []
        self._ok = None

    # ------------------------------------------------------------------ state
    def status(self):
        with self._lock:
            return {
                "busy": self._busy,
                "mode": self._mode,
                "step": self._step,
                "ok": self._ok,
                "log": list(self._log[-14:]),
                "modes": {k: {"label": v["label"], "blurb": v["blurb"]} for k, v in MODES.items()},
            }

    def _say(self, text, step=None):
        log.info(text)
        with self._lock:
            self._log.append(f"{time.strftime('%H:%M:%S')} {text}")
            if step:
                self._step = step

    # ------------------------------------------------------------------ doing
    def start(self, mode):
        if mode not in MODES:
            return False, f"unknown mode {mode!r}"
        with self._lock:
            if self._busy:
                return False, "a flash is already running"
            self._busy, self._ok, self._log, self._step = True, None, [], "starting"
        threading.Thread(target=self._run, args=(mode,), name="flasher", daemon=True).start()
        return True, "started"

    def _run(self, mode):
        spec = MODES[mode]
        ok = False
        try:
            self._say(f"switching to {spec['label']}", "preparing")

            # 1. Stop whatever is driving, so nothing commands the car mid-flash.
            for svc in ALL_SERVICES:
                self._systemctl("stop", svc)
            self._say("driving services stopped")

            # 2. Compile. Done before the port is released, so a broken sketch
            #    costs nothing: the car keeps its working firmware.
            build = os.path.join(config.SKETCH_DIR, spec["build"])
            self._say(f"compiling {spec['sketch']}...", "compiling")
            r = self._run_cmd([config.ARDUINO_CLI, "compile", "--fqbn", config.ESP_FQBN,
                               "--output-dir", build, spec["sketch"]],
                              cwd=config.SKETCH_DIR, timeout=600)
            if r.returncode != 0:
                self._say(f"compile FAILED: {self._tail(r)}")
                return
            self._say([l for l in r.stdout.splitlines() if "Sketch uses" in l][-1:][0]
                      if "Sketch uses" in r.stdout else "compiled")

            binary = os.path.join(build, f"{spec['sketch']}.ino.merged.bin")
            if not os.path.exists(binary):
                self._say(f"no binary at {binary}")
                return

            # 3. Hand the port over, flash, take it back - always take it back,
            #    even if esptool fails, or the dashboard goes deaf.
            self._say("releasing the serial port", "flashing")
            self.serial.pause()
            time.sleep(1.0)
            try:
                r = self._run_cmd([config.ESPTOOL, "--chip", "esp32c3", "-p", self._port(),
                                   "-b", "460800", "write-flash", "0x0", binary], timeout=300)
            finally:
                self.serial.resume()
                self._say("serial port back")
            if r.returncode != 0 or "Hash of data verified" not in r.stdout:
                self._say(f"FLASH FAILED: {self._tail(r)}")
                return
            self._say("flash verified")

            # 4. Point the Pi at the right code, now and at boot.
            self._say("setting the Pi side", "services")
            for svc in ALL_SERVICES:
                if svc == spec["service"]:
                    self._systemctl("enable", svc)
                    self._systemctl("start", svc)
                else:
                    self._systemctl("disable", svc)
            if spec["service"]:
                self._say(f"{spec['service']} running, and enabled at boot")
            else:
                self._say("nothing on the Pi drives in this mode")

            _write_mode(mode)
            with self._lock:
                self._mode = mode
            self._say(f"{spec['label']} ready - {spec['blurb']}", "done")
            ok = True
        except Exception as exc:
            self._say(f"error: {exc}")
        finally:
            with self._lock:
                self._busy, self._ok = False, ok
                if not ok:
                    self._step = "failed"

    # ----------------------------------------------------------------- helpers
    def _port(self):
        import glob
        found = sorted(glob.glob("/dev/serial/by-id/usb-Espressif*")) or \
            sorted(glob.glob("/dev/serial/by-id/*"))
        return found[0] if found else "/dev/ttyACM0"

    def _run_cmd(self, cmd, cwd=None, timeout=300):
        env = dict(os.environ, PATH=f"{os.path.expanduser('~/bin')}:{os.environ.get('PATH', '')}")
        return subprocess.run(cmd, cwd=cwd, env=env, capture_output=True, text=True, timeout=timeout)

    def _systemctl(self, verb, svc):
        """sudo systemctl - needs the NOPASSWD rule in /etc/sudoers.d/wro-mode."""
        r = subprocess.run(["sudo", "-n", "systemctl", verb, svc], capture_output=True, text=True)
        if r.returncode != 0:
            self._say(f"sudo systemctl {verb} {svc} failed: {(r.stderr or '').strip()[:60]}")
        return r.returncode == 0

    @staticmethod
    def _tail(r, n=200):
        out = ((r.stderr or "") + (r.stdout or "")).strip().replace("\n", " | ")
        return out[-n:]


if __name__ == "__main__":
    import sys

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    from serial_link import SerialLink

    want = sys.argv[1] if len(sys.argv) > 1 else "obstacle"
    f = Flasher(SerialLink())
    print(f.start(want))
    while f.status()["busy"]:
        time.sleep(1)
    st = f.status()
    print("\n".join(st["log"]))
    print("OK" if st["ok"] else "FAILED")
