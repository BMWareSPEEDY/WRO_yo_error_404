"""
preview_dashboard.py - open the dashboard on this laptop, with no car attached.

    python3 tools/preview_dashboard.py          # http://localhost:8081/
    python3 tools/preview_dashboard.py 9000     # a different port

The real page, the real stylesheet and the real parameter list straight out of
param_ui.py - only the data behind it is invented. The point is to see how the
thing LOOKS and to click every (i) button without needing the Pi, the ESP32 or
the camera.

It serves the same three endpoints the page asks for:

    /              the page, built exactly as the Pi builds it
    /api/state     made-up telemetry that drifts about so the page looks alive
    /api/params    the defaults from config.py
    /api/mode      a pretend "open round is loaded"

Parameter changes are accepted and echoed back, so the sliders behave, but
nothing is written to disk - `params.json` is left alone.
"""
import json
import math
import os
import random
import sys
import time
import types
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# dashboard.py opens a serial port on import. pyserial is not installed on a
# laptop and there is nothing to open anyway, so stand a hollow module in its
# place. Nothing in this script ever calls it.
if "serial" not in sys.modules:
    fake = types.ModuleType("serial")
    fake.Serial = object
    fake.SerialException = Exception
    sys.modules["serial"] = fake

import config                      # noqa: E402
import dashboard                   # noqa: E402  (for PAGE and build_params_html)
import param_ui                    # noqa: E402

LIVE = dict(config.load_params())


def fake_state():
    """Telemetry that moves, so the page does not look frozen."""
    t = time.time()
    wobble = lambda base, amp, speed: round(base + amp * math.sin(t * speed), 1)
    return {
        "state": "RUNNING", "stage": "TURN_OUT", "master_running": True,
        "decision": "Red pillar at 41 cm → going right round it",
        "pillar_color": "Red", "pillar_conf": 0.91, "pillar_dist": wobble(41, 6, 0.6),
        "turns": 5, "total_turns": config.TOTAL_TURNS, "direction": "CW",
        "runtime": round(t % 90, 1), "target": 180.0,
        "herr": wobble(0, 9, 0.8), "nudge": wobble(0, 4, 1.1),
        "speed": 90, "steer": int(wobble(90, 25, 0.7)),
        "tof": {"front": wobble(120, 60, 0.35), "left": wobble(44, 11, 0.5),
                "right": wobble(52, 14, 0.42), "rear": wobble(150, 40, 0.3)},
        "yaw": round((t * 24) % 360, 1), "cal": {"sys": 3, "gyro": 3, "accel": 2, "mag": 3},
        "esp_speed": 90, "esp_steer": int(wobble(90, 25, 0.7)),
        "esp_connected": True, "port": "/dev/preview", "rate_hz": round(49 + random.random(), 1),
        "telemetry_age": 0.02, "rx_lines": int(t) % 100000, "age": 0.1,
        "start_received": True, "temp_c": wobble(58, 4, 0.2), "ext5v_v": wobble(5.05, 0.06, 0.25),
        "throttled": 0, "health_msg": "preview - no hardware", "brownout": False,
        "log": ["preview mode - none of this is real",
                f"{len(param_ui.keys())} parameters, {len(param_ui.GROUPS)} groups",
                "click any (i) to see what a parameter does"],
    }


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            page = dashboard.PAGE.replace("{{PARAMS}}", dashboard.build_params_html(LIVE))
            return self._send(200, "text/html; charset=utf-8", page.encode())
        if path == "/api/params":
            return self._send(200, "application/json", json.dumps(LIVE).encode())
        if path == "/api/state":
            return self._send(200, "application/json", json.dumps(fake_state()).encode())
        if path == "/api/mode":
            return self._send(200, "application/json", json.dumps({
                "busy": False, "mode": "open", "step": "idle", "ok": None,
                "log": ["preview - the round buttons do nothing here"],
                "modes": {}}).encode())
        if path == "/stream.mjpg":
            return self._send(404, "text/plain", b"no camera in preview")
        return self._send(404, "text/plain", b"not found")

    def do_POST(self):
        path = self.path.split("?")[0]
        if path == "/api/params":
            body = self.rfile.read(int(self.headers.get("Content-Length", 0)))
            try:
                LIVE.update(json.loads(body.decode()))
                config.mirror_green(LIVE)          # so green tracks red here too
            except Exception:
                pass
            return self._send(200, "application/json", json.dumps(LIVE).encode())
        return self._send(200, "application/json", b'{"ok":true,"msg":"preview"}')


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8081
    missing, extra = param_ui.check(config.DEFAULT_PARAMS)
    if missing:
        print(f"WARNING  the car reads these, but the dashboard does not show them: {missing}")
    if extra:
        print(f"WARNING  the dashboard shows these, but the car has no such parameter: {extra}")
    print(f"{len(param_ui.keys())} parameters in {len(param_ui.GROUPS)} groups")
    print(f"\n    http://localhost:{port}/\n\nCtrl+C to stop")
    ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()


if __name__ == "__main__":
    main()
