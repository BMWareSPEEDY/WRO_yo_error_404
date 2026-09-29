"""
dashboard.py - runs from boot (dashboard.service), whether main.py runs or not.

It is the only program that opens the ESP32 serial port and the camera:
    - live web dashboard on http://error404.local:8080/
        /               camera + everything the ESP32 sends + what main.py decided
        /stream.mjpg    camera stream with a HUD strip
        /api/state      the same data as JSON
    - relay for main.py over UDP 127.0.0.1:HUB_PORT: forwards every ESP32 line
      to main.py and writes main.py's commands to the ESP32 (see HubLink in
      serial_link.py). It never sends a drive command of its own.

Frames are only JPEG-encoded while a browser is watching. In competition mode
(flag file exists, rule 11.10) the web page and camera stay off, the relay
keeps running so main.py can still drive.

    python3 dashboard.py
"""
import json
import re
import socket
import logging
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import subprocess
import cv2

import config
from serial_link import SerialLink, command_line
from vision import draw_blocks, draw_rois

log = logging.getLogger("dash")


class _LogTail(logging.Handler):
    def __init__(self, n):
        super().__init__(level=logging.INFO)
        self.lines = deque(maxlen=n)

    def emit(self, record):
        try:
            self.lines.append(f"{time.strftime('%H:%M:%S')} {record.levelname[0]} "
                              f"{record.name}: {record.getMessage()}")
        except Exception:
            pass


LOG_TAIL = _LogTail(config.DASHBOARD_LOG_LINES)   # attached in main() before anything logs


def read_system_health():
    """Reads Pi 5 SoC temperature, EXT5V rail voltage, and throttled bitmask."""
    temp_c, ext5v_v, throttled_val, status_msg, brownout = None, None, 0, "OK", False
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            temp_c = round(int(f.read().strip()) / 1000.0, 1)
    except Exception:
        pass
    try:
        res = subprocess.check_output(["vcgencmd", "pmic_read_adc"], timeout=0.5).decode("ascii", "replace")
        m = re.search(r"EXT5V_V volt\(\d+\)=([0-9.]+)V", res)
        if m:
            ext5v_v = round(float(m.group(1)), 2)
    except Exception:
        pass
    try:
        res = subprocess.check_output(["vcgencmd", "get_throttled"], timeout=0.5).decode("ascii", "replace")
        m = re.search(r"throttled=(0x[0-9a-fA-F]+)", res)
        if m:
            val = int(m.group(1), 16)
            throttled_val = val
            warnings = []
            if val & 0x1:
                warnings.append("UNDERVOLTAGE NOW (<4.63V)")
                brownout = True
            elif val & 0x10000:
                warnings.append("Undervoltage occurred")
            if val & 0x2:
                warnings.append("Arm freq capped")
            if val & 0x4:
                warnings.append("Currently throttled")
            if val & 0x8:
                warnings.append("Soft temp limit")
            status_msg = ", ".join(warnings) if warnings else "OK"
    except Exception:
        pass
    if ext5v_v is not None and ext5v_v < 4.70:
        brownout = True
        if status_msg == "OK":
            status_msg = "LOW VOLTAGE (<4.7V)"
    return {
        "temp_c": temp_c,
        "ext5v_v": ext5v_v,
        "throttled": throttled_val,
        "health_msg": status_msg,
        "brownout": brownout,
    }


class Dashboard:
    def __init__(self, camera=None, vision=None):
        self.camera = camera
        self.vision = vision
        self.hub = None
        self._lock = threading.Lock()
        self._state = {}
        self._health = {"temp_c": None, "ext5v_v": None, "throttled": 0, "health_msg": "reading...", "brownout": False}
        self._jpeg, self._jpeg_t = None, 0.0
        self._wanted_until = 0.0
        self._server = None
        self._logtail = LOG_TAIL
        threading.Thread(target=self._health_poller, name="dash-health", daemon=True).start()

    def _health_poller(self):
        while True:
            try:
                h = read_system_health()
                with self._lock:
                    self._health = h
            except Exception:
                pass
            time.sleep(1.0)

    @staticmethod
    def allowed():
        if not config.DASHBOARD_ENABLE:
            return False, "DASHBOARD_ENABLE is False"
        if os.path.exists(config.COMPETITION_FLAG_FILE):
            return False, f"{config.COMPETITION_FLAG_FILE} exists (competition mode)"
        return True, ""

    def start(self):
        dash = self

        class Handler(_Handler):
            dashboard = dash

        try:
            self._server = _DualStackServer(("::", config.DASHBOARD_PORT), Handler)
        except OSError as exc:
            log.error(f"dashboard could not start on port {config.DASHBOARD_PORT}: {exc}")
            return False
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, name="dash-http", daemon=True).start()
        threading.Thread(target=self._encoder, name="dash-jpeg", daemon=True).start()
        log.info(f"dashboard on http://error404.local:{config.DASHBOARD_PORT}/")
        return True

    def update(self, **state):
        with self._lock:
            self._state.update(state)
            self._state["updated"] = time.time()

    def state_json(self):
        with self._lock:
            st = dict(self._state)
            st.update(self._health)
        st["log"] = list(self._logtail.lines)
        st["age"] = time.time() - st["updated"] if "updated" in st else None
        # fps_measured keeps its last value when the device vanishes mid-run, so
        # the page showed "22 fps, ok" with nothing on the USB bus at all.
        # Report a camera as running only while frames are genuinely arriving.
        cam_ok = bool(self.camera) and getattr(self.camera, "frames_fresh", True)
        st["camera_fps"] = self.camera.fps_measured if cam_ok else None
        return json.dumps(st, default=float).encode()

    # ------------------------------------------------------------ camera stream
    def _encoder(self):
        gap = 1.0 / config.DASHBOARD_STREAM_FPS
        params = [int(cv2.IMWRITE_JPEG_QUALITY), config.DASHBOARD_JPEG_QUALITY]
        last_id = 0
        while True:
            if self.camera is None or time.monotonic() > self._wanted_until:
                time.sleep(0.2)
                continue
            t0 = time.monotonic()
            blocks = None
            if self.vision is not None:
                # the frame YOLO ran on, with its own boxes - they always line up
                v = self.vision.latest()
                if v is None or v[1] == last_id:
                    time.sleep(0.01)
                    continue
                frame, fid, _, blocks = v
            else:
                frame, fid, _ = self.camera.read(newer_than=last_id, timeout=0.5)
                if frame is None or fid == last_id:
                    continue
            last_id = fid
            with self._lock:
                st = dict(self._state)
            img = _hud(frame, st)
            if blocks:
                draw_blocks(img, blocks)      # bands + boxes
            else:
                draw_rois(img)                # bands alone, so they never disappear
            ok, buf = cv2.imencode(".jpg", img, params)
            if ok:
                with self._lock:
                    self._jpeg, self._jpeg_t = buf.tobytes(), time.monotonic()
            time.sleep(max(0.0, gap - (time.monotonic() - t0)))

    def _want_stream(self):
        self._wanted_until = time.monotonic() + 3.0

    def latest_jpeg(self):
        with self._lock:
            return self._jpeg, self._jpeg_t


def _hud(frame, st):
    img = frame.copy()
    w = img.shape[1]
    t = st.get("tof") or {}
    cv2.rectangle(img, (0, 0), (w, 30), (0, 0, 0), -1)
    text = (f"{st.get('state', '?')}  turn {st.get('turns', 0)}/{config.TOTAL_TURNS}  "
            f"F{t.get('front', '-')} L{t.get('left', '-')} R{t.get('right', '-')} B{t.get('rear', '-')}  "
            f"yaw {st.get('yaw') or 0:.0f}  spd {st.get('speed', 0)}  str {st.get('steer', 90)}")
    cv2.putText(img, text, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return img


class _DualStackServer(ThreadingHTTPServer):
    """Listens on IPv6 AND IPv4: error404.local often resolves to the Pi's IPv6
    address, and a plain 0.0.0.0 server never answers that."""
    address_family = socket.AF_INET6

    def server_bind(self):
        self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        super().server_bind()


class _Handler(BaseHTTPRequestHandler):
    dashboard = None
    protocol_version = "HTTP/1.0"

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
        d = self.dashboard
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            p = config.load_params()
            body = (PAGE
                .replace("{{PARAM_VAL_TRIG}}", str(p.get("pillar_trigger_dist_cm", 35)))
                .replace("{{PARAM_VAL_LEFT}}", str(p.get("pillar_steer_left", 45)))
                .replace("{{PARAM_VAL_RIGHT}}", str(p.get("pillar_steer_right", 135)))
                .replace("{{PARAM_VAL_HOLD}}", str(p.get("turn_hold_pct", 30)))
                .replace("{{PARAM_VAL_SENS}}", f"{float(p.get('block_offset_sens', 0.8)):.2f}")
                .replace("{{PARAM_VAL_RECTGT}}", str(p.get("recenter_target_cm", 45.0)))
                .replace("{{PARAM_VAL_RECK}}", f"{float(p.get('recenter_k', 0.8)):.2f}")
                .replace("{{PARAM_VAL_FOCAL}}", str(p.get("pillar_focal_px", 710.0)))
                .replace("{{PARAM_VAL_DPAST}}", str(p.get("dodge_past_cm", 34.0)))
                .replace("{{PARAM_VAL_DSHORT}}", str(p.get("dodge_short_cm", 23.0)))
                .replace("{{PARAM_VAL_DLONG}}", str(p.get("dodge_long_cm", 35.0)))
                .replace("{{PARAM_VAL_CSTEER}}", str(p.get("corner_steer_deg", 30.0)))
                .replace("{{PARAM_VAL_FTURN}}", str(p.get("front_turn_cm", 45.0)))
                .replace("{{PARAM_VAL_PTREV}}", str(p.get("post_turn_reverse_cm", 30.0)))
                .replace("{{PARAM_VAL_BNEAR}}", str(p.get("block_near_cm", 60.0)))
                .replace("{{PARAM_VAL_BHOLD}}", f"{float(p.get('block_no_corner_s', 2.5)):.1f}")
            )
            return self._send(200, "text/html; charset=utf-8", body.encode())
        if path == "/api/state":
            return self._send(200, "application/json", d.state_json())
        if path == "/api/params":
            p = config.load_params()
            return self._send(200, "application/json", json.dumps(p).encode())
        if path == "/stream.mjpg":
            return self._stream()
        return self._send(404, "text/plain", b"not found")

    def _stream(self):
        d = self.dashboard
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last_t = 0.0
        try:
            while True:
                d._want_stream()
                buf, t = d.latest_jpeg()
                if buf is None or t == last_t:
                    time.sleep(0.03)
                    continue
                last_t = t
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(buf)).encode() + b"\r\n\r\n" + buf + b"\r\n")
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def do_POST(self):
        d = self.dashboard
        path = self.path.split("?")[0]
        if path == "/api/params":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length)
            try:
                new_params = json.loads(body.decode("utf-8"))
                save_disk = "save=1" in self.path
                p = config.save_params(new_params, save_to_disk=save_disk)
                if d:
                    if d.hub:
                        d.hub.forward_params(p)
                    if d.vision and "pillar_focal_px" in p:
                        d.vision.set_focal_px(p["pillar_focal_px"])
                return self._send(200, "application/json", json.dumps(p).encode())
            except Exception as exc:
                return self._send(400, "text/plain", str(exc).encode())
        if path == "/api/run":
            action = "toggle"
            if "?" in self.path:
                for param in self.path.split("?", 1)[1].split("&"):
                    if param.startswith("action="):
                        action = param.split("=")[1]
            if d and d.hub:
                if action == "start":
                    d.hub.trigger_start()
                elif action == "stop":
                    d.hub.trigger_stop()
                else:
                    if d.hub.master_state.get("state") == "RUNNING" or d.hub.esp.start_received:
                        d.hub.trigger_stop()
                    else:
                        d.hub.trigger_start()
            return self._send(200, "application/json", b'{"status":"ok"}')
        return self._send(404, "text/plain", b"not found")


PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WRO 2026 - Pi master</title><style>
*{box-sizing:border-box}
body{background:#0d1117;color:#c9d1d9;font:13px/1.45 system-ui,-apple-system,sans-serif;margin:0;padding:14px}
h1{font-size:15px;margin:0 0 12px;color:#e6edf3;font-weight:600}
h1 small{color:#7d8590;font-weight:400;margin-left:8px}
.grid{display:grid;grid-template-columns:minmax(0,1.7fr) minmax(0,1fr) minmax(0,1fr);gap:14px;align-items:start}
@media(max-width:1200px){.grid{grid-template-columns:minmax(0,1.5fr) minmax(0,1fr)}}
@media(max-width:760px){.grid{grid-template-columns:1fr}}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:12px;margin-bottom:14px}
.card h2{font-size:11px;text-transform:uppercase;letter-spacing:.08em;color:#7d8590;margin:0 0 10px;font-weight:600}
img{width:100%;display:block;border-radius:6px;background:#000;min-height:120px}
.row{display:flex;justify-content:space-between;align-items:baseline;padding:4px 0;border-bottom:1px solid #21262d}
.row:last-child{border-bottom:0}.row span{color:#7d8590}
.row b{font-variant-numeric:tabular-nums;color:#e6edf3;font-weight:600}
.big{font-size:24px;font-weight:600;text-align:center;padding:4px 0}
.pill{display:inline-block;padding:2px 9px;border-radius:11px;font-size:11px;font-weight:600}
.ok{background:#1a7f37;color:#fff}.bad{background:#a40e26;color:#fff}.warn{background:#9e6a03;color:#fff}.idle{background:#30363d;color:#a8b1bb}
.red{background:#a40e26;color:#fff}.green{background:#1a7f37;color:#fff}
.car{position:relative;height:210px}
.car .body{position:absolute;left:50%;top:50%;width:54px;height:84px;margin:-42px 0 0 -27px;border:2px solid #7d8590;border-radius:8px;background:#0d1117}
.car .body:after{content:"";position:absolute;left:50%;top:-8px;margin-left:-6px;border:6px solid transparent;border-bottom-color:#7d8590;border-top:0}
.car .v{position:absolute;text-align:center;font-variant-numeric:tabular-nums}
.car .v b{display:block;font-size:20px;color:#e6edf3}.car .v span{font-size:10px;color:#7d8590;text-transform:uppercase;letter-spacing:.06em}
.bar{position:relative;height:22px;background:#0d1117;border:1px solid #30363d;border-radius:5px;overflow:hidden;margin:6px 0}
.bar i{position:absolute;top:0;bottom:0;background:#2f81f7;opacity:.85}.bar u{position:absolute;top:0;bottom:0;left:50%;width:1px;background:#7d8590}
canvas{width:100%;height:140px;display:block}
.leg{display:flex;gap:12px;font-size:11px;color:#7d8590;margin-top:4px}.leg i{display:inline-block;width:10px;height:3px;margin-right:4px;vertical-align:middle}
.lh{display:flex;justify-content:space-between;align-items:center;margin:0 0 10px}.lh h2{margin:0}
.btn{background:#21262d;color:#c9d1d9;border:1px solid #30363d;border-radius:6px;padding:3px 10px;font:600 11px system-ui,sans-serif;cursor:pointer;margin-left:6px}
.btn:hover{background:#30363d}.btn.on{background:#9e6a03;border-color:#9e6a03;color:#fff}
pre{margin:0;font:11px/1.4 ui-monospace,Menlo,monospace;white-space:pre-wrap;color:#a8b1bb;max-height:260px;overflow:auto}
.slider-group{margin-bottom:8px}
.sec{margin:14px 0 8px;padding-top:8px;border-top:1px solid #30363d;font-size:11px;font-weight:700;letter-spacing:.06em;color:#e6edf3;text-transform:uppercase}
.sec:first-child{margin-top:0;padding-top:0;border-top:0}
.sec span{display:block;font-weight:400;text-transform:none;letter-spacing:0;color:#7d8590;margin-top:2px}
.slider-group label{display:flex;justify-content:space-between;font-size:11px;color:#7d8590}
.slider-group input[type=range]{width:100%}
</style></head><body>
<h1>WRO 2026 Pi master <small id="hdr">connecting...</small></h1>
<div class="grid">
  <div>
    <div class="card"><h2>Camera</h2><img src="/stream.mjpg" alt="no camera frame"></div>
    <div class="card"><h2>Decision & Stage</h2>
      <div class="big" id="decBox" style="background:#161b22;border:1px solid #30363d;border-radius:6px;padding:10px;margin-bottom:10px">
        <div id="decText" style="font-size:18px;margin-bottom:4px">No pillar → driving straight</div>
        <div id="stageText" style="font-size:13px;color:#7d8590">Stage: Normal driving</div>
      </div>
    </div>
    <div class="card"><h2>ToF history (cm, last 15 s)</h2><canvas id="hist"></canvas>
      <div class="leg"><span><i style="background:#f0883e"></i>front</span><span><i style="background:#3fb950"></i>left</span><span><i style="background:#58a6ff"></i>right</span><span><i style="background:#bc8cff"></i>rear</span></div></div>
  </div>
  <div>
    <div class="card">
      <div class="lh"><h2>Tunable Parameters</h2><div>
        <button class="btn ok" id="btn_save" onclick="saveParamsToDisk()">Save to Disk</button>
      </div></div>
      <div class="sec">1 · PILLAR DODGE <span>camera sees a block → go round it</span></div>
      <div class="slider-group"><label>Activation dist — dodge starts here (cm): <b id="val_trig">{{PARAM_VAL_TRIG}}</b></label><input type="range" id="param_trig" min="5" max="120" value="{{PARAM_VAL_TRIG}}" oninput="onParamChange()"></div>
      <div class="slider-group"><label>Steer for RED — car goes right (90..140): <b id="val_right">{{PARAM_VAL_RIGHT}}</b></label><input type="range" id="param_right" min="90" max="140" value="{{PARAM_VAL_RIGHT}}" oninput="onParamChange()"></div>
      <div class="slider-group"><label>Steer for GREEN — mirrors red: <b id="val_left">{{PARAM_VAL_LEFT}}</b></label><input type="range" id="param_left" min="30" max="90" value="{{PARAM_VAL_LEFT}}" disabled title="Green always mirrors red about centre (90). Tune the red slider."></div>
      <div class="slider-group"><label>Leg 1 — turn out, normal (cm): <b id="val_dshort">{{PARAM_VAL_DSHORT}}</b></label><input type="range" id="param_dshort" min="5" max="60" step="1" value="{{PARAM_VAL_DSHORT}}" oninput="onParamChange()"></div>
      <div class="slider-group"><label>Leg 1 — turn out, wide arc (cm): <b id="val_dlong">{{PARAM_VAL_DLONG}}</b></label><input type="range" id="param_dlong" min="5" max="80" step="1" value="{{PARAM_VAL_DLONG}}" oninput="onParamChange()"></div>
      <div class="slider-group"><label>Leg 2 — hold the dodge past the block (cm): <b id="val_dpast">{{PARAM_VAL_DPAST}}</b></label><input type="range" id="param_dpast" min="5" max="80" step="1" value="{{PARAM_VAL_DPAST}}" oninput="onParamChange()"></div>
      <div class="slider-group"><label>Hold full lock for first (% of legs 1 &amp; 3): <b id="val_hold">{{PARAM_VAL_HOLD}}</b></label><input type="range" id="param_hold" min="0" max="80" step="5" value="{{PARAM_VAL_HOLD}}" oninput="onParamChange()"></div>
      <div class="slider-group"><label>Aim off-centre sensitivity: <b id="val_sens">{{PARAM_VAL_SENS}}</b></label><input type="range" id="param_sens" min="0.0" max="1.5" step="0.05" value="{{PARAM_VAL_SENS}}" oninput="onParamChange()"></div>

      <div class="sec">2 · CORNER TURNS <span>front ToF sees a wall → 90° corner. Steering is proportional: the cap below is used while the heading error is big, easing off as the car comes round</span></div>
      <div class="slider-group"><label>Corner steer (deg off centre): <b id="val_csteer">{{PARAM_VAL_CSTEER}}</b></label><input type="range" id="param_csteer" min="10" max="50" step="1" value="{{PARAM_VAL_CSTEER}}" oninput="onParamChange()"></div>
      <div class="slider-group"><label>Reverse AFTER each corner (cm): <b id="val_ptrev">{{PARAM_VAL_PTREV}}</b></label><input type="range" id="param_ptrev" min="0" max="150" step="1" value="{{PARAM_VAL_PTREV}}" oninput="onParamChange()"></div>
      <div class="slider-group"><label>Turn when the wall is this close (cm): <b id="val_fturn">{{PARAM_VAL_FTURN}}</b></label><input type="range" id="param_fturn" min="15" max="80" step="1" value="{{PARAM_VAL_FTURN}}" oninput="onParamChange()"></div>

      <div class="sec">3 · BLOCK vs CORNER <span>which one wins when both look close</span></div>
      <div class="slider-group"><label>A block this close counts as "in front" (cm): <b id="val_bnear">{{PARAM_VAL_BNEAR}}</b></label><input type="range" id="param_bnear" min="20" max="120" step="1" value="{{PARAM_VAL_BNEAR}}" oninput="onParamChange()"></div>
      <div class="slider-group"><label>...and blocks corners for (s), 0 = off: <b id="val_bhold">{{PARAM_VAL_BHOLD}}</b></label><input type="range" id="param_bhold" min="0" max="6" step="0.1" value="{{PARAM_VAL_BHOLD}}" oninput="onParamChange()"></div>

      <div class="sec">4 · DRIVING, RE-CENTRING &amp; CAMERA <span>every straight, and the return to the lane after a dodge</span></div>
      <div class="slider-group"><label>Lane centre / re-centre — wall distance to hold (cm): <b id="val_rectgt">{{PARAM_VAL_RECTGT}}</b></label><input type="range" id="param_rectgt" min="30" max="60" step="1" value="{{PARAM_VAL_RECTGT}}" oninput="onParamChange()"></div>
      <div class="slider-group"><label>Lane centre / re-centre sensitivity: <b id="val_reck">{{PARAM_VAL_RECK}}</b></label><input type="range" id="param_reck" min="0.1" max="5.0" step="0.1" value="{{PARAM_VAL_RECK}}" oninput="onParamChange()"></div>
      <div class="slider-group"><label>Camera focal length (px) — distance scale: <b id="val_focal">{{PARAM_VAL_FOCAL}}</b></label><input type="range" id="param_focal" min="200" max="1500" step="5" value="{{PARAM_VAL_FOCAL}}" oninput="onParamChange()"></div>
      <div style="font-size:10px;color:#7d8590;margin-top:4px" id="save_status">Changes apply real-time; click 'Save to Disk' to persist.</div>
    </div>
    <div class="card">
      <div class="lh"><h2>Run Control</h2><div>
        <button class="btn ok" id="btn_run" onclick="toggleRun()">Start Run</button>
      </div></div>
      <div class="big" id="state">--</div>
      <div class="row"><span>turns</span><b id="turns">--</b></div>
      <div class="row"><span>lap</span><b id="lap">--</b></div>
      <div class="row"><span>direction</span><b id="dir">--</b></div>
      <div class="row"><span>run time</span><b id="rt">--</b></div>
    </div>
    <div class="card"><h2>ToF (cm)</h2>
      <div class="car"><div class="body"></div>
        <div class="v" style="left:0;right:0;top:0"><span>front</span><b id="tf">--</b></div>
        <div class="v" style="left:0;top:80px;width:30%"><span>left</span><b id="tl">--</b></div>
        <div class="v" style="right:0;top:80px;width:30%"><span>right</span><b id="tr">--</b></div>
        <div class="v" style="left:0;right:0;bottom:0"><span>rear</span><b id="tb">--</b></div>
      </div>
    </div>
  </div>
  <div>
    <div class="card"><h2>System Health & Power</h2>
      <div class="row"><span>SoC Temperature</span><b id="soc_temp">--</b></div>
      <div class="row"><span>5V Rail Supply</span><b id="ext5v">--</b></div>
      <div class="row"><span>Brownout / Throttle</span><b id="throttled">--</b></div>
    </div>
    <div class="card"><h2>Heading (BNO055)</h2>
      <div class="row"><span>yaw</span><b id="yaw">--</b></div>
      <div class="row"><span>target</span><b id="tgt">--</b></div>
      <div class="row"><span>error</span><b id="herr">--</b></div>
      <div class="row"><span>ToF centring nudge</span><b id="nudge">--</b></div>
      <div class="row"><span>calibration sys gyro acc mag</span><b id="cal">--</b></div>
    </div>
    <div class="card"><h2>Pillars (YOLO)</h2>
      <div class="row"><span>model</span><b id="vis">--</b></div>
      <table style="width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums;margin-top:6px">
        <thead><tr style="color:#7d8590;font-size:10px;text-transform:uppercase;text-align:left"><th>pillar</th><th>conf</th><th>x</th><th>~dist</th></tr></thead>
        <tbody id="blocks"></tbody></table>
    </div>
    <div class="card"><h2>Drive</h2>
      <div class="row"><span>Pi command speed / steer</span><b id="cmd">--</b></div>
      <div class="row"><span>ESP applied speed / steer</span><b id="app">--</b></div>
      <div class="bar"><i id="sbar"></i><u></u></div>
      <div style="display:flex;justify-content:space-between;color:#7d8590;font-size:10px"><span>left 30</span><span>90</span><span>140 right</span></div>
    </div>
    <div class="card"><h2>ESP32 link</h2>
      <div class="row"><span>main.py</span><b id="master">--</b></div>
      <div class="row"><span>port</span><b id="port">--</b></div>
      <div class="row"><span>DATA rate</span><b id="rate">--</b></div>
      <div class="row"><span>telemetry age</span><b id="tage">--</b></div>
      <div class="row"><span>lines / malformed</span><b id="lines">--</b></div>
      <div class="row"><span>START</span><b id="start">--</b></div>
      <div class="row"><span>camera</span><b id="cam">--</b></div>
    </div>
    <div class="card"><div class="lh"><h2>Log</h2><div>
      <button class="btn" id="lpause" onclick="logPause()">Pause</button><button class="btn" onclick="logRefresh()">Refresh</button><button class="btn" onclick="logClear()">Clear</button>
    </div></div><pre id="log"></pre></div>
  </div>
</div>
<script>
const $=id=>document.getElementById(id);
const pill=(t,c)=>`<span class="pill ${c}">${t}</span>`;
const cm=v=>v==null?'--':(v>=999?'far':v);
const deg=v=>v==null?'--':v.toFixed(1)+'\\u00b0';
const H=[],COL={front:'#f0883e',left:'#3fb950',right:'#58a6ff',rear:'#bc8cff'};
function draw(){
  const c=$('hist'),r=devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;
  c.width=w*r;c.height=h*r;const g=c.getContext('2d');g.scale(r,r);
  const now=Date.now(),X=t=>w-(now-t)/15000*w,Y=v=>h-4-Math.min(v,200)/200*(h-8);
  g.strokeStyle='#21262d';g.fillStyle='#7d8590';g.font='10px system-ui';
  for(const v of [50,100,150,200]){g.beginPath();g.moveTo(0,Y(v));g.lineTo(w,Y(v));g.stroke();g.fillText(v,2,Y(v)-2);}
  for(const k in COL){g.strokeStyle=COL[k];g.lineWidth=1.5;g.beginPath();let on=false;
    for(const p of H){const v=p.tof[k];if(v==null||v>=999){on=false;continue;}
      on?g.lineTo(X(p.t),Y(v)):g.moveTo(X(p.t),Y(v));on=true;}g.stroke();}
}
let lastLog=[],logPaused=false,logAfter=null;
function showLog(){let L=lastLog;if(logAfter!==null){const i=L.lastIndexOf(logAfter);L=i>=0?L.slice(i+1):L;}
  $('log').textContent=L.slice().reverse().join('\\n')||'(empty)';}
function logPause(){logPaused=!logPaused;$('lpause').textContent=logPaused?'Resume':'Pause';$('lpause').classList.toggle('on',logPaused);if(!logPaused)showLog();}
function logRefresh(){logAfter=null;tick().then(showLog);}
function logClear(){logAfter=lastLog.length?lastLog[lastLog.length-1]:null;showLog();}

// Every slider, in one place: param name -> the id suffix of its
// <input id="param_X"> and its <b id="val_X"> readout. Everything below walks
// this table, so adding or removing a slider cannot leave a stale reference
// that throws and silently kills the whole handler - which is exactly what
// happened when three sliders were removed.
const PARAM_SLIDERS = {
  pillar_trigger_dist_cm:'trig', pillar_steer_right:'right', turn_hold_pct:'hold',
  block_offset_sens:'sens', recenter_target_cm:'rectgt', recenter_k:'reck',
  pillar_focal_px:'focal', dodge_past_cm:'dpast', dodge_short_cm:'dshort',
  dodge_long_cm:'dlong', 
  corner_steer_deg:'csteer', front_turn_cm:'fturn', post_turn_reverse_cm:'ptrev',
  block_near_cm:'bnear',
  block_no_corner_s:'bhold'
};

function showParamValues(){
  for(const k in PARAM_SLIDERS){
    const inp = $('param_' + PARAM_SLIDERS[k]), out = $('val_' + PARAM_SLIDERS[k]);
    if(inp && out) out.textContent = inp.value;
  }
  mirrorGreen();
}

function getParamsFromUI(){
  const p = {};
  for(const k in PARAM_SLIDERS){
    const inp = $('param_' + PARAM_SLIDERS[k]);
    if(inp) p[k] = parseFloat(inp.value);
  }
  p.pillar_steer_left = 180 - parseInt($('param_right').value);   // server enforces it too
  return p;
}

async function loadParamsUI(){
  try{
    const p=await(await fetch('/api/params?t=' + Date.now(), {cache:'no-store'})).json();
    for(const k in PARAM_SLIDERS){
      const inp = $('param_' + PARAM_SLIDERS[k]);
      if(inp && p[k] != null) inp.value = p[k];
    }
    if(p.pillar_steer_left != null && $('param_left')) $('param_left').value = p.pillar_steer_left;
    showParamValues();
  }catch(e){}
}

let syncTimeout = null;
function mirrorGreen(){
  const g = 180 - parseInt($('param_right').value);
  $('param_left').value = g; $('val_left').textContent = g;
}
function onParamChange(){
  mirrorGreen();
  showParamValues();
  const p = getParamsFromUI();
  $('save_status').textContent = "Updating real-time (unsaved to disk)...";
  $('save_status').style.color = "#d29922";
  
  clearTimeout(syncTimeout);
  syncTimeout = setTimeout(async () => {
    try {
      await fetch('/api/params?save=0', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(p)
      });
      $('save_status').textContent = "Applied real-time (not yet saved to disk).";
    } catch(e) {
      $('save_status').textContent = "Failed to sync real-time.";
      $('save_status').style.color = "#cf222e";
    }
  }, 100);
}

async function saveParamsToDisk(){
  const p = getParamsFromUI();
  const btn = $('btn_save');
  btn.textContent = "Saving...";
  try {
    const res = await fetch('/api/params?save=1', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(p)
    });
    if (res.ok) {
      $('save_status').textContent = "✓ Saved to disk & active!";
      $('save_status').style.color = "#2da44e";
      btn.textContent = "Saved!";
      setTimeout(() => { btn.textContent = "Save to Disk"; }, 1500);
    } else {
      throw new Error("Save returned error");
    }
  } catch(e) {
    $('save_status').textContent = "Failed to save to disk.";
    $('save_status').style.color = "#cf222e";
    btn.textContent = "Error";
  }
}
loadParamsUI();
window.addEventListener('pageshow', loadParamsUI);

async function toggleRun(){
  const btn = $('btn_run');
  const isStop = btn.textContent.includes('Stop');
  btn.textContent = isStop ? 'Stopping...' : 'Starting...';
  try {
    await fetch('/api/run?action=' + (isStop ? 'stop' : 'start'), { method: 'POST' });
  } catch(e) {}
}

async function tick(){
  try{
    const s=await (await fetch('/api/state',{cache:'no-store'})).json();
    const age=s.age??99;
    $('hdr').textContent=age<2?'live':'no update for '+age.toFixed(0)+' s';
    $('master').innerHTML=s.master_running?pill('running','ok'):pill('not running','idle');
    const cls={RUNNING:'ok',TURNING:'warn',REVERSING:'bad',FINISHING:'warn',DONE:'idle',WAITING:'idle'};
    $('state').innerHTML=pill(s.state||'--',cls[s.state]||'idle');
    
    // Run button text and style
    const isRunning = s.state === 'RUNNING' || s.start_received;
    const btn = $('btn_run');
    if(btn){
      btn.textContent = isRunning ? 'Stop Run' : 'Start Run';
      btn.className = isRunning ? 'btn bad' : 'btn ok';
    }

    // System Health & Power
    if(s.temp_c != null){
      const tc = s.temp_c;
      const tCls = tc >= 82 ? 'bad' : (tc >= 72 ? 'warn' : 'ok');
      $('soc_temp').innerHTML = `${tc.toFixed(1)} °C ${pill(tc >= 82 ? 'HIGH' : (tc >= 72 ? 'WARM' : 'NORMAL'), tCls)}`;
    } else { $('soc_temp').textContent = '--'; }

    if(s.ext5v_v != null){
      const v = s.ext5v_v;
      const vCls = v < 4.65 ? 'bad' : (v < 4.80 ? 'warn' : 'ok');
      const vTag = v < 4.65 ? 'BROWNOUT RISK' : (v < 4.80 ? 'LOW' : 'GOOD');
      $('ext5v').innerHTML = `${v.toFixed(2)} V ${pill(vTag, vCls)}`;
    } else { $('ext5v').textContent = '--'; }

    if(s.health_msg){
      const bCls = s.brownout ? 'bad' : (s.health_msg === 'OK' ? 'ok' : 'warn');
      $('throttled').innerHTML = pill(s.health_msg, bCls);
    } else { $('throttled').textContent = '--'; }

    // Decision card updates
    $('decText').textContent=s.decision||'No pillar → driving straight / centring';
    $('stageText').textContent='Stage: '+(s.stage||'Normal driving');
    const db=$('decBox');
    if(s.pillar_color==='Red'){db.style.background='#4c111a';db.style.borderColor='#a40e26';}
    else if(s.pillar_color==='Green'){db.style.background='#0d381e';db.style.borderColor='#1a7f37';}
    else{db.style.background='#161b22';db.style.borderColor='#30363d';}

    $('turns').textContent=(s.turns??'--')+' / '+(s.total_turns??12);
    $('lap').textContent=s.turns!=null?Math.min(3,Math.floor(s.turns/4)+1):'--';
    $('dir').textContent=s.direction||'not locked yet';
    $('rt').textContent=s.runtime!=null?s.runtime.toFixed(1)+' s':'--';
    const t=s.tof||{};$('tf').textContent=cm(t.front);$('tl').textContent=cm(t.left);$('tr').textContent=cm(t.right);$('tb').textContent=cm(t.rear);
    $('yaw').textContent=deg(s.yaw);$('tgt').textContent=deg(s.target);
    $('herr').textContent=s.herr!=null?(s.herr>0?'+':'')+s.herr.toFixed(1)+'\\u00b0':'--';
    $('nudge').textContent=s.nudge!=null?(s.nudge>0?'+':'')+s.nudge.toFixed(1)+'\\u00b0':'off';
    $('cal').textContent=s.cal?s.cal.join(' '):'--';
    $('cmd').textContent=(s.speed??'--')+' / '+(s.steer??'--');
    $('app').textContent=(s.esp_speed??'--')+' / '+(s.esp_steer??'--');
    const st=s.steer??90,x=(st-30)/110*100;$('sbar').style.left=Math.min(50,x)+'%';$('sbar').style.width=Math.abs(x-50)+'%';
    $('port').innerHTML=s.esp_connected?pill(s.port||'open','ok'):pill('no ESP32','bad');
    $('rate').textContent=s.rate_hz!=null?s.rate_hz.toFixed(0)+' Hz':'--';
    $('tage').textContent=s.telemetry_age!=null?(s.telemetry_age*1000).toFixed(0)+' ms':'no data';
    $('lines').textContent=(s.rx_lines??'--')+' / '+(s.bad_lines??'--');
    $('start').innerHTML=s.start_received?pill('pressed','ok'):pill('not pressed','idle');
    $('cam').innerHTML=s.camera_fps!=null?pill(s.camera_fps.toFixed(0)+' fps','ok'):pill(s.camera_msg||'off','bad');
    $('vis').innerHTML=s.vision_fps!=null?pill(s.vision_fps.toFixed(0)+' fps, '+(s.infer_ms||0).toFixed(0)+' ms','ok'):pill(s.vision_msg||'off','bad');
    $('blocks').innerHTML=(s.blocks||[]).map(b=>`<tr><td style="color:${b.class=='Red'?'#ff7b72':'#3fb950'};font-weight:600">${b.class}</td><td>${b.confidence.toFixed(2)}</td><td>${b.cx}</td><td>${b.dist_cm.toFixed(0)} cm</td></tr>`).join('')||'<tr><td colspan=4 style="color:#7d8590">none seen</td></tr>';
    lastLog=s.log||[];if(!logPaused)showLog();
    if(s.tof){H.push({t:Date.now(),tof:s.tof});while(H.length&&Date.now()-H[0].t>15000)H.shift();}
    draw();
  }catch(e){$('hdr').textContent='dashboard not reachable';}
}
setInterval(tick,150);tick();
</script></body></html>"""


_CMD = re.compile(r"^(-?\d+),(\d+)$")


class Hub:
    """Owns the ESP32 port; relays lines/commands to and from main.py."""

    def __init__(self):
        self.esp = SerialLink(on_line=self._forward)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", config.HUB_PORT))
        self._master = None
        self._master_seen = 0.0
        self.master_state = {}
        threading.Thread(target=self._rx, name="hub-rx", daemon=True).start()
        threading.Thread(target=self._status, name="hub-status", daemon=True).start()
        log.info(f"relay for main.py on udp 127.0.0.1:{config.HUB_PORT}")

    @property
    def master_running(self):
        return time.monotonic() - self._master_seen < 1.0

    def _tx(self, text):
        if self._master is not None and self.master_running:
            try:
                self._sock.sendto(text.encode(), self._master)
            except OSError:
                pass

    def forward_vision(self, blocks):
        self._tx("VISION " + json.dumps(blocks, default=float))

    def forward_params(self, params):
        self._tx("PARAMS " + json.dumps(params, default=float))

    def trigger_start(self):
        log.info("START triggered from dashboard")
        self.esp._start.set()
        self.esp.write_line("START\n")
        self._tx("START")

    def trigger_stop(self):
        log.info("STOP triggered from dashboard")
        self.esp._start.clear()
        self.esp.write_line("STOP\n")
        self._tx("STOP")

    def _forward(self, line):
        self._tx(line)

    def _status(self):
        while True:
            # The START state travels with this line so a main.py that starts
            # (or restarts) AFTER the button was pressed still learns about it.
            # The ESP32 only repeats START until its first drive command, so a
            # late main.py would otherwise sit in WAITING with the run already
            # live - which is exactly what "the button does nothing" looked like.
            self._tx(f"HUB,{int(self.esp.connected)},{int(self.esp.start_received)},"
                     f"{self.esp.port_name or ''}")
            time.sleep(0.3)

    def _rx(self):
        while True:
            try:
                data, addr = self._sock.recvfrom(8192)
            except OSError:
                time.sleep(0.2)
                continue
            if not self.master_running:
                log.info("main.py connected")
            self._master, self._master_seen = addr, time.monotonic()
            text = data.decode("utf-8", errors="ignore").strip()
            m = _CMD.match(text)
            if m:
                self.esp.write_line(command_line(int(m.group(1)), int(m.group(2))))
            elif text in ("STOP", "RESET"):
                log.info(f"Hub received {text} from master")
                self.esp._start.clear()
                self.esp.write_line("STOP\n")
            elif text == "START":
                log.info("Hub received START from master")
                self.esp._start.set()
                self.esp.write_line("START\n")
            elif text.startswith("STATE "):
                try:
                    self.master_state = json.loads(text[6:])
                except ValueError:
                    pass


def oled_state(hub, started_at):
    """What the little panel shows: can we run, how many turns, how long.

    Works with either firmware. The obstacle round's master publishes its turn
    count; the standalone open round prints its own ("T:3 ...") and SerialLink
    picks it out. Neither has to know the panel exists.
    """
    esp = hub.esp
    ms = hub.master_state if hub.master_running else {}
    d, _ = esp.latest()

    if not esp.connected:
        return "NO ESP32", "Turns   -", 0.0, "check the USB lead"
    if d is None or d.get("age", 99) > 1.0:
        return "NO DATA", "Turns   -", 0.0, "ESP32 is silent"

    turns = ms.get("turns") if hub.master_running and ms.get("turns") is not None else esp.esp_turns
    turns_text = f"Turns   {turns}/{config.TOTAL_TURNS}" if turns is not None else "Turns   0/12"

    if not esp.start_received:
        return "READY", turns_text, 0.0, "press START"

    elapsed = time.monotonic() - started_at[0] if started_at[0] else 0.0
    state = (ms.get("state") or esp.esp_state or "RUNNING").upper()
    if "DONE" in state or "COMPLETE" in state:
        return "DONE", turns_text, elapsed, state.title()[:21]
    note = (esp.esp_state or ms.get("stage") or "")[:21]
    return "RUNNING", turns_text, elapsed, note


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    logging.getLogger().addHandler(LOG_TAIL)
    hub = Hub()

    # The camera is opened in the BACKGROUND. Nothing waits for it: the web
    # page and, more importantly, the ESP32 relay come up immediately, and a
    # camera plugged in later is picked up without a restart. A dead or absent
    # camera can no longer delay or block the round.
    state = {"camera": None, "cam_msg": "opening...", "vision": None, "vision_msg": "waiting for camera"}

    def open_camera_later():
        while True:
            try:
                from camera import open_camera
                state["camera"] = open_camera()
                state["cam_msg"] = "ok"
            except Exception as exc:
                state["cam_msg"] = str(exc)[:60]
                log.warning(f"camera not available yet ({str(exc)[:60]}) - retrying in 5 s")
                time.sleep(5.0)
                continue
            if config.VISION_ENABLE:
                try:
                    from vision import BlockDetector, VisionWorker
                    v = VisionWorker(state["camera"], BlockDetector())
                    v.start()
                    state["vision"], state["vision_msg"] = v, "running"
                except Exception as exc:
                    state["vision_msg"] = str(exc)[:60]
                    log.error(f"YOLO unavailable: {exc}")
            if dash is not None:
                dash.camera = state["camera"]
                dash.vision = state["vision"]
            log.info("camera ready")
            return

    dash = None
    ok, why = Dashboard.allowed()
    if ok:
        dash = Dashboard(None, None)      # camera arrives later, from the thread
        dash.hub = hub
        if not dash.start():
            dash = None
    else:
        log.info(f"web dashboard off: {why} - relay, camera and YOLO still running")

    threading.Thread(target=open_camera_later, name="camera-open", daemon=True).start()

    # The OLED on the Pi's own I2C bus. Fed from the same state the web page
    # uses, in its own thread, and entirely optional - see oled.py.
    started_at = [None]
    if config.OLED_ENABLE:
        try:
            from oled import OledStatus
            OledStatus(lambda: oled_state(hub, started_at)).start()
        except Exception as exc:
            log.warning(f"OLED not started: {exc}")

    master_was = False
    try:
        while True:
            time.sleep(0.05)
            if hub.esp.start_received and started_at[0] is None:
                started_at[0] = time.monotonic()
            elif not hub.esp.start_received:
                started_at[0] = None

            running = hub.master_running
            if master_was and not running:
                log.warning("main.py stopped (ESP32 watchdog stops the car within 0.5 s)")
            master_was = running

            blocks = (state["vision"].latest() or (None, 0, 0, []))[3] if state["vision"] else []
            if running and blocks:
                hub.forward_vision(blocks)

            if dash is None:
                continue
            d, _ = hub.esp.latest()
            ms = hub.master_state if running else {}
            dash.update(
                state=ms.get("state", "WAITING") if running else "MAIN.PY OFF",
                stage=ms.get("stage", "Normal driving"),
                decision=ms.get("decision", "No pillar → driving straight / centring"),
                pillar_color=ms.get("pillar_color"),
                pillar_conf=ms.get("pillar_conf"),
                pillar_dist=ms.get("pillar_dist"),
                master_running=running,
                turns=ms.get("turns", 0), total_turns=config.TOTAL_TURNS,
                direction=ms.get("direction"), runtime=ms.get("runtime"),
                target=ms.get("target"), herr=ms.get("herr"), nudge=ms.get("nudge"),
                speed=ms.get("speed"), steer=ms.get("steer"),
                tof={k: d[k] for k in ("front", "left", "right", "rear")} if d else None,
                yaw=d["yaw"] if d else None, cal=d["cal"] if d else None,
                esp_speed=d["esp_speed"] if d else None, esp_steer=d["esp_steer"] if d else None,
                esp_connected=hub.esp.connected, port=hub.esp.port_name, rate_hz=hub.esp.rate_hz,
                telemetry_age=d["age"] if d else None, rx_lines=hub.esp.rx_lines,
                bad_lines=hub.esp.bad_lines, start_received=hub.esp.start_received,
                camera_msg=state["cam_msg"], vision_msg=state["vision_msg"],
                vision_fps=state["vision"].fps if state["vision"] else None,
                infer_ms=state["vision"].detector.last_infer_ms if state["vision"] else None,
                blocks=blocks)
    except KeyboardInterrupt:
        pass
    finally:
        if state["camera"]:
            state["camera"].stop()
        hub.esp.close()


if __name__ == "__main__":
    main()

