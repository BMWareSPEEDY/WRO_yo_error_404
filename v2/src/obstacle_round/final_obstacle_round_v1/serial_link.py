"""
serial_link.py - talking to the ESP32-C3 (see esp/open_slave/open_slave.ino).

    ESP32 -> Pi  DATA,<F>,<L>,<R>,<B>,<yaw>,<sys>,<gyro>,<accel>,<mag>,<spd>,<str>   50 Hz, cm
    ESP32 -> Pi  START                    START button pressed
    ESP32 -> Pi  INFO,... / ERR,...       status text
    Pi -> ESP32  <speed>,<steering>       speed -170..170, steering 30..140

Only ONE process can read a serial port, so:

    SerialLink  owns the USB port. Used by dashboard.py, which runs from boot
                (dashboard.service) and is always there.
    HubLink     used by main.py. Talks to dashboard.py over UDP on localhost:
                it gets every ESP32 line forwarded, and its commands are
                written to the ESP32 for it.

Both have the same interface: latest(), wait_new(), send(), start_received.

Stand-alone check (prints live data, never moves the car; needs
dashboard.service stopped, because that owns the port):
    python3 serial_link.py
"""
import glob
import json
import logging
import socket
import threading
import time

import config

log = logging.getLogger("link")


def parse_data(line):
    """DATA line -> dict (cm, degrees), or None if malformed.

    12 fields is the old firmware, 13 adds the free-running wheel encoder
    count. Both are accepted so a Pi running new code against an ESP32 that
    has not been reflashed yet still drives - it just falls back to timers,
    with "ticks" reported as None.
    """
    p = line.split(",")
    if len(p) not in (12, 13):
        return None
    try:
        return {
            "front": int(p[1]), "left": int(p[2]), "right": int(p[3]), "rear": int(p[4]),
            "yaw": float(p[5]),
            "cal": [int(x) for x in p[6:10]],
            "esp_speed": int(p[10]), "esp_steer": int(p[11]),
            "ticks": int(p[12]) if len(p) == 13 else None,
            "t": time.monotonic(),
        }
    except ValueError:
        return None


def command_line(speed, steering):
    """Clamp to the safe limits and format a Pi -> ESP32 command."""
    speed = int(max(-config.MAX_PWM, min(config.MAX_PWM, round(speed))))
    steering = int(max(config.SERVO_MIN, min(config.SERVO_MAX, round(steering))))
    return f"{speed},{steering}\n"


class _Link:
    """Shared line handling: newest DATA, START events, rate counter."""

    def __init__(self, log_status):
        self._cond = threading.Condition()
        self._latest = None
        self._seq = 0
        self._start = threading.Event()
        self._log_status = log_status
        self._n, self._t0 = 0, time.monotonic()
        self.rx_lines = 0
        self.bad_lines = 0
        self.rate_hz = 0.0
        self.esp_turns = None              # "T:3" from the standalone sketch's status line
        self.esp_state = ""                # "TURNING CW", "STRAIGHT", ...

    @property
    def start_received(self):
        return self._start.is_set()

    def clear_start(self):
        self._start.clear()

    def trigger_start(self):
        self._start.set()

    def latest(self):
        """(data dict or None, seq). data["age"] is seconds since it arrived."""
        with self._cond:
            d = None if self._latest is None else dict(self._latest)
            seq = self._seq
        if d is not None:
            d["age"] = time.monotonic() - d["t"]
        return d, seq

    def wait_new(self, last_seq, timeout=0.1):
        """Block until a DATA line newer than last_seq arrives (or timeout)."""
        with self._cond:
            self._cond.wait_for(lambda: self._seq > last_seq, timeout=timeout)
        return self.latest()

    def _handle(self, line):
        if not line:
            return
        self.rx_lines += 1
        now = time.monotonic()
        if now - self._t0 >= 1.0:
            self.rate_hz, self._n, self._t0 = self._n / (now - self._t0), 0, now
        if line.startswith("DATA,"):
            d = parse_data(line)
            if d is None:
                self.bad_lines += 1
                return
            self._n += 1
            with self._cond:
                self._latest = d
                self._seq += 1
                self._cond.notify_all()
        elif line == "START":
            if not self._start.is_set():
                log.info("START event received")
            self._start.set()
        elif line == "STOP":
            if self._start.is_set():
                log.info("STOP event received")
            self._start.clear()
        elif line.startswith("T:"):
            # Standalone open-round status line:
            #   T:3 | F:52 L:31 R:48 | H_Err:2.1 | Ang:118 | State: TURNING CW
            try:
                self.esp_turns = int(line.split("|", 1)[0].split(":", 1)[1])
            except (ValueError, IndexError):
                pass
            if "State:" in line:
                self.esp_state = line.split("State:", 1)[1].strip()
            if self._log_status:
                log.info(f"ESP32: {line}")
        elif self._log_status:
            (log.error if line.startswith("ERR") else log.info)(f"ESP32: {line}")


class SerialLink(_Link):
    """Owns the USB serial port. on_line(line) is called for every line received."""

    def __init__(self, on_line=None):
        super().__init__(log_status=True)
        import serial                  # only the port owner needs pyserial
        self._serial_mod = serial
        self._ser = None
        self._wlock = threading.Lock()
        self._running = True
        self.on_line = on_line
        self.port_name = None
        threading.Thread(target=self._reader, name="serial", daemon=True).start()

    @property
    def connected(self):
        return self._ser is not None

    def write_line(self, text):
        ser = self._ser
        if ser is None:
            return False
        try:
            with self._wlock:
                ser.write(text.encode("ascii"))
            return True
        except Exception as exc:
            log.warning(f"serial write failed: {exc}")
            self._drop()
            return False

    def send(self, speed, steering):
        return self.write_line(command_line(speed, steering))

    def close(self):
        self._running = False
        self._drop()

    def _open(self):
        port = config.SERIAL_PORT
        if not port:
            for pattern in config.SERIAL_PORT_GLOBS:
                found = sorted(glob.glob(pattern))
                if found:
                    port = found[0]
                    break
        if not port:
            return None
        s = self._serial_mod.Serial()
        s.port, s.baudrate, s.timeout = port, config.SERIAL_BAUD, 0.05
        if "ttyUSB" in port:           # USB-UART bridges: stop the open from resetting the ESP32
            s.dtr = s.rts = False
        s.open()
        s.reset_input_buffer()
        self.port_name = port
        log.info(f"ESP32 link open on {port}")
        return s

    def _drop(self):
        s, self._ser = self._ser, None
        if s is not None:
            try:
                s.close()
            except Exception:
                pass

    def _reader(self):
        warned, buf = False, b""
        while self._running:
            if self._ser is None:
                try:
                    s = self._open()
                except Exception as exc:
                    s = None
                    if not warned:
                        log.error(f"cannot open ESP32 port: {exc}")
                if s is None:
                    if not warned:
                        log.error("no ESP32 on the Pi USB - plug the ESP32 cable into the Pi (retrying every 2 s)")
                        warned = True
                    time.sleep(config.SERIAL_RECONNECT_S)
                    continue
                warned, buf = False, b""
                self._ser = s
            try:
                chunk = self._ser.read(self._ser.in_waiting or 1)
            except Exception as exc:
                log.warning(f"serial read failed ({exc}) - reconnecting")
                self._drop()
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", errors="ignore").strip()
                self._handle(line)
                if line and self.on_line:
                    self.on_line(line)
            if len(buf) > 4096:
                buf = b""


class HubLink(_Link):
    """main.py's side: everything goes through dashboard.py over localhost UDP.

    To the hub:   HELLO | <speed>,<steering> | STATE <json>
    From the hub: every ESP32 line, plus  HUB,<esp_connected 0/1>,<started 0/1>,<port>
    """

    def __init__(self):
        super().__init__(log_status=False)   # the hub already logs ESP32 status lines
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.settimeout(0.2)
        self._hub = ("127.0.0.1", config.HUB_PORT)
        self._hub_seen = 0.0
        self._esp_connected = False
        self.port_name = None
        self._running = True
        self._latest_vision = []
        self._vision_t = 0.0            # when that vision frame arrived
        self._vision_seq = 0            # counts DISTINCT frames, not control ticks
        self._latest_params = config.load_params()
        threading.Thread(target=self._reader, name="hub-rx", daemon=True).start()
        threading.Thread(target=self._hello, name="hub-hello", daemon=True).start()

    def latest_vision(self):
        """Blocks from the newest vision frame, or [] if it has gone stale.

        Without the age check the last frame of a run persists indefinitely, so
        the next run picks it up in its very first step - detecting a pillar
        that may have been moved or removed while the car was stopped.
        """
        with self._cond:
            if time.monotonic() - self._vision_t > config.VISION_MAX_AGE_S:
                return []
            return list(self._latest_vision)

    @property
    def vision_seq(self):
        """How many camera frames have arrived. Lets the controller tell a NEW
        look at the world from the same one re-read on the next control tick."""
        return self._vision_seq

    def clear_vision(self):
        """Forget any cached detection. Called when a run starts."""
        with self._cond:
            self._latest_vision = []
            self._vision_t = 0.0

    def latest_params(self):
        with self._cond:
            return dict(self._latest_params)

    @property
    def hub_alive(self):
        return time.monotonic() - self._hub_seen < 1.0

    @property
    def connected(self):
        return self.hub_alive and self._esp_connected

    def _tx(self, text):
        try:
            self._sock.sendto(text.encode(), self._hub)
            return True
        except OSError:
            return False

    def send(self, speed, steering):
        return self._tx(command_line(speed, steering))

    def send_line(self, text):
        return self._tx(text.strip())

    def publish(self, state):
        """Master state for the dashboard page."""
        self._tx("STATE " + json.dumps(state, default=float))

    def close(self):
        self.send(0, config.SERVO_CENTER)
        self._running = False

    def _hello(self):
        while self._running:
            self._tx("HELLO")
            time.sleep(0.2)

    def _reader(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                time.sleep(0.2)
                continue
            self._hub_seen = time.monotonic()
            for line in data.decode("utf-8", errors="ignore").splitlines():
                line = line.strip()
                if line.startswith("HUB,"):
                    p = line.split(",", 3)
                    self._esp_connected = p[1] == "1"
                    # The hub owns the START state: mirror it, so a main.py that
                    # started after the button was pressed picks the run up, and
                    # a STOP always lands even if the ESP32's own line was missed.
                    if len(p) > 3:
                        started = p[2] == "1"
                        if started and not self._start.is_set():
                            log.info("START (latched by the relay)")
                            self._start.set()
                        elif not started and self._start.is_set():
                            log.info("STOP (cleared by the relay)")
                            self._start.clear()
                        self.port_name = p[3] or None
                    else:
                        self.port_name = p[2] if len(p) > 2 and p[2] else None
                elif line.startswith("VISION "):
                    try:
                        v = json.loads(line[7:])
                        with self._cond:
                            self._latest_vision = v
                            self._vision_t = time.monotonic()
                            self._vision_seq += 1
                    except ValueError:
                        pass
                elif line.startswith("PARAMS "):
                    try:
                        p = json.loads(line[7:])
                        with self._cond:
                            self._latest_params.update(p)
                    except ValueError:
                        pass
                else:
                    self._handle(line)



if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    link = SerialLink()
    print("Listening (Ctrl+C to quit). The car never moves in this test.")
    try:
        seq = 0
        while True:
            d, seq = link.wait_new(seq, timeout=1.0)
            if d is None or d["age"] > 1.0:
                print("no telemetry" + ("" if link.connected else " (port not open)"))
                continue
            print(f"F {d['front']:4d}  L {d['left']:4d}  R {d['right']:4d}  B {d['rear']:4d} cm  "
                  f"yaw {d['yaw']:6.1f}  cal {''.join(map(str, d['cal']))}  "
                  f"{link.rate_hz:4.1f} Hz  start={link.start_received}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"lines {link.rx_lines}, malformed {link.bad_lines}")
        link.close()
