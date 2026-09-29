"""
oled.py - the little SSD1306 on the Pi's I2C bus (0x3C on i2c-1).

Three things, which is all it needs to say when you are standing at the track:

    READY            can we run?   (or RUNNING / DONE / NO ESP32 / NO DATA)
    AP ERROR404      which network it is on - its own hotspot, or a Wi-Fi it
                     joined. The name only; the password is never shown.
    172.20.10.7      the address to point a browser or ssh at

The network name matters more than a turn counter when you are standing at the
track with no Wi-Fi: it tells you at a glance whether the Pi came up on its own
hotspot or joined something, and the address saves you hunting for it.

Written straight to the display over smbus2 with a PIL-rendered frame - no
luma, no adafruit stack, nothing to install. A display that is unplugged or
faulty is logged once and then ignored: the round must never depend on it.

Stand-alone test (shows a demo frame for 10 s):
    python3 oled.py
"""
import logging
import subprocess
import threading
import time

import config

log = logging.getLogger("oled")

# SSD1306 command set, only the parts used here.
_CMD = 0x00
_DATA = 0x40
_INIT = (
    0xAE,                    # display off
    0xD5, 0x80,              # clock divide
    0xA8, None,              # multiplex ratio (height - 1, filled in below)
    0xD3, 0x00,              # display offset
    0x40,                    # start line 0
    0x8D, 0x14,              # charge pump on
    0x20, 0x00,              # horizontal addressing
    0xA1,                    # segment remap (left-right the right way round)
    0xC8,                    # scan direction (top-bottom the right way round)
    0xDA, None,              # COM pins (depends on height, filled in below)
    0x81, 0x9F,              # contrast
    0xD9, 0xF1,              # pre-charge
    0xDB, 0x40,              # VCOM detect
    0xA4,                    # resume from RAM
    0xA6,                    # not inverted
    0xAF,                    # display on
)


class Oled:
    def __init__(self, bus=config.OLED_BUS, addr=config.OLED_ADDR,
                 width=config.OLED_WIDTH, height=config.OLED_HEIGHT,
                 rotate_180=config.OLED_ROTATE_180):
        from smbus2 import SMBus, i2c_msg
        from PIL import Image, ImageDraw, ImageFont

        self._SMBus, self._i2c_msg = SMBus, i2c_msg
        self._Image, self._ImageDraw = Image, ImageDraw
        self.addr, self.width, self.height = addr, width, height
        self.rotate_180 = rotate_180
        self.bus = SMBus(bus)

        init = list(_INIT)
        init[init.index(None)] = height - 1                       # multiplex
        init[init.index(None)] = 0x12 if height > 32 else 0x02     # COM pins
        for c in init:
            self._cmd(c)

        # A real font if the system has one, the bitmap default if not.
        self.font_big = self._font(config.OLED_FONT_BIG)
        self.font_small = self._font(config.OLED_FONT_SMALL)
        log.info(f"OLED {width}x{height} at 0x{addr:02x} on i2c-{bus}")

    @staticmethod
    def _font(size):
        from PIL import ImageFont
        for path in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                     "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                continue
        return ImageFont.load_default()

    def _cmd(self, value):
        self.bus.write_byte_data(self.addr, _CMD, value)

    def show(self, status, network, address, note=""):
        """Draw the lines. Any error is the caller's to swallow."""
        img = self._Image.new("1", (self.width, self.height), 0)
        d = self._ImageDraw.Draw(img)

        d.text((0, 0), status[:16], font=self.font_big, fill=1)
        y = config.OLED_FONT_BIG + 4
        d.text((0, y), network[:21], font=self.font_small, fill=1)
        d.text((0, y + config.OLED_FONT_SMALL + 3), address[:21], font=self.font_small, fill=1)
        if note and self.height > 32:
            d.text((0, self.height - config.OLED_FONT_SMALL - 1), note[:21],
                   font=self.font_small, fill=1)

        if self.rotate_180:
            img = img.rotate(180)
        self._blit(img)

    def _blit(self, img):
        """PIL 1-bit image -> SSD1306 page buffer -> I2C."""
        pages = self.height // 8
        buf = bytearray(self.width * pages)
        px = img.load()
        for page in range(pages):
            base = page * self.width
            for x in range(self.width):
                bits = 0
                for bit in range(8):
                    if px[x, page * 8 + bit]:
                        bits |= 1 << bit
                buf[base + x] = bits

        self._cmd(0x21); self._cmd(0); self._cmd(self.width - 1)     # column range
        self._cmd(0x22); self._cmd(0); self._cmd(pages - 1)          # page range
        for i in range(0, len(buf), 128):                            # 128-byte chunks
            chunk = bytes([_DATA]) + bytes(buf[i:i + 128])
            self.bus.i2c_rdwr(self._i2c_msg.write(self.addr, chunk))

    def clear(self):
        try:
            self.show("", "", "")
        except Exception:
            pass

    def close(self):
        try:
            self.clear()
            self._cmd(0xAE)          # display off
            self.bus.close()
        except Exception:
            pass


def wifi_name_and_address(_cache={"t": 0.0, "net": "no network", "addr": ""}):
    """(network, address) for the panel - the SSID only, never the password.

    "AP <name>" when the Pi is running its own hotspot, "<name>" when it has
    joined someone else's. Asked of NetworkManager at most once every few
    seconds, because this runs on the display thread and nmcli is not free.
    """
    now = time.monotonic()
    if now - _cache["t"] < config.OLED_NET_REFRESH_S:
        return _cache["net"], _cache["addr"]
    _cache["t"] = now

    net, addr = "no network", ""
    try:
        out = subprocess.run(
            ["nmcli", "-t", "-f", "TYPE,STATE,CONNECTION,DEVICE", "device"],
            capture_output=True, text=True, timeout=2).stdout
        for line in out.splitlines():
            parts = line.split(":")
            if len(parts) >= 4 and parts[0] == "wifi" and parts[1] == "connected":
                name, dev = parts[2], parts[3]
                mode = subprocess.run(
                    ["nmcli", "-t", "-f", "802-11-wireless.mode", "connection", "show", name],
                    capture_output=True, text=True, timeout=2).stdout.strip()
                net = f"AP {name}" if mode.endswith("ap") else name
                ip = subprocess.run(["nmcli", "-t", "-f", "IP4.ADDRESS", "device", "show", dev],
                                    capture_output=True, text=True, timeout=2).stdout.strip()
                if ":" in ip:
                    addr = ip.split(":", 1)[1].split("/")[0]
                break
    except Exception:
        pass
    _cache["net"], _cache["addr"] = net, addr
    return net, addr


class OledStatus(threading.Thread):
    """Keeps the display fed from a callable returning the dashboard state.

    Never raises: if the panel is missing or dies mid-run it is logged once and
    the thread keeps going quietly, so nothing about the round depends on it.
    """

    def __init__(self, state_fn):
        super().__init__(name="oled", daemon=True)
        self.state_fn = state_fn
        self._dev = None
        self._warned = False

    def run(self):
        while True:
            try:
                if self._dev is None:
                    self._dev = Oled()
                    self._warned = False
                status, note = self.state_fn()
                net, addr = wifi_name_and_address()
                self._dev.show(status, net, addr, note)
            except Exception as exc:
                if not self._warned:
                    self._warned = True
                    log.warning(f"OLED unavailable ({str(exc)[:60]}) - carrying on without it")
                self._dev = None
                time.sleep(3.0)
                continue
            time.sleep(config.OLED_REFRESH_S)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    net, addr = wifi_name_and_address()
    print(f"network: {net}   address: {addr}")
    dev = Oled()
    t0 = time.time()
    try:
        while time.time() - t0 < 10:
            dev.show("READY", net, addr, "oled.py self test")
            time.sleep(0.5)
    finally:
        dev.close()
    print("done - the panel should have shown READY and the network")
