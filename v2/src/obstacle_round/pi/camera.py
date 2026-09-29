"""
camera.py - threaded camera capture. Two kinds of camera, one interface.

    CameraStream      Pi Camera Module on the CSI port, through picamera2
                      (libcamera). OpenCV cannot reach the imx708 on a Pi 5.
    UsbCameraStream   any USB/UVC webcam (e.g. the Logitech C920), through
                      OpenCV V4L2. MJPG by default, because a USB 2.0 bus
                      cannot carry uncompressed 1024x576 at 30 fps.
    open_camera()     picks one. config.CAMERA_BACKEND "auto" (default) tries
                      the CSI camera first and falls back to USB, so the same
                      code runs on either without editing anything.

Both give a background thread that keeps grabbing frames, so nothing else ever
waits on the camera, and the same read() -> (frame, frame_id, t) contract in
BGR, at config.CAMERA_SIZE.

Used only for the dashboard in this step - the driving never waits on it,
and the car still drives if the camera is missing.

Stand-alone test (3 s capture, prints FPS, saves logs/camera_test.jpg):
    python3 camera.py          # whatever open_camera() picks
    python3 camera.py usb      # force the USB webcam
    python3 camera.py csi      # force the CSI camera
"""
import logging
import os
import threading
import time

import config

log = logging.getLogger("camera")


class CameraError(RuntimeError):
    pass


class CameraStream:
    def __init__(self, size=config.CAMERA_SIZE, fmt=config.CAMERA_FORMAT,
                 fps=config.CAMERA_FPS, rotate_180=config.CAMERA_ROTATE_180,
                 retries=config.CAMERA_OPEN_RETRIES):
        self.size, self.fmt, self.fps = tuple(size), fmt, fps
        self.rotate_180, self.retries = rotate_180, retries
        self._picam2 = None
        self._thread = None
        self._running = False
        self._cond = threading.Condition()
        self._frame, self._frame_id, self._frame_t = None, 0, 0.0
        self.fps_measured = 0.0

    def start(self):
        """Open the camera and start the capture thread. Raises CameraError."""
        try:
            from picamera2 import Picamera2
            from libcamera import Transform
        except ImportError as exc:
            raise CameraError(f"picamera2 not available ({exc}) - run on the Pi with the system python3")

        if not Picamera2.global_camera_info():
            raise CameraError("no camera detected by libcamera - check the CSI ribbon "
                              "(contacts, latches, camera port), then REBOOT: cameras are only probed at boot")

        last_exc = None
        for attempt in range(1, self.retries + 1):
            try:
                cam = Picamera2()
                cfg = cam.create_video_configuration(
                    main={"size": self.size, "format": self.fmt},
                    controls={"FrameRate": float(self.fps)},
                    transform=Transform(hflip=1, vflip=1) if self.rotate_180 else Transform(),
                )
                cam.configure(cfg)
                cam.start()
                self._picam2 = cam
                break
            except Exception as exc:
                last_exc = exc
                log.warning(f"camera open attempt {attempt}/{self.retries} failed: {exc}")
                try:
                    cam.close()
                except Exception:
                    pass
                time.sleep(1.0)
        if self._picam2 is None:
            raise CameraError(f"camera would not start: {last_exc}")

        model = self._picam2.camera_properties.get("Model", "?")
        log.info(f"camera {model} started at {self.size[0]}x{self.size[1]} {self.fmt}")
        self._running = True
        self._thread = threading.Thread(target=self._capture, name="camera", daemon=True)
        self._thread.start()
        return self

    def read(self, newer_than=None, timeout=0.5):
        """Return (frame, frame_id, t_monotonic). With newer_than=<id>, wait up to
        timeout for a newer frame. frame is None if nothing is available."""
        with self._cond:
            if newer_than is not None:
                self._cond.wait_for(lambda: self._frame_id > newer_than or not self._running,
                                    timeout=timeout)
            return self._frame, self._frame_id, self._frame_t

    def stop(self):
        self._running = False
        with self._cond:
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._picam2 is not None:
            try:
                self._picam2.stop()
                self._picam2.close()
            except Exception:
                pass
            self._picam2 = None

    def _capture(self):
        n, t0 = 0, time.monotonic()
        fails = 0
        while self._running:
            try:
                frame = self._picam2.capture_array("main")
                fails = 0
            except Exception as exc:
                fails += 1
                log.warning(f"frame capture failed ({fails}): {exc}")
                time.sleep(0.05)
                continue
            now = time.monotonic()
            with self._cond:
                self._frame, self._frame_t = frame, now
                self._frame_id += 1
                self._cond.notify_all()
            n += 1
            if now - t0 >= 2.0:
                self.fps_measured = n / (now - t0)
                n, t0 = 0, now


class UsbCameraStream:
    """Any USB/UVC webcam, through OpenCV V4L2. Same interface as CameraStream.

    MJPG is requested by default: a USB 2.0 bus cannot carry uncompressed
    1024x576 at 30 fps, and asking for YUYV silently drops the camera to a
    few frames a second.
    """

    def __init__(self, device=config.CAMERA_USB_DEVICE, size=config.CAMERA_SIZE,
                 fps=config.CAMERA_FPS, fourcc=config.CAMERA_USB_FOURCC,
                 rotate_180=config.CAMERA_ROTATE_180):
        self.device, self.size, self.fps = device, tuple(size), fps
        self.fourcc, self.rotate_180 = fourcc, rotate_180
        self._cap = None
        self._thread = None
        self._running = False
        self._cond = threading.Condition()
        self._frame, self._frame_id, self._frame_t = None, 0, 0.0
        self.fps_measured = 0.0
        self._last_frame_t = 0.0        # for frames_fresh

    @staticmethod
    def _candidates():
        """/dev/video* nodes that actually capture, lowest first.

        A UVC webcam claims two nodes - video0 for frames and video1 for
        metadata - and a Pi 5 adds a dozen more for its own image pipeline, so
        the node has to be opened and read to know which one is the camera.
        """
        import glob
        import os
        import re
        nodes = sorted(glob.glob("/dev/video*"),
                       key=lambda p: int(re.sub(r"\D", "", p) or 0))

        # USB nodes only. A Pi 5 exposes ~17 of its own ISP nodes (pispbe-*,
        # rpi-hevc-dec) and every one of them takes TEN SECONDS to fail with
        # "select() timeout" when opened as a camera. Scanning them all turned
        # a re-open into a multi-minute stall - worse than the dropout it was
        # meant to recover from. sysfs says which node sits behind USB.
        usb = []
        for dev in nodes:
            try:
                real = os.path.realpath(
                    "/sys/class/video4linux/%s/device" % os.path.basename(dev))
            except OSError:
                continue
            if "/usb" in real:
                usb.append(dev)
        return usb

    def start(self):
        import cv2

        devices = [self.device] if self.device is not None else self._candidates()
        if not devices:
            raise CameraError("no /dev/video* nodes - is the webcam plugged in?")

        last = None
        for dev in devices:
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                last = f"{dev} would not open"
                continue
            if self.fourcc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
            cap.set(cv2.CAP_PROP_FPS, float(self.fps))
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)          # newest frame, not a queue
            ok, frame = cap.read()                        # a node that cannot read is not a camera
            if not ok or frame is None:
                cap.release()
                last = f"{dev} opened but gave no frame"
                continue
            self._cap = cap
            self.device = dev
            got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            log.info(f"USB camera on {dev} at {got[0]}x{got[1]} {self.fourcc or 'default'}")
            if got != self.size:
                log.warning(f"camera gave {got[0]}x{got[1]}, asked for "
                            f"{self.size[0]}x{self.size[1]} - using what it gave")
            break
        if self._cap is None:
            raise CameraError(f"no usable USB camera ({last})")

        self._running = True
        self._thread = threading.Thread(target=self._capture, name="camera-usb", daemon=True)
        self._thread.start()
        return self

    def read(self, newer_than=None, timeout=0.5):
        with self._cond:
            if newer_than is not None:
                self._cond.wait_for(lambda: self._frame_id > newer_than or not self._running,
                                    timeout=timeout)
            return self._frame, self._frame_id, self._frame_t

    @property
    def frames_fresh(self):
        """True while frames are genuinely still arriving.

        fps_measured keeps its last value when the device disappears, so
        anything reading only that reports a healthy camera long after the
        webcam has been unplugged: the page showed "22 fps, ok" with nothing
        on the USB bus at all.
        """
        return (time.monotonic() - self._last_frame_t) < config.CAMERA_STALE_S

    def stop(self):
        self._running = False
        with self._cond:
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def _reopen(self):
        """Re-scan and re-open after the webcam has fallen off the USB bus.

        The C920 on this car disconnects and re-enumerates under load - dmesg
        shows it coming back as a new device number each time. The old capture
        handle is dead for good at that point, and the node it lands on may not
        be the one it had before, so the device is re-scanned rather than
        reused. Without this the car ran blind for the rest of the session on a
        single USB glitch, logging the same warning thousands of times.
        """
        import cv2
        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:
            pass
        self._cap = None
        self.device = None                  # the node can move across a re-enumeration
        try:
            devices = self._candidates()
            for dev in devices:
                cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                if not cap.isOpened():
                    cap.release()
                    continue
                if self.fourcc:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
                cap.set(cv2.CAP_PROP_FPS, float(self.fps))
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                ok, frame = cap.read()
                if not ok or frame is None:
                    cap.release()
                    continue
                self._cap = cap
                self.device = dev
                return True
        except Exception as exc:
            log.warning(f"camera re-open attempt failed: {exc}")
        return False

    def _capture(self):
        import cv2

        n, t0, fails = 0, time.monotonic(), 0
        reopens, last_reopen = 0, 0.0
        while self._running:
            if self._cap is None:
                time.sleep(0.2)
                if self._reopen():
                    log.info(f"camera RECOVERED on {self.device} after {reopens} attempt(s)")
                    fails, reopens = 0, 0
                continue
            ok, frame = self._cap.read()
            if not ok or frame is None:
                fails += 1
                if fails % 20 == 1:
                    log.warning(f"USB frame grab failed ({fails})")
                # A handful of misses is a hiccup; a sustained run means the
                # device has gone. Reconnect instead of warning forever.
                if fails >= config.CAMERA_REOPEN_AFTER_FAILS:
                    now = time.monotonic()
                    if now - last_reopen >= config.CAMERA_REOPEN_PERIOD_S:
                        last_reopen = now
                        reopens += 1
                        log.warning(f"camera gone after {fails} failed grabs - "
                                    f"re-opening (attempt {reopens})")
                        if self._reopen():
                            log.info(f"camera RECOVERED on {self.device}")
                            fails, reopens = 0, 0
                        else:
                            self._cap = None      # keep trying from the top
                time.sleep(0.05)
                continue
            fails = 0
            if self.rotate_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            now = time.monotonic()
            self._last_frame_t = now
            with self._cond:
                self._frame, self._frame_t = frame, now
                self._frame_id += 1
                self._cond.notify_all()
            n += 1
            if now - t0 >= 2.0:
                self.fps_measured = n / (now - t0)
                n, t0 = 0, now


def open_camera(backend=None):
    """Start whichever camera this machine has. Raises CameraError if none.

    "auto" tries the CSI camera first and falls back to the USB webcam, so the
    same code runs on a car with either one fitted.
    """
    backend = (backend or config.CAMERA_BACKEND).lower()
    if backend == "csi":
        return CameraStream().start()
    if backend == "usb":
        return UsbCameraStream().start()

    try:
        return CameraStream().start()
    except Exception as exc:
        log.info(f"no CSI camera ({str(exc)[:70]}) - trying a USB webcam")
    return UsbCameraStream().start()


if __name__ == "__main__":
    import cv2

    import sys

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    want = sys.argv[1] if len(sys.argv) > 1 else None
    try:
        cam = open_camera(want)
    except CameraError as exc:
        print(f"CAMERA ERROR: {exc}")
        raise SystemExit(1)
    try:
        last_id, got, t_end = 0, 0, time.monotonic() + 3.0
        while time.monotonic() < t_end:
            frame, last_id, _ = cam.read(newer_than=last_id)
            got += frame is not None
        print(f"{got} frames in 3 s ({got / 3.0:.1f} fps), shape {None if frame is None else frame.shape}")
        os.makedirs(config.LOG_DIR, exist_ok=True)
        out = os.path.join(config.LOG_DIR, "camera_test.jpg")
        cv2.imwrite(out, frame)
        print(f"saved {out}")
    finally:
        cam.stop()
