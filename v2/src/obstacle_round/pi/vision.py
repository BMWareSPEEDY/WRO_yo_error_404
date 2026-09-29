"""
vision.py - red/green pillar detection with the YOLO ONNX model (onnxruntime).

BlockDetector  frame in -> list of blocks out. Class names and input size come
               from the model's own metadata, so a retrained model drops in.
VisionWorker   background thread: newest camera frame -> detector. latest()
               returns the frame it ran on together with its blocks, so the
               dashboard draws boxes on exactly the frame they came from.

Each block (full-frame pixels, largest = closest first):
    {'class': 'Red'|'Green', 'confidence', 'x1', 'y1', 'x2', 'y2', 'cx', 'cy',
     'w', 'h', 'area', 'dist_cm'}
dist_cm = PILLAR_FOCAL_PX * PILLAR_HEIGHT_CM / box height - rough until
PILLAR_FOCAL_PX is calibrated on this camera.

Runs in dashboard.py, which owns the camera.
"""
import ast
import logging
import os
import threading
import time

import cv2
import numpy as np

import config

log = logging.getLogger("vision")


class BlockDetector:
    def __init__(self, model_path=config.MODEL_PATH):
        import onnxruntime as ort

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"model not found: {model_path}")
        so = ort.SessionOptions()
        so.intra_op_num_threads = config.ONNX_THREADS
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(model_path, sess_options=so, providers=["CPUExecutionProvider"])
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        self.imgsz = int(inp.shape[2])                    # [1, 3, S, S]
        meta = self.session.get_modelmeta().custom_metadata_map
        self.names = {int(k): v for k, v in ast.literal_eval(meta["names"]).items()}
        self.keep_ids = [k for k, v in self.names.items() if v in config.BLOCK_CLASSES]
        self.last_infer_ms = 0.0
        self.focal_px = float(config.load_params().get("pillar_focal_px", config.PILLAR_FOCAL_PX))
        log.info(f"model {os.path.basename(model_path)}: {self.imgsz}px, classes {self.names}, "
                 f"using {[self.names[k] for k in self.keep_ids]}, focal={self.focal_px}")

    def _letterbox(self, frame):
        h, w = frame.shape[:2]
        r = min(self.imgsz / w, self.imgsz / h)
        nw, nh = int(round(w * r)), int(round(h * r))
        canvas = np.full((self.imgsz, self.imgsz, 3), 114, np.uint8)
        dx, dy = (self.imgsz - nw) // 2, (self.imgsz - nh) // 2
        canvas[dy:dy + nh, dx:dx + nw] = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0   # BGR->RGB, CHW
        return np.ascontiguousarray(blob), r, dx, dy

    def detect(self, frame, focal_px=None):
        t0 = time.perf_counter()
        focal = float(focal_px) if focal_px is not None else self.focal_px
        blob, r, dx, dy = self._letterbox(frame)
        preds = self.session.run(None, {self.input_name: blob})[0][0].T   # (N, 4 + classes)
        scores = preds[:, 4:]
        cls, conf = scores.argmax(1), scores.max(1)
        mask = (conf >= config.BLOCK_CONF) & np.isin(cls, self.keep_ids)
        blocks = []
        if mask.any():
            p, cls, conf = preds[mask], cls[mask], conf[mask]
            x1 = (p[:, 0] - p[:, 2] / 2 - dx) / r          # letterbox centre-xywh -> frame corners
            y1 = (p[:, 1] - p[:, 3] / 2 - dy) / r
            bw, bh = p[:, 2] / r, p[:, 3] / r
            idx = cv2.dnn.NMSBoxesBatched(np.stack([x1, y1, bw, bh], 1).tolist(), conf.tolist(),
                                          cls.tolist(), config.BLOCK_CONF, config.BLOCK_IOU)
            fh, fw = frame.shape[:2]
            for i in np.array(idx).flatten():
                bx1, by1 = int(max(0, x1[i])), int(max(0, y1[i]))
                bx2, by2 = int(min(fw - 1, x1[i] + bw[i])), int(min(fh - 1, y1[i] + bh[i]))
                h = max(1, by2 - by1)
                w_px = max(1, bx2 - bx1)

                # Shape tests: a pillar is compact and taller than it is wide.
                # The lines on the mat are long, flat and thin, and the model
                # calls one a red pillar now and again - this is where those go.
                if (w_px * h) < config.BLOCK_MIN_AREA_FRAC * fw * fh:
                    continue
                if (w_px / float(h)) > config.BLOCK_MAX_ASPECT:
                    continue

                # Red, or the orange line on the mat? Ask the pixels.
                name = self.names[int(cls[i])]
                if name == "Red" and config.RED_HUE_CHECK and not looks_red(frame, bx1, by1, bx2, by2):
                    continue

                blocks.append({
                    "class": name, "confidence": round(float(conf[i]), 2),
                    "x1": bx1, "y1": by1, "x2": bx2, "y2": by2,
                    "cx": (bx1 + bx2) // 2, "cy": (by1 + by2) // 2,
                    "w": w_px, "h": h, "area": w_px * h,
                    # A box touching a frame edge is CUT OFF, so its height is
                    # not the pillar's full height and dist_cm reads FARTHER
                    # than the truth - worst exactly when the car is closest.
                    "clipped": bool(bx1 <= 1 or by1 <= 1 or bx2 >= fw - 2 or by2 >= fh - 2),
                    "dist_cm": round(_distance_cm(focal, w_px=w_px, h_px=h,
                                                  clipped_vert=(by1 <= 1 or by2 >= fh - 2)), 1),
                })
        blocks.sort(key=lambda b: b["area"], reverse=True)
        self.last_infer_ms = (time.perf_counter() - t0) * 1000.0
        return blocks


def looks_red(frame, x1, y1, x2, y2):
    """Are the coloured pixels in this box RED, or is it the orange mat line?

    Takes the median hue of the saturated, bright pixels inside the box, which
    ignores the white mat showing around the edges of the detection. Red wraps
    around 0 in OpenCV's 0..179 hue, so both ends count; orange sits just above
    red and is what the line is made of. Returns True when it cannot tell - too
    few colourful pixels to judge - so this can only ever reject a clear orange,
    never a marginal red.
    """
    pad_x, pad_y = max(1, (x2 - x1) // 6), max(1, (y2 - y1) // 6)
    crop = frame[y1 + pad_y:y2 - pad_y, x1 + pad_x:x2 - pad_x]
    if crop.size == 0:
        return True
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    h, s_, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    keep = (s_ >= config.RED_HUE_MIN_SAT) & (v >= config.RED_HUE_MIN_VAL)
    if int(keep.sum()) < config.RED_HUE_MIN_PIXELS:
        return True
    hue = float(np.median(h[keep]))
    return hue <= config.RED_HUE_MAX or hue >= config.RED_HUE_MIN_WRAP


def _distance_cm(focal_px, w_px, h_px, clipped_vert):
    """Distance to a pillar of known size, from its box in pixels.

    dist = focal * real_size / pixel_size. Height (10 cm) is the better
    measurement - it is the bigger dimension, so it is the less noisy one - but
    a box cut off by the top or bottom of the frame is no longer the whole
    pillar, and its height then says the pillar is FARTHER away exactly when
    the car is closest. In that case the width (5 cm), which is still whole,
    is used instead.
    """
    if clipped_vert and w_px > 1:
        return focal_px * config.PILLAR_WIDTH_CM / w_px
    return focal_px * config.PILLAR_HEIGHT_CM / max(1, h_px)


def draw_rois(img, blocks=()):
    """Draw the three region-of-interest bands the dodge decision uses.

    A band lights up when a block's majority area falls in it - the same
    config.block_roi() the controller calls, so what is drawn is exactly what
    is decided on, never a second implementation that can drift out of step.
    """
    fh, fw = img.shape[:2]
    left_end, right_start = config.roi_bounds(fw)   # same bounds the controller decides on
    x1, x2 = int(left_end), int(right_start)

    active = {config.block_roi(b, fw) for b in blocks}
    bands = (("LEFT", 0, x1), ("CENTER", x1, x2), ("RIGHT", x2, fw))

    for name, lo, hi in bands:
        on = name in active
        colour = (0, 210, 255) if on else (150, 150, 150)
        if on:
            # translucent wash so the live band is obvious without hiding the view
            overlay = img.copy()
            cv2.rectangle(overlay, (lo, 0), (hi, fh), colour, -1)
            cv2.addWeighted(overlay, 0.12, img, 0.88, 0, img)
        cv2.putText(img, name, (lo + 12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, colour, 2 if on else 1)

    for x in (x1, x2):
        cv2.line(img, (x, 0), (x, fh), (0, 210, 255), 2)
    return img


def draw_blocks(img, blocks):
    """Draw the ROI bands and the detection boxes onto img in place."""
    draw_rois(img, blocks)
    for i, b in enumerate(blocks):
        colour = (60, 60, 255) if b["class"] == "Red" else (60, 220, 60)
        cv2.rectangle(img, (b["x1"], b["y1"]), (b["x2"], b["y2"]), colour, 4 if i == 0 else 2)
        roi = config.block_roi(b, img.shape[1])
        cv2.putText(img, f"{b['class']} {b['confidence']:.2f}  ~{b['dist_cm']:.0f}cm  [{roi}]",
                    (b["x1"], max(48, b["y1"] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
    return img


class VisionWorker(threading.Thread):
    def __init__(self, camera, detector):
        super().__init__(name="vision", daemon=True)
        self.camera, self.detector = camera, detector
        self._lock = threading.Lock()
        self._latest = None                 # (frame, frame_id, frame_t, blocks)
        self.fps = 0.0

    def set_focal_px(self, focal_px):
        self.detector.focal_px = float(focal_px)
        log.info(f"YOLO focal length updated to {self.detector.focal_px:.1f} px")

    def latest(self):
        with self._lock:
            return self._latest

    def run(self):
        last_id, n, t0 = 0, 0, time.monotonic()
        while True:
            frame, fid, ft = self.camera.read(newer_than=last_id, timeout=0.5)
            if frame is None or fid == last_id:
                continue
            last_id = fid
            try:
                blocks = self.detector.detect(frame)
            except Exception as exc:
                log.error(f"inference failed: {exc}")
                time.sleep(0.2)
                continue
            with self._lock:
                self._latest = (frame, fid, ft, blocks)
            n += 1
            now = time.monotonic()
            if now - t0 >= 1.0:
                self.fps, n, t0 = n / (now - t0), 0, now
