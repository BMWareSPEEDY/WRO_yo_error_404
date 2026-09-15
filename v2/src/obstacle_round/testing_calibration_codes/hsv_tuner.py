"""Simple HSV colour tuner for WRO block detection.

Usage:
    python3 hsv_tuner.py
    python3 hsv_tuner.py --source 0
    python3 hsv_tuner.py --source block_test.mp4 --color green

Keys:
    1 = red preset
    2 = green preset
    3 = magenta preset
    p = print the current NumPy ranges
    s = save the current ranges to hsv_values.txt
    q or ESC = quit
"""

import argparse
from pathlib import Path

import cv2
import numpy as np


WINDOW = "WRO HSV Tuner"
CONTROL = "HSV Controls"

PRESETS = {
    "red": ((0, 70, 60), (10, 255, 255)),
    "green": ((45, 60, 40), (90, 255, 255)),
    "magenta": ((135, 70, 60), (175, 255, 255)),
}


def nothing(_value):
    pass


def create_trackbars(lower, upper):
    cv2.namedWindow(CONTROL)
    cv2.createTrackbar("H lower", CONTROL, lower[0], 179, nothing)
    cv2.createTrackbar("S lower", CONTROL, lower[1], 255, nothing)
    cv2.createTrackbar("V lower", CONTROL, lower[2], 255, nothing)
    cv2.createTrackbar("H upper", CONTROL, upper[0], 179, nothing)
    cv2.createTrackbar("S upper", CONTROL, upper[1], 255, nothing)
    cv2.createTrackbar("V upper", CONTROL, upper[2], 255, nothing)


def set_trackbars(lower, upper):
    values = (*lower, *upper)
    names = ("H lower", "S lower", "V lower", "H upper", "S upper", "V upper")
    for name, value in zip(names, values):
        cv2.setTrackbarPos(name, CONTROL, int(value))


def read_ranges():
    names = ("H lower", "S lower", "V lower", "H upper", "S upper", "V upper")
    values = [cv2.getTrackbarPos(name, CONTROL) for name in names]
    return np.array(values[:3]), np.array(values[3:])


def print_ranges(lower, upper, colour):
    print(f"\n{colour.upper()} HSV ranges:")
    print(f"lower_{colour} = np.array({lower.tolist()})")
    print(f"upper_{colour} = np.array({upper.tolist()})")


def save_ranges(lower, upper, colour, path):
    text = (
        f"# HSV values tuned for {colour}\n"
        f"lower_{colour} = np.array({lower.tolist()})\n"
        f"upper_{colour} = np.array({upper.tolist()})\n"
    )
    path.write_text(text)
    print(f"Saved HSV values to {path}")


def open_source(source):
    try:
        source = int(source)
    except ValueError:
        pass

    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise RuntimeError(f"Could not open camera/video source: {source}")
    return capture


def main():
    parser = argparse.ArgumentParser(description="Tune HSV thresholds for WRO blocks")
    parser.add_argument("--source", default="0", help="Camera index or video path")
    parser.add_argument("--color", choices=tuple(PRESETS), default="green")
    parser.add_argument("--output", default="hsv_values.txt", help="File used by the 's' key")
    args = parser.parse_args()

    capture = open_source(args.source)
    colour = args.color
    lower, upper = PRESETS[colour]
    create_trackbars(lower, upper)
    set_trackbars(lower, upper)

    print("HSV tuner running. Press 1/2/3 for red/green/magenta, p to print, s to save, q to quit.")

    while True:
        ok, frame = capture.read()
        if not ok:
            print("End of video or camera frame unavailable")
            break

        lower, upper = read_ranges()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Keep the three views visible at once: original, mask, and filtered colour.
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        preview = np.hstack((frame, mask_bgr, result))
        cv2.putText(
            preview,
            f"{colour.upper()}  lower={lower.tolist()} upper={upper.tolist()}",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )
        cv2.imshow(WINDOW, preview)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key in (ord("1"), ord("2"), ord("3")):
            colour = {ord("1"): "red", ord("2"): "green", ord("3"): "magenta"}[key]
            lower, upper = PRESETS[colour]
            set_trackbars(lower, upper)
        elif key == ord("p"):
            print_ranges(lower, upper, colour)
        elif key == ord("s"):
            save_ranges(lower, upper, colour, Path(args.output))

    capture.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
