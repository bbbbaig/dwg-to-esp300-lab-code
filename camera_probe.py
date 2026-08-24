"""USB UVC camera probe.

Scans camera indices, grabs a frame from each working one, and reports
resolution plus a brightness reading so we can confirm the right device
index before wiring it into the focus routine.
"""

import cv2
import numpy as np

MAX_INDEX_TO_SCAN = 5


def brightness_metrics(frame: np.ndarray) -> dict:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return {
        "mean": float(np.mean(gray)),
        "max": float(np.max(gray)),
    }


def main():
    found_any = False
    for index in range(MAX_INDEX_TO_SCAN):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
            continue

        ok, frame = cap.read()
        if not ok or frame is None:
            print(f"[index {index}] opened but no frame")
            cap.release()
            continue

        found_any = True
        h, w = frame.shape[:2]
        metrics = brightness_metrics(frame)
        print(
            f"[index {index}] OK  resolution={w}x{h}  "
            f"mean_brightness={metrics['mean']:.2f}  max_brightness={metrics['max']:.2f}"
        )
        cap.release()

    if not found_any:
        print("No camera responded. Check USB connection / Windows camera privacy settings.")


if __name__ == "__main__":
    main()
