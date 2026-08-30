"""Camera-brightness-based Z autofocus calibration.

General-purpose module, independent of any single stage/laser wiring.
Callers supply their own move/capture/pulse callbacks so this works with
the ESP300 stage functions (move_z_abs, laser_on/laser_off) or any other
motion/laser controller without modification here.

Core idea: at a fixed X/Y point, sweep Z through a small range around the
expected focus, fire a brief pulse at each step, measure spark brightness
via camera, and fit the peak to find true focus Z. Works per calibration
point, so it generalizes to any number of surfaces with different focus
heights -- each surface just gets its own calibration call.

Camera angle / mounting is NOT assumed fixed. The spot ROI is not a
hardcoded pixel box -- it is located automatically per calibration call by
diffing a baseline (no-pulse) frame against a pulse frame and finding the
region that lit up. Re-run the same calibrate_surface() call after the
camera has been re-mounted at a different angle and it re-locates the spot
on its own; nothing here needs updating.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

@dataclass
class BrightnessCamera:
    """Thin wrapper over a UVC camera for spot-brightness measurement."""

    index: int
    backend: int = cv2.CAP_MSMF
    roi: Optional[tuple[int, int, int, int]] = None  # (x, y, w, h) in pixels
    warmup_frames: int = 2

    _cap: cv2.VideoCapture = field(init=False, repr=False, default=None)

    def open(self) -> None:
        self._cap = cv2.VideoCapture(self.index, self.backend)
        if not self._cap.isOpened():
            raise RuntimeError(f"camera index {self.index} did not open")
        # Discard a couple of frames so auto-exposure settles.
        for _ in range(self.warmup_frames):
            self._cap.read()

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __enter__(self) -> "BrightnessCamera":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def read_brightness(self, samples: int = 1) -> dict:
        """Grab `samples` frames and return the peak brightness seen.

        Using max-of-samples (not mean-of-samples) matters because the spark
        is a brief transient -- averaging away noise would also average away
        the peak we're trying to catch.
        """
        if self._cap is None:
            raise RuntimeError("camera not open, call open() or use as context manager")

        best_mean = -1.0
        best_max = -1.0
        for _ in range(max(1, samples)):
            ok, frame = self._cap.read()
            if not ok or frame is None:
                continue
            region = self._crop(frame)
            gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
            mean_val = float(np.mean(gray))
            max_val = float(np.max(gray))
            best_mean = max(best_mean, mean_val)
            best_max = max(best_max, max_val)

        if best_max < 0:
            raise RuntimeError("no frame captured during brightness read")
        return {"mean": best_mean, "max": best_max}

    def _crop(self, frame: np.ndarray) -> np.ndarray:
        if self.roi is None:
            return frame
        x, y, w, h = self.roi
        return frame[y:y + h, x:x + w]

    def read_frame(self) -> np.ndarray:
        """Grab one raw frame (no ROI crop). Used by ROI auto-detection."""
        if self._cap is None:
            raise RuntimeError("camera not open, call open() or use as context manager")
        ok, frame = self._cap.read()
        if not ok or frame is None:
            raise RuntimeError("no frame captured")
        return frame

    def auto_locate_roi(
        self,
        fire_pulse: Callable[[], None],
        margin_px: int = 20,
        diff_threshold: int = 30,
        settle_s: float = 0.05,
    ) -> tuple[int, int, int, int]:
        """Find the spark ROI by diffing a baseline frame against a pulse frame.

        Angle-independent by construction: whatever region lit up between
        the two frames is the spot, regardless of where in the image it
        happens to fall. Sets self.roi and returns it.
        """
        baseline = self.read_frame()
        fire_pulse()
        if settle_s > 0:
            time.sleep(settle_s)
        active = self.read_frame()

        roi = locate_spot_roi(baseline, active, margin_px=margin_px, diff_threshold=diff_threshold)
        if roi is None:
            raise RuntimeError(
                "could not locate spark in baseline/pulse frame diff -- "
                "check pulse power, camera framing, or diff_threshold"
            )
        self.roi = roi
        return roi


def locate_spot_roi(
    baseline: np.ndarray,
    active: np.ndarray,
    margin_px: int = 20,
    diff_threshold: int = 30,
    min_area_px: int = 4,
) -> Optional[tuple[int, int, int, int]]:
    """Return the bounding box of the region that got brighter, or None.

    This is what makes the sequence reusable at any camera angle: instead
    of a human picking pixel coordinates for one specific mounting, the
    spark location is derived from the image itself each time.
    """
    gray_base = cv2.cvtColor(baseline, cv2.COLOR_BGR2GRAY)
    gray_active = cv2.cvtColor(active, cv2.COLOR_BGR2GRAY)

    diff = cv2.subtract(gray_active, gray_base)  # only positive (brighter) deltas
    _, mask = cv2.threshold(diff, diff_threshold, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area_px:
        return None

    x, y, w, h = cv2.boundingRect(largest)
    frame_h, frame_w = gray_base.shape[:2]
    x0 = max(0, x - margin_px)
    y0 = max(0, y - margin_px)
    x1 = min(frame_w, x + w + margin_px)
    y1 = min(frame_h, y + h + margin_px)
    return (x0, y0, x1 - x0, y1 - y0)


# ---------------------------------------------------------------------------
# Focus scan
# ---------------------------------------------------------------------------

@dataclass
class FocusScanResult:
    best_z: float
    samples: list[tuple[float, float]]  # (z, brightness) -- coarse + any inserted refinement points, sorted by z
    fit_used: bool
    refined_points: int = 0


def scan_focus_z(
    move_z: Callable[[float], None],
    fire_pulse: Callable[[], None],
    read_brightness: Callable[[], float],
    center_z: float,
    range_mm: float = 0.3,
    step_mm: float = 0.02,
    settle_s: float = 0.05,
    metric: str = "max",
    direction: str = "both",
    adaptive_rise_ratio: float | None = 1.5,
    adaptive_subdivisions: int = 4,
) -> FocusScanResult:
    """Sweep Z around `center_z`, return the brightness-peak Z.

    move_z(z): blocking move to absolute Z.
    fire_pulse(): trigger one short low-power test pulse.
    read_brightness(): return a single scalar brightness reading for the
        most recent pulse (caller decides mean vs max, ROI, sample count).
    metric: label only, informational -- brightness scalar semantics are
        entirely up to read_brightness.
    direction: "both" sweeps center_z +- range_mm (default). "down"/"up"
        sweep only that one side (0 .. -range_mm / 0 .. +range_mm) -- use
        this when the focus surface is known to lie on one side only, so
        the sweep doesn't burn half its range and half its Z-safety budget
        going the wrong way.
    adaptive_rise_ratio: after the coarse pass, any adjacent pair of samples
        whose brightness jumps by more than this ratio (either direction --
        catches both edges of a spike) gets `adaptive_subdivisions` extra
        samples inserted between them, at that pair's own spacing (not the
        coarse step). This is what lets a fast, coarse sweep still resolve
        a spot the beam only lights up sharply on: the scan stays fast
        everywhere it's flat and only slows down where the signal actually
        moves. Set to None (or <= 1) to disable and keep the raw coarse
        grid only.
    adaptive_subdivisions: extra points inserted per flagged interval.

    Returns the parabolic-vertex estimate when the peak sample has a
    neighbor on both sides (sub-step resolution, exact for unevenly spaced
    neighbors too -- see below); otherwise returns the raw best-sample Z.
    """
    if range_mm <= 0 or step_mm <= 0:
        raise ValueError("range_mm and step_mm must be positive")

    n_steps = max(1, int(round(range_mm / step_mm)))
    if direction == "both":
        offsets = [i * step_mm for i in range(-n_steps, n_steps + 1)]
    elif direction == "down":
        offsets = [-i * step_mm for i in range(0, n_steps + 1)]
    elif direction == "up":
        offsets = [i * step_mm for i in range(0, n_steps + 1)]
    else:
        raise ValueError(f"Unknown direction {direction!r}; expected 'both', 'down', or 'up'")

    def measure(z: float) -> tuple[float, float]:
        move_z(z)
        if settle_s > 0:
            time.sleep(settle_s)
        fire_pulse()
        return (z, read_brightness())

    samples: list[tuple[float, float]] = [measure(center_z + offset) for offset in offsets]

    refined_points = 0
    if adaptive_rise_ratio and adaptive_rise_ratio > 1 and adaptive_subdivisions > 0:
        extra: list[tuple[float, float]] = []
        for (z_prev, v_prev), (z_cur, v_cur) in zip(samples, samples[1:]):
            spiked = v_cur > max(v_prev, 1e-9) * adaptive_rise_ratio or v_prev > max(v_cur, 1e-9) * adaptive_rise_ratio
            if not spiked:
                continue
            sub_step = (z_cur - z_prev) / (adaptive_subdivisions + 1)
            for k in range(1, adaptive_subdivisions + 1):
                extra.append(measure(z_prev + sub_step * k))
        if extra:
            refined_points = len(extra)
            samples = sorted(samples + extra, key=lambda s: s[0])

    peak_idx = max(range(len(samples)), key=lambda i: samples[i][1])
    best_z, _ = samples[peak_idx]
    fit_used = False

    if 0 < peak_idx < len(samples) - 1:
        z0, y0 = samples[peak_idx - 1]
        z1, y1 = samples[peak_idx]
        z2, y2 = samples[peak_idx + 1]
        # Vertex of the parabola through the three points, exact for
        # unevenly spaced z0/z1/z2 (falls back to the classic equal-step
        # formula when spacing happens to be uniform).
        d01 = z1 - z0
        d12 = z2 - z1
        if abs(d01) > 1e-12 and abs(d12) > 1e-12:
            f01 = (y1 - y0) / d01
            f12 = (y2 - y1) / d12
            curvature = (f12 - f01) / (z2 - z0)  # nonzero <=> a real local extremum, not a flat/linear run
            if abs(curvature) > 1e-9:
                vertex = (z0 + z1) / 2 - f01 / (2 * curvature)
                lo, hi = min(z0, z2), max(z0, z2)
                best_z = min(max(vertex, lo), hi)  # clamp into the sampled neighborhood
                fit_used = True

    return FocusScanResult(best_z=best_z, samples=samples, fit_used=fit_used, refined_points=refined_points)


# ---------------------------------------------------------------------------
# High-level, reusable per-surface entry point
# ---------------------------------------------------------------------------

def calibrate_surface(
    camera: BrightnessCamera,
    move_z: Callable[[float], None],
    fire_pulse: Callable[[], None],
    center_z: float,
    range_mm: float = 0.3,
    step_mm: float = 0.02,
    settle_s: float = 0.05,
    brightness_samples: int = 3,
    relocate_roi: bool = True,
) -> FocusScanResult:
    """One call per surface: locate the spark, then scan Z for its peak.

    This is the piece meant to be called repeatedly -- once per surface, or
    again after the camera has been re-mounted at a different angle. It
    does not assume anything about where the spot sits in frame: with
    relocate_roi=True (default) it re-derives the ROI from a fresh
    baseline/pulse diff every time, so a changed camera angle is handled
    automatically rather than requiring new hardcoded pixel coordinates.

    camera: an *open* BrightnessCamera (caller manages open()/close() so
        the same camera connection can be reused across many surfaces).
    move_z / fire_pulse: same contract as scan_focus_z.
    center_z: expected/nominal focus Z for this surface before correction.
    """
    if relocate_roi or camera.roi is None:
        camera.auto_locate_roi(fire_pulse, settle_s=settle_s)

    def read_brightness() -> float:
        return camera.read_brightness(samples=brightness_samples)["max"]

    return scan_focus_z(
        move_z=move_z,
        fire_pulse=fire_pulse,
        read_brightness=read_brightness,
        center_z=center_z,
        range_mm=range_mm,
        step_mm=step_mm,
        settle_s=settle_s,
    )


def hill_climb_focus_z(
    move_z: Callable[[float], None],
    fire_pulse: Callable[[], None],
    read_brightness: Callable[[], float],
    start_z: float,
    step_mm: float = 0.02,
    settle_s: float = 0.05,
    max_steps: int = 40,
    min_step_mm: float = 0.002,
) -> FocusScanResult:
    """Faster alternative to scan_focus_z: bidirectional hill-climb.

    Prefer scan_focus_z when noise is a concern (it looks at the whole
    neighborhood); prefer this when speed matters and the brightness curve
    is smooth (e.g. re-calibration after a small known drift).
    """

    def measure(z: float) -> float:
        move_z(z)
        if settle_s > 0:
            time.sleep(settle_s)
        fire_pulse()
        return read_brightness()

    samples: list[tuple[float, float]] = []
    z = start_z
    b = measure(z)
    samples.append((z, b))

    direction = 1.0
    current_step = step_mm
    steps_taken = 0

    while steps_taken < max_steps and current_step >= min_step_mm:
        trial_z = z + direction * current_step
        trial_b = measure(trial_z)
        samples.append((trial_z, trial_b))
        steps_taken += 1

        if trial_b > b:
            z, b = trial_z, trial_b
        else:
            direction *= -1.0
            current_step *= 0.5

    return FocusScanResult(best_z=z, samples=samples, fit_used=False)


# ---------------------------------------------------------------------------
# Standalone tuning entry point
# ---------------------------------------------------------------------------

def _cli():
    import argparse

    parser = argparse.ArgumentParser(description="Camera brightness probe / focus scan tuning (no stage motion, no laser).")
    parser.add_argument("--camera-index", type=int, default=1)
    parser.add_argument("--roi", type=int, nargs=4, metavar=("X", "Y", "W", "H"), default=None)
    parser.add_argument("--samples", type=int, default=5)
    args = parser.parse_args()

    roi = tuple(args.roi) if args.roi else None
    with BrightnessCamera(index=args.camera_index, roi=roi) as cam:
        for i in range(args.samples):
            result = cam.read_brightness(samples=3)
            print(f"[{i}] mean={result['mean']:.2f} max={result['max']:.2f}")
            time.sleep(0.3)


if __name__ == "__main__":
    _cli()
