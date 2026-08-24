from __future__ import annotations

import argparse
import cgi
import importlib.util
import json
import math
import mimetypes
import re
import shutil
import threading
import time
import uuid
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, quote, urlparse

import cv2
import numpy as np
import serial

# MSMF backend spams a C++ warning on every failed grabFrame (e.g. camera
# unplugged) with no Python-side rate limit -- left running overnight this
# fills a log file at ~25 lines/sec. Silence it; failures are still handled
# in CameraController._capture_loop below.
cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)

from dwg_to_esp300 import (
    LaserPath,
    apply_fit_scale,
    cap_scale_to_circle,
    decode_dxf_unicode_escapes,
    distance,
    ensure_dxf,
    extract_paths,
    generate_esp300_script,
    orient_path_from_current,
    raw_bbox,
    rotate_paths,
    select_design_paths,
    transform_paths,
    write_preview_svg,
)


ROOT = Path(__file__).resolve().parent
STATIC_ROOT = ROOT / "visualizer"
UPLOAD_ROOT = ROOT / "uploads"
GENERATED_ROOT = ROOT / "generated"
DEFAULT_SOURCE = Path(r"C:\Users\bjh14\Downloads\adjust25design.dwg")
DEFAULT_LONGEST_SIDE_MM = 25.0
DEFAULT_ROTATION_DEG = 45.0
FIXED_REFERENCE_CIRCLE_DIAMETER_MM = 2.0
DEFAULT_FEED_RATE_MM_S = 0.4
DEFAULT_TRAVEL_RATE_MM_S = 1.0
DEFAULT_FOCUS_Z_MM = 0.0
DEFAULT_Z_DEFOCUS_MM = 1.0
DEFAULT_Z_SPEED_MM_S = 0.3
Z_STEP_BACK_PASSES = 10
Z_STEP_BACK_MM = 0.001
DEFAULT_COMMAND_SETTLE_S = 0.05
DEFAULT_POSITION_TOLERANCE_MM = 0.003
DEFAULT_GLASS_SIZE_MM = (25.0, 25.0)
DEFAULT_USABLE_MARGIN_MM = 0.05
CURVE_FLATTENING_SCALE_HINT = 25.4
SCRIPT_CHOICES = {
    "full": ROOT / "adjust25design_esp300.py",
    "outline": ROOT / "adjust25design_outline_esp300.py",
}

SESSIONS: dict[str, dict] = {}

JOG_AXES = {"x": 1, "y": 2, "z": 3}
JOG_DEFAULT_BAUDRATE = 19200
JOG_MOTOR_SETTLE_S = 0.05
JOG_TP_RETRIES = 5
JOG_TP_RETRY_DELAY = 0.08
JOG_Z_RANGE_MM = 10.0
JOG_STEP_MAX_MM = 5.0
JOG_SPEED_MAX_MM_S = 10.0
JOG_HOME_SPEED_MM_S = 5.0
JOG_HOME_TIMEOUT_S = 60.0
JOG_POSITION_PATTERN = re.compile(r"[-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?")


class JogController:
    """Manual jog interface for the ESP300 stage controller over serial.

    Each /api/jog/move call executes one discrete relative move, bounded by
    the usable glass circle (X/Y) and a fixed safety window (Z). The client
    issues repeated calls while a jog control is held.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.ser: serial.Serial | None = None
        self.dry_run = True
        self.connected = False
        self.port = ""
        self.baudrate = JOG_DEFAULT_BAUDRATE
        self.position = {"x": 12.5, "y": 12.5, "z": 0.0}
        self.raw_position = dict(self.position)
        self.local_offset = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.home_ref = dict(self.position)
        # Raw (machine) Z at the last calibration point (connect/home/set
        # local home), used to bound live focus-trim jogging. focus_trim_mm
        # is the cumulative Z nudge applied via jog since that calibration;
        # it is folded into local_offset["z"] so every already-planned
        # focus/defocus target picks it up automatically (see move()).
        self.z_home_raw = 0.0
        self.focus_trim_mm = 0.0
        self.limit_center = (12.5, 12.5)
        self.limit_radius = 12.5
        self.z_range = JOG_Z_RANGE_MM
        self._poll_generation = 0
        self._run_thread: threading.Thread | None = None
        self._run_stop = threading.Event()
        self._run_state = {"running": False, "index": 0, "total": 0, "error": None, "done": False}

    def _write(self, cmd: str) -> None:
        if self.dry_run or self.ser is None:
            return
        self.ser.write((cmd + "\r\n").encode("ascii"))

    def _query(self, cmd: str) -> str:
        if self.dry_run or self.ser is None:
            return ""
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\r\n").encode("ascii"))
        # Poll without a fixed pre-delay, but require a line terminator
        # (\r or \n) before treating the reply as complete. A reply spanning
        # multiple USB-serial read chunks can otherwise be read as complete
        # after only a partial chunk, leaving trailing bytes in the buffer
        # that corrupt the next query's response.
        response = b""
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            if self.ser.in_waiting > 0:
                response += self.ser.read(self.ser.in_waiting)
                if b"\r" in response or b"\n" in response:
                    time.sleep(0.002)
                    if self.ser.in_waiting > 0:
                        response += self.ser.read(self.ser.in_waiting)
                    break
            else:
                time.sleep(0.003)
        return response.decode("ascii", errors="ignore").strip()

    def _read_axis_position(self, axis: int) -> float:
        last = ""
        for attempt in range(JOG_TP_RETRIES):
            last = self._query(f"{axis}TP")
            match = JOG_POSITION_PATTERN.search(last)
            # Discard out-of-range values (parsing/framing artifacts) rather
            # than accepting them; the configured travel range is well
            # under 200 mm on every axis.
            if match and abs(float(match.group())) <= 250.0:
                return float(match.group())
            if attempt < JOG_TP_RETRIES - 1:
                time.sleep(JOG_TP_RETRY_DELAY)
        raise RuntimeError(f"Axis {axis} position read failed; last reply={last!r}")

    def _raw_of(self, name: str, local_value: float) -> float:
        """Local (reported/design) coordinate -> true machine coordinate."""
        return local_value + self.local_offset[name]

    def _local_of(self, name: str, raw_value: float) -> float:
        """True machine coordinate -> local (reported/design) coordinate."""
        return raw_value - self.local_offset[name]

    def _refresh_limits(self) -> None:
        session = next(iter(SESSIONS.values()), None) if SESSIONS else None
        metadata = session["metadata"] if session else default_glass_metadata()
        center = metadata.get("usable_center_xy_mm") or [12.5, 12.5]
        diameter = float(metadata.get("usable_diameter_mm", 25.0))
        self.limit_center = (float(center[0]), float(center[1]))
        self.limit_radius = max(diameter / 2.0, 0.1)

    def _status_locked(self) -> dict:
        return {
            "connected": self.connected,
            "dryRun": self.dry_run,
            "port": self.port,
            "baudrate": self.baudrate,
            "position": dict(self.position),
            "focusTrimMm": round(self.focus_trim_mm, 6),
            "limit": {
                "centerX": self.limit_center[0],
                "centerY": self.limit_center[1],
                "radiusMm": self.limit_radius,
                "zRangeMm": self.z_range,
            },
        }

    def _disconnect_locked(self) -> None:
        self._poll_generation += 1
        if self.ser is not None:
            try:
                for axis in JOG_AXES.values():
                    self._write(f"{axis}MF")
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connected = False

    def _poll_loop(self, generation: int) -> None:
        """Background thread: refresh self.position from TP reads so status
        queries reflect current motion without blocking on move(). Exits
        when disconnect/estop increments the generation counter."""
        while True:
            for name, axis in JOG_AXES.items():
                with self._lock:
                    if self._poll_generation != generation or not self.connected or self.dry_run or self.ser is None:
                        return
                    try:
                        raw = self._read_axis_position(axis)
                        self.raw_position[name] = raw
                        self.position[name] = self._local_of(name, raw)
                    except RuntimeError:
                        pass
                time.sleep(0.005)
            time.sleep(0.02)

    def status(self) -> dict:
        with self._lock:
            return self._status_locked()

    def connect(
        self,
        port: str,
        baudrate: int,
        dry_run: bool,
        range_xy_mm: float | None = None,
        range_z_mm: float | None = None,
    ) -> dict:
        with self._lock:
            self._disconnect_locked()
            self.dry_run = bool(dry_run)
            self.port = str(port or "COM5")
            self.baudrate = int(baudrate or JOG_DEFAULT_BAUDRATE)
            self._refresh_limits()
            # An explicit range from the client overrides the traced-session
            # glass circle, for jog operations unrelated to the loaded job.
            if range_xy_mm:
                self.limit_radius = max(0.1, float(range_xy_mm))
            self.z_range = max(0.1, float(range_z_mm)) if range_z_mm else JOG_Z_RANGE_MM
            # Fresh connection: local frame starts equal to the real machine
            # frame (offset 0) until set_local_home() is explicitly called.
            # True machine home itself is never touched here or anywhere else.
            self.local_offset = {"x": 0.0, "y": 0.0, "z": 0.0}
            if not self.dry_run:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=2,
                    rtscts=True,
                )
                for axis in JOG_AXES.values():
                    self._write(f"{axis}MO")
                    time.sleep(JOG_MOTOR_SETTLE_S)
                for name, axis in JOG_AXES.items():
                    raw = self._read_axis_position(axis)
                    self.raw_position[name] = raw
                    self.position[name] = raw
            else:
                self.position = {"x": self.limit_center[0], "y": self.limit_center[1], "z": 0.0}
                self.raw_position = dict(self.position)
            # Recenter the soft limit on the current stage position, in case
            # it does not match the last traced session's glass center.
            self.limit_center = (self.position["x"], self.position["y"])
            self.home_ref = dict(self.position)
            self.z_home_raw = self.raw_position["z"]
            self.focus_trim_mm = 0.0
            self.connected = True
            self._poll_generation += 1
            generation = self._poll_generation
            if not self.dry_run:
                threading.Thread(target=self._poll_loop, args=(generation,), daemon=True).start()
            return self._status_locked()

    def disconnect(self) -> dict:
        with self._lock:
            self._disconnect_locked()
            return self._status_locked()

    def move(self, axis_name: str, delta_mm: float, speed_mm_s: float) -> dict:
        if axis_name not in JOG_AXES:
            raise ValueError(f"Unknown jog axis {axis_name!r}.")
        with self._lock:
            if not self.connected:
                raise RuntimeError("Jog not connected. Connect first.")
            axis = JOG_AXES[axis_name]
            delta = max(-JOG_STEP_MAX_MM, min(JOG_STEP_MAX_MM, float(delta_mm)))
            speed = max(0.01, min(JOG_SPEED_MAX_MM_S, float(speed_mm_s)))

            if axis_name == "z":
                # Z jog is a live focus trim: it physically moves the stage
                # by delta, but folds that delta into local_offset["z"]
                # instead of self.position["z"]. Any focus/defocus target
                # already baked into the loaded plan is a fixed *local* Z
                # value, converted to a raw move via _raw_of() at the moment
                # it is sent (including by the "Run on machine" worker) -
                # so shifting the offset means every future scripted Z move
                # picks up this correction automatically instead of
                # snapping the stage back to the un-trimmed focus the next
                # time the plan touches Z. This is what lets the user
                # refocus mid-run without the correction being undone.
                new_raw = self.raw_position["z"] + delta
                if abs(new_raw - self.z_home_raw) > self.z_range:
                    raise RuntimeError("Move blocked: outside Z safety range.")
                if not self.dry_run:
                    self._write(f"{axis}VA{speed:.6f}")
                    time.sleep(0.02)
                    self._write(f"{axis}PA{new_raw:.6f}")
                self.raw_position["z"] = new_raw
                self.local_offset["z"] += delta
                self.focus_trim_mm += delta
                return self._status_locked()

            target = dict(self.position)
            target[axis_name] = self.position[axis_name] + delta

            dx = target["x"] - self.limit_center[0]
            dy = target["y"] - self.limit_center[1]
            if math.hypot(dx, dy) > self.limit_radius:
                raise RuntimeError("Move blocked: outside usable glass circle.")

            if not self.dry_run:
                self._write(f"{axis}VA{speed:.6f}")
                time.sleep(0.02)
                # PA (absolute move) is used in place of PR (relative move);
                # PR does not produce motion on this controller.
                self._write(f"{axis}PA{self._raw_of(axis_name, target[axis_name]):.6f}")
                # The command is issued without waiting for completion; the
                # background poll loop keeps self.position current from TP
                # reads for the duration of the move.
                self.raw_position[axis_name] = self._raw_of(axis_name, target[axis_name])
            self.position = target
            return self._status_locked()

    def estop(self) -> dict:
        self._run_stop.set()
        with self._lock:
            if not self.dry_run and self.ser is not None:
                try:
                    self._write("AB")
                    for axis in JOG_AXES.values():
                        self._write(f"{axis}MF")
                except Exception:
                    pass
            # Require an explicit reconnect after E-STOP before jogging resumes.
            self.connected = False
            self._poll_generation += 1
            return self._status_locked()

    def diagnose(self) -> dict:
        """Read-only ESP300 status query. Sends no motion commands."""
        with self._lock:
            if self.dry_run or self.ser is None:
                raise RuntimeError("Live connection required for diagnostics.")
            report = {}
            report["error_buffer"] = self._query("TB?")
            for name, axis in JOG_AXES.items():
                report[f"axis_{name}_motor_on"] = self._query(f"{axis}MO?")
                report[f"axis_{name}_motion_done"] = self._query(f"{axis}MD?")
                report[f"axis_{name}_axis_error"] = self._query(f"{axis}TE?")
                report[f"axis_{name}_stage_id"] = self._query(f"{axis}ID?")
            return report

    def _send_absolute_locked(self, target: dict, speed: float) -> None:
        """Issue VA+PA for all three axes and return without waiting for
        arrival. Caller must hold self._lock. Confirmation of arrival is
        handled by the background poll loop."""
        if not self.dry_run:
            for axis in JOG_AXES.values():
                self._write(f"{axis}VA{max(0.01, float(speed)):.6f}")
                time.sleep(0.02)
            for name, axis in JOG_AXES.items():
                raw = self._raw_of(name, target[name])
                self._write(f"{axis}PA{raw:.6f}")
                self.raw_position[name] = raw
        else:
            # Keep raw_position consistent with the local frame in dry-run
            # too, so a live Z focus trim (which is bounded against
            # raw_position/z_home_raw) behaves the same whether or not
            # hardware is attached.
            for name in target:
                self.raw_position[name] = self._raw_of(name, target[name])
        self.position = dict(target)

    def home(self) -> dict:
        """Move X/Y/Z to the local coordinate origin (0,0,0). This move is
        not subject to the jog soft limits, as the origin is the reference
        point by definition."""
        with self._lock:
            if not self.connected:
                raise RuntimeError("Jog not connected. Connect first.")
            target = {"x": 0.0, "y": 0.0, "z": 0.0}
            self._send_absolute_locked(target, JOG_HOME_SPEED_MM_S)
            self.limit_center = (0.0, 0.0)
            self.home_ref = dict(target)
            # Homing redefines the reference point, so any live focus trim
            # accumulated before this point is now baked into position 0;
            # restart the trim counter and Z safety-window anchor from here.
            self.z_home_raw = self.raw_position["z"]
            self.focus_trim_mm = 0.0
            return self._status_locked()

    def set_local_home(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> dict:
        """Assign local coordinates (x,y,z) to the current physical position
        without motion or any serial command. This adjusts only the
        software-side coordinate offset; the ESP300's absolute machine
        coordinate system is not modified. Passing the loaded design's
        center aligns reported/limit/Run-on-machine coordinates with the
        design frame."""
        with self._lock:
            if not self.connected:
                raise RuntimeError("Jog not connected. Connect first.")
            ref = {"x": float(x), "y": float(y), "z": float(z)}
            self.local_offset = {name: self.raw_position[name] - ref[name] for name in ref}
            self.position = dict(ref)
            self.limit_center = (ref["x"], ref["y"])
            self.home_ref = dict(ref)
            self.z_home_raw = self.raw_position["z"]
            self.focus_trim_mm = 0.0
            return self._status_locked()

    def go_to_local_home(self) -> dict:
        """Move X/Y/Z to the currently labeled local-home reference point
        (home_ref) - the position last marked via set_local_home(), e.g. the
        design center. Distinct from home(), which always targets literal
        local (0,0,0) regardless of what set_local_home() has labeled."""
        with self._lock:
            if not self.connected:
                raise RuntimeError("Jog not connected. Connect first.")
            target = dict(self.home_ref)
            self._send_absolute_locked(target, JOG_HOME_SPEED_MM_S)
            return self._status_locked()

    def goto(self, x: float, y: float, z: float) -> dict:
        """Move to an absolute X/Y/Z (mm), subject to the usable glass
        circle and Z safety window."""
        with self._lock:
            if not self.connected:
                raise RuntimeError("Jog not connected. Connect first.")
            target = {"x": float(x), "y": float(y), "z": float(z)}
            dx = target["x"] - self.limit_center[0]
            dy = target["y"] - self.limit_center[1]
            if math.hypot(dx, dy) > self.limit_radius:
                raise RuntimeError("Move blocked: outside usable glass circle.")
            if abs(target["z"] - self.home_ref["z"]) > self.z_range:
                raise RuntimeError("Move blocked: outside Z safety range.")
            self._send_absolute_locked(target, JOG_SPEED_MAX_MM_S)
            return self._status_locked()

    def run_plan(self, segments: list[dict]) -> dict:
        """Execute the given motion segments on the stage in order, with
        each segment's arrival confirmed before the next is issued. Runs in
        a background thread; poll run_status() for progress."""
        with self._lock:
            if not self.connected:
                raise RuntimeError("Jog not connected. Connect first.")
            if self._run_thread is not None and self._run_thread.is_alive():
                raise RuntimeError("A run is already in progress.")
            if not segments:
                raise RuntimeError("Nothing to run.")
            self._run_stop.clear()
            self._run_state = {"running": True, "index": 0, "total": len(segments), "error": None, "done": False}
            generation = self._poll_generation
            thread = threading.Thread(target=self._run_worker, args=(list(segments), generation), daemon=True)
            self._run_thread = thread
            thread.start()
            return dict(self._run_state)

    def _run_worker(self, segments: list[dict], generation: int) -> None:
        tolerance = 0.02
        for index, segment in enumerate(segments):
            if self._run_stop.is_set():
                break
            to3 = segment.get("to3")
            if not to3:
                with self._lock:
                    self._run_state["index"] = index + 1
                continue
            duration = max(float(segment.get("duration", 0.1)), 0.05)
            z_only = bool(segment.get("zOnly"))
            axes_to_move: list[tuple[str, int, float]] = []

            with self._lock:
                if self._poll_generation != generation or not self.connected:
                    self._run_state["error"] = "Connection lost."
                    break
                cur = dict(self.position)
                target = {"x": float(to3[0]), "y": float(to3[1]), "z": float(to3[2])}
                if z_only:
                    if abs(target["z"] - cur["z"]) > 1e-6:
                        speed = max(0.01, abs(target["z"] - cur["z"]) / duration)
                        raw_z = self._raw_of("z", target["z"])
                        self._write(f"3VA{speed:.6f}")
                        time.sleep(0.02)
                        self._write(f"3PA{raw_z:.6f}")
                        self.raw_position["z"] = raw_z
                        axes_to_move.append(("z", 3, raw_z))
                else:
                    if abs(target["x"] - cur["x"]) > 1e-6:
                        self._write(f"1VA{max(0.01, abs(target['x'] - cur['x']) / duration):.6f}")
                        time.sleep(0.02)
                    if abs(target["y"] - cur["y"]) > 1e-6:
                        self._write(f"2VA{max(0.01, abs(target['y'] - cur['y']) / duration):.6f}")
                        time.sleep(0.02)
                    if abs(target["x"] - cur["x"]) > 1e-6:
                        raw_x = self._raw_of("x", target["x"])
                        self._write(f"1PA{raw_x:.6f}")
                        self.raw_position["x"] = raw_x
                        axes_to_move.append(("x", 1, raw_x))
                    if abs(target["y"] - cur["y"]) > 1e-6:
                        raw_y = self._raw_of("y", target["y"])
                        self._write(f"2PA{raw_y:.6f}")
                        self.raw_position["y"] = raw_y
                        axes_to_move.append(("y", 2, raw_y))
                self.position = target

            if not self.dry_run and axes_to_move:
                deadline = time.monotonic() + max(3.0, duration * 4 + 2.0)
                while time.monotonic() < deadline and not self._run_stop.is_set():
                    arrived = True
                    with self._lock:
                        if self._poll_generation != generation or not self.connected:
                            break
                        for name, axis, raw_value in axes_to_move:
                            try:
                                actual_raw = self._read_axis_position(axis)
                                self.raw_position[name] = actual_raw
                                self.position[name] = self._local_of(name, actual_raw)
                            except RuntimeError:
                                arrived = False
                                continue
                            if abs(actual_raw - raw_value) > tolerance:
                                arrived = False
                    if arrived:
                        break
                    time.sleep(0.05)
            elif self.dry_run:
                time.sleep(min(duration, 0.03))

            with self._lock:
                self._run_state["index"] = index + 1

        with self._lock:
            self._run_state["running"] = False
            if self._run_stop.is_set() and not self._run_state["error"]:
                self._run_state["error"] = "Stopped."
            self._run_state["done"] = not self._run_stop.is_set() and not self._run_state["error"]

    def run_status(self) -> dict:
        with self._lock:
            return dict(self._run_state)

    def run_stop(self) -> dict:
        self._run_stop.set()
        with self._lock:
            return dict(self._run_state)


JOG = JogController()


CAMERA_STREAM_FPS = 15
CAMERA_JPEG_QUALITY = 80
CAMERA_BOUNDARY = "focusframe"


class CameraController:
    """Background UVC capture for the live focus-brightness preview.

    A single capture thread keeps grabbing frames while connected, so the
    MJPEG stream endpoint and the brightness status endpoint both just read
    the latest already-decoded frame instead of contending for the device.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.cap: cv2.VideoCapture | None = None
        self.index: int | None = None
        self.connected = False
        self.roi: tuple[int, int, int, int] | None = None
        self._latest_jpeg: bytes | None = None
        self._latest_stats = {"mean": None, "max": None}
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    def connect(self, index: int, backend: int = cv2.CAP_MSMF) -> dict:
        with self._lock:
            self._disconnect_locked()
            cap = cv2.VideoCapture(int(index), backend)
            if not cap.isOpened():
                cap.release()
                raise RuntimeError(f"Camera index {index} did not open.")
            self.cap = cap
            self.index = int(index)
            self.connected = True
            self._stop.clear()
            thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread = thread
            thread.start()
            return self._status_locked()

    def _disconnect_locked(self) -> None:
        self._stop.set()
        if self.cap is not None:
            self.cap.release()
        self.cap = None
        self.connected = False
        self._latest_jpeg = None
        self._latest_stats = {"mean": None, "max": None}

    def disconnect(self) -> dict:
        with self._lock:
            self._disconnect_locked()
            return self._status_locked()

    def set_roi(self, roi: tuple[int, int, int, int] | None) -> dict:
        with self._lock:
            self.roi = roi
            return self._status_locked()

    def _capture_loop(self) -> None:
        interval = 1.0 / CAMERA_STREAM_FPS
        consecutive_failures = 0
        max_consecutive_failures = CAMERA_STREAM_FPS * 5  # ~5s of dead reads
        while not self._stop.is_set():
            started = time.monotonic()
            with self._lock:
                cap = self.cap
            if cap is None:
                break
            ok, frame = cap.read()
            if ok and frame is not None:
                consecutive_failures = 0
                with self._lock:
                    roi = self.roi
                region = frame
                if roi is not None:
                    x, y, w, h = roi
                    cropped = frame[max(0, y) : y + h, max(0, x) : x + w]
                    if cropped.size > 0:
                        region = cropped
                gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
                stats = {"mean": float(np.mean(gray)), "max": float(np.max(gray))}
                ok2, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, CAMERA_JPEG_QUALITY])
                if ok2:
                    with self._lock:
                        self._latest_jpeg = buf.tobytes()
                        self._latest_stats = stats
            else:
                consecutive_failures += 1
                if consecutive_failures >= max_consecutive_failures:
                    print(f"[camera] {consecutive_failures} consecutive grabFrame failures, disconnecting")
                    with self._lock:
                        self._disconnect_locked()
                    break
            elapsed = time.monotonic() - started
            time.sleep(max(0.0, interval - elapsed))

    def _status_locked(self) -> dict:
        return {
            "connected": self.connected,
            "index": self.index,
            "roi": list(self.roi) if self.roi else None,
            "brightness": dict(self._latest_stats),
        }

    def status(self) -> dict:
        with self._lock:
            return self._status_locked()

    def latest_jpeg(self) -> bytes | None:
        with self._lock:
            return self._latest_jpeg


CAMERA = CameraController()


def optional_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    return float(value)


def float_or_default(value: object, default: float) -> float:
    parsed = optional_float(value)
    return default if parsed is None else parsed


def safe_file_stem(name: object) -> str:
    stem = Path(str(name or "design")).stem
    safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", stem).strip("._")
    return safe or "design"


def path_length(points: list[tuple[float, float]]) -> float:
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def path_bbox(points: list[tuple[float, float]]) -> dict:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return {
        "minX": min(xs),
        "minY": min(ys),
        "maxX": max(xs),
        "maxY": max(ys),
        "width": max(xs) - min(xs),
        "height": max(ys) - min(ys),
    }


def bbox_payload(paths: list[LaserPath]) -> dict:
    (min_x, min_y), (max_x, max_y) = raw_bbox(paths)
    return {
        "minX": min_x,
        "minY": min_y,
        "maxX": max_x,
        "maxY": max_y,
        "width": max_x - min_x,
        "height": max_y - min_y,
    }


def default_bbox_payload(size: float = DEFAULT_LONGEST_SIDE_MM) -> dict:
    return {
        "minX": 0.0,
        "minY": 0.0,
        "maxX": size,
        "maxY": size,
        "width": size,
        "height": size,
    }


def glass_size_from_metadata(metadata: dict | None = None) -> tuple[float, float]:
    raw = (metadata or {}).get("glass_size_mm", DEFAULT_GLASS_SIZE_MM)
    if isinstance(raw, (int, float)):
        width = height = float(raw)
    else:
        values = list(raw) if isinstance(raw, (list, tuple)) else list(DEFAULT_GLASS_SIZE_MM)
        width = float(values[0]) if values else DEFAULT_GLASS_SIZE_MM[0]
        height = float(values[1]) if len(values) > 1 else width
    return max(width, 0.001), max(height, 0.001)


def glass_start_xy_from_metadata(metadata: dict | None = None) -> tuple[float, float]:
    width, height = glass_size_from_metadata(metadata)
    raw = (metadata or {}).get("start_xy_mm")
    if isinstance(raw, (list, tuple)) and len(raw) >= 2:
        return float(raw[0]), float(raw[1])
    usable_center = (metadata or {}).get("usable_center_xy_mm")
    if isinstance(usable_center, (list, tuple)) and len(usable_center) >= 2:
        return float(usable_center[0]), float(usable_center[1])
    return width / 2.0, height / 2.0


def default_glass_metadata() -> dict:
    width, height = DEFAULT_GLASS_SIZE_MM
    center = [width / 2.0, height / 2.0]
    return {
        "glass_size_mm": [width, height],
        "start_xy_mm": center,
        "usable_area_shape": "circle",
        "usable_diameter_mm": min(width, height),
        "usable_center_xy_mm": center,
        "usable_margin_mm": DEFAULT_USABLE_MARGIN_MM,
    }


def glass_bbox_payload(metadata: dict | None = None) -> dict:
    width, height = glass_size_from_metadata(metadata)
    return {
        "minX": 0.0,
        "minY": 0.0,
        "maxX": width,
        "maxY": height,
        "width": width,
        "height": height,
    }


def detected_circle_diameters(paths: list[LaserPath]) -> list[float]:
    diameters: list[float] = []
    for laser_path in paths:
        if len(laser_path.points) < 12:
            continue
        if not laser_path.closed and distance(laser_path.points[0], laser_path.points[-1]) > 1e-6:
            continue
        box = path_bbox(laser_path.points)
        diameter = (box["width"] + box["height"]) / 2.0
        if diameter <= 0:
            continue
        roundness = abs(box["width"] - box["height"]) / diameter
        circumference = path_length(laser_path.points)
        circle_error = abs(circumference - math.pi * diameter) / max(math.pi * diameter, 1e-9)
        if roundness <= 0.08 and circle_error <= 0.15:
            diameters.append(diameter)
    return sorted(diameters)


def circle_check(paths: list[LaserPath], target_diameter: float | None) -> dict | None:
    diameters = detected_circle_diameters(paths)
    if not diameters:
        return None
    median = diameters[len(diameters) // 2]
    result = {
        "count": len(diameters),
        "detected_diameter_mm": median,
        "min_diameter_mm": min(diameters),
        "max_diameter_mm": max(diameters),
    }
    if target_diameter:
        result["target_diameter_mm"] = target_diameter
        result["difference_mm"] = median - target_diameter
        result["difference_percent"] = ((median - target_diameter) / target_diameter) * 100.0
    return result


def laser_path_to_payload(index: int, laser_path: LaserPath, *, passes: int = 1, selected: bool = True) -> dict:
    points = [(float(x), float(y)) for x, y in laser_path.points]
    return {
        "id": index,
        "layer": decode_dxf_unicode_escapes(laser_path.layer),
        "rawLayer": laser_path.layer,
        "entityType": laser_path.entity_type,
        "closed": laser_path.closed,
        "points": [[round(x, 6), round(y, 6)] for x, y in points],
        "length": path_length(points),
        "bbox": path_bbox(points),
        "passes": max(1, int(passes)),
        "selected": bool(selected),
    }


def payload_to_laser_path(payload: dict) -> LaserPath:
    return LaserPath(
        points=[(float(x), float(y)) for x, y in payload["points"]],
        layer=payload.get("rawLayer") or payload.get("layer", "0"),
        entity_type=payload.get("entityType", "PATH"),
        closed=bool(payload.get("closed", False)),
    )


def load_generated_script(script_path: Path) -> tuple[list[dict], dict]:
    spec = importlib.util.spec_from_file_location(f"generated_{script_path.stem}", script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load {script_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.PATHS, module.METADATA


def build_segments(
    paths: list[LaserPath],
    metadata: dict,
    *,
    base_ids: list[int] | None = None,
    pass_indices: list[int] | None = None,
) -> dict:
    feed_rate = float(metadata.get("feed_rate", DEFAULT_FEED_RATE_MM_S))
    travel_rate = float(metadata.get("travel_rate", DEFAULT_TRAVEL_RATE_MM_S))
    focus_z = float(metadata.get("focus_z_mm", DEFAULT_FOCUS_Z_MM))
    z_defocus_mm = abs(float(metadata.get("z_defocus_mm", DEFAULT_Z_DEFOCUS_MM)))
    defocus_z = focus_z - z_defocus_mm
    z_speed = max(float(metadata.get("z_speed_mm_s", DEFAULT_Z_SPEED_MM_S)), 0.001)
    position_tolerance = max(float(metadata.get("position_tolerance_mm", DEFAULT_POSITION_TOLERANCE_MM)), 0.0)
    segments: list[dict] = []
    cut_length = 0.0
    travel_length = 0.0
    z_length = 0.0
    current = glass_start_xy_from_metadata(metadata)
    current_z = focus_z
    all_points = [current]

    def point3(point: tuple[float, float], z: float) -> list[float]:
        return [round(float(point[0]), 6), round(float(point[1]), 6), round(float(z), 6)]

    def add_z_segment(kind: str, point: tuple[float, float], target_z: float, source_id: int, layer: str) -> None:
        nonlocal current_z, z_length
        distance_z = abs(target_z - current_z)
        if distance_z <= 1e-9:
            current_z = target_z
            return
        segments.append(
            {
                "kind": kind,
                "pathIndex": source_id,
                "from": [round(float(point[0]), 6), round(float(point[1]), 6)],
                "to": [round(float(point[0]), 6), round(float(point[1]), 6)],
                "from3": point3(point, current_z),
                "to3": point3(point, target_z),
                "length": distance_z,
                "duration": distance_z / z_speed,
                "layer": layer,
                "laserOn": False,
                "zOnly": True,
            }
        )
        z_length += distance_z
        current_z = target_z

    for index, laser_path in enumerate(paths):
        points = [(float(x), float(y)) for x, y in laser_path.points]
        if len(points) < 2:
            continue
        source_id = base_ids[index] if base_ids and index < len(base_ids) else index
        layer = decode_dxf_unicode_escapes(laser_path.layer)
        first = points[0]
        travel = distance(current, first)
        if travel > position_tolerance:
            add_z_segment("z-defocus", current, defocus_z, source_id, layer)
            segments.append(
                {
                    "kind": "travel",
                    "pathIndex": source_id,
                    "from": [round(current[0], 6), round(current[1], 6)],
                    "to": [round(first[0], 6), round(first[1], 6)],
                    "from3": point3(current, current_z),
                    "to3": point3(first, current_z),
                    "length": travel,
                    "duration": travel / travel_rate,
                    "layer": layer,
                    "laserOn": False,
                }
            )
            travel_length += travel
            all_points.append(first)
        current = first
        # Focus retreats by Z_STEP_BACK_MM every Z_STEP_BACK_PASSES repeats
        # of a path, along the defocus direction, to compensate for
        # material recession over multiple passes.
        pass_index = pass_indices[index] if pass_indices and index < len(pass_indices) else 0
        pass_focus_z = focus_z - (pass_index // Z_STEP_BACK_PASSES) * Z_STEP_BACK_MM
        add_z_segment("z-focus", current, pass_focus_z, source_id, layer)

        for start, end in zip(points, points[1:]):
            length = distance(start, end)
            if length <= 1e-9:
                continue
            segments.append(
                {
                    "kind": "cut",
                    "pathIndex": source_id,
                    "from": [round(start[0], 6), round(start[1], 6)],
                    "to": [round(end[0], 6), round(end[1], 6)],
                    "from3": point3(start, current_z),
                    "to3": point3(end, current_z),
                    "length": length,
                    "duration": length / feed_rate,
                    "layer": layer,
                    "laserOn": True,
                }
            )
            cut_length += length
            all_points.extend([start, end])
            current = end

    if paths:
        final_source_id = base_ids[-1] if base_ids else len(paths) - 1
        add_z_segment("z-defocus", current, defocus_z, final_source_id, "final")

    glass_width, glass_height = glass_size_from_metadata(metadata)
    box = path_bbox(all_points + [(0.0, 0.0), (glass_width, glass_height)])
    return {
        "metadata": metadata,
        "bbox": box,
        "stats": {
            "segments": len(segments),
            "scheduledPaths": len(paths),
            "cutLength": cut_length,
            "travelLength": travel_length,
            "zLength": z_length,
            "totalLength": cut_length + travel_length,
            "motionLength3d": cut_length + travel_length + z_length,
            "duration": sum(segment["duration"] for segment in segments),
            "points": sum(len(path.points) for path in paths),
        },
        "segments": segments,
    }


def build_motion_from_script(script_key: str) -> dict:
    script_path = SCRIPT_CHOICES.get(script_key, SCRIPT_CHOICES["full"])
    paths, metadata = load_generated_script(script_path)
    laser_paths = [
        LaserPath(
            points=[(float(x), float(y)) for x, y in path["points"]],
            layer=path.get("layer", "0"),
            entity_type=path.get("entity_type", "PATH"),
            closed=bool(path.get("closed", False)),
        )
        for path in paths
    ]
    payload = build_segments(laser_paths, metadata)
    payload["script"] = script_key
    payload["scriptPath"] = str(script_path)
    payload["stats"]["paths"] = len(laser_paths)
    return payload


def expand_passes(
    order: list[LaserPath], pass_counts: dict[int, int], ordered_ids: list[int]
) -> tuple[list[LaserPath], list[int], list[int]]:
    scheduled: list[LaserPath] = []
    scheduled_ids: list[int] = []
    scheduled_pass_indices: list[int] = []
    current: tuple[float, float] | None = None

    for laser_path, path_id in zip(order, ordered_ids):
        passes = max(0, int(pass_counts.get(path_id, 1)))
        if passes <= 0:
            continue

        points = list(laser_path.points)
        if current is not None:
            forward = distance(current, points[0])
            backward = distance(current, points[-1])
            if backward < forward:
                points = list(reversed(points))

        for pass_index in range(passes):
            pass_points = points if pass_index % 2 == 0 or laser_path.closed else list(reversed(points))
            scheduled.append(
                LaserPath(
                    points=list(pass_points),
                    layer=laser_path.layer,
                    entity_type=laser_path.entity_type,
                    closed=laser_path.closed,
                )
            )
            scheduled_ids.append(path_id)
            scheduled_pass_indices.append(pass_index)
            current = pass_points[-1]

    return scheduled, scheduled_ids, scheduled_pass_indices


def endpoint_after_passes(laser_path: LaserPath, passes: int) -> tuple[float, float]:
    if laser_path.closed or passes % 2 == 1:
        return laser_path.points[-1]
    return laser_path.points[0]


def item_travel_cost(
    items: list[tuple[int, LaserPath]],
    pass_counts: dict[int, int],
    start: tuple[float, float] = (0.0, 0.0),
) -> float:
    current = start
    cost = 0.0
    for path_id, laser_path in items:
        oriented = orient_path_from_current(laser_path, current)
        cost += distance(current, oriented.points[0])
        current = endpoint_after_passes(oriented, max(1, int(pass_counts.get(path_id, 1))))
    return cost


def orient_items(
    items: list[tuple[int, LaserPath]],
    pass_counts: dict[int, int],
    start: tuple[float, float] = (0.0, 0.0),
) -> tuple[list[LaserPath], list[int]]:
    current = start
    oriented_paths: list[LaserPath] = []
    ordered_ids: list[int] = []
    for path_id, laser_path in items:
        oriented = orient_path_from_current(laser_path, current)
        oriented_paths.append(oriented)
        ordered_ids.append(path_id)
        current = endpoint_after_passes(oriented, max(1, int(pass_counts.get(path_id, 1))))
    return oriented_paths, ordered_ids


def optimize_items(
    items: list[tuple[int, LaserPath]],
    pass_counts: dict[int, int],
    start: tuple[float, float] = (0.0, 0.0),
) -> tuple[list[LaserPath], list[int]]:
    remaining = list(items)
    ordered: list[tuple[int, LaserPath]] = []
    current = start

    while remaining:
        best_index = min(
            range(len(remaining)),
            key=lambda index: distance(current, orient_path_from_current(remaining[index][1], current).points[0]),
        )
        path_id, laser_path = remaining.pop(best_index)
        oriented = orient_path_from_current(laser_path, current)
        ordered.append((path_id, oriented))
        current = endpoint_after_passes(oriented, max(1, int(pass_counts.get(path_id, 1))))

    best_cost = item_travel_cost(ordered, pass_counts, start)
    for _ in range(2):
        improved = False
        for i in range(0, len(ordered) - 2):
            for j in range(i + 2, len(ordered)):
                candidate = ordered[:i] + list(reversed(ordered[i : j + 1])) + ordered[j + 1 :]
                cost = item_travel_cost(candidate, pass_counts, start)
                if cost + 1e-6 < best_cost:
                    ordered = candidate
                    best_cost = cost
                    improved = True
                    break
            if improved:
                break
        if not improved:
            break

    return orient_items(ordered, pass_counts, start)


def build_plan(session: dict, request: dict) -> dict:
    if "paths" in request:
        selected = request.get("paths") or []
    else:
        selected = [
            {"id": path["id"], "passes": path.get("passes", 1)}
            for path in session["paths"]
            if path.get("selected", True)
        ]

    pass_counts = {int(item["id"]): max(0, int(item.get("passes", 1))) for item in selected}
    selected_ids = [path_id for path_id, count in pass_counts.items() if count > 0]
    path_by_id = {int(path["id"]): payload_to_laser_path(path) for path in session["paths"]}
    selected_paths = [path_by_id[path_id] for path_id in selected_ids if path_id in path_by_id]
    selected_ids = [path_id for path_id in selected_ids if path_id in path_by_id]

    if not selected_paths:
        raise ValueError("No selected paths with pass count > 0.")

    start_xy = glass_start_xy_from_metadata(session["metadata"])
    optimize = bool(request.get("optimize", True))
    if optimize:
        optimized_paths, ordered_ids = optimize_items(list(zip(selected_ids, selected_paths)), pass_counts, start=start_xy)
    else:
        optimized_paths = selected_paths
        ordered_ids = selected_ids

    scheduled_paths, scheduled_ids, scheduled_pass_indices = expand_passes(optimized_paths, pass_counts, ordered_ids)
    metadata = {
        **default_glass_metadata(),
        **session["metadata"],
        "feed_rate": float(request.get("feedRate", session["metadata"].get("feed_rate", DEFAULT_FEED_RATE_MM_S))),
        "travel_rate": float(request.get("travelRate", session["metadata"].get("travel_rate", DEFAULT_TRAVEL_RATE_MM_S))),
        "selected_path_count": len(selected_paths),
        "scheduled_path_count": len(scheduled_paths),
        "path_passes": pass_counts,
        "optimize": optimize,
        "focus_z_mm": float(request.get("focusZMm", session["metadata"].get("focus_z_mm", DEFAULT_FOCUS_Z_MM))),
        "z_defocus_mm": float(request.get("zDefocusMm", session["metadata"].get("z_defocus_mm", DEFAULT_Z_DEFOCUS_MM))),
        "z_speed_mm_s": float(request.get("zSpeedMmS", session["metadata"].get("z_speed_mm_s", DEFAULT_Z_SPEED_MM_S))),
        "position_tolerance_mm": float(
            request.get("positionTolerance", session["metadata"].get("position_tolerance_mm", DEFAULT_POSITION_TOLERANCE_MM))
        ),
    }
    payload = build_segments(scheduled_paths, metadata, base_ids=scheduled_ids, pass_indices=scheduled_pass_indices)
    payload["sessionId"] = session["id"]
    payload["selectedIds"] = selected_ids
    payload["scheduledIds"] = scheduled_ids
    payload["stats"]["paths"] = len(selected_paths)
    payload["stats"]["passes"] = sum(pass_counts[path_id] for path_id in selected_ids)
    payload["paths"] = [
        {
            "id": path_id,
            "passes": pass_counts[path_id],
            "length": path_by_id[path_id].length,
        }
        for path_id in selected_ids
    ]
    payload["_scheduledPaths"] = scheduled_paths
    return payload


def trace_file(source: Path, params: dict) -> dict:
    if not source.exists():
        raise FileNotFoundError(source)

    trace_mode = params.get("traceMode", "design")
    fit_width_mm = optional_float(params.get("fitWidthMm"))
    fit_height_mm = optional_float(params.get("fitHeightMm"))
    fit_longest_mm = float_or_default(params.get("fitLongestMm"), DEFAULT_LONGEST_SIDE_MM)
    flatness_mm = float_or_default(params.get("flatnessMm"), 0.02)
    min_length_mm = float_or_default(params.get("minLengthMm"), 0.001)
    feed_rate = float_or_default(params.get("feedRate"), DEFAULT_FEED_RATE_MM_S)
    travel_rate = float_or_default(params.get("travelRate"), DEFAULT_TRAVEL_RATE_MM_S)
    rotation_deg = float_or_default(params.get("rotationDeg"), DEFAULT_ROTATION_DEG)

    session_id = uuid.uuid4().hex[:12]
    output_stub = GENERATED_ROOT / f"{source.stem}_{session_id}_esp300.py"
    dxf_path = ensure_dxf(source, output_stub)
    skipped: dict[str, int] = {}
    raw_paths, _doc, skipped = extract_paths(
        dxf_path,
        include_layers=params.get("includeLayers", []),
        exclude_layers=params.get("excludeLayers", []),
        flatness_mm=flatness_mm,
        unit_scale=CURVE_FLATTENING_SCALE_HINT,
        min_length_mm=min_length_mm,
    )
    if trace_mode == "design" and not params.get("includeLayers"):
        raw_paths = select_design_paths(raw_paths, skipped)
    if not raw_paths:
        raise RuntimeError("No drawable paths found after trace filters.")

    # Rotate the drawing about its own center before fit scaling; for a
    # non-square layout this can increase usable margin within the
    # circular working area.
    raw_paths = rotate_paths(raw_paths, rotation_deg)

    glass_meta = default_glass_metadata()
    usable_diameter_mm = float(params.get("usableDiameterMm", glass_meta["usable_diameter_mm"]))
    usable_margin_mm = float(params.get("usableMarginMm", glass_meta["usable_margin_mm"]))
    glass_meta["usable_diameter_mm"] = usable_diameter_mm
    glass_meta["usable_margin_mm"] = usable_margin_mm
    usable_center_xy = glass_meta["usable_center_xy_mm"]

    scale = apply_fit_scale(
        raw_paths,
        CURVE_FLATTENING_SCALE_HINT,
        fit_width_mm,
        fit_height_mm,
        fit_longest_mm,
    )
    scale = cap_scale_to_circle(raw_paths, scale, diameter_mm=usable_diameter_mm, margin_mm=usable_margin_mm)
    machine_paths, source_bbox, machine_bbox = transform_paths(
        raw_paths,
        unit_scale=scale,
        origin=(float(usable_center_xy[0]), float(usable_center_xy[1])),
        anchor="center",
        flip_x=bool(params.get("flipX", False)),
        flip_y=bool(params.get("flipY", False)),
        max_segment_mm=float(params.get("maxSegmentMm", 0.0)),
    )

    payload_paths = [laser_path_to_payload(index, laser_path) for index, laser_path in enumerate(machine_paths)]
    circle_info = circle_check(machine_paths, FIXED_REFERENCE_CIRCLE_DIAMETER_MM)
    metadata = {
        **glass_meta,
        "source": str(source),
        "dxf": str(dxf_path),
        "unit_scale": scale,
        "machine_unit": "mm",
        "origin": [float(usable_center_xy[0]), float(usable_center_xy[1])],
        "anchor": "center",
        "feed_rate": feed_rate,
        "travel_rate": travel_rate,
        "focus_z_mm": DEFAULT_FOCUS_Z_MM,
        "z_defocus_mm": DEFAULT_Z_DEFOCUS_MM,
        "z_speed_mm_s": DEFAULT_Z_SPEED_MM_S,
        "trace_mode": trace_mode,
        "fit_width_mm": fit_width_mm,
        "fit_height_mm": fit_height_mm,
        "fit_longest_mm": fit_longest_mm,
        "rotation_deg": rotation_deg,
        "reference_circle_diameter_mm": FIXED_REFERENCE_CIRCLE_DIAMETER_MM,
        "circle_check": circle_info,
        "path_count": len(machine_paths),
        "point_count": sum(len(path.points) for path in machine_paths),
        "total_length_mm": round(sum(path.length for path in machine_paths), 6),
        "skipped": skipped,
    }
    session = {
        "id": session_id,
        "sourceName": source.name,
        "sourcePath": str(source),
        "dxfPath": str(dxf_path),
        "metadata": metadata,
        "bbox": glass_bbox_payload(metadata),
        "rawBbox": {
            "minX": source_bbox[0][0],
            "minY": source_bbox[0][1],
            "maxX": source_bbox[1][0],
            "maxY": source_bbox[1][1],
        },
        "machineBbox": {
            "minX": machine_bbox[0][0],
            "minY": machine_bbox[0][1],
            "maxX": machine_bbox[1][0],
            "maxY": machine_bbox[1][1],
        },
        "paths": payload_paths,
    }
    SESSIONS[session_id] = session
    return session


def manual_path_from_payload(payload: dict) -> tuple[LaserPath, int, bool] | None:
    raw_points = payload.get("points") or []
    points: list[tuple[float, float]] = []
    for raw_point in raw_points:
        if not isinstance(raw_point, (list, tuple)) or len(raw_point) < 2:
            continue
        x = float(raw_point[0])
        y = float(raw_point[1])
        if math.isfinite(x) and math.isfinite(y):
            points.append((x, y))

    points = close_duplicate_points(points)
    closed = bool(payload.get("closed", False))
    if closed and len(points) > 2 and distance(points[0], points[-1]) > 1e-7:
        points.append(points[0])

    if len(points) < 2 or path_length(points) <= 1e-9:
        return None

    layer = str(payload.get("layer") or "직접 조합")
    entity_type = str(payload.get("entityType") or payload.get("entity_type") or "MANUAL")
    passes = max(1, int(payload.get("passes", 1)))
    selected = bool(payload.get("selected", True))
    return LaserPath(points=points, layer=layer, entity_type=entity_type, closed=closed), passes, selected


def close_duplicate_points(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    cleaned: list[tuple[float, float]] = []
    for point in points:
        if not cleaned or distance(cleaned[-1], point) > 1e-9:
            cleaned.append(point)
    return cleaned


def build_manual_session(request: dict) -> dict:
    session_id = uuid.uuid4().hex[:12]
    name = str(request.get("name") or "manual_design")
    feed_rate = float_or_default(request.get("feedRate"), DEFAULT_FEED_RATE_MM_S)
    travel_rate = float_or_default(request.get("travelRate"), DEFAULT_TRAVEL_RATE_MM_S)

    parsed_paths: list[tuple[LaserPath, int, bool]] = []
    for payload in request.get("paths") or []:
        if not isinstance(payload, dict):
            continue
        parsed = manual_path_from_payload(payload)
        if parsed is not None:
            parsed_paths.append(parsed)

    laser_paths = [item[0] for item in parsed_paths]
    payload_paths = [
        laser_path_to_payload(index, laser_path, passes=passes, selected=selected)
        for index, (laser_path, passes, selected) in enumerate(parsed_paths)
    ]
    box = bbox_payload(laser_paths) if laser_paths else default_bbox_payload()
    circle_info = circle_check(laser_paths, FIXED_REFERENCE_CIRCLE_DIAMETER_MM) if laser_paths else None
    metadata = {
        **default_glass_metadata(),
        "source": name,
        "dxf": None,
        "source_type": "manual",
        "unit_scale": 1.0,
        "machine_unit": "mm",
        "origin": [0.0, 0.0],
        "anchor": "drawing-origin",
        "feed_rate": feed_rate,
        "travel_rate": travel_rate,
        "focus_z_mm": DEFAULT_FOCUS_Z_MM,
        "z_defocus_mm": DEFAULT_Z_DEFOCUS_MM,
        "z_speed_mm_s": DEFAULT_Z_SPEED_MM_S,
        "trace_mode": "manual",
        "fit_width_mm": None,
        "fit_height_mm": None,
        "fit_longest_mm": None,
        "reference_circle_diameter_mm": FIXED_REFERENCE_CIRCLE_DIAMETER_MM,
        "circle_check": circle_info,
        "path_count": len(laser_paths),
        "point_count": sum(len(path.points) for path in laser_paths),
        "total_length_mm": round(sum(path.length for path in laser_paths), 6),
        "skipped": {},
    }
    session = {
        "id": session_id,
        "sourceName": name,
        "sourcePath": None,
        "dxfPath": None,
        "metadata": metadata,
        "bbox": glass_bbox_payload(metadata),
        "rawBbox": box,
        "machineBbox": box,
        "paths": payload_paths,
    }
    SESSIONS[session_id] = session
    return session


class VisualizerHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/motion":
            query = parse_qs(parsed.query)
            script_key = query.get("script", ["full"])[0]
            self.send_json(build_motion_from_script(script_key))
            return
        if parsed.path == "/api/default":
            self.handle_default()
            return
        if parsed.path == "/api/session":
            query = parse_qs(parsed.query)
            session_id = query.get("id", [""])[0]
            session = SESSIONS.get(session_id)
            if not session:
                self.send_error_json(404, "Unknown session.")
                return
            self.send_json(session)
            return
        if parsed.path == "/api/download":
            self.handle_download(parsed)
            return
        if parsed.path == "/api/jog/status":
            self.send_json(JOG.status())
            return
        if parsed.path == "/api/run/status":
            self.send_json(JOG.run_status())
            return
        if parsed.path == "/api/camera/status":
            self.send_json(CAMERA.status())
            return
        if parsed.path == "/api/camera/stream":
            self.handle_camera_stream()
            return

        path = parsed.path.lstrip("/") or "index.html"
        target = (STATIC_ROOT / path).resolve()
        if not str(target).startswith(str(STATIC_ROOT.resolve())) or not target.exists():
            self.send_error(404)
            return

        content_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        data = target.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        try:
            if parsed.path == "/api/trace":
                self.handle_trace()
            elif parsed.path == "/api/manual":
                self.send_json(build_manual_session(self.read_json()))
            elif parsed.path == "/api/plan":
                request = self.read_json()
                session = SESSIONS.get(request.get("sessionId", ""))
                if not session:
                    self.send_error_json(404, "Unknown session.")
                    return
                plan = build_plan(session, request)
                plan.pop("_scheduledPaths", None)
                self.send_json(plan)
            elif parsed.path == "/api/export":
                self.handle_export()
            elif parsed.path == "/api/jog/connect":
                request = self.read_json()
                self.send_json(
                    JOG.connect(
                        request.get("port", "COM5"),
                        request.get("baudrate", JOG_DEFAULT_BAUDRATE),
                        request.get("dryRun", True),
                        request.get("rangeXYMm"),
                        request.get("rangeZMm"),
                    )
                )
            elif parsed.path == "/api/jog/disconnect":
                self.send_json(JOG.disconnect())
            elif parsed.path == "/api/jog/move":
                request = self.read_json()
                self.send_json(JOG.move(request.get("axis", ""), request.get("deltaMm", 0), request.get("speedMmS", 0.5)))
            elif parsed.path == "/api/jog/estop":
                self.send_json(JOG.estop())
            elif parsed.path == "/api/jog/diag":
                self.send_json(JOG.diagnose())
            elif parsed.path == "/api/jog/home":
                self.send_json(JOG.home())
            elif parsed.path == "/api/jog/go-local-home":
                self.send_json(JOG.go_to_local_home())
            elif parsed.path == "/api/jog/set-local-home":
                request = self.read_json()
                self.send_json(JOG.set_local_home(request.get("x", 0), request.get("y", 0), request.get("z", 0)))
            elif parsed.path == "/api/jog/goto":
                request = self.read_json()
                self.send_json(JOG.goto(request.get("x", 0), request.get("y", 0), request.get("z", 0)))
            elif parsed.path == "/api/run/start":
                request = self.read_json()
                self.send_json(JOG.run_plan(request.get("segments", [])))
            elif parsed.path == "/api/run/stop":
                self.send_json(JOG.run_stop())
            elif parsed.path == "/api/camera/connect":
                request = self.read_json()
                self.send_json(CAMERA.connect(int(request.get("index", 1))))
            elif parsed.path == "/api/camera/disconnect":
                self.send_json(CAMERA.disconnect())
            elif parsed.path == "/api/camera/roi":
                request = self.read_json()
                roi = request.get("roi")
                self.send_json(CAMERA.set_roi(tuple(int(v) for v in roi) if roi else None))
            else:
                self.send_error(404)
        except Exception as exc:
            self.send_error_json(400, str(exc))

    def handle_default(self) -> None:
        if DEFAULT_SOURCE.exists():
            params = {
                "fitLongestMm": DEFAULT_LONGEST_SIDE_MM,
                "traceMode": "design",
                "feedRate": DEFAULT_FEED_RATE_MM_S,
                "travelRate": DEFAULT_TRAVEL_RATE_MM_S,
            }
            self.send_json(trace_file(DEFAULT_SOURCE, params))
            return

        self.send_error_json(404, "Default sample DWG not found. Upload a DWG/DXF file.")

    def handle_trace(self) -> None:
        content_type = self.headers.get("Content-Type", "")
        if content_type.startswith("multipart/form-data"):
            form = cgi.FieldStorage(
                fp=self.rfile,
                headers=self.headers,
                environ={"REQUEST_METHOD": "POST", "CONTENT_TYPE": content_type},
            )
            params = {key: form.getvalue(key) for key in form.keys() if key != "file"}
            file_item = form["file"] if "file" in form else None
            if file_item is not None and getattr(file_item, "filename", ""):
                UPLOAD_ROOT.mkdir(exist_ok=True)
                safe_name = Path(file_item.filename).name
                source = UPLOAD_ROOT / f"{uuid.uuid4().hex[:8]}_{safe_name}"
                with source.open("wb") as handle:
                    shutil.copyfileobj(file_item.file, handle)
            else:
                source = Path(str(params.get("sourcePath", ""))).expanduser()
        else:
            params = self.read_json()
            source = Path(str(params.get("sourcePath", ""))).expanduser()

        self.send_json(trace_file(source, params))

    def handle_export(self) -> None:
        request = self.read_json()
        session = SESSIONS.get(request.get("sessionId", ""))
        if not session:
            self.send_error_json(404, "Unknown session.")
            return
        plan = build_plan(session, request)
        scheduled_paths = plan.pop("_scheduledPaths")
        pass_by_id = {int(item["id"]): int(item["passes"]) for item in plan["paths"]}
        base_paths: list[LaserPath] = []
        base_passes: list[int] = []
        seen_ids: set[int] = set()
        for path_id, laser_path in zip(plan["scheduledIds"], scheduled_paths):
            if path_id in seen_ids:
                continue
            seen_ids.add(path_id)
            base_paths.append(laser_path)
            base_passes.append(pass_by_id.get(int(path_id), 1))

        uniform_passes = base_passes[0] if base_passes and all(count == base_passes[0] for count in base_passes) else None
        embedded_path_passes = None if uniform_passes is not None else base_passes
        GENERATED_ROOT.mkdir(exist_ok=True)
        stem = "manual_design" if session["metadata"].get("source_type") == "manual" else safe_file_stem(session["sourceName"])
        output = GENERATED_ROOT / f"{stem}_{session['id']}_selected_esp300.py"
        preview = output.with_suffix(".svg")
        metadata = {
            **plan["metadata"],
            "path_count": len(base_paths),
            "scheduled_path_count": sum(base_passes),
            "point_count": sum(len(path.points) for path in base_paths),
            "base_total_length_mm": round(sum(path.length for path in base_paths), 6),
            "total_length_mm": round(sum(path.length * passes for path, passes in zip(base_paths, base_passes)), 6),
        }
        generate_esp300_script(
            base_paths,
            output=output,
            metadata=metadata,
            port=str(request.get("port", "COM5")),
            baudrate=int(request.get("baudrate", 19200)),
            feed_rate=float(metadata.get("feed_rate", DEFAULT_FEED_RATE_MM_S)),
            travel_rate=float(metadata.get("travel_rate", DEFAULT_TRAVEL_RATE_MM_S)),
            passes=uniform_passes,
            position_tolerance=float(request.get("positionTolerance", 0.003)),
            command_settle=float(request.get("commandSettle", DEFAULT_COMMAND_SETTLE_S)),
            dry_run_default=True,
            path_passes=embedded_path_passes,
            z_defocus_mm=float(metadata.get("z_defocus_mm", DEFAULT_Z_DEFOCUS_MM)),
            z_speed=float(metadata.get("z_speed_mm_s", DEFAULT_Z_SPEED_MM_S)),
            wait_mode="position",
        )
        write_preview_svg(base_paths, preview)
        self.send_json(
            {
                "scriptPath": str(output),
                "previewPath": str(preview),
                "fileName": output.name,
                "downloadUrl": f"/api/download?file={quote(output.name)}",
                "plan": plan,
            }
        )

    def handle_download(self, parsed) -> None:
        query = parse_qs(parsed.query)
        file_name = Path(query.get("file", [""])[0]).name
        target = (GENERATED_ROOT / file_name).resolve()
        generated_root = GENERATED_ROOT.resolve()
        if not str(target).startswith(str(generated_root)) or target.suffix != ".py" or not target.exists():
            self.send_error_json(404, "Generated code file not found.")
            return

        data = target.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "text/x-python; charset=utf-8")
        self.send_header("Content-Disposition", f'attachment; filename="{target.name}"')
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def handle_camera_stream(self) -> None:
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={CAMERA_BOUNDARY}")
        self.end_headers()
        interval = 1.0 / CAMERA_STREAM_FPS
        try:
            while True:
                frame = CAMERA.latest_jpeg()
                if frame is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(f"--{CAMERA_BOUNDARY}\r\n".encode("ascii"))
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                time.sleep(interval)
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            pass

    def read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length).decode("utf-8")
        return json.loads(raw or "{}")

    def send_json(self, payload: dict) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_error_json(self, status: int, message: str) -> None:
        data = json.dumps({"error": message}, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, format: str, *args) -> None:
        return


def main() -> int:
    parser = argparse.ArgumentParser(description="Serve the ESP300 motion visualizer.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    UPLOAD_ROOT.mkdir(exist_ok=True)
    GENERATED_ROOT.mkdir(exist_ok=True)
    server = ThreadingHTTPServer((args.host, args.port), VisualizerHandler)
    print(f"http://{args.host}:{args.port}", flush=True)
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
