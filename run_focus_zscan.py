"""One-off live Z-focus scan, driven entirely through the running
visualizer server's HTTP API (localhost:8766).

Why HTTP and not a direct serial connection: the ESP300's COM port is a
single exclusive connection, and the visualizer server's Jog panel already
holds it live (dryRun off). A second pyserial.Serial(COM5) from this script
would fail to open (or worse, contend with it). Driving through
/api/jog/move reuses that same open connection, and keeps the server's
position/offset bookkeeping (focus trim, local home) consistent instead of
being silently invalidated by an out-of-band raw move.

Laser is CW (continuously emitting) for this setup, so there is no pulse to
trigger per step -- brightness is read straight from
GET /api/camera/status, which reflects the visualizer's already-open UVC
camera connection.

Two-stage search:
  1. Coarse: +-2.5 mm around the current position, 0.1 mm step -- locates
     the neighborhood on a surface whose focus offset isn't known yet.
  2. Fine: +-0.05 mm around the coarse peak, 0.002 mm step -- micrometer
     refinement via scan_focus_z's parabolic fit.

All motion is RELATIVE (matches how the server's Z jog works: it trims a
local offset rather than addressing an absolute machine Z), so this script
tracks its own cumulative offset from the starting position and issues
delta moves; it never assumes or resets the server's absolute frame.

Set RUN_COARSE = False to skip straight to the fine stage once a surface's
rough focus is already known from a previous scan.
"""

import json
import time
import urllib.request

import focus_autocal

BASE_URL = "http://127.0.0.1:8766"
Z_SPEED_MM_S = 0.3

RUN_COARSE = True
# Neighborhood already known from the previous max-metric scan (real signal
# band was roughly -1.3..0.0 mm from that run's start); current position
# sits inside it, so this pass only needs to bracket that band, not the
# full +-2.5 mm blind range.
COARSE_RANGE_MM = 0.8
COARSE_STEP_MM = 0.03
FINE_RANGE_MM = 0.05
FINE_STEP_MM = 0.002
SETTLE_S = 0.15


def post_json(path: str, payload: dict) -> dict:
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        f"{BASE_URL}{path}", data=data, headers={"Content-Type": "application/json"}, method="POST"
    )
    with urllib.request.urlopen(req, timeout=5) as resp:
        return json.loads(resp.read().decode("utf-8"))


def get_json(path: str) -> dict:
    with urllib.request.urlopen(f"{BASE_URL}{path}", timeout=5) as resp:
        return json.loads(resp.read().decode("utf-8"))


class RelativeZDriver:
    """Wraps /api/jog/move so scan_focus_z's move_z(absolute_z) contract
    works over a connection that only exposes relative deltas. Tracks the
    running offset from the scan's own starting point -- 'absolute_z' here
    is scan-local (0 == wherever the scan began), not machine Z."""

    def __init__(self) -> None:
        self.current_offset = 0.0

    def move_to(self, target_offset: float) -> None:
        delta = target_offset - self.current_offset
        if abs(delta) <= 1e-9:
            return
        move_time = abs(delta) / Z_SPEED_MM_S
        post_json("/api/jog/move", {"axis": "z", "deltaMm": delta, "speedMmS": Z_SPEED_MM_S})
        time.sleep(move_time + SETTLE_S)
        self.current_offset = target_offset


def fire_pulse() -> None:
    return  # laser already emitting continuously; nothing to trigger


def read_brightness() -> float:
    data = get_json("/api/camera/status")
    brightness = data.get("brightness") or {}
    # "max" saturates at 255 across a wide plateau near true focus once the
    # ROI is tight enough to actually see the spark (verified against this
    # rig already) -- "mean" keeps discriminating within that plateau since
    # the glow's spatial extent/intensity still varies even after its
    # brightest pixel clips.
    mean_val = brightness.get("mean")
    if mean_val is None:
        raise RuntimeError("No brightness reading -- is the camera connected in the visualizer UI?")
    return float(mean_val)


def print_samples(result: focus_autocal.FocusScanResult) -> None:
    for offset, b in result.samples:
        marker = " <-- peak" if abs(offset - result.best_z) < 1e-9 else ""
        print(f"    offset={offset:+.6f} mm  brightness={b:.1f}{marker}")


def main() -> None:
    jog_status = get_json("/api/jog/status")
    if not jog_status.get("connected"):
        raise RuntimeError("Jog is not connected in the visualizer UI. Connect it (live, not dry run) first.")
    if jog_status.get("dryRun"):
        raise RuntimeError("Jog is connected in dry-run mode -- no real motion would happen. Reconnect live.")
    camera_status = get_json("/api/camera/status")
    if not camera_status.get("connected"):
        raise RuntimeError("Camera is not connected in the visualizer UI. Connect it first.")

    print(f"Starting Z (server-reported): {jog_status['position']['z']:.6f} mm")
    driver = RelativeZDriver()

    center_offset = 0.0
    if RUN_COARSE:
        print(f"Stage 1: coarse scan, +-{COARSE_RANGE_MM} mm, step {COARSE_STEP_MM} mm")
        coarse = focus_autocal.scan_focus_z(
            move_z=driver.move_to,
            fire_pulse=fire_pulse,
            read_brightness=read_brightness,
            center_z=0.0,
            range_mm=COARSE_RANGE_MM,
            step_mm=COARSE_STEP_MM,
            settle_s=0.0,  # settle already handled inside RelativeZDriver.move_to
        )
        print(f"  coarse peak offset: {coarse.best_z:+.6f} mm")
        print_samples(coarse)
        center_offset = coarse.best_z

    print(f"Stage 2: fine scan, +-{FINE_RANGE_MM} mm, step {FINE_STEP_MM} mm")
    fine = focus_autocal.scan_focus_z(
        move_z=driver.move_to,
        fire_pulse=fire_pulse,
        read_brightness=read_brightness,
        center_z=center_offset,
        range_mm=FINE_RANGE_MM,
        step_mm=FINE_STEP_MM,
        settle_s=0.0,
    )
    print(f"  fine focus offset: {fine.best_z:+.6f} mm (parabolic fit used={fine.fit_used})")
    print_samples(fine)

    driver.move_to(fine.best_z)
    final_status = get_json("/api/jog/status")
    print(f"Moved to detected focus offset {fine.best_z:+.6f} mm from scan start.")
    print(f"Final Z (server-reported): {final_status['position']['z']:.6f} mm")


if __name__ == "__main__":
    main()
