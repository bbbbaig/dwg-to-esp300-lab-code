"""One-off live Z-focus scan, driven entirely through the running
visualizer server's HTTP API (localhost:8768).

Why HTTP and not a direct serial connection: the ESP300's COM port is a
single exclusive connection, and the visualizer server's Jog panel already
holds it live (dryRun off). A second pyserial.Serial(COM5) from this script
would fail to open (or worse, contend with it). Driving through
/api/jog/move reuses that same open connection, and keeps the server's
position/offset bookkeeping (focus trim, local home) consistent instead of
being silently invalidated by an out-of-band raw move.

This drives the server's own /api/focus/scan/custom endpoint rather than
re-walking Z and re-reading brightness itself (an earlier version of this
script did that, via /api/camera/status's whole-ROI mean/max). That path
skipped everything the server-side FocusScanner already does to keep the
curve clean: the tracked red-glow+saturated-core blob score instead of
whole-frame mean, a wait for a frame *timestamped after* the move settles
instead of a blind sleep, and a median-of-N-fresh-frames read instead of a
single frame -- see FocusScanner._read_brightness in visualizer_server.py.
Re-implementing a second, worse version of that per script is exactly the
kind of drift that made the graph jagged in the first place; call the one
that's already right.

Two-stage search:
  1. Coarse: +-2.5 mm around the current position, 0.1 mm step -- locates
     the neighborhood on a surface whose focus offset isn't known yet.
  2. Fine: +-0.05 mm around the coarse peak, 0.002 mm step -- micrometer
     refinement via scan_focus_z's parabolic fit.

Set RUN_COARSE = False to skip straight to the fine stage once a surface's
rough focus is already known from a previous scan (fine then scans around
wherever the stage already sits).
"""

import json
import time
import urllib.error
import urllib.request

BASE_URL = "http://127.0.0.1:8768"

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
POLL_INTERVAL_S = 0.2


def post_json(path: str, payload: dict) -> dict:
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        f"{BASE_URL}{path}", data=data, headers={"Content-Type": "application/json"}, method="POST"
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        raise RuntimeError(json.loads(exc.read().decode("utf-8")).get("error", str(exc))) from exc


def get_json(path: str) -> dict:
    with urllib.request.urlopen(f"{BASE_URL}{path}", timeout=5) as resp:
        return json.loads(resp.read().decode("utf-8"))


def run_scan(range_mm: float, step_mm: float, settle_s: float, center_offset_mm: float = 0.0) -> dict:
    """Kick off a custom focus scan and block until it finishes, returning
    FocusScanner.last_result (see visualizer_server.py's _finish_scan)."""
    post_json(
        "/api/focus/scan/custom",
        {"rangeMm": range_mm, "stepMm": step_mm, "direction": "both", "settleS": settle_s, "centerOffsetMm": center_offset_mm},
    )
    while True:
        status = get_json("/api/focus/status")
        if not status["scanning"]:
            if status.get("error"):
                raise RuntimeError(status["error"])
            return status["lastResult"]
        time.sleep(POLL_INTERVAL_S)


def print_samples(samples: list[list[float]], best_offset: float) -> None:
    for offset, score in samples:
        marker = " <-- peak" if abs(offset - best_offset) < 1e-9 else ""
        print(f"    offset={offset:+.6f} mm  score={score:.1f}{marker}")


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

    if RUN_COARSE:
        print(f"Stage 1: coarse scan, +-{COARSE_RANGE_MM} mm, step {COARSE_STEP_MM} mm")
        coarse = run_scan(COARSE_RANGE_MM, COARSE_STEP_MM, SETTLE_S)
        print(f"  coarse peak offset: {coarse['bestOffsetMm']:+.6f} mm")
        print_samples(coarse["samples"]["narrow"], coarse["bestOffsetMm"])

    # run_custom always leaves the stage at that scan's own detected peak
    # (see FocusScanner._run_stage), so the fine pass below is already
    # centered on the coarse peak -- no extra centering move needed here.
    print(f"Stage 2: fine scan, +-{FINE_RANGE_MM} mm, step {FINE_STEP_MM} mm")
    fine = run_scan(FINE_RANGE_MM, FINE_STEP_MM, SETTLE_S)
    print(f"  fine focus offset (from fine scan's own start): {fine['bestOffsetMm']:+.6f} mm (parabolic fit used={fine['fitUsed']})")
    print_samples(fine["samples"]["narrow"], fine["bestOffsetMm"])

    final_status = get_json("/api/jog/status")
    print(f"Final Z (server-reported): {final_status['position']['z']:.6f} mm")


if __name__ == "__main__":
    main()
