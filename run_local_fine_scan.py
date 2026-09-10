"""Bidirectional fine Z scan around the current position: +-2 mm, 0.01 mm
step. Report-only -- moves back to the starting position afterward.

Driven through the running visualizer server's HTTP API (COM5 and the
camera are both already held open by that process -- see
run_focus_zscan.py for why a direct serial/cv2 connection isn't used).

Uses the server's own /api/focus/scan/custom endpoint instead of reading
/api/camera/status's whole-ROI brightness directly: that endpoint already
does the tracked-blob score + fresh-frame wait + median-of-N-samples that
keeps the curve from being dominated by single-frame dropouts (a earlier
version of this script read raw ROI "mean" once per step with a blind
sleep, which is what made the graph jagged -- see visualizer_server.py's
FocusScanner._read_brightness).
"""

import json
import time
import urllib.error
import urllib.request

BASE_URL = "http://127.0.0.1:8768"
RANGE_MM = 2.0
STEP_MM = 0.01
SETTLE_S = 0.1
POLL_INTERVAL_S = 0.2
Z_SPEED_MM_S = 0.3


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


def main() -> None:
    jog_status = get_json("/api/jog/status")
    if not jog_status.get("connected") or jog_status.get("dryRun"):
        raise RuntimeError("Jog must be connected live (not dry run).")
    camera_status = get_json("/api/camera/status")
    if not camera_status.get("connected"):
        raise RuntimeError("Camera must be connected.")

    print(f"Start Z (server-reported): {jog_status['position']['z']:.6f} mm")
    print(f"Scanning +-{RANGE_MM} mm around here, step {STEP_MM} mm")

    post_json(
        "/api/focus/scan/custom",
        {"rangeMm": RANGE_MM, "stepMm": STEP_MM, "direction": "both", "settleS": SETTLE_S, "centerOffsetMm": 0.0},
    )
    while True:
        status = get_json("/api/focus/status")
        if not status["scanning"]:
            if status.get("error"):
                raise RuntimeError(status["error"])
            result = status["lastResult"]
            break
        time.sleep(POLL_INTERVAL_S)

    best_offset = result["bestOffsetMm"]
    samples = result["samples"]["narrow"]
    print(f"\nPeak offset: {best_offset:+.4f} mm (parabolic fit used={result['fitUsed']})")
    offsets = [z for z, _ in samples]
    values = [v for _, v in samples]
    baseline = sorted(values)[: max(1, len(values) // 10)]
    baseline_avg = sum(baseline) / len(baseline)
    peak_val = max(values)
    print(f"Baseline (lowest 10%) avg score: {baseline_avg:.1f}")
    print(f"Peak score: {peak_val:.1f}  (ratio to baseline: {peak_val / max(baseline_avg, 0.1):.2f}x)")

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(offsets, values, color="tab:blue")
        ax.axvline(best_offset, color="tab:red", linestyle="--", label=f"peak {best_offset:+.4f} mm")
        ax.set_xlabel("Offset from current position (mm)")
        ax.set_ylabel("Tracked-blob score (median of fresh frames)")
        ax.set_title(f"Local fine scan: +-{RANGE_MM} mm, step {STEP_MM} mm")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig("local_fine_scan.png", dpi=150)
        print("Saved plot: local_fine_scan.png")
    except ImportError:
        pass

    # /api/focus/scan/custom leaves the stage at the scan's own detected
    # peak (not the last sample), so "back to start" is a plain relative
    # move of -best_offset, not the sample-walk RelativeZDriver used to do.
    print("\nReturning to the starting position (report-only, as requested).")
    move_time = abs(best_offset) / Z_SPEED_MM_S
    post_json("/api/jog/move", {"axis": "z", "deltaMm": -best_offset, "speedMmS": Z_SPEED_MM_S})
    time.sleep(move_time + SETTLE_S)
    print(f"Back at start. Z (server-reported): {get_json('/api/jog/status')['position']['z']:.6f} mm")


if __name__ == "__main__":
    main()
