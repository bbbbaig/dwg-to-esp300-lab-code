"""Bidirectional fine Z scan around the current position: +-1 mm, 0.01 mm step.

Driven through the running visualizer server's HTTP API (COM5 and the
camera are both already held open by that process -- see
run_focus_zscan.py for why a direct serial/cv2 connection isn't used).

Uses "mean" ROI brightness (not "max"): max saturates at 255 across a wide
band near real signal on this rig (confirmed earlier this session), mean
keeps discriminating through that band.
"""

import json
import time
import urllib.request

import focus_autocal

BASE_URL = "http://127.0.0.1:8766"
RANGE_MM = 2.0
STEP_MM = 0.01
SPEED_MM_S = 0.3
SETTLE_S = 0.1


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
    def __init__(self) -> None:
        self.current_offset = 0.0

    def move_to(self, target_offset: float) -> None:
        delta = target_offset - self.current_offset
        if abs(delta) <= 1e-9:
            return
        move_time = abs(delta) / SPEED_MM_S
        post_json("/api/jog/move", {"axis": "z", "deltaMm": delta, "speedMmS": SPEED_MM_S})
        time.sleep(move_time + SETTLE_S)
        self.current_offset = target_offset


def fire_pulse() -> None:
    return  # laser is CW, already emitting


def read_brightness() -> float:
    data = get_json("/api/camera/status")
    brightness = data.get("brightness") or {}
    mean_val = brightness.get("mean")
    if mean_val is None:
        raise RuntimeError("No brightness reading -- is the camera connected?")
    return float(mean_val)


def main() -> None:
    jog_status = get_json("/api/jog/status")
    if not jog_status.get("connected") or jog_status.get("dryRun"):
        raise RuntimeError("Jog must be connected live (not dry run).")
    camera_status = get_json("/api/camera/status")
    if not camera_status.get("connected"):
        raise RuntimeError("Camera must be connected.")

    print(f"Start Z (server-reported): {jog_status['position']['z']:.6f} mm")
    print(f"Scanning +-{RANGE_MM} mm around here, step {STEP_MM} mm")

    driver = RelativeZDriver()
    result = focus_autocal.scan_focus_z(
        move_z=driver.move_to,
        fire_pulse=fire_pulse,
        read_brightness=read_brightness,
        center_z=0.0,
        range_mm=RANGE_MM,
        step_mm=STEP_MM,
        settle_s=0.0,  # settle handled inside RelativeZDriver.move_to
    )

    print(f"\nPeak offset: {result.best_z:+.4f} mm (parabolic fit used={result.fit_used})")
    offsets = [z for z, _ in result.samples]
    values = [b for _, b in result.samples]
    baseline = sorted(values)[: max(1, len(values) // 10)]
    baseline_avg = sum(baseline) / len(baseline)
    peak_val = max(values)
    print(f"Baseline (lowest 10%) avg brightness: {baseline_avg:.1f}")
    print(f"Peak brightness: {peak_val:.1f}  (ratio to baseline: {peak_val / max(baseline_avg, 0.1):.2f}x)")

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(offsets, values, color="tab:blue")
        ax.axvline(result.best_z, color="tab:red", linestyle="--", label=f"peak {result.best_z:+.4f} mm")
        ax.set_xlabel("Offset from current position (mm)")
        ax.set_ylabel("ROI mean brightness")
        ax.set_title(f"Local fine scan: +-{RANGE_MM} mm, step {STEP_MM} mm")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig("local_fine_scan.png", dpi=150)
        print("Saved plot: local_fine_scan.png")
    except ImportError:
        pass

    print("\nReturning to the starting position (no move-to-peak -- reporting only, as requested).")
    driver.move_to(0.0)
    print(f"Back at start. Z (server-reported): {get_json('/api/jog/status')['position']['z']:.6f} mm")


if __name__ == "__main__":
    main()
