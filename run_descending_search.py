"""One-directional descending Z search: -0.1 mm steps only, 0.5 s dwell per
step for the beam/camera to settle before reading brightness. Stops as soon
as a real peak is detected (brightness rose well above baseline, then fell
back down) -- does not keep going past it. Hard safety cap at 10 mm total
travel in case no peak shows up.

Driven through the running visualizer server's HTTP API (see
run_focus_zscan.py for why: COM5 and the camera are both already held open
by that process).
"""

import json
import time
import urllib.request

BASE_URL = "http://127.0.0.1:8766"
STEP_MM = 0.1
DWELL_S = 0.5
SPEED_MM_S = 0.3
MAX_TOTAL_MM = 10.0
PEAK_RISE_RATIO = 1.4   # running max must exceed baseline by this much to count as a real bump
PEAK_FALL_FRACTION = 0.7  # current reading must drop below this fraction of the running max to call it "past the peak"


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


def read_mean_brightness() -> float:
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

    start_z = jog_status["position"]["z"]
    baseline = read_mean_brightness()
    print(f"Start Z (server-reported): {start_z:.6f} mm")
    print(f"Baseline brightness: {baseline:.1f}")
    print(f"Descending only, step {STEP_MM} mm, dwell {DWELL_S}s, cap {MAX_TOTAL_MM} mm")

    offsets = [0.0]
    values = [baseline]
    running_max = baseline
    running_max_offset = 0.0
    peak_found = None
    move_time = STEP_MM / SPEED_MM_S
    n_steps = int(round(MAX_TOTAL_MM / STEP_MM))

    for i in range(1, n_steps + 1):
        post_json("/api/jog/move", {"axis": "z", "deltaMm": -STEP_MM, "speedMmS": SPEED_MM_S})
        time.sleep(move_time + DWELL_S)

        offset = -i * STEP_MM
        value = read_mean_brightness()
        offsets.append(offset)
        values.append(value)
        print(f"  offset={offset:+.2f} mm  brightness={value:.1f}")

        if value > running_max:
            running_max = value
            running_max_offset = offset

        if running_max > baseline * PEAK_RISE_RATIO and value < running_max * PEAK_FALL_FRACTION:
            peak_found = running_max_offset
            print(f"\nPeak detected at offset {peak_found:+.2f} mm (brightness {running_max:.1f}), stopping.")
            break
    else:
        print(f"\nNo clear peak within {MAX_TOTAL_MM} mm cap. Highest seen: {running_max:.1f} at {running_max_offset:+.2f} mm.")

    final_status = get_json("/api/jog/status")
    print(f"Final Z (server-reported): {final_status['position']['z']:.6f} mm")
    print("No return-to-start move issued -- staying here, as this was a one-directional search.")

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(offsets, values, color="tab:blue", marker="o", markersize=3)
        if peak_found is not None:
            ax.axvline(peak_found, color="tab:red", linestyle="--", label=f"peak {peak_found:+.2f} mm")
            ax.legend()
        ax.set_xlabel("Offset from search start (mm, negative = descending)")
        ax.set_ylabel("ROI mean brightness")
        ax.set_title("Descending Z search (-0.1 mm steps, 0.5s dwell)")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig("descending_search.png", dpi=150)
        print("Saved plot: descending_search.png")
    except ImportError:
        pass


if __name__ == "__main__":
    main()
