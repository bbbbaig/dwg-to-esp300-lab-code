"""Forward-only Z diagnostic sweep: 0.1 mm steps, up to 10 mm total.

Purpose: verify the Z axis is actually moving as commanded (log the
server-reported position after every step, not just assume commanded ==
actual) and produce a plain brightness-vs-Z plot over a wide range so the
real curve shape can be inspected visually instead of guessed at from
narrow automated scans.

Driven through the running visualizer server's HTTP API (same reasoning as
run_focus_zscan.py: COM5 and the camera are both already held open by that
process, so this reuses those connections instead of opening new ones).

No return-to-start move at the end -- this only goes forward, as requested.
"""

import json
import time
import urllib.request
import urllib.error

BASE_URL = "http://127.0.0.1:8766"
STEP_MM = 0.1
MAX_TOTAL_MM = 10.0
SPEED_MM_S = 0.3
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


def main() -> None:
    jog_status = get_json("/api/jog/status")
    if not jog_status.get("connected") or jog_status.get("dryRun"):
        raise RuntimeError("Jog must be connected live (not dry run).")
    camera_status = get_json("/api/camera/status")
    if not camera_status.get("connected"):
        raise RuntimeError("Camera must be connected.")

    start_z = jog_status["position"]["z"]
    start_trim = jog_status["focusTrimMm"]
    print(f"Start Z (server-reported, does NOT move on Z-jog by design): {start_z:.6f} mm")
    print(f"Start focus trim (this DOES reflect real Z jog movement): {start_trim:.6f} mm")
    print(f"Sweeping forward: step {STEP_MM} mm, up to {MAX_TOTAL_MM} mm total")

    n_steps = int(round(MAX_TOTAL_MM / STEP_MM))
    move_time = STEP_MM / SPEED_MM_S

    commanded_offsets: list[float] = [0.0]
    actual_trim: list[float] = [start_trim]
    brightness_mean: list[float] = [camera_status["brightness"].get("mean") or 0.0]
    brightness_max: list[float] = [camera_status["brightness"].get("max") or 0.0]

    stopped_early = None
    for i in range(1, n_steps + 1):
        try:
            move_result = post_json("/api/jog/move", {"axis": "z", "deltaMm": STEP_MM, "speedMmS": SPEED_MM_S})
        except urllib.error.HTTPError as exc:
            stopped_early = f"move rejected at step {i} ({exc.read().decode('utf-8', errors='ignore')})"
            break
        time.sleep(move_time + SETTLE_S)

        cam = get_json("/api/camera/status")
        commanded_offsets.append(i * STEP_MM)
        actual_trim.append(move_result["focusTrimMm"])
        brightness_mean.append(cam["brightness"].get("mean") or 0.0)
        brightness_max.append(cam["brightness"].get("max") or 0.0)
        print(
            f"  step {i:3d}  commanded_offset={i * STEP_MM:+.2f} mm  "
            f"actual_trim={move_result['focusTrimMm']:.6f} mm  "
            f"brightness mean={brightness_mean[-1]:.1f} max={brightness_max[-1]:.1f}"
        )

    if stopped_early:
        print(f"Stopped early: {stopped_early}")

    actual_delta = actual_trim[-1] - actual_trim[0]
    print(f"\nTotal commanded forward distance: {commanded_offsets[-1]:.2f} mm")
    print(f"Total actual Z movement (focus trim delta): {actual_delta:.6f} mm")
    if abs(actual_delta - commanded_offsets[-1]) > 0.01:
        print("  MISMATCH: actual movement does not match what was commanded.")

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax1.plot(commanded_offsets, brightness_mean, label="mean", color="tab:blue")
        ax1.plot(commanded_offsets, brightness_max, label="max", color="tab:red", alpha=0.6)
        ax1.set_ylabel("ROI brightness")
        ax1.set_title("Brightness vs. commanded Z offset (forward sweep)")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        ax2.plot(commanded_offsets, commanded_offsets, "--", color="gray", alpha=0.5, label="ideal (1:1)")
        ax2.plot(commanded_offsets, [t - actual_trim[0] for t in actual_trim], color="tab:green", label="actual trim")
        ax2.set_xlabel("Commanded forward offset (mm)")
        ax2.set_ylabel("Actual Z movement (mm)")
        ax2.set_title("Actual Z movement vs. commanded offset (movement sanity check)")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        fig.tight_layout()
        out_path = "zaxis_diagnostic.png"
        fig.savefig(out_path, dpi=150)
        print(f"\nSaved plot: {out_path}")
    except ImportError:
        print("\nmatplotlib not available -- skipped plot, raw data printed above.")


if __name__ == "__main__":
    main()
