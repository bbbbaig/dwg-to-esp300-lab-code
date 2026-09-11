"""2026-09-11 repeat-scan run: CV autofocus every INTERVAL_S for DURATION_S,
tracking how much the found focus offset drifts as the environment (heat,
vibration, whatever) acts on the real rig. Run directly:
`python repeat_focus_scan.py`.

Each scan is already logged server-side (focus_scan_log.csv) -- this script
only paces the repeats, saves a snapshot photo + z-scan graph per cycle
(레이저 세라믹 가공/실험 기록 매뉴얼.md convention), and produces one drift-over-time
graph at the end from the offsets it collected.
"""
import json
import time
import urllib.request
from datetime import datetime
from pathlib import Path

from plot_experiment import plot_drift_over_time, plot_z_scan

BASE = "http://127.0.0.1:8768"
INTERVAL_S = 5 * 60
DURATION_S = 60 * 60
PHOTO_DIR = Path(r"C:\Users\bjh14\Documents\Obsidian Vault\레이저 세라믹 가공\9.11 실험 사진")


def _post(path: str, payload: dict) -> dict:
    data = json.dumps(payload).encode()
    req = urllib.request.Request(BASE + path, data=data, headers={"Content-Type": "application/json"}, method="POST")
    return json.loads(urllib.request.urlopen(req, timeout=10).read())


def _get(path: str) -> dict:
    return json.loads(urllib.request.urlopen(BASE + path, timeout=10).read())


def run_scan() -> dict:
    _post("/api/focus/scan", {"narrowOnly": True})
    while True:
        time.sleep(2)
        status = _get("/api/focus/status")
        if status["error"]:
            raise RuntimeError(status["error"])
        if not status["scanning"]:
            return status["lastResult"]


def _extract_first_jpeg(buf: bytes) -> bytes | None:
    """Pull one full JPEG (SOI..EOI) out of raw MJPEG multipart bytes, or
    None if `buf` doesn't contain a complete frame yet."""
    start = buf.find(b"\xff\xd8")
    if start < 0:
        return None
    end = buf.find(b"\xff\xd9", start + 2)
    if end < 0:
        return None
    return buf[start : end + 2]


def grab_snapshot(out_path: Path, max_bytes: int = 400_000) -> None:
    """No single-shot endpoint exists -- read /api/camera/stream (MJPEG)
    until one full frame is assembled, then stop. Good enough for a
    once-per-cycle photo, not a real stream client."""
    with urllib.request.urlopen(BASE + "/api/camera/stream", timeout=10) as resp:
        buf = b""
        while len(buf) < max_bytes:
            buf += resp.read(4096)
            frame = _extract_first_jpeg(buf)
            if frame is not None:
                out_path.write_bytes(frame)
                return
    raise RuntimeError("no full JPEG frame found in stream")


def main() -> None:
    PHOTO_DIR.mkdir(parents=True, exist_ok=True)
    run_tag = datetime.now().strftime("%H%M")
    records: list[tuple[float, float]] = []  # (elapsed_min, best_offset_mm)
    started = time.monotonic()
    i = 0
    while True:
        i += 1
        elapsed_min = (time.monotonic() - started) / 60
        result = run_scan()
        records.append((elapsed_min, result["bestOffsetMm"]))

        grab_snapshot(PHOTO_DIR / f"scan_{run_tag}_{i:02d}.jpg")
        plot_z_scan(
            result["samples"]["narrow"],
            PHOTO_DIR / f"scan_{run_tag}_{i:02d}_curve.png",
            peak_z=result["bestOffsetMm"],
            title=f"Focus scan #{i} (t={elapsed_min:.1f}min)",
        )
        print(f"[{i}] t={elapsed_min:.1f}min offset={result['bestOffsetMm']:.4f}mm peak={result['peakBrightness']:.0f}")

        elapsed_s = time.monotonic() - started
        if elapsed_s >= DURATION_S:
            break
        time.sleep(min(INTERVAL_S, DURATION_S - elapsed_s))

    plot_drift_over_time(records, PHOTO_DIR / f"drift_{run_tag}.png", title=f"Focus drift over {DURATION_S // 60}min ({run_tag})")
    print("done ->", PHOTO_DIR)


if __name__ == "__main__":
    main()
