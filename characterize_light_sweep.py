"""Exploratory: sweep one XY axis across the ceramic edge and record a
richer light-signal bundle per step than the existing mean/max, so the
edge-detection method for auto center-finding can be picked from real data
instead of guessed. See 레이저 세라믹 가공/9.11 실험 세팅.md 0단계.

Not run against hardware yet -- prepped ahead of time (ponytail: this loop
+ per-step signal math is non-trivial enough to write once calmly instead
of live under time pressure tomorrow). Run it manually once the ceramic is
actually in frame; it does NOT auto-search for the ceramic.

Records per step (CSV + one raw JPEG):
  - grayscale mean/max/p50/p90/p99 (percentiles catch a rising bump in the
    upper tail before the plain mean moves -- same lesson as the focus-score
    fix on 2026-09-09, where mean-only missed the real signal shape)
  - HSV channel means -- scattered light may have a distinct hue/saturation
    signature vs. the focused red+white flash the existing blob detector
    is tuned for
  - Laplacian variance -- a standard sharpness/edge-density metric; scatter
    often shows as increased local texture before mean brightness moves
  - std across N repeat frames at the same position (repeatability/noise floor)

Safety: goto() calls the server's own /api/jog/goto, which enforces the
XY radius / Z range caps server-side before any serial command goes out
(same as every other move in this codebase) -- this script adds no motion
path that bypasses that.
"""
import argparse
import csv
import json
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

BASE = "http://127.0.0.1:8768"


def api(path, payload=None):
    req = urllib.request.Request(
        BASE + path,
        data=json.dumps(payload).encode() if payload is not None else None,
        headers={"Content-Type": "application/json"},
        method="POST" if payload is not None else "GET",
    )
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.load(r)


def grab_frame():
    req = urllib.request.urlopen(BASE + "/api/camera/stream", timeout=5)
    buf = b""
    while len(buf) < 400_000:
        chunk = req.read(65536)
        if not chunk:
            break
        buf += chunk
        if buf.count(b"\xff\xd8\xff") >= 1 and buf.count(b"\xff\xd9") >= 1 and len(buf) > 20000:
            break
    req.close()
    start = buf.find(b"\xff\xd8\xff")
    end = buf.find(b"\xff\xd9", start)
    jpeg = buf[start:end + 2]
    arr = np.frombuffer(jpeg, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR), jpeg


def bundle_at_point(n_repeats=3, settle_between_s=0.05):
    grays, hsvs, laplacians = [], [], []
    last_jpeg = None
    for _ in range(n_repeats):
        frame, jpeg = grab_frame()
        last_jpeg = jpeg
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        grays.append(gray)
        hsvs.append(hsv)
        laplacians.append(float(cv2.Laplacian(gray, cv2.CV_64F).var()))
        time.sleep(settle_between_s)
    gray_stack = np.stack(grays).astype(np.float64)
    mean_gray = gray_stack.mean(axis=0)
    return {
        "mean": float(mean_gray.mean()),
        "max": float(mean_gray.max()),
        "p50": float(np.percentile(mean_gray, 50)),
        "p90": float(np.percentile(mean_gray, 90)),
        "p99": float(np.percentile(mean_gray, 99)),
        "h_mean": float(np.mean([h[:, :, 0] for h in hsvs])),
        "s_mean": float(np.mean([h[:, :, 1] for h in hsvs])),
        "v_mean": float(np.mean([h[:, :, 2] for h in hsvs])),
        "laplacian_var_mean": float(np.mean(laplacians)),
        "laplacian_var_std": float(np.std(laplacians)),
        "gray_std_across_repeats": float(gray_stack.std(axis=0).mean()),
    }, last_jpeg


def sweep(axis, start, end, step, x_fixed, y_fixed, z_fixed, out_dir, settle_s=0.3, n_repeats=3):
    before = api("/api/jog/status")
    print(f"current position before sweep: {before['position']}, limit: {before['limit']}")

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    direction = 1 if end >= start else -1
    pos = start
    while (pos <= end) if direction > 0 else (pos >= end):
        x = pos if axis == "x" else x_fixed
        y = pos if axis == "y" else y_fixed
        api("/api/jog/goto", {"x": x, "y": y, "z": z_fixed})
        time.sleep(settle_s)

        stats, jpeg = bundle_at_point(n_repeats)
        actual = api("/api/jog/status")
        stats["pos_mm"] = pos
        stats["actual_x"] = actual["position"]["x"]
        stats["actual_y"] = actual["position"]["y"]
        rows.append(stats)

        (out_dir / f"{axis}_{pos:+.3f}mm.jpg").write_bytes(jpeg)
        print(f"{axis}={pos:+.3f}mm  mean={stats['mean']:.1f} p99={stats['p99']:.1f} "
              f"lap={stats['laplacian_var_mean']:.1f} sat={stats['s_mean']:.1f}")

        pos = round(pos + direction * step, 6)

    csv_path = out_dir / f"{axis}_sweep.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print("saved", csv_path, f"({len(rows)} points)")
    return rows


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--axis", choices=["x", "y"], required=True)
    p.add_argument("--start", type=float, required=True, help="mm, sweep-axis start")
    p.add_argument("--end", type=float, required=True, help="mm, sweep-axis end")
    p.add_argument("--step", type=float, default=0.5)
    p.add_argument("--x-fixed", type=float, required=True, help="mm, held constant if sweeping y")
    p.add_argument("--y-fixed", type=float, required=True, help="mm, held constant if sweeping x")
    p.add_argument("--z-fixed", type=float, required=True, help="mm, held constant -- pass the current known-safe z")
    p.add_argument("--out-dir", required=True)
    p.add_argument("--settle-s", type=float, default=0.3)
    p.add_argument("--repeats", type=int, default=3)
    args = p.parse_args()
    sweep(args.axis, args.start, args.end, args.step, args.x_fixed, args.y_fixed, args.z_fixed,
          args.out_dir, args.settle_s, args.repeats)
