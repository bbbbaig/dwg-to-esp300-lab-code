"""One-shot focus check for experiment transitions (not a timer) -- run this
right before/after moving from one step of today's plan to the next, e.g.:
    python check_focus.py before-priority2
    python check_focus.py after-priority2

Does one narrow CV autofocus scan, saves a snapshot + z-scan graph tagged
with the label, and prints the drift vs the last logged scan (server already
appends every scan to focus_scan_log.csv -- this just diffs against that).
"""
import csv
import sys
from datetime import datetime
from pathlib import Path

from plot_experiment import plot_z_scan
from repeat_focus_scan import grab_snapshot, run_scan

LOG_PATH = Path(__file__).parent / "focus_scan_log.csv"
PHOTO_DIR = Path(r"C:\Users\bjh14\Documents\Obsidian Vault\레이저 세라믹 가공\9.11 실험 사진")


def _last_logged_offset() -> float | None:
    if not LOG_PATH.exists():
        return None
    with LOG_PATH.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return float(rows[-1]["best_offset_mm"]) if rows else None


def main() -> None:
    label = sys.argv[1] if len(sys.argv) > 1 else "check"
    prev_offset = _last_logged_offset()

    result = run_scan()
    offset = result["bestOffsetMm"]

    PHOTO_DIR.mkdir(parents=True, exist_ok=True)
    tag = f"{label}_{datetime.now().strftime('%H%M%S')}"
    grab_snapshot(PHOTO_DIR / f"{tag}.jpg")
    plot_z_scan(result["samples"]["narrow"], PHOTO_DIR / f"{tag}_curve.png", peak_z=offset, title=f"Focus check: {label}")

    if prev_offset is None:
        print(f"[{label}] offset={offset:.4f}mm (no prior scan to compare)")
    else:
        drift = offset - prev_offset
        print(f"[{label}] offset={offset:.4f}mm  prev={prev_offset:.4f}mm  drift={drift:+.4f}mm")


if __name__ == "__main__":
    main()
