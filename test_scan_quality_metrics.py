"""Self-check for the auto-collected data added 2026-09-09: focus-scan
quality metrics (_scan_quality_metrics) and the shared CSV appender
(_append_csv_row) that FOCUS_QUALITY_LOG_PATH / CAMERA_FREEZE_LOG_PATH /
SERIAL_RETRY_LOG_PATH all go through. Run directly: `python
test_scan_quality_metrics.py`. No pytest -- this repo has no existing test
harness, just assert-based smoke checks like this one.
"""

import csv
import tempfile
from pathlib import Path

from visualizer_server import _append_csv_row, _scan_quality_metrics


def test_scan_quality_metrics() -> None:
    # Empty scan (e.g. an aborted stage) -- must not divide by zero.
    empty = _scan_quality_metrics([])
    assert empty == {"n": 0, "zeroCount": 0, "zeroPct": 0.0, "roughness": 0.0}

    # Perfectly flat, no zeros: 0% dropout, 0 roughness (no variation at all).
    flat = _scan_quality_metrics([(0.0, 100.0), (0.01, 100.0), (0.02, 100.0)])
    assert flat["zeroCount"] == 0 and flat["zeroPct"] == 0.0
    assert flat["roughness"] == 0.0

    # The exact failure mode this metric exists to catch: half the samples
    # hard-dropped to 0.0 between real readings -- the 2026-09-09 rig showed
    # 34.1% before the fix, 0% after.
    jagged = _scan_quality_metrics([(0.0, 0.0), (0.01, 100.0), (0.02, 0.0), (0.03, 100.0)])
    assert jagged["n"] == 4
    assert jagged["zeroCount"] == 2
    assert jagged["zeroPct"] == 50.0
    assert jagged["roughness"] == 3.0  # (100+100+100)/100 == 3.0

    # A smooth ramp up to the peak should score far rougher-immune than the
    # jagged case above at the same peak and sample count.
    smooth = _scan_quality_metrics([(0.0, 0.0), (0.01, 50.0), (0.02, 100.0), (0.03, 50.0)])
    assert smooth["zeroCount"] == 1  # one legitimate baseline zero, not a dropout run
    assert smooth["roughness"] < jagged["roughness"]


def test_append_csv_row_writes_header_once() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "log.csv"
        header = ["timestamp_utc", "value"]
        _append_csv_row(path, header, ["2026-09-09T00:00:00+00:00", 1])
        _append_csv_row(path, header, ["2026-09-09T00:00:05+00:00", 2])

        with path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.reader(handle))

        assert rows[0] == header  # header written exactly once, not per row
        assert len(rows) == 3
        assert rows[1][1] == "1" and rows[2][1] == "2"


if __name__ == "__main__":
    test_scan_quality_metrics()
    test_append_csv_row_writes_header_once()
    print("OK")
