"""Self-check for repeat_focus_scan._extract_first_jpeg -- the MJPEG
byte-stream parsing added 2026-09-11 for the once-per-cycle snapshot photo.
Run directly: `python test_repeat_focus_scan_helpers.py`. No pytest, same
assert-based style as test_scan_quality_metrics.py.
"""
from repeat_focus_scan import _extract_first_jpeg


def test_extract_first_jpeg() -> None:
    # No frame at all yet.
    assert _extract_first_jpeg(b"") is None
    assert _extract_first_jpeg(b"garbage bytes, no markers here") is None

    # A real frame is SOI (FFD8) .. EOI (FFD9); multipart boundary/header
    # text can come before it and more stream bytes can trail after.
    frame = b"\xff\xd8fake-jpeg-bytes\xff\xd9"
    buf = b"--boundary\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n--boundary\r\n...next part not complete"
    assert _extract_first_jpeg(buf) == frame

    # Only the first, complete frame is returned even if a second one
    # (or the start of one) trails in the same buffer.
    two_frames = frame + b"junk" + b"\xff\xd8partial-next-frame-no-eoi-yet"
    assert _extract_first_jpeg(two_frames) == frame

    # SOI present but EOI hasn't arrived yet (frame still streaming in).
    assert _extract_first_jpeg(b"\xff\xd8still-arriving") is None


if __name__ == "__main__":
    test_extract_first_jpeg()
    print("OK")
