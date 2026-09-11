"""Self-check for the 2026-09-11 fix to locate_bright_regions(): a
distance gate and a noise-area gate on the prior_xy nearest-match, added to
stop the reported spike (real target briefly "jumps" to a small stray blob
just because that blob happens to sit closest to the previous frame's
position). Run directly: `python test_locate_bright_regions_gate.py`. No
pytest -- same assert-based smoke-check style as test_scan_quality_metrics.py.
"""

import numpy as np

from visualizer_server import locate_bright_regions


def _blob(frame: np.ndarray, center: tuple[int, int], red_radius: int, core_radius: int | None = None) -> None:
    import cv2

    cv2.circle(frame, center, red_radius, (0, 0, 255), -1)  # BGR red -> hue 0
    if core_radius:
        cv2.circle(frame, center, core_radius, (255, 255, 255), -1)  # white -> gray 255


def test_area_gate_ignores_small_noise_blob_near_prior() -> None:
    # No core anywhere -> hits the red-only fallback branch. A tiny noise
    # blob sits exactly on the prior position; the only sizeable red glow is
    # far away. Old code (nearest-by-distance, no area floor) would lock
    # onto the noise blob since it's closest -- that's the reported bug.
    frame = np.zeros((300, 300, 3), dtype=np.uint8)
    _blob(frame, (50, 50), red_radius=2)  # area ~12px, below MIN_CANDIDATE_AREA_PX (16)
    _blob(frame, (200, 200), red_radius=8)  # area ~200px, the real (off-focus) glow

    regions = locate_bright_regions(frame, prior_xy=(50, 50))

    assert regions["x"] is not None
    assert abs(regions["x"] - 200) < 5 and abs(regions["y"] - 200) < 5


def test_distance_gate_falls_back_to_area_when_nearest_is_implausible() -> None:
    # Both blobs have a saturated core -> hits the candidates branch. prior_xy
    # is far from both (e.g. stage just jumped, or a frame was dropped), and
    # the small spurious blob happens to be the nearer of the two. Old code
    # (nearest-by-distance, no gate) would lock onto that spurious blob just
    # for being closer; the gate should reject it and fall back to the
    # larger, real target instead.
    frame = np.zeros((400, 400, 3), dtype=np.uint8)
    _blob(frame, (150, 150), red_radius=4, core_radius=1)  # spurious, nearer to prior_xy
    _blob(frame, (200, 200), red_radius=20, core_radius=5)  # real target, farther but bigger

    regions = locate_bright_regions(frame, prior_xy=(0, 0))

    assert regions["x"] is not None
    assert abs(regions["x"] - 200) < 5 and abs(regions["y"] - 200) < 5


if __name__ == "__main__":
    test_area_gate_ignores_small_noise_blob_near_prior()
    test_distance_gate_falls_back_to_area_when_nearest_is_implausible()
    print("OK")
