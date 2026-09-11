"""Self-check for the 2026-09-11 velocity-prediction addition to the camera
capture loop: _predict_xy / _update_velocity_xy. Run directly:
`python test_prior_xy_prediction.py`. No pytest, same assert-based style as
test_scan_quality_metrics.py.
"""
from visualizer_server import _predict_xy, _update_velocity_xy


def test_predict_xy() -> None:
    assert _predict_xy(None, (5.0, -2.0)) is None  # no anchor yet -> no prediction
    assert _predict_xy((100.0, 50.0), (0.0, 0.0)) == (100.0, 50.0)  # zero velocity -> static prior
    assert _predict_xy((100.0, 50.0), (4.0, -1.0)) == (104.0, 49.0)  # moving -> extrapolated ahead


def test_update_velocity_xy() -> None:
    # No prior anchor -> nothing to derive a displacement from.
    assert _update_velocity_xy(None, (3.0, 3.0), (10.0, 10.0)) == (0.0, 0.0)

    # First real displacement folds in at half weight against a zero start.
    v = _update_velocity_xy((0.0, 0.0), (0.0, 0.0), (10.0, 0.0))
    assert v == (5.0, 0.0)

    # A steady, consistent displacement should converge toward that value
    # over repeated updates rather than jumping straight to it.
    prior, velocity = (0.0, 0.0), (0.0, 0.0)
    for _ in range(20):
        new_xy = (prior[0] + 4.0, prior[1])
        velocity = _update_velocity_xy(prior, velocity, new_xy)
        prior = new_xy
    assert abs(velocity[0] - 4.0) < 0.01, velocity  # converged close to the true constant velocity
    assert velocity[1] == 0.0

    # One noisy outlier displacement should only nudge the estimate, not
    # replace it outright (that's the whole point of the smoothing).
    v_before = (4.0, 0.0)
    v_after = _update_velocity_xy((0.0, 0.0), v_before, (100.0, 0.0))  # wild one-off jump
    assert v_after == (52.0, 0.0)  # halfway toward the outlier, not equal to it


if __name__ == "__main__":
    test_predict_xy()
    test_update_velocity_xy()
    print("OK")
