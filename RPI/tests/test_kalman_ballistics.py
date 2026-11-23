"""Tests for non-mutating Kalman predictions, horizon geometry, parallax, and drag ballistics."""

import math
import pytest

from bombardeer_pkg.config import load_config
from bombardeer_pkg.control.ballistics import (
    compute_drag_ballistics,
    compute_horizon_y_720,
    evaluate_ground_plane,
    image_px_to_world_mrad,
    project_world_to_image_px,
)
from bombardeer_pkg.control.kalman import TargetKalmanFilter


def test_kalman_predict_is_non_mutating_and_ignores_out_of_order():
    kf = TargetKalmanFilter(process_noise=35.0, measurement_noise=10.0)
    assert kf.predict_state(100.0) is None

    for step in range(10):
        t = 10.0 + step * 0.1
        kf.update_measurement(int(100 * step * 0.1), 0, t)

    p1 = kf.predict_state(11.1)
    p2 = kf.predict_state(11.1)
    assert p1 == p2

    p3 = kf.predict_state(11.3)
    assert p3[0] > p1[0]
    assert p3[2] > 50.0

    # Stale out-of-order frame must not corrupt velocity
    kf.update_measurement(-500, 0, 10.2)
    assert kf.predict_state(11.1) == p1


def test_kalman_clamp_position_zeroes_boundary_velocity():
    kf = TargetKalmanFilter()
    kf.update_measurement(0, 0, 1.0)
    kf.update_measurement(200, 100, 1.1)
    kf.clamp_position(True, 3141, False, 0)
    pan, _, v_pan, _ = kf.predict_state(1.1)
    assert pan == 3141
    assert v_pan == 0.0


def test_horizon_moves_down_when_tilted_up():
    cfg = load_config(env={})
    y_level = compute_horizon_y_720(0.0, cfg.camera, cfg.optics)
    y_up = compute_horizon_y_720(100.0, cfg.camera, cfg.optics)
    y_down = compute_horizon_y_720(-100.0, cfg.camera, cfg.optics)

    assert pytest.approx(y_level, abs=0.1) == 360.0
    assert y_up > y_level
    assert y_down < y_level


def test_pixel_world_roundtrip_consistency():
    cfg = load_config(env={})
    orig_px, orig_py = 820.0, 410.0
    cur_pan, cur_tilt, rng = 150, -30, 25.0

    w_pan, w_tilt = image_px_to_world_mrad(orig_px, orig_py, cur_pan, cur_tilt, rng, cfg.camera, cfg.optics)
    rec_px, rec_py = project_world_to_image_px(w_pan, w_tilt, cur_pan, cur_tilt, rng, cfg.camera, cfg.optics)

    assert abs(rec_px - orig_px) < 1.5
    assert abs(rec_py - orig_py) < 1.5


def test_drag_ballistics_and_ground_plane_vetoes():
    cfg = load_config(env={})
    tof_15, drop_15, lead_15, _ = compute_drag_ballistics(15.0, 0.0, 100.0, 0.0, cfg.ballistics, cfg.optics)
    tof_35, drop_35, lead_35, _ = compute_drag_ballistics(35.0, 0.0, 100.0, 0.0, cfg.ballistics, cfg.optics)
    assert tof_35 > tof_15 > 0.0
    assert drop_35 > drop_15 > 0.0
    assert lead_35 > lead_15 > 0.0

    # Feet far above horizon must be vetoed
    ok_sky, reason_sky, _ = evaluate_ground_plane([500, 100, 560, 200], 0.0, 20.0, 1.50, cfg.camera, cfg.optics)
    assert not ok_sky and "ABOVE_HORIZON" in reason_sky

    # Standing target at 20m: depression ~ atan(1.15/20) -> y_feet ~ 391
    y_feet = int(round(360 + 540.4 * math.tan(math.atan2(1.15, 20.0))))
    ok_deer, _, fused_rng = evaluate_ground_plane([600, y_feet - 27, 640, y_feet], 0.0, 20.0, 1.50, cfg.camera, cfg.optics)
    ok_human, reason_h, _ = evaluate_ground_plane([600, y_feet - 55, 625, y_feet], 0.0, 20.0, 1.50, cfg.camera, cfg.optics)

    assert ok_deer and 18.0 <= fused_rng <= 22.0
    assert not ok_human and "HEIGHT" in reason_h