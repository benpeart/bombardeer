"""Tests for non-mutating Kalman predictions, deer-kinematics bounds, horizon geometry, parallax, and drag ballistics."""

import math
import pytest

from bombardeer_pkg.config import load_config
from bombardeer_pkg.control.ballistics import (
    compute_drag_ballistics,
    compute_horizon_y_720,
    compute_target_kinematics,
    evaluate_ground_plane,
    image_px_to_world_mrad,
    project_world_to_image_px,
)
from bombardeer_pkg.control.kalman import TargetKalmanFilter


def test_kalman_predict_is_non_mutating_and_ignores_out_of_order():
    kf = TargetKalmanFilter(process_noise=35.0, measurement_noise=10.0)
    assert kf.predict_state(100.0) is None
    assert kf.get_last_state() is None

    for step in range(10):
        t = 10.0 + step * 0.1
        kf.update_measurement(int(100 * step * 0.1), 0, t)

    p1 = kf.predict_state(11.1)
    p2 = kf.predict_state(11.1)
    assert p1 == p2

    p3 = kf.predict_state(11.3)
    assert p3[0] > p1[0]
    assert p3[2] > 50.0

    # Un-extrapolated last state returns the exact capture timestamp
    last_pan, last_tilt, last_ts = kf.get_last_state()
    assert pytest.approx(last_ts, abs=1e-6) == 10.9
    assert last_tilt == 0 and last_pan > 0

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


def test_target_kinematics_scales_with_range_and_ground_plane():
    cfg = load_config(env={})
    k_close = compute_target_kinematics(5.0, cfg.optics)
    k_mid = compute_target_kinematics(15.0, cfg.optics)
    k_far = compute_target_kinematics(40.0, cfg.optics)

    # Pan angular speed decreases with range and respects hardware cap
    assert k_close.max_v_pan_mrad_s <= 850.0
    assert k_close.max_v_pan_mrad_s > k_mid.max_v_pan_mrad_s > k_far.max_v_pan_mrad_s > 0.0

    # Ground-plane tilt speed/acceleration is strictly tighter than pan at mid/far range
    assert k_mid.max_v_tilt_mrad_s < k_mid.max_v_pan_mrad_s
    assert k_far.max_v_tilt_mrad_s < k_far.max_v_pan_mrad_s
    assert k_mid.accel_tilt_mrad_s2 < k_mid.accel_pan_mrad_s2

    # Optics-derived measurement noise reflects pixel jitter (> old 10.0 mrad^2)
    assert k_mid.meas_var_tilt >= k_mid.meas_var_pan > 15.0


def test_kalman_encapsulated_velocity_clamp_and_innovation_gate():
    """Prevents non-physical jumps from injecting runaway coast velocity into statePost."""
    cfg = load_config(env={})
    kin = compute_target_kinematics(30.0, cfg.optics)
    kf = TargetKalmanFilter()

    kf.update_measurement(100.0, -180.0, 10.0, kinematics=kin)
    pan_est, tilt_est, v_pan, v_tilt = kf.update_measurement(650.0, 220.0, 10.14, kinematics=kin)

    # Encapsulated velocity ceilings enforced atomically on statePost
    assert abs(v_pan) <= kin.max_v_pan_mrad_s + 1e-3
    assert abs(v_tilt) <= kin.max_v_tilt_mrad_s + 1e-3
    _, _, pred_vp, pred_vt = kf.predict_state(10.14)
    assert abs(pred_vp) <= kin.max_v_pan_mrad_s + 1e-3
    assert abs(pred_vt) <= kin.max_v_tilt_mrad_s + 1e-3

    # Innovation gate prevented position from teleporting to the outlier
    assert pan_est < 250
    assert tilt_est < -100


def test_kalman_anisotropic_damps_close_range_vertical_jitter():
    """Reproduces the 7m stationary-deer tilt jitter scenario and confirms damped v_tilt."""
    cfg = load_config(env={})
    kin = compute_target_kinematics(7.0, cfg.optics)

    kf_old = TargetKalmanFilter(accel_noise_mrad_s2=150.0, measurement_noise=10.0)
    kf_new = TargetKalmanFilter(accel_noise_mrad_s2=150.0, measurement_noise=35.0)

    jitter = [0.0, 14.0, -14.0, 16.0, -16.0, 12.0, -12.0, 15.0, -15.0]
    max_v_old = 0.0
    max_v_new = 0.0
    for idx, dy in enumerate(jitter):
        t = 100.0 + idx * 0.135
        _, _, _, vt_old = kf_old.update_measurement(854.0, -182.0 + dy, t, kinematics=None)
        _, _, _, vt_new = kf_new.update_measurement(854.0, -182.0 + dy, t, kinematics=kin)
        max_v_old = max(max_v_old, abs(vt_old))
        max_v_new = max(max_v_new, abs(vt_new))

    assert max_v_new < (0.65 * max_v_old)


def test_kalman_predict_extrapolates_beyond_2s_for_350ms_latency():
    """With 1.8s coast + 0.35s NPU latency, predict_state must not freeze at 2.0s."""
    kf = TargetKalmanFilter(accel_noise_mrad_s2=150.0, measurement_noise=10.0)
    for step in range(5):
        kf.update_measurement(step * 20.0, 0.0, 10.0 + step * 0.2)

    p_20 = kf.predict_state(10.8 + 2.0)
    p_23 = kf.predict_state(10.8 + 2.3)
    assert p_23[0] > p_20[0]


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

    # Standing target at 20m: depression ~ atan(H/20) -> y_feet
    y_feet = int(round(360 + cfg.optics.focal_length_px * math.tan(math.atan2(cfg.optics.camera_mount_height_m, 20.0))))
    ok_deer, _, fused_rng = evaluate_ground_plane([600, y_feet - 27, 640, y_feet], 0.0, 20.0, 1.50, cfg.camera, cfg.optics)
    ok_human, reason_h, _ = evaluate_ground_plane([600, y_feet - 55, 625, y_feet], 0.0, 20.0, 1.50, cfg.camera, cfg.optics)

    assert ok_deer and 18.0 <= fused_rng <= 22.0
    assert not ok_human and "HEIGHT" in reason_h

    # Tall buck / slight horizon variance (~1.58m): rejected at 1.50m, accepted at 2.0m
    bh_buck = int(round(1.58 * cfg.optics.focal_length_px / 20.0))
    ok_150, rej_150, _ = evaluate_ground_plane([600, y_feet - bh_buck, 640, y_feet], 0.0, 20.0, 1.50, cfg.camera, cfg.optics)
    ok_200, rej_200, _ = evaluate_ground_plane([600, y_feet - bh_buck, 640, y_feet], 0.0, 20.0, 2.00, cfg.camera, cfg.optics)
    assert not ok_150 and "HEIGHT" in rej_150
    assert ok_200 and rej_200 is None
