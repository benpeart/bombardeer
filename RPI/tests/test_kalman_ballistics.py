"""Tests for damped-coast Kalman predictions, Western VA white-tailed deer kinematics, horizon geometry, parallax, and drag ballistics."""

import math
import pytest

from bombardeer_pkg.config import load_config
from bombardeer_pkg.control.ballistics import (
    compute_drag_ballistics,
    compute_horizon_y_720,
    compute_target_elevation_ceiling_mrad,
    compute_target_kinematics,
    evaluate_ground_plane,
    image_px_to_world_mrad,
    project_world_to_image_px,
)
from bombardeer_pkg.control.kalman import TargetKalmanFilter


def test_kalman_predict_is_non_mutating_and_ignores_out_of_order():
    kf = TargetKalmanFilter(accel_noise_mrad_s2=150.0, measurement_noise=10.0)
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

    # Ground-plane tilt speed/acceleration is strictly tighter than pan and capped <= 180 mrad/s
    assert k_close.max_v_tilt_mrad_s <= 180.0
    assert k_mid.max_v_tilt_mrad_s < k_mid.max_v_pan_mrad_s
    assert k_far.max_v_tilt_mrad_s < k_far.max_v_pan_mrad_s
    assert k_mid.accel_tilt_mrad_s2 < k_mid.accel_pan_mrad_s2

    # Even on an elevated 2.29m mount at 9m, tilt ceiling stays <= 180 mrad/s (never 385 mrad/s)
    cfg.optics.camera_mount_height_m = 2.29
    k_elev = compute_target_kinematics(9.0, cfg.optics)
    assert k_elev.max_v_tilt_mrad_s <= 180.0

    # Optics-derived measurement noise reflects pixel jitter (> old 10.0 mrad^2)
    assert k_mid.meas_var_tilt >= k_mid.meas_var_pan > 15.0


def test_kalman_encapsulated_velocity_clamp_and_innovation_gate():
    """Prevents non-physical jumps from injecting runaway coast velocity into statePost."""
    cfg = load_config(env={})
    kin = compute_target_kinematics(30.0, cfg.optics)
    kf = TargetKalmanFilter()

    kf.update_measurement(100.0, -180.0, 10.0, kinematics=kin)
    pan_est, tilt_est, v_pan, v_tilt = kf.update_measurement(650.0, 220.0, 10.14, kinematics=kin)

    # Encapsulated velocity ceilings enforced atomically on statePost and outlier velocity damped
    assert abs(v_pan) <= kin.max_v_pan_mrad_s + 1e-3
    assert abs(v_tilt) <= kin.max_v_tilt_mrad_s + 1e-3
    assert v_tilt == 0.0
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


def test_kalman_damped_coast_preserves_350ms_lead_and_bounds_long_dropouts():
    """
    Healthy 350ms SCAN-12MP lead stays intact via pan_hold_sec, while extended
    dropouts smoothly decelerate to a bounded wait position instead of linearly
    flying into mechanical limits.
    """
    kf = TargetKalmanFilter(accel_noise_mrad_s2=150.0, measurement_noise=10.0)
    for step in range(6):
        kf.update_measurement(step * 40.0, step * 20.0, 10.0 + step * 0.2)

    t_last = 11.0
    p_0 = kf.predict_state(t_last, pan_hold_sec=0.42)
    p_35 = kf.predict_state(t_last + 0.35, pan_hold_sec=0.42)
    p_15 = kf.predict_state(t_last + 1.50, pan_hold_sec=0.42)
    p_23 = kf.predict_state(t_last + 2.30, pan_hold_sec=0.42)

    # Full pan velocity lead preserved across 350ms NPU latency
    assert p_35[0] > p_0[0] + 50
    assert pytest.approx(p_35[2], abs=1.0) == p_0[2]

    # Pan smoothly decelerates and bounds displacement over 1.5s-2.3s occlusion
    assert p_15[2] < 0.05 * p_0[2]
    assert abs(p_23[0] - p_15[0]) <= 10

    # Tilt decays even faster: virtually zero velocity by 0.8s and bounded displacement
    p_tilt_08 = kf.predict_state(t_last + 0.80, pan_hold_sec=0.42)
    assert abs(p_tilt_08[3]) < 0.03 * abs(p_0[3])
    assert abs(p_23[1] - p_0[1]) < 45


def test_kalman_064259_slingshot_replay_stays_bounded_after_dropout():
    """
    Replays the 06:43:19.55 -> 06:43:20.67 1.12s dropout + reverse deer detection.
    With damped prediction in update_measurement() + capped innovation gate dt,
    the filter must NOT slingshot into +385 mrad/s roof climbs or -485 mrad/s spins.
    """
    cfg = load_config(env={})
    cfg.optics.camera_mount_height_m = 2.29
    kin = compute_target_kinematics(9.0, cfg.optics)
    kf = TargetKalmanFilter()

    # Establish fast pan/tilt state right before the 1.12s dropout
    kf.update_measurement(-200.0, -140.0, 19.35, kinematics=kin)
    kf.update_measurement(-130.0, -170.0, 19.55, kinematics=kin)

    # 1.12s dropout: blind coast must stay well inside the yard (never hit -420)
    pred_dropout = kf.predict_state(20.67)
    assert pred_dropout[1] > -300

    # Reverse detection arrives near original deer at (-260, -80) after 1.12s
    pan_est, tilt_est, v_pan, v_tilt = kf.update_measurement(-260.0, -80.0, 20.67, kinematics=kin)
    assert abs(v_pan) < 60.0
    assert abs(v_tilt) < 30.0

    # Following 1.46s blind coast must hold near the yard (never climb toward +400 porch roof)
    pred_after = kf.predict_state(20.67 + 1.46)
    assert pred_after[1] < 0
    assert pred_after[0] > -350

    # Next live frame 0.14s later immediately tracks real walking speed (-40 mrad/s)
    _, _, v_pan_walk, _ = kf.update_measurement(-266.0, -80.0, 20.81, kinematics=kin)
    assert -60.0 < v_pan_walk < -20.0


def test_kalman_walking_sprinting_occlusion_leap_and_uturn_physics():
    """Validates real Western VA deer gaits: walk, sprint, tree occlusion, fence leap, and abrupt freeze/U-turn."""
    cfg = load_config(env={})

    # 1. Slow grazing walk (1.0 m/s @ 15 m -> 66.7 mrad/s) & brisk walk (2.0 m/s @ 10 m -> 200 mrad/s)
    kin_15 = compute_target_kinematics(15.0, cfg.optics)
    kf_walk = TargetKalmanFilter()
    for i in range(12):
        t = 10.0 + i * 0.135
        kf_walk.update_measurement(-200.0 + 66.7 * (i * 0.135), -85.0, t, kinematics=kin_15)
    p_walk = kf_walk.predict_state(10.0 + 11 * 0.135 + 0.10)
    expected_pan = -200.0 + 66.7 * (11 * 0.135 + 0.10)
    assert abs(p_walk[0] - expected_pan) < 12.0

    # 2. Spooked sprint (12 m/s @ 15 m -> 800 mrad/s) remains inside kinematic/innovation gates
    kf_sprint = TargetKalmanFilter()
    for i in range(10):
        t = 20.0 + i * 0.12
        kf_sprint.update_measurement(-400.0 + 800.0 * (i * 0.12), -80.0, t, kinematics=kin_15)
    _, _, v_pan_sprint, _ = kf_sprint.predict_state(20.0 + 9 * 0.12)
    assert 650.0 <= v_pan_sprint <= 800.1

    # 3. Tree-trunk occlusion (1.2s dropout while walking 2 m/s @ 15 m = 133 mrad/s)
    kf_occ = TargetKalmanFilter()
    for i in range(8):
        t = 30.0 + i * 0.135
        kf_occ.update_measurement(133.3 * (i * 0.135), -90.0, t, kinematics=kin_15)
    t_disappear = 30.0 + 7 * 0.135
    p_disappear = kf_occ.predict_state(t_disappear)
    p_wait = kf_occ.predict_state(t_disappear + 1.2)
    # Coasts modestly past the tree trunk (~60-110 mrad), NOT 160+ mrad runaway, and slows to near-stop
    assert 40.0 <= (p_wait[0] - p_disappear[0]) <= 120.0
    assert abs(p_wait[2]) < 15.0

    # 4. Bounding / fence leap (2.4 m arc @ 12 m, <0.7s up): tracks live climb, decays promptly at apex
    kin_12 = compute_target_kinematics(12.0, cfg.optics)
    kf_leap = TargetKalmanFilter()
    for i in range(6):
        dt_l = i * 0.12
        z_m = max(0.0, 6.8 * dt_l - 4.905 * dt_l * dt_l)  # ~2.36 m peak at t=0.69s
        tilt_mrad = -90.0 + (z_m / 12.0) * 1000.0
        kf_leap.update_measurement(50.0 + 180.0 * dt_l, tilt_mrad, 40.0 + dt_l, kinematics=kin_12)
    p_apex = kf_leap.predict_state(40.0 + 5 * 0.12)
    p_drop = kf_leap.predict_state(40.0 + 5 * 0.12 + 1.0)
    # If detections drop at the leap apex, tilt stops climbing within <45 mrad (never hits porch roof)
    assert abs(p_drop[1] - p_apex[1]) < 45.0
    assert abs(p_drop[3]) < 5.0

    # 5. Alert freeze / sudden U-turn from brisk walk -> reverses cleanly without runaway overshoot
    kf_turn = TargetKalmanFilter()
    for i in range(6):
        kf_turn.update_measurement(200.0 * (i * 0.13), -80.0, 50.0 + i * 0.13, kinematics=kin_15)
    for j in range(1, 7):
        t_u = 50.0 + (5 + j) * 0.13
        kf_turn.update_measurement(130.0 - 160.0 * (j * 0.13), -80.0, t_u, kinematics=kin_15)
    p_rev = kf_turn.predict_state(50.0 + 11 * 0.13)
    assert p_rev[2] < -60.0


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


def test_target_elevation_ceiling_mrad_on_elevated_and_sawhorse_mounts():
    cfg = load_config(env={})

    # Elevated mount (H=2.29m, uphill slope=-42.8mrad): deer at 9m stays well below porch roof (+400)
    cfg.optics.camera_mount_height_m = 2.29
    cfg.optics.ground_slope_mrad = -42.8
    ceil_elev_9m = compute_target_elevation_ceiling_mrad(9.0, cfg.optics, max_deer_height_m=1.85, far_ground_mrad=46.0)
    assert 0.0 < ceil_elev_9m < 60.0

    # Sawhorse mount (H=1.00m, flat slope=0mrad): 9m buck head fits (~+117 mrad), 4m buck fits (~+235 mrad)
    cfg.optics.camera_mount_height_m = 1.00
    cfg.optics.ground_slope_mrad = 0.0
    ceil_saw_9m = compute_target_elevation_ceiling_mrad(9.0, cfg.optics, max_deer_height_m=1.85, far_ground_mrad=46.0)
    ceil_saw_4m = compute_target_elevation_ceiling_mrad(4.0, cfg.optics, max_deer_height_m=1.85, far_ground_mrad=46.0)
    assert 95.0 < ceil_saw_9m < 140.0
    assert 205.0 < ceil_saw_4m < 260.0
    assert ceil_saw_4m > ceil_saw_9m


def test_drag_ballistics_and_ground_plane_vetoes():
    cfg = load_config(env={})
    tof_15, drop_15, lead_15, _ = compute_drag_ballistics(15.0, 0.0, 100.0, 0.0, cfg.ballistics)
    tof_35, drop_35, lead_35, _ = compute_drag_ballistics(35.0, 0.0, 100.0, 0.0, cfg.ballistics)
    assert tof_35 > tof_15 > 0.0
    assert drop_35 > drop_15 > 0.0
    assert lead_35 > lead_15 > 0.0

    # Feet far above horizon must be vetoed
    ok_sky, reason_sky, _ = evaluate_ground_plane([500, 100, 560, 200], 0.0, 20.0, 1.85, cfg.camera, cfg.optics)
    assert not ok_sky and "ABOVE_HORIZON" in reason_sky

    # Standing target at 20m: depression ~ atan(H/20) -> y_feet
    y_feet = int(round(360 + cfg.optics.focal_length_px * math.tan(math.atan2(cfg.optics.camera_mount_height_m, 20.0))))
    ok_deer, _, fused_rng = evaluate_ground_plane([600, y_feet - 27, 640, y_feet], 0.0, 20.0, 1.85, cfg.camera, cfg.optics)
    # 2.15m anomaly rejected at Western VA 1.85m ceiling; 1.70m mature buck with antlers accepted
    bh_buck = int(round(1.70 * cfg.optics.focal_length_px / 20.0))
    bh_tall = int(round(2.15 * cfg.optics.focal_length_px / 20.0))
    ok_buck, rej_buck, _ = evaluate_ground_plane([600, y_feet - bh_buck, 640, y_feet], 0.0, 20.0, 1.85, cfg.camera, cfg.optics)
    ok_tall, rej_tall, _ = evaluate_ground_plane([600, y_feet - bh_tall, 625, y_feet], 0.0, 20.0, 1.85, cfg.camera, cfg.optics)

    assert ok_deer and 18.0 <= fused_rng <= 22.0
    assert ok_buck and rej_buck is None
    assert not ok_tall and "HEIGHT" in rej_tall
