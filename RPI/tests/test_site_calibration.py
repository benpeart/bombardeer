"""Tests for 100% software/camera-only site self-calibration (boot sweep, 5'9" walk-test, passive stadia fit, roll, and persistence)."""

import math
import time
import cv2
import numpy as np
import pytest

from bombardeer_pkg.config import load_config
from bombardeer_pkg.control.ballistics import compute_deer_head_ceiling_mrad, compute_horizon_y_720, evaluate_ground_plane
from bombardeer_pkg.control.site_calibration import (
    DEFAULT_OPERATOR_HEIGHT_M,
    SiteCalibrationManager,
    compare_scene_fingerprints,
    compute_scene_fingerprint,
    estimate_daylight_horizon_and_roll,
    fit_ground_plane_huber,
)
from bombardeer_pkg.state import SystemState


def _make_synthetic_scene(hor_y=340, roll_deg=0.0, dark=False):
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    if dark:
        img[:, :] = (10, 10, 10)
        return img
    yy, xx = np.mgrid[0:720, 0:1280]
    rad = math.radians(roll_deg)
    hor_line = float(hor_y) + (xx - 640.0) * math.tan(rad)
    img[yy < hor_line] = (205, 205, 200)
    img[yy >= hor_line] = (95, 115, 100)
    for x0 in range(150, 950, 110):
        dx_top = int(round(160 * math.sin(rad)))
        dy_top = int(round(-160 * math.cos(rad)))
        cv2.line(img, (x0, int(hor_y + 40)), (x0 + dx_top, int(hor_y + 40) + dy_top), (40, 40, 40), 6, cv2.LINE_AA)
    return img


def _project_person_box(z_w, x_w, h_cam, slope_mrad, tilt_mrad, person_h, cam, optics, roll_mrad=0.0):
    s = slope_mrad / 1000.0
    alpha = tilt_mrad / 1000.0
    y_feet_w = h_cam + z_w * math.tan(s)
    y_head_w = y_feet_w - person_h
    pts = []
    for y_w in (y_head_w, y_feet_w):
        y_c = y_w * math.cos(alpha) + z_w * math.sin(alpha)
        z_c = -y_w * math.sin(alpha) + z_w * math.cos(alpha)
        y_px = cam.optical_center[1] + optics.focal_length_px * (y_c / z_c)
        x_px = cam.optical_center[0] + optics.focal_length_px * (x_w / z_c)
        pts.append((x_px, y_px))
    (xh, yh), (xf, yf) = pts
    xc = 0.5 * (xh + xf)
    roll_dy = (xc - cam.optical_center[0]) * math.tan(roll_mrad / 1000.0) if abs(roll_mrad) > 1e-4 else 0.0
    yh += roll_dy
    yf += roll_dy
    bw = max(16.0, (yf - yh) * 0.32)
    return [xc - 0.5 * bw, yh, xc + 0.5 * bw, yf]


def test_default_operator_height_is_5ft_9in():
    assert pytest.approx(DEFAULT_OPERATOR_HEIGHT_M, abs=1e-4) == 1.7526


def test_daylight_horizon_and_roll_estimation_and_night_rejection():
    cfg = load_config(env={})

    # Clean flat daylight scene
    img_320 = _make_synthetic_scene(hor_y=320, roll_deg=0.0)
    y_est, yt_mrad, roll_mrad, conf = estimate_daylight_horizon_and_roll(img_320, cfg.camera, cfg.optics, tilt_mrad=0.0)
    assert conf >= 0.85
    assert abs(y_est - 320.0) <= 5.0
    assert abs(roll_mrad) <= 10.0

    # Rolled mount (+4.5 deg = +78.5 mrad)
    img_roll = _make_synthetic_scene(hor_y=360, roll_deg=4.5)
    _, _, roll_mrad_45, conf_r = estimate_daylight_horizon_and_roll(img_roll, cfg.camera, cfg.optics, tilt_mrad=0.0)
    assert conf_r >= 0.70
    assert abs(math.degrees(roll_mrad_45 / 1000.0) - 4.5) <= 0.8

    # Nighttime dark IR scene must return None with conf=0.0
    img_dark = _make_synthetic_scene(dark=True)
    y_n, yt_n, _, conf_n = estimate_daylight_horizon_and_roll(img_dark, cfg.camera, cfg.optics, tilt_mrad=0.0)
    assert y_n is None and yt_n is None and conf_n == 0.0


def test_5ft9_walk_test_solves_exact_height_and_slope_with_outliers(tmp_path):
    """5'9" (1.7526m) walk-test recovers camera height & yard slope accurately even with outlier frames."""
    cfg = load_config(env={})
    cfg.base_dir = str(tmp_path)
    state = SystemState(cfg)
    cal = SiteCalibrationManager(cfg, cal_path=str(tmp_path / "site_calibration.json"), auto_boot_sweep=False)
    state.site_calibration = cal

    # Arm first, then verify start_walk_test immediately disarms for operator safety
    state.set_turret_mode("ARMED")
    ok, _ = cal.start_walk_test(state)  # defaults to 5'9" (1.7526m)
    assert ok
    assert state.get_turret_mode()[0] == "OFF"
    assert cal.is_motion_owned() and cal.is_walk_test_active()

    true_h = 2.15
    true_slope_mrad = 28.0
    t_base = 100.0
    # Operator walks out from 6m to 24m with a couple of noisy/crouched outlier frames mixed in
    distances = [6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 19.0, 22.0, 24.0, 18.0, 11.0]
    for i, z_w in enumerate(distances):
        box = _project_person_box(z_w, (i % 3 - 1) * 1.2, true_h, true_slope_mrad, 0.0, DEFAULT_OPERATOR_HEIGHT_M, cfg.camera, cfg.optics)
        if i == 4:
            # Inject a crouched/bad bounding box outlier -> Huber IRLS must resist it
            box[1] += 28.0
        cal.ingest_walk_test_detections(
            [{"class": "PERSON", "score": 0.88, "box": box}],
            tilt_at_cap=0.0,
            eff_capture_mono=t_base + i * 0.5,
            state=state,
        )

    ok_fin, msg = cal.finish_walk_test(state)
    assert ok_fin, msg
    assert cal.data.metric_source == "WALK_TEST"
    assert abs(cal.data.camera_mount_height_m - true_h) < 0.08
    assert abs(cal.data.ground_slope_mrad - true_slope_mrad) < 5.0
    assert not cal.is_motion_owned()

    # Verify persistence across reboot
    cal_reloaded = SiteCalibrationManager(cfg, cal_path=str(tmp_path / "site_calibration.json"), auto_boot_sweep=False)
    assert cal_reloaded.data.metric_source == "WALK_TEST"
    assert pytest.approx(cal_reloaded.data.camera_mount_height_m, abs=1e-3) == cal.data.camera_mount_height_m
    assert pytest.approx(cal_reloaded.data.ground_slope_mrad, abs=1e-2) == cal.data.ground_slope_mrad


def test_passive_stadia_convergence_and_safe_uncalibrated_mode(tmp_path):
    cfg = load_config(env={})
    cfg.base_dir = str(tmp_path)
    cal = SiteCalibrationManager(cfg, cal_path=str(tmp_path / "site_calibration.json"), auto_boot_sweep=True)

    # Prior to convergence on a new site, use_ground_range must be False (stadia-only safety)
    _, _, _, use_ground = cal.get_ground_plane_params(cfg.optics, pan_mrad=0.0)
    assert use_ground is False

    # Feed 18 wildlife observations across 7m..28m on a H=1.85m, slope=-20mrad yard
    true_h = 1.85
    true_slope_mrad = -20.0
    s = true_slope_mrad / 1000.0
    converged = False
    for i in range(18):
        z_w = 7.0 + (i * 1.2)
        y_feet_w = true_h + z_w * math.tan(s)
        y_px = cfg.camera.optical_center[1] + cfg.optics.focal_length_px * (y_feet_w / z_w)
        bh = (0.95 * cfg.optics.focal_length_px) / z_w
        box = [600.0, y_px - bh, 660.0, y_px]
        if cal.observe_passive_wildlife(box, tilt_at_cap=0.0, stadia_range_m=z_w, capture_mono=10.0 + i * 1.0):
            converged = True

    assert converged
    assert cal.data.metric_source == "PASSIVE_STADIA"
    assert abs(cal.data.camera_mount_height_m - true_h) < 0.05
    assert abs(cal.data.ground_slope_mrad - true_slope_mrad) < 3.0

    metric_opt, _, _, use_ground_after = cal.get_ground_plane_params(cfg.optics, pan_mrad=0.0)
    assert use_ground_after is True
    assert abs(metric_opt.camera_mount_height_m - true_h) < 0.05


def test_boot_sweep_maps_pan_treeline_and_detects_site_relocation(tmp_path):
    cfg = load_config(env={})
    cfg.base_dir = str(tmp_path)
    state = SystemState(cfg)
    cal = SiteCalibrationManager(cfg, cal_path=str(tmp_path / "site_calibration.json"), auto_boot_sweep=True)
    cal.SWEEP_SETTLE_SEC = 0.0  # Instant settle in unit test
    state.site_calibration = cal
    state.update_health(serial_connected=True)

    # Boot at night -> stays PENDING_DAYLIGHT, does NOT move turret
    dark_img = _make_synthetic_scene(dark=True)
    cal.step_boot_sweep(dark_img, lux=3.0, is_monochrome=True, pan_cap=0, tilt_cap=0, hw_moving=False, state=state, mono_now=1.0, safe_for_sweep=True)
    assert cal._sweep_state == "PENDING_DAYLIGHT"
    assert state.command_bus.drain() == []

    # First daylight frame triggers sweep across pan stops with varying treeline heights
    t = 10.0
    cur_pan = 0
    for step in range(40):
        # Treeline slopes from y=290 on the left to y=390 on the right
        hor_y = 340 + int(round((cur_pan / 750.0) * 50.0))
        day_img = _make_synthetic_scene(hor_y=hor_y, roll_deg=0.0)
        cal.step_boot_sweep(day_img, lux=150.0, is_monochrome=False, pan_cap=cur_pan, tilt_cap=0, hw_moving=False, state=state, mono_now=t, safe_for_sweep=True)
        cmds = state.command_bus.drain()
        for c in cmds:
            if c["cmd"] == "SETPOINT":
                cur_pan = c["pan"]
        t += 0.2
        if cal._sweep_state == "DONE":
            break

    assert cal._sweep_state == "DONE"
    assert cal.data.boot_sweep_completed is True
    assert len(cal.data.yard_top_profile) >= 5

    # Verify pan-interpolated tiling optics follow left vs right treeline
    opt_left = cal.get_tiling_optics(cfg.optics, pan_mrad=-700.0)
    opt_right = cal.get_tiling_optics(cfg.optics, pan_mrad=+700.0)
    assert opt_left.ground_slope_mrad < opt_right.ground_slope_mrad

    # Reboot at SAME location -> CHECK_SITE matches fingerprint and skips sweep
    cal_same = SiteCalibrationManager(cfg, cal_path=str(tmp_path / "site_calibration.json"), auto_boot_sweep=True)
    home_img = _make_synthetic_scene(hor_y=340, roll_deg=0.0)
    cal_same.step_boot_sweep(home_img, lux=150.0, is_monochrome=False, pan_cap=0, tilt_cap=0, hw_moving=False, state=state, mono_now=50.0, safe_for_sweep=True)
    assert cal_same._sweep_state == "DONE"

    # Reboot at a NEW location (inverted / drastically different scene) -> triggers fresh boot sweep
    cal_new = SiteCalibrationManager(cfg, cal_path=str(tmp_path / "site_calibration.json"), auto_boot_sweep=True)
    new_site_img = np.full((720, 1280, 3), 100, dtype=np.uint8)
    new_site_img[420:, :] = 210  # Bright bottom, dark top -> negative correlation
    cal_new.step_boot_sweep(new_site_img, lux=150.0, is_monochrome=False, pan_cap=0, tilt_cap=0, hw_moving=False, state=state, mono_now=60.0, safe_for_sweep=True)
    assert cal_new._sweep_state == "PENDING_DAYLIGHT"
    assert cal_new.data.metric_source == "UNSET"


def test_evaluate_ground_plane_supports_roll_and_separate_yard_ceiling():
    cfg = load_config(env={})
    yt_mrad = 160.0
    y_ceiling = cfg.camera.optical_center[1] + cfg.optics.focal_length_px * math.tan(yt_mrad / 1000.0)
    min_feet_y = y_ceiling - cfg.optics.horizon_feet_margin_720
    y_bird = int(round(min_feet_y - 12.0))

    # Bird sits below metric horizon (y=360) yet above treeline ceiling -> vetoed by yard_top_mrad
    ok_bird, rej_bird, _ = evaluate_ground_plane(
        [600, y_bird - 25, 630, y_bird],
        tilt_at_cap=0.0,
        stadia_range_m=20.0,
        max_animal_height_m=2.0,
        cam=cfg.camera,
        optics=cfg.optics,
        yard_top_mrad=yt_mrad,
    )
    assert not ok_bird and "ABOVE_HORIZON" in rej_bird

    # Positive roll (+80 mrad) tilts horizon up on the left and down on the right
    y_left = compute_horizon_y_720(0.0, cfg.camera, cfg.optics, x_px=100.0, roll_mrad=80.0)
    y_right = compute_horizon_y_720(0.0, cfg.camera, cfg.optics, x_px=1180.0, roll_mrad=80.0)
    assert y_right > y_left + 70.0

    # With use_ground_range=False (new site before height fit), deer below ceiling returns stadia range safely
    y_deer_bot = int(round(y_ceiling + 60.0))
    ok_deer, rej_deer, rng = evaluate_ground_plane(
        [600, y_deer_bot - 50, 660, y_deer_bot],
        tilt_at_cap=0.0,
        stadia_range_m=14.5,
        max_animal_height_m=1.5,
        cam=cfg.camera,
        optics=cfg.optics,
        yard_top_mrad=yt_mrad,
        use_ground_range=False,
    )
    assert ok_deer and rej_deer is None and pytest.approx(rng, abs=1e-4) == 14.5


def test_computed_deer_head_ceiling_accounts_for_near_and_far_perspective():
    cfg = load_config(env={})
    # Case 1: Sawhorse mount (~1.0 m) < 6'0" (1.83 m) deer -> near deer rises much higher than far-edge deer
    cfg.optics.camera_mount_height_m = 1.00
    cfg.optics.ground_slope_mrad = 0.0
    ceil_low, far_low, near_low = compute_deer_head_ceiling_mrad(cfg.optics, max_deer_height_m=1.8288, far_ground_mrad=46.0)
    assert near_low < far_low < 0.0
    assert -50.0 < far_low < -25.0  # Far-edge 6'0" buck head sits just ~20px above horizon, NOT -237 mrad
    assert ceil_low < near_low

    # Case 2: Elevated mount (2.20 m) > 6'0" (1.83 m) deer -> deer head stays BELOW horizon at all ranges
    cfg.optics.camera_mount_height_m = 2.20
    ceil_hi, far_hi, near_hi = compute_deer_head_ceiling_mrad(cfg.optics, max_deer_height_m=1.8288, far_ground_mrad=50.0)
    assert near_hi > far_hi > 0.0
    assert ceil_hi > ceil_low + 100.0

    # Case 3: Uphill far edge (above metric horizon) keeps far_head strictly above far_ground_mrad
    cfg.optics.camera_mount_height_m = 1.00
    _, far_uphill, _ = compute_deer_head_ceiling_mrad(cfg.optics, max_deer_height_m=1.8288, far_ground_mrad=-25.0)
    assert far_uphill < -55.0


def test_walk_test_ignores_horizontal_barrel_false_person_box(tmp_path):
    """A horizontal VETO:PERSON box on the paintball barrel tip must NOT block Walk Calibration."""
    cfg = load_config(env={})
    cfg.base_dir = str(tmp_path)
    state = SystemState(cfg)
    cal = SiteCalibrationManager(cfg, cal_path=str(tmp_path / "site_calibration.json"), auto_boot_sweep=False)
    cal.start_walk_test(state)

    barrel_false_person = {"class": "PERSON", "score": 0.57, "box": [860.0, 232.0, 960.0, 279.0]}
    real_person_box = _project_person_box(10.0, 0.0, 1.80, 10.0, 0.0, DEFAULT_OPERATOR_HEIGHT_M, cfg.camera, cfg.optics)
    cal.ingest_walk_test_detections(
        [barrel_false_person, {"class": "PERSON", "score": 0.89, "box": real_person_box}],
        tilt_at_cap=0.0,
        eff_capture_mono=50.0,
        state=state,
    )
    assert cal.get_status_dict()["walk_test_samples"] == 1


def test_start_walk_test_levels_tilt_without_halt_dropping_setpoint(tmp_path):
    """start_walk_test() must disarm trigger AND deliver SETPOINT(cur_pan, 0) without _halt_pending dropping it."""
    cfg = load_config(env={})
    cfg.base_dir = str(tmp_path)
    state = SystemState(cfg)
    state.update_turret_telemetry(pan=220, tilt=-140, moving=False, rx_mono=10.0)
    cal = SiteCalibrationManager(cfg, cal_path=str(tmp_path / "site_calibration.json"), auto_boot_sweep=False)

    ok, _ = cal.start_walk_test(state)
    assert ok
    cmds = state.command_bus.drain()
    assert any(c["cmd"] == "TRIGGER" and c["state"] == 0 for c in cmds)
    assert any(c["cmd"] == "SETPOINT" and c["pan"] == 220 and c["tilt"] == 0 for c in cmds)


def test_boot_sweep_triggered_at_night_defers_instead_of_locking_motion(tmp_path):
    """Clicking Boot Sweep at night must defer to PENDING_DAYLIGHT instead of locking is_motion_owned() all night."""
    cfg = load_config(env={})
    cfg.base_dir = str(tmp_path)
    state = SystemState(cfg)
    state.update_health(serial_connected=True)
    cal = SiteCalibrationManager(cfg, cal_path=str(tmp_path / "site_calibration.json"), auto_boot_sweep=False)

    ok, _ = cal.start_boot_sweep(force=True)
    assert ok and cal.is_motion_owned()

    dark_img = _make_synthetic_scene(dark=True)
    cal.step_boot_sweep(dark_img, lux=2.0, is_monochrome=True, pan_cap=0, tilt_cap=0, hw_moving=False, state=state, mono_now=5.0, safe_for_sweep=True)
    assert cal._sweep_state == "PENDING_DAYLIGHT"
    assert not cal.is_motion_owned()
