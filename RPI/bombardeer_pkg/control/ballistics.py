"""Pure mathematical functions for horizon geometry, parallax, drag ballistics, projections, and target kinematics."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np

from bombardeer_pkg.config import BallisticsConfig, CameraConfig, OpticsConfig


@dataclass(frozen=True)
class TargetKinematics:
    """Range- and optics-derived angular kinematic bounds for a live deer."""

    max_v_pan_mrad_s: float
    max_v_tilt_mrad_s: float
    accel_pan_mrad_s2: float
    accel_tilt_mrad_s2: float
    meas_var_pan: float
    meas_var_tilt: float
    max_step_margin_pan_mrad: float
    max_step_margin_tilt_mrad: float


def compute_target_kinematics(
    range_m: float,
    optics: OpticsConfig,
    max_deer_speed_m_s: float = 10.0,
    max_deer_accel_m_s2: float = 3.0,
    max_vertical_speed_m_s: float = 1.0,
    max_vertical_accel_m_s2: float = 0.6,
    bbox_jitter_pan_px: float = 3.0,
    bbox_jitter_tilt_px: float = 4.0,
    hw_max_v_pan_mrad_s: float = 850.0,
    hw_max_v_tilt_mrad_s: float = 450.0,
) -> TargetKinematics:
    """
    Derives world-space angular velocity/acceleration limits and measurement
    variances from live deer speeds, target range, and camera optics.
    """
    r = float(np.clip(range_m, max(1.0, optics.min_valid_range_m), optics.max_valid_range_m))
    h = max(0.5, float(optics.camera_mount_height_m))
    mrad_per_px = float(optics.mrad_per_pixel)

    # Pan: lateral running speed & acceleration projected at range r
    max_v_pan = min(hw_max_v_pan_mrad_s, (1000.0 * max_deer_speed_m_s) / r)
    accel_pan = float(np.clip((1000.0 * max_deer_accel_m_s2) / r, 25.0, 400.0))

    # Tilt: ground-plane radial motion (H*v/r^2) + modest vertical bounding (vz/r)
    ground_v_tilt = (1000.0 * h * max_deer_speed_m_s) / (r * r)
    bound_v_tilt = (1000.0 * max_vertical_speed_m_s) / r
    max_v_tilt = min(hw_max_v_tilt_mrad_s, ground_v_tilt + bound_v_tilt)

    ground_a_tilt = (1000.0 * h * max_deer_accel_m_s2) / (r * r)
    bound_a_tilt = (1000.0 * max_vertical_accel_m_s2) / r
    accel_tilt = float(np.clip(ground_a_tilt + bound_a_tilt, 15.0, 120.0))

    # Measurement variance from pixel box-center uncertainty and lens mrad/px
    sigma_pan = max(1.5, bbox_jitter_pan_px * mrad_per_px)
    sigma_tilt = max(2.0, bbox_jitter_tilt_px * mrad_per_px)

    return TargetKinematics(
        max_v_pan_mrad_s=max_v_pan,
        max_v_tilt_mrad_s=max_v_tilt,
        accel_pan_mrad_s2=accel_pan,
        accel_tilt_mrad_s2=accel_tilt,
        meas_var_pan=sigma_pan * sigma_pan,
        meas_var_tilt=sigma_tilt * sigma_tilt,
        max_step_margin_pan_mrad=3.0 * sigma_pan,
        max_step_margin_tilt_mrad=3.0 * sigma_tilt,
    )


def compute_horizon_y_720(tilt_mrad: float, cam: CameraConfig, optics: OpticsConfig) -> float:
    """Computes horizon Y coordinate in canonical 1280x720 space for given turret tilt (mrad)."""
    angle_rad = (float(tilt_mrad) + optics.ground_slope_mrad) / 1000.0
    y_hor = cam.optical_center[1] + (optics.focal_length_px * math.tan(angle_rad)) + optics.horizon_y_offset_720
    return float(np.clip(y_hor, 0.0, float(cam.frame_height)))


def compute_parallax_mrad(range_m: float, optics: OpticsConfig) -> Tuple[float, float]:
    """Returns (parallax_pan_mrad, parallax_tilt_mrad) for camera offset relative to bore."""
    safe_range_m = max(0.5, float(range_m))
    pan_mrad = (-optics.camera_offset_x_mm / (safe_range_m * 1000.0)) * 1000.0
    tilt_mrad = (optics.camera_offset_y_mm / (safe_range_m * 1000.0)) * 1000.0
    return pan_mrad, tilt_mrad


def image_px_to_world_mrad(
    cx: float,
    cy: float,
    pan_at_capture: float,
    tilt_at_capture: float,
    range_m: float,
    cam: CameraConfig,
    optics: OpticsConfig,
) -> Tuple[int, int]:
    """Converts a canonical 1280x720 image point into parallax-compensated world (pan, tilt) mrad."""
    p_pan, p_tilt = compute_parallax_mrad(range_m, optics)
    dx = -(cx - cam.optical_center[0])
    dy = -(cy - cam.optical_center[1])
    opt_pan = math.atan2(dx, optics.focal_length_px) * 1000.0 + p_pan
    opt_tilt = math.atan2(dy, optics.focal_length_px) * 1000.0 + p_tilt
    return int(round(pan_at_capture + opt_pan)), int(round(tilt_at_capture + opt_tilt))


def project_world_to_image_px(
    world_pan: float,
    world_tilt: float,
    cur_pan: float,
    cur_tilt: float,
    range_m: float,
    cam: CameraConfig,
    optics: OpticsConfig,
) -> Tuple[float, float]:
    """
    Inverse projection: projects a world-space target (pan, tilt) back into current
    canonical 1280x720 pixel coordinates using live turret telemetry. Prevents stale
    tracking crops during rapid turret slews.
    """
    p_pan, p_tilt = compute_parallax_mrad(range_m, optics)
    opt_pan = float(np.clip((world_pan - cur_pan) - p_pan, -1100.0, 1100.0))
    opt_tilt = float(np.clip((world_tilt - cur_tilt) - p_tilt, -1100.0, 1100.0))
    dx = optics.focal_length_px * math.tan(opt_pan / 1000.0)
    dy = optics.focal_length_px * math.tan(opt_tilt / 1000.0)
    cx = float(np.clip(cam.optical_center[0] - dx, 0.0, float(cam.frame_width)))
    cy = float(np.clip(cam.optical_center[1] - dy, 0.0, float(cam.frame_height)))
    return cx, cy


def compute_drag_ballistics(
    range_m: float,
    current_tilt_mrad: float,
    tgt_v_pan: float,
    tgt_v_tilt: float,
    bal: BallisticsConfig,
    optics: OpticsConfig,
) -> Tuple[float, float, float, float]:
    """
    Computes (tof_sec, drop_mrad, lead_pan_mrad, lead_tilt_mrad) under exponential drag.
    """
    if range_m <= 0.0:
        return 0.0, 0.0, 0.0, 0.0
    tof_sec = (math.exp(bal.drag_k * range_m) - 1.0) / (bal.drag_k * bal.muzzle_velocity_m_s)
    tilt_rad = current_tilt_mrad / 1000.0
    drop_m = 0.5 * (bal.gravity_m_s2 * math.cos(tilt_rad)) * (tof_sec ** 2)
    drop_mrad = (drop_m / range_m) * 1000.0
    return tof_sec, drop_mrad, tgt_v_pan * tof_sec, tgt_v_tilt * tof_sec


def evaluate_ground_plane(
    box_xyxy,
    tilt_at_cap: float,
    stadia_range_m: float,
    max_animal_height_m: float,
    cam: CameraConfig,
    optics: OpticsConfig,
) -> Tuple[bool, Optional[str], float]:
    """
    Checks above-horizon feet veto, ground-plane physical height veto, and returns
    (accepted, reject_reason, fused_range_m).
    """
    _, by1, _, by2 = box_xyxy
    bh_px = float(max(1, by2 - by1))
    y_bottom = float(by2)

    y_hor = compute_horizon_y_720(tilt_at_cap, cam, optics)
    min_feet_y = y_hor - optics.horizon_feet_margin_720
    if y_bottom < min_feet_y:
        return False, f"ABOVE_HORIZON(y2:{by2}<{min_feet_y:.0f})", stadia_range_m

    opt_dep_mrad = math.atan2(y_bottom - cam.optical_center[1], optics.focal_length_px) * 1000.0
    net_dep_mrad = opt_dep_mrad - float(tilt_at_cap) - optics.ground_slope_mrad
    if net_dep_mrad >= 25.0 and y_bottom < (cam.frame_height - 8):
        r_ground = optics.camera_mount_height_m / math.sin(net_dep_mrad / 1000.0)
        est_height_m = (bh_px * r_ground) / optics.focal_length_px
        if est_height_m > max_animal_height_m:
            return False, f"HEIGHT({est_height_m:.2f}m)", stadia_range_m
        r_fused = (0.70 * r_ground) + (0.30 * stadia_range_m)
        clamped = float(np.clip(r_fused, optics.min_valid_range_m, optics.max_valid_range_m))
        return True, None, clamped

    return True, None, stadia_range_m
