"""Pure mathematical functions for horizon geometry, parallax, drag ballistics, projections, and target kinematics."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np

from bombardeer_pkg.config import BallisticsConfig, CameraConfig, OpticsConfig

# Western Virginia mature white-tailed buck head-up + antlers (~6'0" = 1.8288 m)
DEFAULT_MAX_DEER_HEIGHT_M = 6.0 * 12.0 * 0.0254


@dataclass(frozen=True)
class TargetKinematics:
    """Range- and optics-derived angular kinematic bounds for a live white-tailed deer."""

    max_v_pan_mrad_s: float
    max_v_tilt_mrad_s: float
    accel_pan_mrad_s2: float
    accel_tilt_mrad_s2: float
    meas_var_pan: float
    meas_var_tilt: float
    max_step_margin_pan_mrad: float
    max_step_margin_tilt_mrad: float
    max_radial_speed_m_s: float = 6.0


def compute_target_kinematics(
    range_m: float,
    optics: OpticsConfig,
    max_deer_speed_m_s: float = 12.0,
    max_deer_accel_m_s2: float = 3.5,
    max_radial_speed_m_s: float = 6.0,
    max_vertical_speed_m_s: float = 0.6,
    max_vertical_accel_m_s2: float = 0.5,
    bbox_jitter_pan_px: float = 3.0,
    bbox_jitter_tilt_px: float = 4.5,
    hw_max_v_pan_mrad_s: float = 850.0,
    hw_max_v_tilt_mrad_s: float = 180.0,
) -> TargetKinematics:
    """
    Derives world-space angular velocity/acceleration limits and measurement
    variances from live white-tailed deer speeds, target range, and camera optics.

    Physical basis (Western Virginia white-tailed deer):
      - Lateral sprint up to ~12 m/s (~27 mph) projected at range r (capped at HW 850 mrad/s).
      - Radial charge/retreat up to ~6 m/s gives ground-plane tilt rate
        omega_tilt = 1000 * H * v_r / (r^2 + H^2), plus modest vertical bounding.
      - Sustained tilt velocity is capped at 180 mrad/s so vertical box jitter or
        transient leaps can never inject runaway upward/downward coasting.
    """
    r = float(np.clip(range_m, max(1.0, optics.min_valid_range_m), optics.max_valid_range_m))
    h = max(0.5, float(optics.camera_mount_height_m))
    mrad_per_px = float(optics.mrad_per_pixel)

    # Pan: lateral running speed & acceleration projected at range r
    max_v_pan = min(hw_max_v_pan_mrad_s, (1000.0 * max_deer_speed_m_s) / r)
    accel_pan = float(np.clip((1000.0 * max_deer_accel_m_s2) / r, 25.0, 400.0))

    # Tilt: exact ground-plane radial motion H*v_r/(r^2 + H^2) + modest vertical bounding
    r2_h2 = (r * r) + (h * h)
    ground_v_tilt = (1000.0 * h * max_radial_speed_m_s) / r2_h2
    bound_v_tilt = (1000.0 * max_vertical_speed_m_s) / r
    max_v_tilt = min(hw_max_v_tilt_mrad_s, ground_v_tilt + bound_v_tilt)

    ground_a_tilt = (1000.0 * h * min(max_deer_accel_m_s2, 2.5)) / r2_h2
    bound_a_tilt = (1000.0 * max_vertical_accel_m_s2) / r
    accel_tilt = float(np.clip(ground_a_tilt + bound_a_tilt, 12.0, 80.0))

    # Measurement variance from pixel box-center uncertainty and lens mrad/px
    sigma_pan = max(1.5, bbox_jitter_pan_px * mrad_per_px)
    sigma_tilt = max(2.5, bbox_jitter_tilt_px * mrad_per_px)

    return TargetKinematics(
        max_v_pan_mrad_s=max_v_pan,
        max_v_tilt_mrad_s=max_v_tilt,
        accel_pan_mrad_s2=accel_pan,
        accel_tilt_mrad_s2=accel_tilt,
        meas_var_pan=sigma_pan * sigma_pan,
        meas_var_tilt=sigma_tilt * sigma_tilt,
        max_step_margin_pan_mrad=3.0 * sigma_pan,
        max_step_margin_tilt_mrad=3.0 * sigma_tilt,
        max_radial_speed_m_s=float(max_radial_speed_m_s),
    )


def compute_horizon_y_720(
    tilt_mrad: float,
    cam: CameraConfig,
    optics: OpticsConfig,
    x_px: Optional[float] = None,
    roll_mrad: float = 0.0,
) -> float:
    """Computes horizon Y coordinate in canonical 1280x720 space for given turret tilt and optional camera roll."""
    angle_rad = (float(tilt_mrad) + optics.ground_slope_mrad) / 1000.0
    y_hor = cam.optical_center[1] + (optics.focal_length_px * math.tan(angle_rad)) + optics.horizon_y_offset_720
    if x_px is not None and abs(roll_mrad) > 1e-4:
        y_hor += (float(x_px) - cam.optical_center[0]) * math.tan(float(roll_mrad) / 1000.0)
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
    roll_mrad: float = 0.0,
    yard_top_mrad: Optional[float] = None,
    use_ground_range: bool = True,
) -> Tuple[bool, Optional[str], float]:
    """
    Checks above-horizon/treeline feet veto, ground-plane physical height veto, and returns
    (accepted, reject_reason, fused_range_m).

    - When yard_top_mrad is provided (from daylight boot sweep), it defines the visual
      yard ceiling for ABOVE_HORIZON while optics.ground_slope_mrad defines the metric
      ground plane for r_ground and HEIGHT.
    - When use_ground_range is False (new site prior to walk-test or passive convergence),
      ABOVE_HORIZON still protects against sky/treetop false positives while returning
      stadia_range_m without risking false HEIGHT rejections from a stale mount height.
    """
    bx1, by1, bx2, by2 = box_xyxy
    bh_px = float(max(1, by2 - by1))
    x_center = 0.5 * float(bx1 + bx2)
    y_bottom = float(by2)

    roll_dy = (
        (x_center - cam.optical_center[0]) * math.tan(float(roll_mrad) / 1000.0)
        if abs(roll_mrad) > 1e-4
        else 0.0
    )

    if yard_top_mrad is not None:
        ceil_rad = (float(tilt_at_cap) + float(yard_top_mrad)) / 1000.0
        y_ceiling = float(np.clip(
            cam.optical_center[1] + (optics.focal_length_px * math.tan(ceil_rad)) + roll_dy,
            0.0,
            float(cam.frame_height),
        ))
    else:
        y_ceiling = compute_horizon_y_720(tilt_at_cap, cam, optics, x_px=x_center, roll_mrad=roll_mrad)

    min_feet_y = y_ceiling - optics.horizon_feet_margin_720
    if y_bottom < min_feet_y:
        return False, f"ABOVE_HORIZON(y2:{by2}<{min_feet_y:.0f})", stadia_range_m

    if not use_ground_range:
        return True, None, float(np.clip(stadia_range_m, optics.min_valid_range_m, optics.max_valid_range_m))

    y_bottom_unrolled = y_bottom - roll_dy
    opt_dep_mrad = math.atan2(y_bottom_unrolled - cam.optical_center[1], optics.focal_length_px) * 1000.0
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


def compute_deer_head_ceiling_mrad(
    optics: OpticsConfig,
    max_deer_height_m: float = DEFAULT_MAX_DEER_HEIGHT_M,
    far_ground_mrad: Optional[float] = None,
    terrain_margin_mrad: float = 14.0,
) -> Tuple[float, float, float]:
    """
    Computes the highest elevation angle (in depression-positive mrad; lower / more
    negative = higher in image) that a standing deer of physical height
    max_deer_height_m can reach anywhere between optics.min_valid_range_m and the
    far yard edge.

    Pinhole perspective law on a ground plane with slope s and camera height H_cam:
      dep_head(Z) = s + atan((H_cam - H_deer) / Z)

    Returns (ceiling_mrad, far_head_mrad, near_head_mrad) in depression-positive mrad.
    """
    h_cam = max(0.35, float(optics.camera_mount_height_m))
    h_deer = max(0.50, float(max_deer_height_m))
    s_mrad = float(optics.ground_slope_mrad)
    r_min = max(2.0, float(optics.min_valid_range_m))
    r_max = max(r_min + 1.0, float(optics.max_valid_range_m))

    r_far = float(np.clip(optics.max_engage_range_m, r_min, r_max))
    if far_ground_mrad is not None:
        net_far_dep_mrad = float(far_ground_mrad) - s_mrad
        if net_far_dep_mrad >= 15.0:
            r_est = h_cam / math.sin(min(1.2, net_far_dep_mrad / 1000.0))
            r_far = float(np.clip(r_est, r_min, r_max))

    dh = h_cam - h_deer
    near_head_mrad = s_mrad + math.atan2(dh, r_min) * 1000.0
    far_head_mrad = s_mrad + math.atan2(dh, r_far) * 1000.0
    if far_ground_mrad is not None:
        far_head_from_edge = float(far_ground_mrad) - (math.atan2(h_deer, r_far) * 1000.0)
        far_head_mrad = min(far_head_mrad, far_head_from_edge)

    ceiling_mrad = min(near_head_mrad, far_head_mrad) - float(terrain_margin_mrad)
    return float(ceiling_mrad), float(far_head_mrad), float(near_head_mrad)


def compute_target_elevation_ceiling_mrad(
    range_m: float,
    optics: OpticsConfig,
    max_deer_height_m: float = DEFAULT_MAX_DEER_HEIGHT_M,
    far_ground_mrad: Optional[float] = None,
    margin_mrad: float = 25.0,
) -> float:
    """
    Computes the maximum physically valid target elevation angle in turret
    world_tilt coordinates (+mrad = UP, -mrad = DOWN) for a deer at range_m.

    Negates depression-positive ground-plane geometry so motion_servo.py can
    clamp target base_tilt directly (before adding ballistic drop + trim),
    preventing upward slews into overhanging eaves or sky on both elevated
    (~2.3 m) and sawhorse (~1.0 m) mounts.
    """
    r_min = max(2.0, float(optics.min_valid_range_m))
    r_max = max(r_min + 1.0, float(optics.max_valid_range_m))
    r = float(np.clip(range_m, r_min, r_max))
    h_cam = max(0.35, float(optics.camera_mount_height_m))
    h_deer = max(0.50, float(max_deer_height_m))
    s_mrad = float(optics.ground_slope_mrad)

    dep_head_mrad = s_mrad + math.atan2(h_cam - h_deer, r) * 1000.0
    if far_ground_mrad is not None:
        r_far = float(np.clip(optics.max_engage_range_m, r_min, r_max))
        net_far_dep_mrad = float(far_ground_mrad) - s_mrad
        if net_far_dep_mrad >= 15.0:
            r_est = h_cam / math.sin(min(1.2, net_far_dep_mrad / 1000.0))
            r_far = float(np.clip(r_est, r_min, r_max))
        dep_from_edge = float(far_ground_mrad) - (math.atan2(h_deer, r_far) * 1000.0)
        dep_head_mrad = min(dep_head_mrad, dep_from_edge)

    # Convert depression-positive angle to elevation-positive turret tilt + margin
    return float(-dep_head_mrad + float(margin_mrad))
