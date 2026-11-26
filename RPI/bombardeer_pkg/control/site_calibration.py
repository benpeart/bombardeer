"""
100% software / camera-only site self-calibration for zero-config deployments:
  1. Daylight Boot Sweep: pan-indexed visual yard-ceiling / treeline profile
     (yard_top_mrad(pan)) + camera roll estimation + home-view scene fingerprint
     to detect site relocation across reboots.
  2. 5'9" (1.7526 m) Walk-Test Metric Fit: exact projective Huber-IRLS regression
     from MegaDetector PERSON boxes to solve camera_mount_height_m and
     ground_slope_mrad with zero small-angle approximation.
  3. Passive Stadia-to-Ground Convergence: fully hands-off fallback that uses
     stadia range safely on an uncalibrated site until enough wildlife observations
     across depth converge on (camera_mount_height_m, ground_slope_mrad).
"""

from dataclasses import asdict, dataclass, field, replace
import json
import logging
import math
import os
import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from bombardeer_pkg.config import BombardeerConfig, CameraConfig, OpticsConfig

log = logging.getLogger("Bombardeer")

# Operator default height: 5'9" = 69 inches = 1.7526 m
DEFAULT_OPERATOR_HEIGHT_M = 69.0 * 0.0254  # 1.7526 m


@dataclass
class SiteCalibrationData:
    """Persisted site-specific calibration profile saved in site_calibration.json."""

    version: int = 1
    updated_unix: float = 0.0
    boot_sweep_completed: bool = False
    site_fingerprint: List[float] = field(default_factory=list)
    yard_top_profile: Dict[str, float] = field(default_factory=dict)  # str(pan_mrad) -> yard_top_mrad
    roll_mrad: float = 0.0
    metric_source: str = "UNSET"  # "UNSET" | "PASSIVE_STADIA" | "WALK_TEST"
    camera_mount_height_m: Optional[float] = None
    ground_slope_mrad: Optional[float] = None
    operator_height_m: float = DEFAULT_OPERATOR_HEIGHT_M
    walk_test_samples: int = 0
    walk_test_residual_mrad: float = 0.0
    passive_samples: int = 0


def compute_scene_fingerprint(frame_bgr_or_rgb: np.ndarray) -> List[float]:
    """
    Computes a lighting-invariant 16x9 normalized luma signature of the upper/mid
    scene (excluding the lower foreground band and right-side barrel) so rebooting
    in a new physical location triggers fresh auto-calibration.
    """
    h, w = frame_bgr_or_rgb.shape[:2]
    gray = cv2.cvtColor(frame_bgr_or_rgb, cv2.COLOR_BGR2GRAY) if frame_bgr_or_rgb.ndim == 3 else frame_bgr_or_rgb
    roi = gray[int(0.08 * h):int(0.75 * h), int(0.05 * w):int(0.78 * w)]
    small = cv2.resize(roi, (16, 9), interpolation=cv2.INTER_AREA).astype(np.float32).reshape(-1)
    mean = float(small.mean())
    std = float(small.std())
    if std < 1e-3:
        return [0.0] * int(small.size)
    normed = (small - mean) / std
    return [round(float(v), 4) for v in normed]


def compare_scene_fingerprints(fp_a: List[float], fp_b: List[float]) -> float:
    """Returns Pearson correlation in [-1.0, 1.0] between two scene fingerprints."""
    if not fp_a or not fp_b or len(fp_a) != len(fp_b):
        return 0.0
    a = np.asarray(fp_a, dtype=np.float64)
    b = np.asarray(fp_b, dtype=np.float64)
    na = float(np.linalg.norm(a))
    nb = float(np.linalg.norm(b))
    if na < 1e-6 or nb < 1e-6:
        return 0.0
    return float(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))


def estimate_daylight_horizon_and_roll(
    frame_bgr_or_rgb: np.ndarray,
    cam: CameraConfig,
    optics: OpticsConfig,
    tilt_mrad: float = 0.0,
) -> Tuple[Optional[float], Optional[float], float, float]:
    """
    Analyzes a canonical 1280x720 daylight frame to estimate:
      - yard_top_y_720: top row of walkable ground / treeline base (below sky & branches)
      - yard_top_mrad: tilt-referenced ceiling angle (so compute_horizon_y_720 matches yard_top_y_720)
      - roll_mrad: camera roll angle (positive = horizon slopes down to the right)
      - confidence: [0.0, 1.0] quality score (0.0 on dark/IR frames)
    """
    h, w = frame_bgr_or_rgb.shape[:2]
    gray = cv2.cvtColor(frame_bgr_or_rgb, cv2.COLOR_BGR2GRAY) if frame_bgr_or_rgb.ndim == 3 else frame_bgr_or_rgb

    mean_luma = float(gray.mean())
    std_luma = float(gray.std())
    if mean_luma < 45.0 or std_luma < 18.0:
        return None, None, 0.0, 0.0

    blur = cv2.GaussianBlur(gray, (9, 9), 0).astype(np.float32)
    gx_raw = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    abs_gx = np.abs(gx_raw)

    # Exclude far-right barrel zone and lower foreground mount/rail band
    x_start = int(0.05 * w)
    x_end = int(0.78 * w)
    y_min = int(0.10 * h)
    y_max = int(0.76 * h)

    # Ground appearance model sampled from lower walkable yard band (58%..76% height)
    if frame_bgr_or_rgb.ndim == 3:
        lab = cv2.cvtColor(frame_bgr_or_rgb, cv2.COLOR_BGR2LAB).astype(np.float32)
        g_patch = lab[int(0.58 * h):int(0.76 * h), x_start:x_end].reshape(-1, 3)
        g_med = np.median(g_patch, axis=0)
        g_mad = np.median(np.abs(g_patch - g_med), axis=0)
        d_ground = np.linalg.norm(lab - g_med, axis=2)
        ground_dist_thresh = max(24.0, float(np.linalg.norm(g_mad)) * 3.2)
    else:
        g_patch = blur[int(0.58 * h):int(0.76 * h), x_start:x_end]
        g_med = float(np.median(g_patch))
        g_mad = float(np.median(np.abs(g_patch - g_med)))
        d_ground = np.abs(blur - g_med)
        ground_dist_thresh = max(22.0, g_mad * 3.2)

    n_strips = 24
    strip_edges = np.linspace(x_start, x_end, n_strips + 1, dtype=int)
    strip_y = []

    top_luma_ref = float(np.percentile(blur[:int(0.22 * h), x_start:x_end], 75))
    sky_thresh = max(125.0, 0.78 * top_luma_ref)

    ground_gx_ref = float(np.median(abs_gx[int(0.60 * h):int(0.76 * h), x_start:x_end]))
    canopy_gx_thresh = max(25.0, ground_gx_ref * 2.4)
    kernel = np.ones(17, dtype=np.float32) / 17.0

    for i in range(n_strips):
        xs, xe = int(strip_edges[i]), int(strip_edges[i + 1])
        if xe <= xs:
            continue
        col_luma = blur[:, xs:xe].mean(axis=1)
        col_gx = np.convolve(abs_gx[:, xs:xe].mean(axis=1), kernel, mode="same")
        col_dg = np.convolve(d_ground[:, xs:xe].mean(axis=1), kernel, mode="same")

        is_non_ground = (
            (col_luma >= sky_thresh)
            | (col_gx >= canopy_gx_thresh)
            | (col_dg >= ground_dist_thresh)
        ).astype(np.float32)
        cs = np.concatenate([[0.0], np.cumsum(is_non_ground)])
        ys = np.arange(y_min + 20, y_max - 20)
        if ys.size == 0:
            continue
        above_non_ground = cs[ys] - cs[y_min]
        below_ground = (y_max - ys) - (cs[y_max] - cs[ys])
        best_y = int(ys[int(np.argmax(above_non_ground + below_ground))])
        strip_y.append(float(best_y))

    if len(strip_y) < 4:
        return None, None, 0.0, 0.0

    arr_y = np.asarray(strip_y, dtype=np.float64)
    med_y = float(np.median(arr_y))
    mad_y = float(np.median(np.abs(arr_y - med_y)))
    inliers = arr_y[np.abs(arr_y - med_y) <= max(25.0, 2.5 * mad_y)]
    # 35th percentile tracks the actual far ground edge (tree bases) cleanly
    yard_top_y = float(np.percentile(inliers, 35)) if inliers.size >= 4 else med_y

    # Scale to canonical 1280x720 if frame height differs
    y_720 = yard_top_y * (float(cam.frame_height) / float(h))
    opt_dep_mrad = math.atan2(y_720 - cam.optical_center[1], optics.focal_length_px) * 1000.0
    yard_top_mrad = float(np.clip(opt_dep_mrad - float(tilt_mrad), -320.0, 320.0))

    # --- Camera Roll Estimation from Smooth Near-Vertical Edges (trunks, posts, corners) ---
    roll_roi = cv2.GaussianBlur(gray[int(0.12 * h):int(0.72 * h), x_start:x_end], (5, 5), 1.2)
    rgx = cv2.Sobel(roll_roi, cv2.CV_32F, 1, 0, ksize=5)
    rgy = cv2.Sobel(roll_roi, cv2.CV_32F, 0, 1, ksize=5)
    rmag = np.hypot(rgx, rgy)
    rmag_thresh = max(80.0, float(np.percentile(rmag, 92)))
    vert_mask = (rmag >= rmag_thresh) & (np.abs(rgx) > 3.0 * np.abs(rgy))
    roll_rad = 0.0
    if np.count_nonzero(vert_mask) >= 150:
        angles = np.arctan2(rgy[vert_mask], rgx[vert_mask])
        angles = np.where(angles > (math.pi / 2.0), angles - math.pi, angles)
        angles = np.where(angles < -(math.pi / 2.0), angles + math.pi, angles)
        near_v = angles[np.abs(angles) <= math.radians(12.0)]
        if near_v.size >= 80:
            roll_rad = float(np.median(near_v))

    roll_mrad = float(np.clip(roll_rad * 1000.0, -210.0, 210.0))
    spread_score = max(0.0, 1.0 - (mad_y / 100.0))
    luma_score = min(1.0, (mean_luma - 45.0) / 50.0)
    confidence = float(np.clip(0.55 * spread_score + 0.45 * luma_score, 0.0, 1.0))
    return y_720, yard_top_mrad, roll_mrad, confidence


def fit_ground_plane_huber(
    inv_z_samples: List[float],
    tan_dep_samples: List[float],
    min_samples: int = 6,
    min_depth_span_m: float = 3.0,
    min_height_m: float = 0.35,
    max_height_m: float = 6.0,
    max_abs_slope_mrad: float = 250.0,
) -> Optional[Tuple[float, float, float]]:
    """
    Exact projective Huber-IRLS fit of the ground plane equation:
        tan(dep_feet) = tan(ground_slope) + camera_mount_height_m * (1 / Z)

    Returns (camera_mount_height_m, ground_slope_mrad, rms_residual_mrad) or None
    if sample count, depth spread, or physical bounds are not satisfied.
    """
    if len(inv_z_samples) < min_samples or len(inv_z_samples) != len(tan_dep_samples):
        return None

    u = np.asarray(inv_z_samples, dtype=np.float64)
    v = np.asarray(tan_dep_samples, dtype=np.float64)
    valid = np.isfinite(u) & np.isfinite(v) & (u > 0.01) & (u < 0.60)
    u = u[valid]
    v = v[valid]
    if u.size < min_samples:
        return None

    depths = 1.0 / u
    depth_span = float(np.percentile(depths, 90) - np.percentile(depths, 10))
    if depth_span < min_depth_span_m:
        return None

    A = np.column_stack([np.ones_like(u), u])
    w = np.ones_like(u)
    sol = np.zeros(2, dtype=np.float64)
    delta = 0.015  # ~15 mrad Huber threshold

    for _ in range(8):
        sw = np.sqrt(w)
        Aw = A * sw[:, None]
        vw = v * sw
        try:
            sol, _, _, _ = np.linalg.lstsq(Aw, vw, rcond=None)
        except np.linalg.LinAlgError:
            return None
        resid = v - (A @ sol)
        abs_r = np.abs(resid)
        w = np.where(abs_r <= delta, 1.0, delta / np.maximum(abs_r, 1e-9))

    tan_slope = float(sol[0])
    height_m = float(sol[1])
    slope_mrad = math.atan(tan_slope) * 1000.0

    if not (min_height_m <= height_m <= max_height_m):
        return None
    if abs(slope_mrad) > max_abs_slope_mrad:
        return None

    # Weighted angular RMS residual in mrad
    pred_ang = np.arctan(A @ sol) * 1000.0
    obs_ang = np.arctan(v) * 1000.0
    rms_mrad = float(math.sqrt(float(np.average((obs_ang - pred_ang) ** 2, weights=w))))
    return height_m, slope_mrad, rms_mrad


def _interpolate_pan_profile(profile: Dict[str, float], pan_mrad: float) -> Optional[float]:
    if not profile:
        return None
    pts = sorted((float(k), float(v)) for k, v in profile.items())
    if len(pts) == 1 or pan_mrad <= pts[0][0]:
        return pts[0][1]
    if pan_mrad >= pts[-1][0]:
        return pts[-1][1]
    for i in range(len(pts) - 1):
        p0, v0 = pts[i]
        p1, v1 = pts[i + 1]
        if p0 <= pan_mrad <= p1:
            if abs(p1 - p0) < 1e-6:
                return v0
            alpha = (pan_mrad - p0) / (p1 - p0)
            return v0 + alpha * (v1 - v0)
    return pts[-1][1]


class SiteCalibrationManager:
    """
    Thread-safe lifecycle manager for zero-config site calibration:
      - Auto-loads/saves site_calibration.json alongside config.yaml.
      - Runs automatic daylight boot sweep when uncalibrated or relocated.
      - Runs interactive 5'9" Walk-Test calibration safely (disarmed + stationary).
      - Runs passive stadia-to-ground convergence as a hands-off fallback.
    """

    SWEEP_SETTLE_SEC = 0.35
    SWEEP_STOP_TIMEOUT_SEC = 4.5
    WALK_TEST_TIMEOUT_SEC = 45.0
    WALK_TEST_MIN_SAMPLES = 8
    WALK_TEST_AUTO_FINISH_SAMPLES = 16
    WALK_TEST_MIN_DEPTH_SPAN_M = 4.0
    PASSIVE_MIN_SAMPLES = 15
    PASSIVE_MIN_DEPTH_SPAN_M = 5.0
    SITE_FINGERPRINT_MATCH_CORR = 0.65
    ROLL_WARN_MRAD = 44.0  # ~2.5 deg

    def __init__(self, cfg: BombardeerConfig, cal_path: Optional[str] = None, auto_boot_sweep: bool = True):
        self.cfg = cfg
        self.cal_path = cal_path or os.path.join(cfg.base_dir, "site_calibration.json")
        self.auto_boot_sweep = bool(auto_boot_sweep)
        self._lock = threading.Lock()
        self.data = SiteCalibrationData()

        # Boot sweep state machine
        self._sweep_state = "IDLE"  # IDLE | CHECK_SITE | SWEEPING | DONE
        self._sweep_stops: List[int] = []
        self._sweep_idx = 0
        self._sweep_cmd_sent = False
        self._sweep_cmd_mono = 0.0
        self._sweep_settled_since: Optional[float] = None
        self._sweep_results: Dict[str, float] = {}
        self._sweep_rolls: List[float] = []
        self._sweep_returning_home = False

        # Interactive 5'9" Walk-Test state
        self._walk_active = False
        self._walk_start_mono = 0.0
        self._walk_operator_height_m = DEFAULT_OPERATOR_HEIGHT_M
        self._walk_inv_z: List[float] = []
        self._walk_tan_dep: List[float] = []
        self._walk_last_sample_mono = 0.0

        # Passive stadia-to-ground observation buffer
        self._passive_inv_z: List[float] = []
        self._passive_tan_dep: List[float] = []
        self._passive_last_sample_mono = 0.0

        self._load()
        if self.auto_boot_sweep:
            # Always check site fingerprint on first daylight home frame; sweep if new/uncalibrated
            self._sweep_state = "CHECK_SITE" if self.data.boot_sweep_completed else "PENDING_DAYLIGHT"

    # --- Persistence ---

    def _load(self):
        if not os.path.exists(self.cal_path):
            return
        try:
            with open(self.cal_path, "r", encoding="utf-8") as fh:
                raw = json.load(fh)
            if not isinstance(raw, dict):
                return
            d = self.data
            d.version = int(raw.get("version", 1))
            d.updated_unix = float(raw.get("updated_unix", 0.0))
            d.boot_sweep_completed = bool(raw.get("boot_sweep_completed", False))
            d.site_fingerprint = [float(x) for x in raw.get("site_fingerprint", [])]
            d.yard_top_profile = {str(k): float(v) for k, v in raw.get("yard_top_profile", {}).items()}
            d.roll_mrad = float(raw.get("roll_mrad", 0.0))
            d.metric_source = str(raw.get("metric_source", "UNSET"))
            h_val = raw.get("camera_mount_height_m")
            s_val = raw.get("ground_slope_mrad")
            d.camera_mount_height_m = float(h_val) if h_val is not None else None
            d.ground_slope_mrad = float(s_val) if s_val is not None else None
            d.operator_height_m = float(raw.get("operator_height_m", DEFAULT_OPERATOR_HEIGHT_M))
            d.walk_test_samples = int(raw.get("walk_test_samples", 0))
            d.walk_test_residual_mrad = float(raw.get("walk_test_residual_mrad", 0.0))
            d.passive_samples = int(raw.get("passive_samples", 0))
            log.info(
                f"[SITE-CAL] Loaded {self.cal_path} | Sweep:{d.boot_sweep_completed} "
                f"Metric:{d.metric_source} H:{d.camera_mount_height_m} Slope:{d.ground_slope_mrad} Roll:{d.roll_mrad:.1f}mrad"
            )
        except Exception as exc:
            log.warning(f"[SITE-CAL] Failed to load {self.cal_path}: {exc}")

    def _save_unlocked(self):
        self.data.updated_unix = time.time()
        parent = os.path.dirname(os.path.abspath(self.cal_path))
        if parent:
            os.makedirs(parent, exist_ok=True)
        tmp_path = f"{self.cal_path}.tmp"
        try:
            with open(tmp_path, "w", encoding="utf-8") as fh:
                json.dump(asdict(self.data), fh, indent=2)
            os.replace(tmp_path, self.cal_path)
        except Exception as exc:
            log.error(f"[SITE-CAL] Failed to save {self.cal_path}: {exc}")

    # --- Motion Ownership & Status ---

    def is_motion_owned(self) -> bool:
        """True while boot sweep or walk-test owns turret motion (MotionServo must yield & stay safe)."""
        with self._lock:
            return self._walk_active or (self._sweep_state == "SWEEPING")

    def is_walk_test_active(self) -> bool:
        with self._lock:
            return self._walk_active

    def get_hud_banner(self) -> Optional[str]:
        with self._lock:
            if self._walk_active:
                depths = [1.0 / u for u in self._walk_inv_z if u > 0.01]
                span = (max(depths) - min(depths)) if len(depths) >= 2 else 0.0
                return (
                    f"WALK-TEST ({self._walk_operator_height_m * 3.28084:.2f}ft): "
                    f"{len(self._walk_inv_z)}/{self.WALK_TEST_MIN_SAMPLES}+ pts | Span:{span:.1f}m"
                )
            if self._sweep_state == "SWEEPING":
                if self._sweep_returning_home:
                    return "SITE BOOT SWEEP: Returning home..."
                return f"SITE BOOT SWEEP: Stop {min(self._sweep_idx + 1, len(self._sweep_stops))}/{len(self._sweep_stops)}"
            return None

    def get_status_dict(self) -> dict:
        with self._lock:
            depths = [1.0 / u for u in self._walk_inv_z if u > 0.01]
            span = round((max(depths) - min(depths)) if len(depths) >= 2 else 0.0, 2)
            return {
                "sweep_state": self._sweep_state,
                "boot_sweep_completed": self.data.boot_sweep_completed,
                "yard_top_stops": len(self.data.yard_top_profile),
                "roll_mrad": round(self.data.roll_mrad, 1),
                "roll_deg": round(math.degrees(self.data.roll_mrad / 1000.0), 2),
                "metric_source": self.data.metric_source,
                "camera_mount_height_m": (
                    round(self.data.camera_mount_height_m, 3)
                    if self.data.camera_mount_height_m is not None
                    else None
                ),
                "ground_slope_mrad": (
                    round(self.data.ground_slope_mrad, 1)
                    if self.data.ground_slope_mrad is not None
                    else None
                ),
                "operator_height_m": round(self.data.operator_height_m, 4),
                "walk_test_active": self._walk_active,
                "walk_test_samples": len(self._walk_inv_z) if self._walk_active else self.data.walk_test_samples,
                "walk_test_depth_span_m": span,
                "walk_test_residual_mrad": round(self.data.walk_test_residual_mrad, 2),
                "passive_samples": len(self._passive_inv_z) if self.data.metric_source == "UNSET" else self.data.passive_samples,
            }

    # --- Optics & Ground-Plane Projections for Runtime Consumers ---

    def get_tiling_optics(self, base_optics: OpticsConfig, pan_mrad: float) -> OpticsConfig:
        """
        Returns an OpticsConfig view for tiling.py so SCAN-12MP/SCAN-3MP tiles
        automatically hug the scanned treeline/yard ceiling at pan_mrad.
        """
        with self._lock:
            yt = _interpolate_pan_profile(self.data.yard_top_profile, pan_mrad)
            if yt is not None:
                return replace(base_optics, ground_slope_mrad=yt, horizon_y_offset_720=0.0)
            if self.data.ground_slope_mrad is not None:
                return replace(base_optics, ground_slope_mrad=self.data.ground_slope_mrad, horizon_y_offset_720=0.0)
            return base_optics

    def get_ground_plane_params(
        self, base_optics: OpticsConfig, pan_mrad: float
    ) -> Tuple[OpticsConfig, Optional[float], float, bool]:
        """
        Returns (metric_optics, yard_top_mrad, roll_mrad, use_ground_range) for
        evaluate_ground_plane().
        """
        with self._lock:
            yt = _interpolate_pan_profile(self.data.yard_top_profile, pan_mrad)
            roll = float(self.data.roll_mrad)

            if (
                self.data.metric_source in ("WALK_TEST", "PASSIVE_STADIA")
                and self.data.camera_mount_height_m is not None
                and self.data.ground_slope_mrad is not None
            ):
                metric_optics = replace(
                    base_optics,
                    camera_mount_height_m=self.data.camera_mount_height_m,
                    ground_slope_mrad=self.data.ground_slope_mrad,
                    horizon_y_offset_720=0.0,
                )
                return metric_optics, yt, roll, True

            # Uncalibrated new site: use stadia range safely until walk-test or passive convergence
            if self.data.boot_sweep_completed or self._sweep_state in ("PENDING_DAYLIGHT", "SWEEPING"):
                return base_optics, yt, roll, False

            # Legacy fallback when auto_boot_sweep is disabled and no site file exists
            return base_optics, yt, roll, True

    # --- Manual Controls (Web API / CLI) ---

    def _build_sweep_stops(self) -> List[int]:
        pmin = max(self.cfg.control.pan_min_mrad, -750)
        pmax = min(self.cfg.control.pan_max_mrad, 750)
        if pmax - pmin < 200:
            return [0]
        raw = np.linspace(pmin, pmax, 7)
        stops = sorted({int(round(x / 50.0) * 50) for x in raw} | {0})
        return stops

    def start_boot_sweep(self, force: bool = True) -> Tuple[bool, str]:
        with self._lock:
            if self._walk_active:
                return False, "Walk-test is currently active."
            if not force and self.data.boot_sweep_completed:
                return False, "Boot sweep already completed."
            self._sweep_stops = self._build_sweep_stops()
            self._sweep_idx = 0
            self._sweep_cmd_sent = False
            self._sweep_settled_since = None
            self._sweep_results = {}
            self._sweep_rolls = []
            self._sweep_returning_home = False
            self._sweep_state = "SWEEPING"
            log.info(f"[SITE-CAL] Starting daylight boot sweep across {len(self._sweep_stops)} pan stops: {self._sweep_stops}")
            return True, f"Boot sweep started ({len(self._sweep_stops)} stops)."

    def start_walk_test(self, state, operator_height_m: Optional[float] = None) -> Tuple[bool, str]:
        """
        Safely disarms turret (OFF + TRIGGER 0 + clear_motion + level tilt=0) and begins collecting
        5'9" (1.7526 m) operator PERSON detections to solve height & ground slope.
        """
        h_m = float(operator_height_m) if operator_height_m is not None else self.data.operator_height_m
        if not (1.20 <= h_m <= 2.30):
            return False, "Operator height must be between 1.20m and 2.30m."

        # Safety first: disarm turret, force trigger OFF, clear motion, and level to tilt=0
        state.set_turret_mode("OFF")
        state.clear_primary_target()
        state.target_filter.reset()
        state.command_bus.request_trigger(0)
        state.command_bus.clear_motion()

        with self._lock:
            self._sweep_state = "DONE" if self.data.boot_sweep_completed else "PENDING_DAYLIGHT"
            self._walk_active = True
            self._walk_start_mono = time.monotonic()
            self._walk_operator_height_m = h_m
            self._walk_inv_z = []
            self._walk_tan_dep = []
            self._walk_last_sample_mono = 0.0
            self.data.operator_height_m = h_m

        # Level turret to tilt=0 at current pan (not dropped by _halt_pending)
        cur_pan, _, _, _ = state.get_instantaneous_telemetry()
        state.command_bus.request_setpoint(cur_pan, 0)
        state.set_alert(f"WALK-TEST ACTIVE (H={h_m:.3f}m): Walk 5-25m out & back", duration=6.0)
        log.info(f"[SITE-CAL] Walk-test started (operator_height={h_m:.4f}m / {h_m * 39.3701:.1f}in). Turret disarmed.")
        return True, f"Walk-test started for {h_m:.4f}m operator. Walk out 5-25m into the yard."

    def finish_walk_test(self, state=None) -> Tuple[bool, str]:
        with self._lock:
            if not self._walk_active:
                return False, "No walk-test is currently active."
            self._walk_active = False
            n = len(self._walk_inv_z)
            fit = fit_ground_plane_huber(
                self._walk_inv_z,
                self._walk_tan_dep,
                min_samples=self.WALK_TEST_MIN_SAMPLES,
                min_depth_span_m=self.WALK_TEST_MIN_DEPTH_SPAN_M,
            )
            if fit is None:
                depths = [1.0 / u for u in self._walk_inv_z if u > 0.01]
                span = (max(depths) - min(depths)) if len(depths) >= 2 else 0.0
                msg = (
                    f"Walk-test incomplete ({n} pts, {span:.1f}m span; need >= "
                    f"{self.WALK_TEST_MIN_SAMPLES} pts and >= {self.WALK_TEST_MIN_DEPTH_SPAN_M:.1f}m span)."
                )
                log.warning(f"[SITE-CAL] {msg}")
                if state is not None:
                    state.set_alert(msg, duration=5.0)
                return False, msg

            height_m, slope_mrad, rms_mrad = fit
            self.data.camera_mount_height_m = height_m
            self.data.ground_slope_mrad = slope_mrad
            self.data.metric_source = "WALK_TEST"
            self.data.walk_test_samples = n
            self.data.walk_test_residual_mrad = rms_mrad
            self._save_unlocked()

        msg = (
            f"SITE CAL LOCKED: H={height_m:.2f}m | Slope={slope_mrad:+.1f}mrad "
            f"(n={n}, rms={rms_mrad:.1f}mrad)"
        )
        log.info(f"[SITE-CAL] {msg}")
        if state is not None:
            state.set_alert(msg, duration=6.0)
        return True, msg

    def cancel_calibration(self, state=None):
        with self._lock:
            self._walk_active = False
            if self._sweep_state == "SWEEPING":
                self._sweep_state = "DONE" if self.data.boot_sweep_completed else "PENDING_DAYLIGHT"
                self._sweep_returning_home = False
        if state is not None:
            state.command_bus.request_halt()
            state.set_alert("SITE CALIBRATION CANCELED", duration=3.0)

    def reset_calibration(self, trigger_sweep: bool = True):
        with self._lock:
            op_h = self.data.operator_height_m
            self.data = SiteCalibrationData(operator_height_m=op_h)
            self._walk_active = False
            self._walk_inv_z.clear()
            self._walk_tan_dep.clear()
            self._passive_inv_z.clear()
            self._passive_tan_dep.clear()
            self._sweep_state = "PENDING_DAYLIGHT" if trigger_sweep else "IDLE"
            if os.path.exists(self.cal_path):
                try:
                    os.remove(self.cal_path)
                except OSError:
                    pass
        log.info("[SITE-CAL] Site calibration reset.")

    # --- Runtime Hooks Called from VisionMain & HailoWildlifeDetector ---

    def step_boot_sweep(
        self,
        frame_lores: np.ndarray,
        lux: float,
        is_monochrome: bool,
        pan_cap: int,
        tilt_cap: int,
        hw_moving: bool,
        state,
        mono_now: float,
        safe_for_sweep: bool,
    ):
        """
        Advances the automatic daylight boot sweep state machine cleanly on Core-0
        without blocking camera capture.
        """
        with self._lock:
            sw_state = self._sweep_state
            walk_active = self._walk_active

        if walk_active or sw_state in ("IDLE", "DONE"):
            return

        is_daylight = (not is_monochrome) and (lux >= 60.0) and (float(frame_lores.mean()) >= 45.0)
        if not is_daylight:
            if sw_state == "SWEEPING":
                with self._lock:
                    self._sweep_state = "PENDING_DAYLIGHT"
                    self._sweep_returning_home = False
                    self._sweep_cmd_sent = False
                state.set_alert("BOOT SWEEP DEFERRED: Waiting for daylight", duration=4.0)
                log.info("[SITE-CAL] Boot sweep deferred until daylight (insufficient Lux/color).")
            return

        # 1. Check whether saved calibration matches the current physical site
        if sw_state == "CHECK_SITE":
            if abs(pan_cap) <= 25 and abs(tilt_cap) <= 25 and (not hw_moving):
                fp_now = compute_scene_fingerprint(frame_lores)
                with self._lock:
                    corr = compare_scene_fingerprints(self.data.site_fingerprint, fp_now)
                    if corr >= self.SITE_FINGERPRINT_MATCH_CORR:
                        self._sweep_state = "DONE"
                        log.info(f"[SITE-CAL] Scene matches saved site (corr={corr:.2f}). Skipping boot sweep.")
                        return
                    log.warning(
                        f"[SITE-CAL] New deployment location detected (scene corr={corr:.2f} < "
                        f"{self.SITE_FINGERPRINT_MATCH_CORR:.2f}). Resetting metric height and scheduling boot sweep."
                    )
                    self.data.boot_sweep_completed = False
                    self.data.yard_top_profile = {}
                    self.data.metric_source = "UNSET"
                    self.data.camera_mount_height_m = None
                    self.data.ground_slope_mrad = None
                    self._passive_inv_z.clear()
                    self._passive_tan_dep.clear()
                    self._sweep_state = "PENDING_DAYLIGHT"
            return

        # 2. Start sweep once conditions are safe (connected, standby, no target/recording)
        if sw_state == "PENDING_DAYLIGHT":
            if safe_for_sweep and state.is_serial_connected():
                self.start_boot_sweep(force=True)
            return

        # 3. Execute multi-stop pan sweep at tilt=0
        if sw_state == "SWEEPING":
            if not state.is_serial_connected():
                self.cancel_calibration(state)
                return
            mode, _ = state.get_turret_mode()
            if mode in ("ARMED", "CALIBRATE"):
                self.cancel_calibration(state)
                return

            with self._lock:
                returning = self._sweep_returning_home
                stops = list(self._sweep_stops)
                idx = self._sweep_idx
                cmd_sent = self._sweep_cmd_sent
                cmd_mono = self._sweep_cmd_mono
                settled_since = self._sweep_settled_since

            if returning:
                if (abs(pan_cap) <= 15 and abs(tilt_cap) <= 15 and not hw_moving) or (mono_now - cmd_mono > self.SWEEP_STOP_TIMEOUT_SEC):
                    fp_home = compute_scene_fingerprint(frame_lores)
                    with self._lock:
                        self.data.site_fingerprint = fp_home
                        self.data.yard_top_profile = dict(self._sweep_results)
                        self.data.roll_mrad = float(np.median(self._sweep_rolls)) if self._sweep_rolls else 0.0
                        self.data.boot_sweep_completed = bool(self._sweep_results)
                        self._sweep_state = "DONE"
                        self._sweep_returning_home = False
                        roll_mrad = self.data.roll_mrad
                        n_stops = len(self.data.yard_top_profile)
                        self._save_unlocked()

                    log.info(
                        f"[SITE-CAL] Boot sweep complete: {n_stops} treeline stops mapped | "
                        f"Roll:{math.degrees(roll_mrad / 1000.0):+.2f}deg"
                    )
                    if abs(roll_mrad) >= self.ROLL_WARN_MRAD:
                        side = "right" if roll_mrad > 0 else "left"
                        state.set_alert(
                            f"LEVEL MOUNT: Tilted {abs(math.degrees(roll_mrad / 1000.0)):.1f}deg {side}",
                            duration=8.0,
                        )
                    else:
                        state.set_alert(f"SITE SWEEP OK ({n_stops} stops)", duration=3.5)
                return

            if idx >= len(stops):
                state.command_bus.request_setpoint(0, 0)
                with self._lock:
                    self._sweep_returning_home = True
                    self._sweep_cmd_mono = mono_now
                return

            target_pan = stops[idx]
            if not cmd_sent:
                state.command_bus.request_setpoint(target_pan, 0)
                with self._lock:
                    self._sweep_cmd_sent = True
                    self._sweep_cmd_mono = mono_now
                    self._sweep_settled_since = None
                return

            at_stop = (abs(pan_cap - target_pan) <= 18) and (abs(tilt_cap) <= 18) and (not hw_moving)
            timed_out = (mono_now - cmd_mono) > self.SWEEP_STOP_TIMEOUT_SEC
            if at_stop:
                if settled_since is None:
                    with self._lock:
                        self._sweep_settled_since = mono_now
                    return
                if (mono_now - settled_since) < self.SWEEP_SETTLE_SEC:
                    return
            elif not timed_out:
                return

            # Measure treeline/yard ceiling and roll at this pan stop
            _, yt_mrad, r_mrad, conf = estimate_daylight_horizon_and_roll(
                frame_lores, self.cfg.camera, self.cfg.optics, tilt_mrad=tilt_cap
            )
            with self._lock:
                if yt_mrad is not None and conf >= 0.38:
                    self._sweep_results[str(int(pan_cap))] = round(yt_mrad, 1)
                    self._sweep_rolls.append(r_mrad)
                self._sweep_idx += 1
                self._sweep_cmd_sent = False
                self._sweep_settled_since = None

    def ingest_walk_test_detections(
        self,
        veto_boxes: List[dict],
        tilt_at_cap: float,
        eff_capture_mono: float,
        state=None,
    ) -> bool:
        """
        Ingests PERSON detections during an active 5'9" Walk-Test.
        Returns True if the walk-test auto-completed on this batch.
        """
        with self._lock:
            if not self._walk_active:
                return False
            elapsed = time.monotonic() - self._walk_start_mono
            op_h = self._walk_operator_height_m
            roll_mrad = self.data.roll_mrad
            last_ts = self._walk_last_sample_mono

        if elapsed > self.WALK_TEST_TIMEOUT_SEC:
            self.finish_walk_test(state)
            return True

        if eff_capture_mono <= last_ts + 0.15:
            return False

        cam, optics = self.cfg.camera, self.cfg.optics
        # Filter to valid standing PERSON boxes FIRST so horizontal non-person artifacts
        # (such as the paintball barrel tip in the upper-right) never block walk calibration
        standing_persons = []
        for v in veto_boxes:
            if str(v.get("class", "")).upper() != "PERSON" or float(v.get("score", 0.0)) < 0.45:
                continue
            bx1, by1, bx2, by2 = v["box"]
            bw = float(bx2 - bx1)
            bh = float(by2 - by1)
            if by1 < 10 or by2 > (cam.frame_height - 10):
                continue
            if bh < 30.0 or bw < 10.0 or (bh / bw) < 1.30:
                continue
            standing_persons.append((bx1, by1, bx2, by2))

        # Require a single unambiguous standing operator in view
        if len(standing_persons) != 1:
            return False

        bx1, by1, bx2, by2 = standing_persons[0]

        xc = 0.5 * float(bx1 + bx2)
        roll_dy = (xc - cam.optical_center[0]) * math.tan(roll_mrad / 1000.0) if abs(roll_mrad) > 1e-4 else 0.0
        alpha = float(tilt_at_cap) / 1000.0
        dep_f = math.atan2((float(by2) - roll_dy) - cam.optical_center[1], optics.focal_length_px) - alpha
        dep_h = math.atan2((float(by1) - roll_dy) - cam.optical_center[1], optics.focal_length_px) - alpha

        inv_z = (math.tan(dep_f) - math.tan(dep_h)) / op_h
        if not (0.02 <= inv_z <= 0.35):  # ~2.8m to 50m operator distance
            return False

        auto_done = False
        with self._lock:
            if not self._walk_active:
                return False
            self._walk_inv_z.append(float(inv_z))
            self._walk_tan_dep.append(float(math.tan(dep_f)))
            self._walk_last_sample_mono = eff_capture_mono
            if len(self._walk_inv_z) >= self.WALK_TEST_AUTO_FINISH_SAMPLES:
                depths = [1.0 / u for u in self._walk_inv_z]
                span = float(np.percentile(depths, 90) - np.percentile(depths, 10))
                if span >= (self.WALK_TEST_MIN_DEPTH_SPAN_M + 1.5):
                    auto_done = True

        if auto_done:
            self.finish_walk_test(state)
            return True
        return False

    def observe_passive_wildlife(
        self,
        box_xyxy,
        tilt_at_cap: float,
        stadia_range_m: float,
        capture_mono: float,
    ) -> bool:
        """
        Hands-off fallback: accumulates (1 / stadia_range_m, tan(dep_feet)) from
        accepted wildlife detections when no Walk-Test has been run. Once 15+
        observations span >= 5 m in depth, solves (camera_mount_height_m,
        ground_slope_mrad) automatically and persists site_calibration.json.
        """
        with self._lock:
            if self.data.metric_source == "WALK_TEST":
                return False
            if capture_mono <= self._passive_last_sample_mono + 0.40:
                return False
            roll_mrad = self.data.roll_mrad

        bx1, by1, bx2, by2 = box_xyxy
        cam, optics = self.cfg.camera, self.cfg.optics
        if by1 < 10 or by2 > (cam.frame_height - 12):
            return False
        if not (optics.min_valid_range_m <= stadia_range_m <= optics.max_valid_range_m):
            return False

        xc = 0.5 * float(bx1 + bx2)
        roll_dy = (xc - cam.optical_center[0]) * math.tan(roll_mrad / 1000.0) if abs(roll_mrad) > 1e-4 else 0.0
        alpha = float(tilt_at_cap) / 1000.0
        dep_f = math.atan2((float(by2) - roll_dy) - cam.optical_center[1], optics.focal_length_px) - alpha
        inv_z = 1.0 / float(stadia_range_m)

        with self._lock:
            if self.data.metric_source == "WALK_TEST":
                return False
            self._passive_inv_z.append(inv_z)
            self._passive_tan_dep.append(float(math.tan(dep_f)))
            if len(self._passive_inv_z) > 120:
                self._passive_inv_z.pop(0)
                self._passive_tan_dep.pop(0)
            self._passive_last_sample_mono = capture_mono

            if len(self._passive_inv_z) < self.PASSIVE_MIN_SAMPLES:
                return False

            fit = fit_ground_plane_huber(
                self._passive_inv_z,
                self._passive_tan_dep,
                min_samples=self.PASSIVE_MIN_SAMPLES,
                min_depth_span_m=self.PASSIVE_MIN_DEPTH_SPAN_M,
            )
            if fit is None:
                return False

            height_m, slope_mrad, rms_mrad = fit
            if rms_mrad > 35.0:
                return False

            first_lock = self.data.metric_source != "PASSIVE_STADIA"
            self.data.camera_mount_height_m = height_m
            self.data.ground_slope_mrad = slope_mrad
            self.data.metric_source = "PASSIVE_STADIA"
            self.data.passive_samples = len(self._passive_inv_z)
            if first_lock or (self.data.passive_samples % 15 == 0):
                self._save_unlocked()
                log.info(
                    f"[SITE-CAL] Passive ground-plane converged: H={height_m:.2f}m | "
                    f"Slope={slope_mrad:+.1f}mrad (n={self.data.passive_samples}, rms={rms_mrad:.1f}mrad)"
                )
            return True
