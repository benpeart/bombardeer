"""Abstract base interface and tracking policy contract for Bombardeer perception engines."""

import abc
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class TrackingPolicy:
    """Detector-supplied motion/servo constraints consumed by MotionServo."""

    enforce_elevation_ceiling: bool = False
    max_target_height_m: float = 0.0
    tilt_enter_deadband_mrad: float = 4.0
    tilt_exit_deadband_mrad: float = 4.0
    pan_coast_hold_sec: float = 0.35


def default_tracking_policy(cfg, npu_lat_sec: float = 0.0) -> TrackingPolicy:
    """Unconstrained precision policy (default for fiducial / ArUco calibration)."""
    vel_db = float(cfg.control.velocity_deadband_mrad)
    return TrackingPolicy(
        enforce_elevation_ceiling=False,
        max_target_height_m=0.0,
        tilt_enter_deadband_mrad=vel_db,
        tilt_exit_deadband_mrad=vel_db,
        pan_coast_hold_sec=max(0.35, float(npu_lat_sec) + 0.08),
    )


def wildlife_tracking_policy(cfg, mode: str, npu_lat_sec: float = 0.0) -> TrackingPolicy:
    """Physics-grounded wildlife policy: elevation ceiling + grazing tilt deadband hysteresis."""
    vel_db = float(cfg.control.velocity_deadband_mrad)
    tilt_enter = (
        max(vel_db, float(cfg.control.tilt_velocity_deadband_mrad))
        if mode != "CALIBRATE"
        else vel_db
    )
    return TrackingPolicy(
        enforce_elevation_ceiling=True,
        max_target_height_m=float(cfg.wildlife.max_animal_height_m),
        tilt_enter_deadband_mrad=tilt_enter,
        tilt_exit_deadband_mrad=vel_db,
        pan_coast_hold_sec=max(0.35, float(npu_lat_sec) + 0.08),
    )


def get_default_tracking_policy(
    detector_type: str, cfg, mode: str, npu_lat_sec: float = 0.0
) -> TrackingPolicy:
    """Fallback policy lookup when SystemState is used standalone (e.g. unit tests)."""
    if str(detector_type).upper() == "WILDLIFE":
        return wildlife_tracking_policy(cfg, mode, npu_lat_sec)
    return default_tracking_policy(cfg, npu_lat_sec)


class BaseTargetDetector(abc.ABC):
    @property
    @abc.abstractmethod
    def name(self) -> str:
        pass

    @property
    @abc.abstractmethod
    def is_available(self) -> bool:
        pass

    @property
    def uses_native_roi(self) -> bool:
        """True if VisionMain should capture minimal native-resolution main-stream ROIs."""
        return False

    @property
    def prefers_high_res_day_sensor(self) -> bool:
        """True if the detector benefits from switching DUSK_3MP -> DAY_12MP in daylight."""
        return False

    @property
    def saves_acquire_audit_snapshot(self) -> bool:
        """True if initial lock acquisition should log an audit JPEG snapshot."""
        return False

    def is_idle(self) -> bool:
        return True

    def is_ready_for_frame(self) -> bool:
        return True

    def discard_older_than(self, cutoff_mono: float):
        pass

    def set_active(self, active: bool):
        pass

    def prepare_plan(self, sensor_w: int, sensor_h: int, tilt_mrad: float, track_box, pan_mrad: float = 0.0):
        return None

    @abc.abstractmethod
    def detect(
        self,
        frame_bgr: np.ndarray,
        capture_mono_time: float,
        is_monochrome: bool = False,
        tilt_mrad: float = 0.0,
        track_box=None,
        sensor_size=None,
        plan_bundle=None,
    ):
        pass

    def get_tracking_policy(self, cfg, mode: str, npu_lat_sec: float = 0.0) -> TrackingPolicy:
        return default_tracking_policy(cfg, npu_lat_sec)

    def get_kinematics(self, range_m: float, optics):
        return None

    def get_association_gate(
        self, prim_rng: float, dt_eff: float, optics, ctrl
    ) -> Tuple[float, Optional[float], Optional[float]]:
        """Returns (max_jump_x_px, max_jump_y_px_or_None, max_range_delta_m_or_None)."""
        return float(ctrl.confirmation_max_jump_px), None, None

    def get_telemetry(self) -> dict:
        return {}

    def get_last_debug_info(self) -> dict:
        return {"veto_boxes": [], "rejected_animals": []}

    def format_hud_telemetry(self, cam_tag: str, health: dict) -> str:
        return f"{cam_tag} DET:{self.name}"

    def format_standby_telemetry(self) -> str:
        return ""
