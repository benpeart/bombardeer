"""Tests for polymorphic BaseTargetDetector capability/policy contracts and decoupling of MotionServo / VisionMain."""

import inspect

from bombardeer_pkg.config import load_config
from bombardeer_pkg.control import motion_servo
from bombardeer_pkg.hardware import vision_pipeline
from bombardeer_pkg.perception.aruco import ArucoTargetDetector
from bombardeer_pkg.perception.base import BaseTargetDetector, TrackingPolicy
from bombardeer_pkg.perception.hailo_detector import HailoWildlifeDetector
from bombardeer_pkg.state import SystemState


class _DummyCustomDetector(BaseTargetDetector):
    @property
    def name(self) -> str:
        return "THERMAL"

    @property
    def is_available(self) -> bool:
        return True

    def detect(self, frame_bgr, capture_mono_time, is_monochrome=False, tilt_mrad=0.0, track_box=None, sensor_size=None, plan_bundle=None):
        return [], capture_mono_time

    def get_tracking_policy(self, cfg, mode: str, npu_lat_sec: float = 0.0) -> TrackingPolicy:
        return TrackingPolicy(
            enforce_elevation_ceiling=True,
            max_target_height_m=1.25,
            tilt_enter_deadband_mrad=12.0,
            tilt_exit_deadband_mrad=5.0,
            pan_coast_hold_sec=0.40,
        )


def test_motion_servo_and_vision_pipeline_have_no_hardcoded_detector_names():
    """Encapsulation guard: control and pipeline threads must not branch on detector names."""
    ms_src = inspect.getsource(motion_servo)
    vp_src = inspect.getsource(vision_pipeline)
    assert '"WILDLIFE"' not in ms_src and '"ARUCO"' not in ms_src
    assert '"WILDLIFE"' not in vp_src and '"ARUCO"' not in vp_src


def test_detector_policies_and_capabilities():
    cfg = load_config(env={})
    aruco = ArucoTargetDetector(cfg)
    hailo = HailoWildlifeDetector.__new__(HailoWildlifeDetector)
    hailo.cfg = cfg

    # Capabilities
    assert aruco.uses_native_roi is False
    assert aruco.prefers_high_res_day_sensor is False
    assert aruco.saves_acquire_audit_snapshot is False
    assert hailo.uses_native_roi is True
    assert hailo.prefers_high_res_day_sensor is True
    assert hailo.saves_acquire_audit_snapshot is True

    # Tracking policies
    p_aruco = aruco.get_tracking_policy(cfg, "ON", 0.0)
    assert p_aruco.enforce_elevation_ceiling is False
    assert p_aruco.tilt_enter_deadband_mrad == cfg.control.velocity_deadband_mrad

    p_wl_on = hailo.get_tracking_policy(cfg, "ON", 0.345)
    assert p_wl_on.enforce_elevation_ceiling is True
    assert p_wl_on.max_target_height_m == cfg.wildlife.max_animal_height_m
    assert p_wl_on.tilt_enter_deadband_mrad == cfg.control.tilt_velocity_deadband_mrad
    assert p_wl_on.tilt_exit_deadband_mrad == cfg.control.velocity_deadband_mrad
    assert p_wl_on.pan_coast_hold_sec >= 0.42

    # CALIBRATE mode keeps crisp tilt deadband even on wildlife detector
    p_wl_cal = hailo.get_tracking_policy(cfg, "CALIBRATE", 0.11)
    assert p_wl_cal.tilt_enter_deadband_mrad == cfg.control.velocity_deadband_mrad

    # Kinematics & association gates
    assert aruco.get_kinematics(15.0, cfg.optics) is None
    assert hailo.get_kinematics(15.0, cfg.optics) is not None
    ax, ay, ar = aruco.get_association_gate(15.0, 0.25, cfg.optics, cfg.control)
    wx, wy, wr = hailo.get_association_gate(15.0, 0.25, cfg.optics, cfg.control)
    assert ay is None and ar is None
    assert wy is not None and wr is not None and wx >= ax

    # HUD formatting
    assert aruco.format_hud_telemetry("3MP", {}) == "3MP DET:ARUCO"
    assert "SCAN-12MP" in hailo.format_hud_telemetry(
        "12MP", {"npu_scan_mode": "SCAN-12MP", "npu_fps": 2.9, "npu_latency_ms": 344.0}
    )


def test_state_delegates_policy_to_registered_custom_detector(tmp_path):
    cfg = load_config(env={})
    cfg.base_dir = str(tmp_path)
    state = SystemState(cfg)
    custom = _DummyCustomDetector()
    state.register_detector(custom)
    state.set_active_detector_type("THERMAL")

    policy = state.get_tracking_policy("ON")
    assert policy.enforce_elevation_ceiling is True
    assert policy.max_target_height_m == 1.25
    assert policy.tilt_enter_deadband_mrad == 12.0
    assert policy.tilt_exit_deadband_mrad == 5.0
