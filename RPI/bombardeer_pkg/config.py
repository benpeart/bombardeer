"""Typed configuration dataclasses with YAML loading and environment overrides."""

from dataclasses import dataclass, field, fields
import logging
import os
from typing import List, Tuple

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

log = logging.getLogger("Bombardeer")


@dataclass
class CameraConfig:
    frame_width: int = 1280
    frame_height: int = 720
    display_width: int = 640
    display_height: int = 360
    sensor_day_size: Tuple[int, int] = (4608, 2592)
    sensor_dusk_size: Tuple[int, int] = (2304, 1296)
    main_stream_is_rgb: bool = True
    manual_lens_position: float = 0.0
    target_fps: float = 30.0
    hud_max_fps: float = 15.0
    day_12mp_enter_lux: float = 60.0
    day_12mp_enter_max_gain: float = 3.5
    day_12mp_exit_lux: float = 35.0
    day_12mp_exit_min_gain: float = 5.0
    sensor_switch_dwell_sec: float = 60.0
    sensor_boot_eval_sec: float = 5.0

    @property
    def scale_x(self) -> float:
        return self.display_width / float(self.frame_width)

    @property
    def scale_y(self) -> float:
        return self.display_height / float(self.frame_height)

    @property
    def optical_center(self) -> Tuple[int, int]:
        return (self.frame_width // 2, self.frame_height // 2)

    @property
    def max_sensor_width(self) -> int:
        return max(self.sensor_day_size[0], self.sensor_dusk_size[0])

    @property
    def max_sensor_height(self) -> int:
        return max(self.sensor_day_size[1], self.sensor_dusk_size[1])


@dataclass
class OpticsConfig:
    camera_offset_x_mm: float = -170.0
    camera_offset_y_mm: float = -60.0
    tilt_calibration_trim_mrad: float = -8.3
    focal_length_px: float = 540.4
    camera_mount_height_m: float = 1.15
    ground_slope_mrad: float = 0.0
    min_valid_range_m: float = 3.0
    max_valid_range_m: float = 75.0
    max_engage_range_m: float = 40.0
    default_fallback_range_m: float = 20.0
    horizon_y_offset_720: float = 0.0
    horizon_headroom_720: float = 90.0
    horizon_far_headroom_720: float = 50.0
    horizon_feet_margin_720: float = 45.0

    @property
    def mrad_per_pixel(self) -> float:
        return 1000.0 / self.focal_length_px


@dataclass
class BallisticsConfig:
    muzzle_velocity_m_s: float = 85.0
    gravity_m_s2: float = 9.81
    drag_k: float = 0.0128


@dataclass
class ControlConfig:
    ballistic_acquire_mrad: float = 120.0
    ballistic_abort_mrad: float = 260.0
    velocity_deadband_mrad: float = 4.0
    fire_deadband_mrad: float = 40.0
    calibrate_deadband_mrad: float = 15.0
    max_fire_slew_speed_mrad_s: float = 180.0
    motion_loop_interval_sec: float = 0.020
    setpoint_min_interval_sec: float = 0.040
    tilt_min_mrad: int = -420
    tilt_max_mrad: int = 400
    pan_min_mrad: int = -3141
    pan_max_mrad: int = 3141
    max_trigger_duration_sec: float = 2.0
    trigger_cooldown_sec: float = 1.5
    confirmation_frames: int = 4
    confirmation_max_jump_px: float = 120.0
    home_timeout_sec: float = 60.0
    fire_data_freshness_sec: float = 0.220
    coast_decay_onset_sec: float = 0.650
    coast_max_horizon_sec: float = 1.800
    coast_exp_decay_rate: float = 2.5
    track_crop_hold_sec: float = 0.700


@dataclass
class WildlifeConfig:
    infer_size: int = 1280
    stack_tile_w: int = 1280
    stack_tile_h: int = 620
    stack_divider_h: int = 40
    stack_divider_gray: int = 114
    conf_thresh: float = 0.55
    monochrome_conf_thresh: float = 0.55
    iou_thresh: float = 0.45
    person_veto_thresh: float = 0.20
    vehicle_veto_thresh: float = 0.20
    veto_padding_px: int = 24
    class_margin: float = 0.25
    max_animal_width_frac: float = 0.55
    max_animal_area_frac: float = 0.40
    max_animal_aspect_ratio: float = 1.65
    max_non_animal_prob: float = 0.02
    max_animal_height_m: float = 1.50
    assumed_target_height_mm: float = 950.0
    seam_clip_margin_px: float = 6.0
    max_npu_restarts: int = 5

    @property
    def stack_slot1_y(self) -> int:
        return self.stack_tile_h + self.stack_divider_h


@dataclass
class SerialConfig:
    baud_rate: int = 921600
    heartbeat_timeout_sec: float = 2.0
    reconnect_no_rx_sec: float = 5.0
    max_consecutive_tx_errors: int = 3
    ports: List[str] = field(default_factory=lambda: [
        "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
        "/dev/ttyUSB0",
        "/dev/ttyACM0",
        "/dev/ttyUSB1",
    ])


@dataclass
class StorageConfig:
    record_pre_roll_sec: float = 2.0
    record_post_roll_sec: float = 3.0
    min_lock_frames_to_record: int = 3
    max_recording_duration_sec: float = 30.0
    min_free_space_gb: float = 5.0
    max_recordings_storage_gb: float = 10.0
    max_debug_snapshots: int = 200
    audit_veto_cooldown_sec: float = 2.0


@dataclass
class WebConfig:
    default_user: str = "admin"
    default_pass: str = "bombardeer"
    auth_user: str = "admin"
    auth_pass: str = "bombardeer"
    armed_allowed: bool = False


@dataclass
class BombardeerConfig:
    base_dir: str = "."
    static_dir: str = "./static"
    recordings_dir: str = "./recordings"
    debug_snapshot_dir: str = "./recordings/debug_snapshots"
    camera: CameraConfig = field(default_factory=CameraConfig)
    optics: OpticsConfig = field(default_factory=OpticsConfig)
    ballistics: BallisticsConfig = field(default_factory=BallisticsConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    wildlife: WildlifeConfig = field(default_factory=WildlifeConfig)
    serial: SerialConfig = field(default_factory=SerialConfig)
    storage: StorageConfig = field(default_factory=StorageConfig)
    web: WebConfig = field(default_factory=WebConfig)


def _apply_section(dc_obj, data: dict):
    if not isinstance(data, dict):
        return
    valid_fields = {f.name: f for f in fields(dc_obj)}
    for k, v in data.items():
        if k not in valid_fields:
            log.warning(f"[CONFIG] Ignoring unknown key '{k}' in {type(dc_obj).__name__}")
            continue
        cur = getattr(dc_obj, k)
        if isinstance(cur, tuple) and isinstance(v, list):
            v = tuple(v)
        setattr(dc_obj, k, v)


def load_config(yaml_path: str = None, base_dir: str = None, env: dict = None) -> BombardeerConfig:
    """Loads BombardeerConfig from YAML (if present) and applies environment auth rules."""
    resolved_base = os.path.abspath(base_dir or (os.path.dirname(yaml_path) if yaml_path else "."))
    cfg = BombardeerConfig(
        base_dir=resolved_base,
        static_dir=os.path.join(resolved_base, "static"),
        recordings_dir=os.path.join(resolved_base, "recordings"),
        debug_snapshot_dir=os.path.join(resolved_base, "recordings", "debug_snapshots"),
    )

    if yaml_path and os.path.exists(yaml_path):
        if YAML_AVAILABLE:
            try:
                with open(yaml_path, "r", encoding="utf-8") as fh:
                    raw = yaml.safe_load(fh) or {}
                for sec_name in ("camera", "optics", "ballistics", "control", "wildlife", "serial", "storage", "web"):
                    if sec_name in raw:
                        _apply_section(getattr(cfg, sec_name), raw[sec_name])
                log.info(f"[CONFIG] Loaded configuration from {yaml_path}")
            except Exception as exc:
                log.error(f"[CONFIG] Failed parsing {yaml_path} ({exc}); using built-in defaults.")
        else:
            log.warning("[CONFIG] PyYAML not installed; using built-in defaults.")

    env_map = os.environ if env is None else env
    env_user = env_map.get("BOMBARDEER_USER")
    env_pass = env_map.get("BOMBARDEER_PASS")

    cfg.web.auth_user = env_user if env_user is not None else cfg.web.default_user
    cfg.web.auth_pass = env_pass if env_pass is not None else cfg.web.default_pass

    # Refuse ARMED mode unless BOMBARDEER_PASS is explicitly set to a non-empty, non-default password
    cfg.web.armed_allowed = bool(
        env_pass is not None
        and env_pass.strip() != ""
        and env_pass != cfg.web.default_pass
    )
    if not cfg.web.armed_allowed:
        log.warning(
            "[SECURITY] BOMBARDEER_PASS is unset or using default password — "
            "ARMED live-fire mode is DISABLED until a custom password is configured."
        )

    return cfg