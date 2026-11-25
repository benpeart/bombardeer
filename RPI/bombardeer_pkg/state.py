"""Segregated thread-safe state planes and non-droppable TurretCommandBus."""

from collections import deque
import logging
import math
import threading
import time
from typing import List, Optional, Tuple

from bombardeer_pkg.config import BombardeerConfig
from bombardeer_pkg.control.kalman import TargetKalmanFilter
from bombardeer_pkg.control.site_calibration import SiteCalibrationManager

log = logging.getLogger("Bombardeer")


class TurretCommandBus:
    """
    Priority, non-droppable command channel between producers (MotionServo, Web API,
    Safety Veto) and TurretSerial.

    Guarantees:
      1. HALT ('X') and TRIGGER OFF ('T 0') can NEVER be dropped due to queue saturation.
      2. HALT immediately clears any pending motion setpoint/velocity and forces trigger OFF.
      3. Motion commands coalesce to the latest SETPOINT/VELOCITY without unbounded backlog.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._halt_pending = False
        self._trigger_pending: Optional[dict] = None
        self._motion_pending: Optional[dict] = None

    def request_halt(self):
        with self._lock:
            self._halt_pending = True
            self._motion_pending = None
            self._trigger_pending = {"cmd": "TRIGGER", "state": 0, "pulse_len": None, "timestamp_mono": time.monotonic()}

    def request_trigger(self, state: int, pulse_len: Optional[float] = None, timestamp_mono: Optional[float] = None):
        ts = time.monotonic() if timestamp_mono is None else timestamp_mono
        with self._lock:
            # Never allow a TRIGGER 1 to overwrite a pending HALT or an unconsumed safety TRIGGER 0
            if state == 1 and self._halt_pending:
                return
            if state == 1 and self._trigger_pending is not None and self._trigger_pending["state"] == 0:
                return
            self._trigger_pending = {
                "cmd": "TRIGGER",
                "state": 1 if state else 0,
                "pulse_len": pulse_len,
                "timestamp_mono": ts,
            }

    def request_setpoint(self, pan: int, tilt: int):
        with self._lock:
            if self._halt_pending:
                return
            self._motion_pending = {"cmd": "SETPOINT", "pan": int(pan), "tilt": int(tilt)}

    def request_velocity(self, pan_s: int, tilt_s: int):
        with self._lock:
            if self._halt_pending:
                return
            self._motion_pending = {"cmd": "VELOCITY", "pan_s": int(pan_s), "tilt_s": int(tilt_s)}

    def drain(self) -> List[dict]:
        """Atomically returns ordered commands: [HALT?, TRIGGER?, MOTION?]."""
        out: List[dict] = []
        with self._lock:
            if self._halt_pending:
                out.append({"cmd": "HALT"})
                self._halt_pending = False
            if self._trigger_pending is not None:
                out.append(self._trigger_pending)
                self._trigger_pending = None
            if self._motion_pending is not None:
                out.append(self._motion_pending)
                self._motion_pending = None
        return out

    def clear_motion(self):
        with self._lock:
            self._motion_pending = None


class SystemState:
    """Centralized container for the 4 isolated lock planes + command bus + Kalman filter."""

    def __init__(self, cfg: BombardeerConfig):
        self.cfg = cfg
        self.start_mono = time.monotonic()
        self.shutdown_event = threading.Event()
        self.command_bus = TurretCommandBus()
        self.target_filter = TargetKalmanFilter(accel_noise_mrad_s2=150.0, measurement_noise=35.0)
        self.wildlife_detector = None
        self.site_calibration = SiteCalibrationManager(cfg)

        # Lock 0: Engagement mode & active detector
        self._mode_lock = threading.Lock()
        self._turret_mode = "OFF"
        self._calibrate_primed = False
        self._active_detector_type = "WILDLIFE"

        # Lock 1: Web streaming buffer & viewer count
        self._frame_lock = threading.Lock()
        self._latest_jpeg: Optional[bytes] = None
        self._active_web_clients = 0

        # Lock 2: High-speed ESP32 hardware telemetry (300 samples = 3-6s at 50-100 Hz)
        self._telemetry_lock = threading.Lock()
        self._turret_telemetry = {
            "pan_mrad": 0,
            "tilt_mrad": 0,
            "is_moving": False,
            "is_firing": False,
        }
        self._telemetry_history = deque(maxlen=300)

        # Lock 3: Target & health plane
        self._target_lock = threading.Lock()
        self._primary_target: Optional[dict] = None
        self._system_health = {
            "serial_connected": False,
            "serial_tx_count": 0,
            "serial_rx_count": 0,
            "recorder_active": False,
            "recorder_error": None,
            "last_alert": None,
            "alert_until_mono": 0.0,
            "free_disk_gb": 0.0,
            "hailo_available": False,
            "npu_fps": 0.0,
            "npu_latency_ms": 0.0,
            "npu_scan_mode": "IDLE",
            "npu_restarts": 0,
            "sensor_mode": "DUSK_3MP",
            "sensor_fps": 0.0,
        }

    # --- Mode & Detector Plane ---

    def get_turret_mode(self) -> Tuple[str, bool]:
        with self._mode_lock:
            return self._turret_mode, self._calibrate_primed

    def set_turret_mode(self, new_mode: str, primed: Optional[bool] = None):
        with self._mode_lock:
            self._turret_mode = new_mode
            if primed is not None:
                self._calibrate_primed = primed
            elif new_mode == "CALIBRATE":
                self._calibrate_primed = True
            elif new_mode in ("OFF", "ON"):
                self._calibrate_primed = False

    def consume_calibration_prime(self) -> bool:
        with self._mode_lock:
            if self._calibrate_primed:
                self._calibrate_primed = False
                return True
            return False

    def get_active_detector_type(self) -> str:
        with self._mode_lock:
            return self._active_detector_type

    def set_active_detector_type(self, det_type: str):
        with self._mode_lock:
            self._active_detector_type = det_type

    # --- Web Frame Plane ---

    def set_latest_jpeg(self, jpeg_bytes: bytes):
        with self._frame_lock:
            self._latest_jpeg = jpeg_bytes

    def get_latest_jpeg(self) -> Optional[bytes]:
        with self._frame_lock:
            return self._latest_jpeg

    def adjust_web_clients(self, delta: int):
        with self._frame_lock:
            self._active_web_clients = max(0, self._active_web_clients + delta)

    def has_web_clients(self) -> bool:
        with self._frame_lock:
            return self._active_web_clients > 0

    # --- Telemetry Plane ---

    def update_turret_telemetry(self, pan: int, tilt: int, moving: bool, rx_mono: float):
        with self._telemetry_lock:
            self._turret_telemetry["pan_mrad"] = pan
            self._turret_telemetry["tilt_mrad"] = tilt
            self._turret_telemetry["is_moving"] = moving
            self._telemetry_history.append((rx_mono, pan, tilt))

    def set_firing_state(self, firing: bool):
        with self._telemetry_lock:
            self._turret_telemetry["is_firing"] = firing

    def get_instantaneous_telemetry(self) -> Tuple[int, int, bool, bool]:
        with self._telemetry_lock:
            t = self._turret_telemetry
            return t["pan_mrad"], t["tilt_mrad"], t["is_moving"], t["is_firing"]

    def get_turret_pos_at_time(self, target_mono: float) -> Tuple[int, int]:
        with self._telemetry_lock:
            hist = self._telemetry_history
            n = len(hist)
            if n == 0:
                return self._turret_telemetry["pan_mrad"], self._turret_telemetry["tilt_mrad"]
            if target_mono >= hist[-1][0]:
                return hist[-1][1], hist[-1][2]
            if target_mono <= hist[0][0]:
                return hist[0][1], hist[0][2]
            for i in range(n - 2, -1, -1):
                t0, p0, tilt0 = hist[i]
                t1, p1, tilt1 = hist[i + 1]
                if t0 <= target_mono <= t1:
                    dt = t1 - t0
                    if dt <= 1e-6:
                        return p0, tilt0
                    alpha = (target_mono - t0) / dt
                    return int(round(p0 + alpha * (p1 - p0))), int(round(tilt0 + alpha * (tilt1 - tilt0)))
            return self._turret_telemetry["pan_mrad"], self._turret_telemetry["tilt_mrad"]

    def get_measured_turret_speed_mrad_s(self) -> float:
        with self._telemetry_lock:
            hist = self._telemetry_history
            if len(hist) < 2:
                return 0.0
            t1, p1, tilt1 = hist[-1]
            t0, p0, tilt0 = hist[max(0, len(hist) - 3)]
            dt = t1 - t0
            if dt <= 0.005:
                return 0.0
            return math.hypot(p1 - p0, tilt1 - tilt0) / dt

    # --- Target & Health Plane ---

    def get_primary_target(self) -> Optional[dict]:
        with self._target_lock:
            return dict(self._primary_target) if self._primary_target is not None else None

    def set_primary_target(self, target_dict: dict):
        with self._target_lock:
            self._primary_target = target_dict

    def clear_primary_target(self):
        with self._target_lock:
            self._primary_target = None

    def set_alert(self, message: str, duration: float = 3.0):
        with self._target_lock:
            self._system_health["last_alert"] = message
            self._system_health["alert_until_mono"] = time.monotonic() + duration
        log.warning(f"[ALERT] {message}")

    def update_health(self, **kwargs):
        with self._target_lock:
            self._system_health.update(kwargs)

    def inc_health(self, key: str, delta: int = 1):
        with self._target_lock:
            self._system_health[key] = self._system_health.get(key, 0) + delta

    def get_health_snapshot(self) -> dict:
        with self._target_lock:
            snap = dict(self._system_health)
        now_mono = time.monotonic()
        snap["active_alert"] = snap["last_alert"] if now_mono < snap["alert_until_mono"] else None
        snap["uptime_sec"] = round(now_mono - self.start_mono, 1)
        snap["site_calibration"] = self.site_calibration.get_status_dict()
        return snap

    def get_npu_latency_sec(self) -> float:
        """Returns active NPU inference latency in seconds (0.0 when running CPU ArUco)."""
        if self.get_active_detector_type() != "WILDLIFE":
            return 0.0
        with self._target_lock:
            return max(0.0, float(self._system_health.get("npu_latency_ms", 0.0)) / 1000.0)

    def is_serial_connected(self) -> bool:
        with self._target_lock:
            return bool(self._system_health["serial_connected"])

    def is_hailo_ready(self) -> bool:
        if self.wildlife_detector is not None:
            return self.wildlife_detector.is_available
        with self._target_lock:
            return bool(self._system_health["hailo_available"])
