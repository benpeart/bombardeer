#!/usr/bin/env python3
"""
Bombardeer Turret: Real-Time Dual-Mode Controller (Mission-Critical Edition)
----------------------------------------------------------------------------
- Architecture:
    * Polymorphic Perception Engine (Runtime Switchable via Web UI):
        - ArucoTargetDetector: OpenCV sub-pixel fiducial tracker for calibration/testing.
        - HailoWildlifeDetector: Hardware-accelerated Hailo-8 (26 TOPS) NPU running
          MegaDetector v6 (MDV6-yolov9-c-1280.hef) with zero-copy 16:9 letterboxing.
    * Asynchronous Perception & Motion Decoupling:
        - Vision Worker: Processes frames at sensor/inference rate (15-30 Hz).
        - Motion Servoing Loop: Runs deterministically at 50 Hz (20ms ticks),
          predicting target state and feeding the ESP32 continuous velocity
          vectors regardless of optical frame cadence or ML inference lag.
    * Concurrency Architecture (Segregated Lock Planes):
        - telemetry_lock: High-speed motor angles, moving flags, and history deque.
        - target_lock: Perception target state, supervisory health, and storage metrics.
        - frame_lock: Flask JPEG streaming and client tracking.
        - mode_lock: Engagement model (OFF -> ON -> ARMED / CALIBRATE) & active detector.
"""

import os
os.environ["LIBCAMERA_LOG_LEVELS"] = "IPARPI:FATAL"

import abc
import glob
import time
import math
import hmac
import ctypes
import shutil
import datetime
import threading
import multiprocessing as mp
from multiprocessing import shared_memory
from multiprocessing import resource_tracker
import signal
import atexit
import queue
import logging
import traceback
from collections import deque
import numpy as np
import cv2
import serial
from flask import Flask, Response, render_template_string, jsonify, request, send_from_directory

from picamera2 import Picamera2
from libcamera import Transform

# Optional Hardware NPU Import (Graceful Fallback if HailoRT is absent)
try:
    from hailo_platform import (
        HEF,
        VDevice,
        ConfigureParams,
        InferVStreams,
        InputVStreamParams,
        OutputVStreamParams,
        FormatType,
        HailoStreamInterface,
    )
    HAILO_AVAILABLE = True
except ImportError:
    HAILO_AVAILABLE = False

# -------------------------------------------------------------------------
# OpenCV Deterministic Core Clamping
# -------------------------------------------------------------------------
cv2.setNumThreads(2)
cv2.ocl.setUseOpenCL(False)

# =========================================================================
# Structured Logging & OS Helpers
# =========================================================================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("Bombardeer")

def set_os_thread_name(name: str, cpu_cores: list = None):
    """Sets the Linux OS-level thread name visible in ps, top, and htop and pins CPU core affinity on the Raspberry Pi 5."""
    try:
        libc = ctypes.CDLL("libc.so.6")
        libc.prctl(15, name[:15].encode("utf-8"), 0, 0, 0)
    except Exception:
        pass
    if cpu_cores is not None:
        try:
            os.sched_setaffinity(0, set(cpu_cores))
        except Exception:
            pass

# =========================================================================
# Configuration & Hardware Calibration
# =========================================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
RECORDINGS_DIR = os.path.join(BASE_DIR, "recordings")
DEBUG_SNAPSHOT_DIR = os.path.join(RECORDINGS_DIR, "debug_snapshots")
MAX_DEBUG_SNAPSHOTS = 200
AUDIT_VETO_COOLDOWN_SEC = 2.0

try:
    os.makedirs(RECORDINGS_DIR, exist_ok=True)
    os.makedirs(DEBUG_SNAPSHOT_DIR, exist_ok=True)
    test_touch = os.path.join(RECORDINGS_DIR, ".write_test")
    with open(test_touch, "w") as f:
        f.write("ok")
    os.remove(test_touch)
    log.info(f"[STORAGE] Storage directory verified writable: {RECORDINGS_DIR}")
except Exception as e:
    log.error(f"[STORAGE ERROR] Cannot write to storage path {RECORDINGS_DIR}: {e}")

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 360

SCALE_X = DISPLAY_WIDTH / FRAME_WIDTH
SCALE_Y = DISPLAY_HEIGHT / FRAME_HEIGHT
OPTICAL_CENTER = (FRAME_WIDTH // 2, FRAME_HEIGHT // 2)

# --- PHYSICAL PARALLAX OFFSETS (Camera relative to Barrel) ---
CAMERA_OFFSET_X_MM = -170.0  # Camera 170mm Left of Bore
CAMERA_OFFSET_Y_MM = -60.0   # Camera 60mm Below Bore

# Ballistic and Mechanical Bore Trim
TILT_CALIBRATION_TRIM_MRAD = -8.3  # -8.3 mrad lowers impact point by 4" at 40 ft

# Camera Optical Intrinsics
FOCAL_LENGTH_PX = 540.4
MRAD_PER_PIXEL_X = 1000.0 / FOCAL_LENGTH_PX
MRAD_PER_PIXEL_Y = 1000.0 / FOCAL_LENGTH_PX

MIN_VALID_RANGE_M = 3.0
MAX_VALID_RANGE_M = 75.0
DEFAULT_FALLBACK_RANGE_M = 20.0

CAMERA_MOUNT_HEIGHT_M = 1.15   # Physical height of camera lens above ground (meters)
GROUND_SLOPE_MRAD = 0.0        # Terrain slope in mrad (+ if uphill away from turret, - if downhill)

# Dynamic Targeting, Hysteresis & Deadbands
BALLISTIC_ACQUIRE_MRAD = 120.0
BALLISTIC_ABORT_MRAD = 260.0
VELOCITY_DEADBAND_MRAD = 4.0
FIRE_DEADBAND_MRAD = 40.0
MAX_FIRE_SLEW_SPEED_MRAD_S = 180.0
MOTION_LOOP_INTERVAL_SEC = 0.020
SETPOINT_MIN_INTERVAL_SEC = 0.040

# Software Travel Limits (Physical Axis Protection)
TILT_MIN_MRAD = -420
TILT_MAX_MRAD = 400
PAN_MIN_MRAD = -3141
PAN_MAX_MRAD = 3141

MAX_TRIGGER_DURATION_SEC = 2.0
TRIGGER_COOLDOWN_SEC = 1.5

CONFIRMATION_FRAMES = 4
CONFIRMATION_MAX_JUMP_PX = 120.0
HOME_TIMEOUT_SEC = 60.0

# Persistence & Coasting Horizons
COAST_MAX_HORIZON_SEC = 1.500
COAST_EXP_DECAY_RATE = 2.5

DEFAULT_SERIAL_PORTS = ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1"]
BAUD_RATE = 921600
SERIAL_HEARTBEAT_TIMEOUT_SEC = 2.5

TARGET_FPS = 30.0
RECORD_PRE_ROLL_SEC = 2.0
RECORD_POST_ROLL_SEC = 3.0
MIN_LOCK_FRAMES_TO_RECORD = 3
MAX_RECORDING_DURATION_SEC = 30.0

MIN_FREE_SPACE_GB = 5.0
MAX_RECORDINGS_STORAGE_GB = 10.0

STREAM_PART_HEADER = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
STREAM_PART_FOOTER = b"\r\n"

# Web UI HTTP Basic Authentication (override via systemd Environment=BOMBARDEER_USER / BOMBARDEER_PASS)
WEB_AUTH_USER = os.environ.get("BOMBARDEER_USER", "admin")
WEB_AUTH_PASS = os.environ.get("BOMBARDEER_PASS", "bombardeer")

# =========================================================================
# Concurrency Architecture: Isolated Lock Planes
# =========================================================================
app = Flask(__name__)

# Lock 0: Operational Engagement & Active Perception Strategy
mode_lock = threading.Lock()
turret_mode = "OFF"             # OFF, ON, ARMED, CALIBRATE
calibrate_primed = False        # Single-shot authorization token
active_detector_type = "ARUCO"  # "ARUCO" or "WILDLIFE"

def get_turret_mode():
    with mode_lock:
        return turret_mode, calibrate_primed

def set_turret_mode(new_mode, primed=None):
    global turret_mode, calibrate_primed
    with mode_lock:
        turret_mode = new_mode
        if primed is not None:
            calibrate_primed = primed
        elif new_mode == "CALIBRATE":
            calibrate_primed = True
        elif new_mode in ("OFF", "ON"):
            calibrate_primed = False

def consume_calibration_prime():
    global calibrate_primed
    with mode_lock:
        if calibrate_primed:
            calibrate_primed = False
            return True
        return False

def get_active_detector_type():
    with mode_lock:
        return active_detector_type

def set_active_detector_type(det_type: str):
    global active_detector_type
    with mode_lock:
        active_detector_type = det_type

# Lock 1: Web Streaming Buffer and Client Counter
frame_lock = threading.Lock()
latest_jpeg = None
active_web_clients = 0
app_shutting_down = threading.Event()

# Lock 2: Real-Time High-Speed Hardware Telemetry Plane
telemetry_lock = threading.Lock()
turret_telemetry = {
    "pan_mrad": 0,
    "tilt_mrad": 0,
    "is_moving": False,
    "is_firing": False,
}
telemetry_history = deque(maxlen=60)

# Lock 3: Perception Target & Supervisory Status Plane
target_lock = threading.Lock()
primary_target = None
system_health = {
    "serial_connected": False,
    "serial_tx_count": 0,
    "serial_rx_count": 0,
    "recorder_active": False,
    "recorder_error": None,
    "last_alert": None,
    "alert_time": 0.0,
    "free_disk_gb": 0.0,
    "hailo_available": False,
    "npu_fps": 0.0,
    "npu_latency_ms": 0.0,
}

global_turret_queue = None
global_target_filter = None
global_wildlife_detector = None

def is_hailo_npu_ready() -> bool:
    """Reads the live cross-process shared-memory availability flag from the NPU subprocess."""
    global global_wildlife_detector
    if global_wildlife_detector is not None:
        return global_wildlife_detector.is_available
    with target_lock:
        return system_health["hailo_available"]

def set_alert(message, duration=3.0):
    with target_lock:
        system_health["last_alert"] = message
        system_health["alert_time"] = time.time() + duration
    log.warning(f"[ALERT] {message}")

def clear_queue(q):
    try:
        while not q.empty():
            q.get_nowait()
    except Exception:
        pass

# --- Atomic Telemetry Accessors ---

def update_turret_telemetry(pan, tilt, moving, rx_mono_time):
    with telemetry_lock:
        turret_telemetry["pan_mrad"] = pan
        turret_telemetry["tilt_mrad"] = tilt
        turret_telemetry["is_moving"] = moving
        telemetry_history.append((rx_mono_time, pan, tilt))

def set_firing_state(firing):
    with telemetry_lock:
        turret_telemetry["is_firing"] = firing

def get_instantaneous_telemetry():
    with telemetry_lock:
        return (
            turret_telemetry["pan_mrad"],
            turret_telemetry["tilt_mrad"],
            turret_telemetry["is_moving"],
            turret_telemetry["is_firing"]
        )

def get_turret_pos_at_time(target_mono_sec):
    with telemetry_lock:
        hist_len = len(telemetry_history)
        if hist_len == 0:
            return turret_telemetry["pan_mrad"], turret_telemetry["tilt_mrad"]

        if target_mono_sec >= telemetry_history[-1][0]:
            return telemetry_history[-1][1], telemetry_history[-1][2]

        if target_mono_sec <= telemetry_history[0][0]:
            return telemetry_history[0][1], telemetry_history[0][2]

        for i in range(hist_len - 2, -1, -1):
            t0, p0, tilt0 = telemetry_history[i]
            t1, p1, tilt1 = telemetry_history[i + 1]
            if t0 <= target_mono_sec <= t1:
                dt = t1 - t0
                if dt <= 1e-6:
                    return p0, tilt0
                alpha = (target_mono_sec - t0) / dt
                interp_pan = p0 + alpha * (p1 - p0)
                interp_tilt = tilt0 + alpha * (tilt1 - tilt0)
                return int(round(interp_pan)), int(round(interp_tilt))

        return turret_telemetry["pan_mrad"], turret_telemetry["tilt_mrad"]

# --- Atomic Target Accessors ---

def get_primary_target_snapshot():
    with target_lock:
        return dict(primary_target) if primary_target is not None else None

def set_primary_target(target_dict):
    global primary_target
    with target_lock:
        primary_target = target_dict

def clear_primary_target():
    global primary_target
    with target_lock:
        primary_target = None

def get_serial_connected():
    with target_lock:
        return system_health["serial_connected"]

def set_serial_connected(status):
    with target_lock:
        system_health["serial_connected"] = status

# =========================================================================
# Thread-Safe World-Space Kalman Filter (State: Pan, Tilt, vPan, vTilt)
# =========================================================================
class TargetKalmanFilter:
    def __init__(self, process_noise=35.0, measurement_noise=10.0):
        self.lock = threading.Lock()
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.transitionMatrix = np.eye(4, dtype=np.float32)

        self.kf.measurementMatrix = np.zeros((2, 4), dtype=np.float32)
        self.kf.measurementMatrix[0, 0] = 1.0
        self.kf.measurementMatrix[1, 1] = 1.0

        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * process_noise
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * measurement_noise
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 50.0

        self.last_update_time = None
        self.is_initialized = False

    def reset(self):
        with self.lock:
            self.is_initialized = False
            self.last_update_time = None

    def update_measurement(self, measured_pan, measured_tilt, current_time):
        with self.lock:
            if not self.is_initialized:
                self.kf.statePre = np.array([[float(measured_pan)], [float(measured_tilt)], [0.0], [0.0]], dtype=np.float32)
                self.kf.statePost = np.array([[float(measured_pan)], [float(measured_tilt)], [0.0], [0.0]], dtype=np.float32)
                self.kf.errorCovPre = np.eye(4, dtype=np.float32) * 50.0
                self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 50.0
                self.last_update_time = current_time
                self.is_initialized = True
                return int(measured_pan), int(measured_tilt), 0.0, 0.0

            dt = max(0.001, min(0.3, current_time - self.last_update_time))
            self.last_update_time = current_time

            self.kf.transitionMatrix[0, 2] = dt
            self.kf.transitionMatrix[1, 3] = dt

            self.kf.predict()
            measurement = np.array([[float(measured_pan)], [float(measured_tilt)]], dtype=np.float32)
            estimated = self.kf.correct(measurement)

            return (
                int(round(float(estimated[0, 0]))),
                int(round(float(estimated[1, 0]))),
                float(estimated[2, 0]),
                float(estimated[3, 0])
            )

    def predict_state(self, current_time):
        with self.lock:
            if not self.is_initialized or self.last_update_time is None:
                return None

            dt = max(0.001, min(0.1, current_time - self.last_update_time))
            self.last_update_time = current_time

            self.kf.transitionMatrix[0, 2] = dt
            self.kf.transitionMatrix[1, 3] = dt

            predicted = self.kf.predict()
            return (
                int(round(float(predicted[0, 0]))),
                int(round(float(predicted[1, 0]))),
                float(predicted[2, 0]),
                float(predicted[3, 0])
            )

# =========================================================================
# Polymorphic Perception Architecture (Strategy Pattern)
# =========================================================================

class BaseTargetDetector(abc.ABC):
    """
    Abstract Base Interface for all Bombardeer target perception engines.
    Guarantees a uniform return schema and lifecycle across optical and NPU models.
    """
    @property
    @abc.abstractmethod
    def name(self) -> str:
        pass

    @property
    @abc.abstractmethod
    def is_available(self) -> bool:
        pass

    @abc.abstractmethod
    def detect(self, frame_bgr: np.ndarray, capture_mono_time: float, is_monochrome: bool = False):
        """
        Processes a 1280x720 BGR/RGB frame and returns:
            (detections_list, effective_capture_mono_time)
        Each item in detections_list follows:
            {
                "box": [x1, y1, x2, y2],   # 1280x720 coordinate space
                "center": (cx, cy),        # 1280x720 coordinate space
                "score": float,
                "area": float,
                "range_m": float,
                "id": int,
                "label": str
            }
        """
        pass

    def get_telemetry(self) -> dict:
        return {}

    def get_last_debug_info(self) -> dict:
        return {"veto_boxes": [], "rejected_animals": []}


class ArucoTargetDetector(BaseTargetDetector):
    """
    Encapsulated ArUco Fiducial Detector for precision calibration and range testing.
    All ArUco-specific physical dimensions and thresholds are isolated inside this class.
    """
    ARUCO_REAL_WIDTH_MM = 365.0
    TARGET_MARKER_ID = 0
    MIN_EDGE_PX = 8.0

    def __init__(self, target_id=None, marker_width_mm=None):
        self.target_id = self.TARGET_MARKER_ID if target_id is None else target_id
        self.marker_width_mm = self.ARUCO_REAL_WIDTH_MM if marker_width_mm is None else marker_width_mm

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.errorCorrectionRate = 0.55
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 19
        self.parameters.adaptiveThreshWinSizeStep = 4

        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # Pre-compute Gamma LUT (Gamma < 1.0 lifts shadows with zero CPU penalty)
        gamma = 0.6
        inv_gamma = 1.0 / gamma
        self.gamma_lut = np.array([
            ((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)
        ]).astype("uint8")

    @property
    def name(self) -> str:
        return "ARUCO"

    @property
    def is_available(self) -> bool:
        return True

    def detect(self, frame_bgr: np.ndarray, capture_mono_time: float, is_monochrome: bool = False):
        try:
            lifted = cv2.LUT(frame_bgr, self.gamma_lut)
            gray = cv2.cvtColor(lifted, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)
            detections = []

            if ids is not None and len(ids) > 0:
                for marker_corners, marker_id in zip(corners, ids.flatten()):
                    if self.target_id is not None and marker_id != self.target_id:
                        continue

                    pts = marker_corners.reshape((4, 2))
                    d01 = math.hypot(pts[0][0] - pts[1][0], pts[0][1] - pts[1][1])
                    d12 = math.hypot(pts[1][0] - pts[2][0], pts[1][1] - pts[2][1])

                    if d01 < self.MIN_EDGE_PX or d12 < self.MIN_EDGE_PX:
                        continue

                    ratio = d01 / max(1.0, d12)
                    if not (0.35 <= ratio <= 2.85):
                        continue

                    cx = float(np.mean(pts[:, 0]))
                    cy = float(np.mean(pts[:, 1]))
                    x1 = int(np.min(pts[:, 0]))
                    y1 = int(np.min(pts[:, 1]))
                    x2 = int(np.max(pts[:, 0]))
                    y2 = int(np.max(pts[:, 1]))
                    area = float(cv2.contourArea(pts.astype(np.float32)))

                    marker_px = max(1.0, (d01 + d12) / 2.0)
                    estimated_range_m = (self.marker_width_mm * FOCAL_LENGTH_PX) / (marker_px * 1000.0)
                    clamped_range_m = float(np.clip(estimated_range_m, MIN_VALID_RANGE_M, MAX_VALID_RANGE_M))

                    detections.append({
                        "box": [x1, y1, x2, y2],
                        "center": (cx, cy),
                        "score": 0.99,
                        "area": area,
                        "range_m": clamped_range_m,
                        "id": int(marker_id),
                        "label": f"ID:{int(marker_id)}"
                    })

                # Sort largest/closest marker first
                detections.sort(key=lambda d: d["area"], reverse=True)

            return detections, capture_mono_time
        except Exception as e:
            log.error(f"[ARUCO DETECTOR ERROR] Vision parsing failed: {e}")
            return [], capture_mono_time

def save_audit_snapshot(frame_bgr, tag, dets, debug_info):
    """Saves an annotated JPEG with Animal (green), Vetoed (orange), and Person/Vehicle (red) boxes."""
    try:
        vis = frame_bgr.copy()
        for v in debug_info.get("veto_boxes", []):
            x1, y1, x2, y2 = v["box"]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(
                vis,
                f"VETO:{v['class']} {v['score']:.2f}",
                (x1, max(y1 - 6, 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2,
            )
        for r in debug_info.get("rejected_animals", []):
            x1, y1, x2, y2 = r["box"]
            pa, pp, pv = r["probs"]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 165, 255), 2)
            cv2.putText(
                vis,
                f"REJ:{r['reason']} A:{pa:.2f} P:{pp:.2f} V:{pv:.2f}",
                (x1, max(y1 - 6, 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 165, 255),
                2,
            )
        for d in dets:
            x1, y1, x2, y2 = d["box"]
            pa, pp, pv = d.get("probs", (d["score"], 0.0, 0.0))
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                vis,
                f"ACQ A:{pa:.2f} P:{pp:.2f} V:{pv:.2f}",
                (x1, max(y1 - 6, 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        out_path = os.path.join(DEBUG_SNAPSHOT_DIR, f"{tag}_{ts}.jpg")
        cv2.imwrite(out_path, vis, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        log.info(f"[AUDIT] Saved {tag} snapshot -> {out_path}")

        # Keep debug_snapshots bounded so storage never fills up
        snaps = glob.glob(os.path.join(DEBUG_SNAPSHOT_DIR, "*.jpg"))
        if len(snaps) > MAX_DEBUG_SNAPSHOTS:
            snaps.sort(key=os.path.getmtime)
            for old_snap in snaps[:-MAX_DEBUG_SNAPSHOTS]:
                try:
                    os.remove(old_snap)
                except OSError:
                    pass
    except Exception as e:
        log.warning(f"[AUDIT] Failed to save snapshot: {e}")


class MDV6Decoder1280:
    """
    Zero-Allocation Quantized MegaDetector v6 (YOLOv9-c 1280x1280) Post-Processor.
    Classes in MDV6:
        0: animal
        1: person
        2: vehicle
    Enforces strict class dominance, dual-class (person + vehicle) zero-overlap
    spatial veto with safety padding, maximum size sanity checks, and full audit
    metadata for post-mortem false-positive debugging.
    """
    CLASS_LAYERS = ("MDV6-yolov9-c/conv100", "MDV6-yolov9-c/conv122", "MDV6-yolov9-c/conv144")
    BOX_LAYERS = ("MDV6-yolov9-c/conv98", "MDV6-yolov9-c/conv121", "MDV6-yolov9-c/conv143")

    VETO_PADDING_PX = 24          # Expand person/vehicle boxes by 24px in all directions
    CLASS_MARGIN = 0.25           # p_anim must beat both p_pers and p_veh by >= 0.25
    MAX_ANIMAL_WIDTH_FRAC = 0.55  # Reject vehicle-sized boxes (> 55% frame width)
    MAX_ANIMAL_AREA_FRAC = 0.40   # Reject boxes covering > 40% of total frame
    MAX_ANIMAL_ASPECT_RATIO = 1.65  # Reject tall/narrow standing-human silhouettes (h/w > 1.65)
    MAX_NON_ANIMAL_PROB = 0.02      # Reject animal candidates if person or vehicle prob >= 0.02

    def __init__(
        self,
        hef,
        input_size=1280,
        y_pad_offset=280,
        conf_thresh=0.55,
        iou_thresh=0.45,
        person_veto_thresh=0.20,
        vehicle_veto_thresh=0.20,
    ):
        self.input_size = input_size
        self.y_pad_offset = y_pad_offset
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh
        self.person_veto_thresh = person_veto_thresh
        self.vehicle_veto_thresh = vehicle_veto_thresh
        self.strides = (8, 16, 32)
        self.reg_max = 16
        self.project = np.arange(self.reg_max, dtype=np.float32)

        # Logit thresholds: sigmoid(x) > p  <=>  x > -ln(1/p - 1)
        self.logit_thresh = -math.log((1.0 / self.conf_thresh) - 1.0)
        self.person_logit_thresh = -math.log((1.0 / self.person_veto_thresh) - 1.0)
        self.vehicle_logit_thresh = -math.log((1.0 / self.vehicle_veto_thresh) - 1.0)

        # Pre-compute per-stride anchor grids
        self.head_anchors = []
        for stride in self.strides:
            grid_size = self.input_size // stride
            grid_y, grid_x = np.meshgrid(
                np.arange(grid_size, dtype=np.float32),
                np.arange(grid_size, dtype=np.float32),
                indexing="ij",
            )
            anchor = np.stack((grid_x + 0.5, grid_y + 0.5), axis=-1) * stride
            self.head_anchors.append(anchor.reshape(-1, 2))

        # Extract hardware quantization parameters (qp_scale, qp_zp) from HEF vstream infos
        self.quant_params = {}
        self.u8_animal_thresholds = {}
        self.u8_person_thresholds = {}
        self.u8_vehicle_thresholds = {}
        try:
            for vinfo in hef.get_output_vstream_infos():
                qp = vinfo.quant_info
                scale = float(qp.qp_scale)
                zp = float(qp.qp_zp)
                self.quant_params[vinfo.name] = (scale, zp)
                if vinfo.name in self.CLASS_LAYERS:
                    raw_anim_u8 = math.ceil((self.logit_thresh / scale) + zp)
                    raw_pers_u8 = math.ceil((self.person_logit_thresh / scale) + zp)
                    raw_veh_u8 = math.ceil((self.vehicle_logit_thresh / scale) + zp)
                    self.u8_animal_thresholds[vinfo.name] = int(max(0, min(255, raw_anim_u8)))
                    self.u8_person_thresholds[vinfo.name] = int(max(0, min(255, raw_pers_u8)))
                    self.u8_vehicle_thresholds[vinfo.name] = int(max(0, min(255, raw_veh_u8)))
            self.use_uint8_fastpath = (len(self.u8_animal_thresholds) == 3)
        except Exception as e:
            log.warning(f"[HAILO] Quantization metadata unavailable ({e}); using float32 fallback.")
            self.use_uint8_fastpath = False

    @staticmethod
    def _softmax(x, axis=-1):
        e_x = np.exp(x - np.max(x, axis=axis, keepdims=True))
        return e_x / np.sum(e_x, axis=axis, keepdims=True)

    @classmethod
    def _check_veto_overlap(cls, anim_box, veto_boxes):
        """
        Returns (True, veto_dict) if anim_box [x1, y1, x2, y2] has ANY overlap
        with any padded person or vehicle box.
        """
        ax1, ay1, ax2, ay2 = anim_box
        pad = cls.VETO_PADDING_PX
        for v in veto_boxes:
            vx1, vy1, vx2, vy2 = v["box"]
            if (ax1 < vx2 + pad) and (ax2 > vx1 - pad) and (ay1 < vy2 + pad) and (ay2 > vy1 - pad):
                return True, v
        return False, None

    def decode(self, raw_outputs, assumed_height_mm):
        cand_probs_list = []
        cand_boxes_list = []
        cand_anchors_list = []
        cand_scales_list = []

        for idx, stride in enumerate(self.strides):
            cls_name = self.CLASS_LAYERS[idx]
            box_name = self.BOX_LAYERS[idx]

            cls_tensor = raw_outputs[cls_name].reshape(-1, 3)
            animal_col = cls_tensor[:, 0]
            person_col = cls_tensor[:, 1]
            vehicle_col = cls_tensor[:, 2]

            if self.use_uint8_fastpath and cls_tensor.dtype == np.uint8:
                u8_anim_t = self.u8_animal_thresholds[cls_name]
                u8_pers_t = self.u8_person_thresholds[cls_name]
                u8_veh_t = self.u8_vehicle_thresholds[cls_name]

                # Collect ALL anchors exceeding animal, person, OR vehicle thresholds
                anim_mask = (animal_col >= u8_anim_t) & (animal_col > person_col) & (animal_col > vehicle_col)
                pers_mask = (person_col >= u8_pers_t)
                veh_mask = (vehicle_col >= u8_veh_t)
                combined_mask = anim_mask | pers_mask | veh_mask
                if not np.any(combined_mask):
                    continue

                valid_indices = np.flatnonzero(combined_mask)
                c_scale, c_zp = self.quant_params[cls_name]
                sub_cls_f32 = (cls_tensor[valid_indices].astype(np.float32) - c_zp) * c_scale

                b_scale, b_zp = self.quant_params[box_name]
                raw_box_u8 = raw_outputs[box_name].reshape(-1, 4, self.reg_max)[valid_indices]
                raw_boxes = (raw_box_u8.astype(np.float32) - b_zp) * b_scale
            else:
                anim_mask = (animal_col > self.logit_thresh) & (animal_col > person_col) & (animal_col > vehicle_col)
                pers_mask = (person_col > self.person_logit_thresh)
                veh_mask = (vehicle_col > self.vehicle_logit_thresh)
                combined_mask = anim_mask | pers_mask | veh_mask
                if not np.any(combined_mask):
                    continue

                valid_indices = np.flatnonzero(combined_mask)
                sub_cls_f32 = cls_tensor[valid_indices]
                raw_boxes = raw_outputs[box_name].reshape(-1, 4, self.reg_max)[valid_indices]

            probs = 1.0 / (1.0 + np.exp(-sub_cls_f32))
            cand_probs_list.append(probs)
            cand_boxes_list.append(raw_boxes)
            cand_anchors_list.append(self.head_anchors[idx][valid_indices])
            cand_scales_list.append(np.full(len(valid_indices), stride, dtype=np.float32))

        debug_info = {"veto_boxes": [], "rejected_animals": []}
        if not cand_probs_list:
            return [], debug_info

        cand_probs = np.vstack(cand_probs_list)
        cand_boxes = np.vstack(cand_boxes_list)
        cand_anchors = np.vstack(cand_anchors_list)
        cand_scales = np.concatenate(cand_scales_list)

        dist = self._softmax(cand_boxes, axis=-1)
        ltrb = np.dot(dist, self.project) * cand_scales[:, None]

        x1 = np.clip(cand_anchors[:, 0] - ltrb[:, 0], 0, FRAME_WIDTH)
        y1 = np.clip(cand_anchors[:, 1] - ltrb[:, 1] - self.y_pad_offset, 0, FRAME_HEIGHT)
        x2 = np.clip(cand_anchors[:, 0] + ltrb[:, 2], 0, FRAME_WIDTH)
        y2 = np.clip(cand_anchors[:, 1] + ltrb[:, 3] - self.y_pad_offset, 0, FRAME_HEIGHT)

        w = x2 - x1
        h = y2 - y1

        # 1. Separate decoded boxes into NMS-filtered Veto Zones (PERSON & VEHICLE) vs Animal Candidates
        veto_cv_boxes, veto_scores, veto_meta = [], [], []
        anim_cv_boxes, anim_scores, anim_probs = [], [], []

        for i in range(len(cand_probs)):
            bw_i = int(w[i])
            bh_i = int(h[i])
            if bw_i < 8 or bh_i < 8:
                continue
            bx_i = int(x1[i])
            by_i = int(y1[i])
            p_anim = float(cand_probs[i, 0])
            p_pers = float(cand_probs[i, 1])
            p_veh = float(cand_probs[i, 2])

            if p_pers >= self.person_veto_thresh:
                veto_cv_boxes.append([bx_i, by_i, bw_i, bh_i])
                veto_scores.append(p_pers)
                veto_meta.append(("PERSON", p_pers))
            if p_veh >= self.vehicle_veto_thresh:
                veto_cv_boxes.append([bx_i, by_i, bw_i, bh_i])
                veto_scores.append(p_veh)
                veto_meta.append(("VEHICLE", p_veh))

            # Strict animal eligibility: above conf_thresh AND beats both person & vehicle by CLASS_MARGIN
            if (
                p_anim >= self.conf_thresh
                and p_anim >= (p_pers + self.CLASS_MARGIN)
                and p_anim >= (p_veh + self.CLASS_MARGIN)
            ):
                anim_cv_boxes.append([bx_i, by_i, bw_i, bh_i])
                anim_scores.append(p_anim)
                anim_probs.append((p_anim, p_pers, p_veh))

        veto_boxes = []
        if veto_cv_boxes:
            v_idx = cv2.dnn.NMSBoxes(
                veto_cv_boxes,
                veto_scores,
                min(self.person_veto_thresh, self.vehicle_veto_thresh),
                0.45,
            )
            if len(v_idx) > 0:
                for vi in v_idx.flatten():
                    vx, vy, vw, vh = veto_cv_boxes[vi]
                    v_cls, v_score = veto_meta[vi]
                    veto_boxes.append({
                        "class": v_cls,
                        "score": v_score,
                        "box": [vx, vy, vx + vw, vy + vh],
                    })

        debug_info["veto_boxes"] = veto_boxes
        if not anim_cv_boxes:
            return [], debug_info

        # 2. NMS on Animal Candidates + Zero-Overlap Veto & Size Sanity Check
        indices = cv2.dnn.NMSBoxes(anim_cv_boxes, anim_scores, self.conf_thresh, self.iou_thresh)

        detections = []
        frame_area = float(FRAME_WIDTH * FRAME_HEIGHT)

        if len(indices) > 0:
            for idx in indices.flatten():
                bx, by, bw, bh = anim_cv_boxes[idx]
                box_xyxy = [bx, by, bx + bw, by + bh]
                p_anim, p_pers, p_veh = anim_probs[idx]
                area = float(bw * bh)

                # Size sanity check (reject vehicle-sized boxes)
                if (bw / FRAME_WIDTH) > self.MAX_ANIMAL_WIDTH_FRAC or (area / frame_area) > self.MAX_ANIMAL_AREA_FRAC:
                    debug_info["rejected_animals"].append({
                        "reason": "SIZE_LIMIT",
                        "box": box_xyxy,
                        "probs": (p_anim, p_pers, p_veh),
                    })
                    continue

                # Non-animal probability ceiling (reject if person or vehicle prob >= 0.02)
                if p_pers >= self.MAX_NON_ANIMAL_PROB or p_veh >= self.MAX_NON_ANIMAL_PROB:
                    debug_info["rejected_animals"].append({
                        "reason": f"NON_ANIMAL_PROB(P:{p_pers:.2f},V:{p_veh:.2f})",
                        "box": box_xyxy,
                        "probs": (p_anim, p_pers, p_veh),
                    })
                    continue

                # Standing-human aspect ratio filter (height / width > 1.65)
                aspect_ratio = float(bh) / max(1.0, float(bw))
                if aspect_ratio > self.MAX_ANIMAL_ASPECT_RATIO:
                    debug_info["rejected_animals"].append({
                        "reason": f"ASPECT_RATIO({aspect_ratio:.2f})",
                        "box": box_xyxy,
                        "probs": (p_anim, p_pers, p_veh),
                    })
                    continue

                # Zero-overlap spatial veto against any PERSON or VEHICLE
                is_vetoed, veto_hit = self._check_veto_overlap(box_xyxy, veto_boxes)
                if is_vetoed:
                    debug_info["rejected_animals"].append({
                        "reason": f"OVERLAP_{veto_hit['class']}({veto_hit['score']:.2f})",
                        "box": box_xyxy,
                        "probs": (p_anim, p_pers, p_veh),
                    })
                    continue

                cx = bx + (bw / 2.0)
                cy = by + (bh * 0.60)
                est_range_m = (assumed_height_mm * FOCAL_LENGTH_PX) / (max(12.0, float(bh)) * 1000.0)
                clamped_range_m = float(np.clip(est_range_m, MIN_VALID_RANGE_M, MAX_VALID_RANGE_M))

                detections.append({
                    "box": box_xyxy,
                    "center": (cx, cy),
                    "score": p_anim,
                    "probs": (p_anim, p_pers, p_veh),
                    "area": area,
                    "range_m": clamped_range_m,
                    "id": 1,
                    "label": f"ANIMAL {p_anim*100:.0f}%"
                })

            detections.sort(key=lambda d: (d["score"], d["area"]), reverse=True)

        return detections, debug_info

def _hailo_subprocess_entry(model_path, shm_name, cap_mono_val, active_flag, avail_flag, stop_flag, conn_tx):
    """
    Isolated OS Process for Hailo-8 PCIe DMA Inference.
    Runs in its own Python interpreter with its own GIL so InferVStreams.infer()
    never pauses the 30 FPS camera thread or 50 Hz motion servo loop.
    Handles SIGTERM/SIGINT cleanly so systemd stops in < 0.5 seconds.
    """
    set_os_thread_name("HailoNPUProc", cpu_cores=[2])

    # Catch SIGTERM/SIGINT from systemd and flip stop_flag to exit context managers cleanly
    def _sig_handler(signum, frame):
        stop_flag.value = 1

    signal.signal(signal.SIGTERM, _sig_handler)
    signal.signal(signal.SIGINT, _sig_handler)

    shm = None
    try:
        shm = shared_memory.SharedMemory(name=shm_name)
        resource_tracker.unregister(shm._name, "shared_memory")
        shared_frame = np.ndarray((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8, buffer=shm.buf)

        hef = HEF(model_path)
        decoder = MDV6Decoder1280(
            hef=hef,
            input_size=HailoWildlifeDetector.INFER_SIZE,
            y_pad_offset=HailoWildlifeDetector.Y_PAD_OFFSET,
            conf_thresh=HailoWildlifeDetector.CONF_THRESH,
            iou_thresh=HailoWildlifeDetector.IOU_THRESH,
            person_veto_thresh=HailoWildlifeDetector.PERSON_VETO_THRESH,
            vehicle_veto_thresh=HailoWildlifeDetector.VEHICLE_VETO_THRESH,
        )

        params = VDevice.create_params()
        with VDevice(params) as target:
            configure_params = ConfigureParams.create_from_hef(
                hef, interface=HailoStreamInterface.PCIe
            )
            network_group = target.configure(hef, configure_params)[0]
            network_group_params = network_group.create_params()

            input_vparams = InputVStreamParams.make(
                network_group, quantized=True, format_type=FormatType.UINT8
            )
            out_fmt = FormatType.UINT8 if decoder.use_uint8_fastpath else FormatType.FLOAT32
            output_vparams = OutputVStreamParams.make(
                network_group, quantized=decoder.use_uint8_fastpath, format_type=out_fmt
            )
            input_name = hef.get_input_vstream_infos()[0].name

            letterbox_tensor = np.zeros(
                (1, HailoWildlifeDetector.INFER_SIZE, HailoWildlifeDetector.INFER_SIZE, 3),
                dtype=np.uint8
            )
            y_off = HailoWildlifeDetector.Y_PAD_OFFSET
            letterbox_roi = letterbox_tensor[0, y_off:y_off + FRAME_HEIGHT, :, :]

            with network_group.activate(network_group_params):
                with InferVStreams(network_group, input_vparams, output_vparams) as pipeline:
                    avail_flag.value = 1
                    log.info(
                        f"[HAILO NPU PROCESS] Isolated PID:{os.getpid()} active "
                        f"({'UINT8 FastPath' if decoder.use_uint8_fastpath else 'FLOAT32'}): {model_path}"
                    )

                    last_cap_seen = 0.0
                    last_fps_time = time.time()
                    frames_processed = 0
                    cur_fps = 0.0

                    # Exit immediately when stop_flag is set or parent process dies (ppid == 1)
                    while stop_flag.value == 0 and os.getppid() != 1:
                        if active_flag.value == 0:
                            time.sleep(0.05)
                            continue

                        cap_mono = cap_mono_val.value
                        if cap_mono <= last_cap_seen:
                            time.sleep(0.002)
                            continue

                        last_cap_seen = cap_mono
                        t0 = time.perf_counter()

                        cv2.cvtColor(shared_frame, cv2.COLOR_BGR2RGB, dst=letterbox_roi)

                        raw_outputs = pipeline.infer({input_name: letterbox_tensor})
                        dets, debug_info = decoder.decode(raw_outputs, HailoWildlifeDetector.ASSUMED_TARGET_HEIGHT_MM)
                        dt_ms = (time.perf_counter() - t0) * 1000.0

                        frames_processed += 1
                        now = time.time()
                        elapsed = now - last_fps_time
                        if elapsed >= 1.0:
                            cur_fps = frames_processed / elapsed
                            frames_processed = 0
                            last_fps_time = now

                        try:
                            conn_tx.send((dets, debug_info, cap_mono, dt_ms, cur_fps))
                        except (BrokenPipeError, EOFError, OSError):
                            break

    except Exception as e:
        if stop_flag.value == 0:
            log.error(f"[HAILO SUBPROCESS FATAL] {e}\n{traceback.format_exc()}")
    finally:
        avail_flag.value = 0
        if shm is not None:
            try:
                shm.close()
            except Exception:
                pass
        log.info(f"[HAILO NPU PROCESS] PID:{os.getpid()} shut down cleanly.")

class HailoWildlifeDetector(BaseTargetDetector):
    """
    Process-Isolated Hailo-8 (26 TOPS) Wildlife Detector.
    Uses POSIX SharedMemory so the NPU runs in a separate OS process with zero
    GIL contention against VisionMain (30 FPS) and MotionServo (50 Hz).
    """
    INFER_SIZE = 1280
    Y_PAD_OFFSET = (INFER_SIZE - FRAME_HEIGHT) // 2
    CONF_THRESH = 0.55
    MONOCHROME_CONF_THRESH = 0.55
    MAX_ANIMAL_HEIGHT_M = 1.50
    IOU_THRESH = 0.45
    PERSON_VETO_THRESH = 0.20
    VEHICLE_VETO_THRESH = 0.20
    ASSUMED_TARGET_HEIGHT_MM = 950.0

    CANDIDATE_PATHS = [
        os.path.join(BASE_DIR, "MDV6-yolov9-c-1280.hef"),
        os.path.join(BASE_DIR, "models", "MDV6-yolov9-c-1280.hef"),
        "/home/ben/Bombardeer/MDV6-yolov9-c-1280.hef",
        "/home/ben/Bombardeer/models/MDV6-yolov9-c-1280.hef",
        "/home/ben/bombardeer/MDV6-yolov9-c-1280.hef",
        "/home/ben/bombardeer/models/MDV6-yolov9-c-1280.hef",
    ]
    
    def shutdown(self):
        """Signals the NPU subprocess to exit cleanly and unlinks POSIX shared memory."""
        try:
            if hasattr(self, "_stop_flag") and self._stop_flag is not None:
                self._stop_flag.value = 1
            if hasattr(self, "_proc") and self._proc is not None and self._proc.is_alive():
                self._proc.join(timeout=1.0)
                if self._proc.is_alive():
                    self._proc.terminate()
            if hasattr(self, "_shm") and self._shm is not None:
                self._shm.close()
                self._shm.unlink()
        except Exception:
            pass

    def __init__(self):
        self._lock = threading.Lock()
        self._latest_detections = []
        self._latest_debug_info = {"veto_boxes": [], "rejected_animals": []}
        self._latest_capture_mono = 0.0
        self._npu_latency_ms = 0.0
        self._npu_fps = 0.0

        self.model_path = next((p for p in self.CANDIDATE_PATHS if os.path.exists(p)), None)
        if not HAILO_AVAILABLE or not self.model_path:
            log.warning("[HAILO] HailoRT or HEF model unavailable.")
            self._avail_flag = None
            return

        ctx = mp.get_context("fork")
        frame_bytes = FRAME_HEIGHT * FRAME_WIDTH * 3
        self._shm = shared_memory.SharedMemory(create=True, size=frame_bytes)
        self._shm_array = np.ndarray((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8, buffer=self._shm.buf)

        self._cap_mono_val = ctx.Value("d", 0.0, lock=False)
        self._active_flag = ctx.Value("i", 0, lock=False)
        self._avail_flag = ctx.Value("i", 0, lock=False)
        self._stop_flag = ctx.Value("i", 0, lock=False)
        self._conn_rx, conn_tx = ctx.Pipe(duplex=False)

        self._proc = ctx.Process(
            target=_hailo_subprocess_entry,
            args=(
                self.model_path,
                self._shm.name,
                self._cap_mono_val,
                self._active_flag,
                self._avail_flag,
                self._stop_flag,
                conn_tx
            ),
            daemon=True,
            name="HailoNPUProcess"
        )
        self._proc.start()
        atexit.register(self.shutdown)

    @property
    def name(self) -> str:
        return "WILDLIFE"

    @property
    def is_available(self) -> bool:
        if self._avail_flag is None:
            return False
        is_ok = (self._avail_flag.value == 1)
        with target_lock:
            system_health["hailo_available"] = is_ok
        return is_ok

    def set_active(self, active: bool):
        if self._avail_flag is None:
            return
        self._active_flag.value = 1 if active else 0
        if not active:
            with self._lock:
                self._latest_detections = []
                self._latest_debug_info = {"veto_boxes": [], "rejected_animals": []}
                self._npu_fps = 0.0

    def get_telemetry(self) -> dict:
        with self._lock:
            return {
                "npu_fps": self._npu_fps,
                "npu_latency_ms": self._npu_latency_ms,
                "available": self.is_available
            }

    def get_last_debug_info(self) -> dict:
        with self._lock:
            return {
                "veto_boxes": list(self._latest_debug_info.get("veto_boxes", [])),
                "rejected_animals": list(self._latest_debug_info.get("rejected_animals", [])),
            }

    def detect(self, frame_bgr: np.ndarray, capture_mono_time: float, is_monochrome: bool = False):
        if not self.is_available:
            return [], capture_mono_time

        self._active_flag.value = 1

        # 1. Drain any completed inference results from the NPU subprocess pipe (non-blocking)
        while self._conn_rx.poll():
            try:
                dets, debug_info, eff_mono, dt_ms, cur_fps = self._conn_rx.recv()
                if dets:
                    _, tilt_at_cap = get_turret_pos_at_time(eff_mono - 0.040)
                    rejected_animals = debug_info.setdefault("rejected_animals", [])
                    filtered_dets = []
                    for d in dets:
                        pa, pp, pv = d.get("probs", (d["score"], 0.0, 0.0))

                        # Monochrome / low-light confidence gate
                        if is_monochrome and d["score"] < self.MONOCHROME_CONF_THRESH:
                            rejected_animals.append({
                                "reason": f"MONO_CONF({d['score']:.2f})",
                                "box": d["box"],
                                "probs": (pa, pp, pv),
                            })
                            continue

                        # Ground-plane physical height veto & trigonometric range fusion
                        bx1, by1, bx2, by2 = d["box"]
                        bh_px = float(max(1, by2 - by1))
                        y_bottom = float(by2)
                        opt_dep_mrad = math.atan2(y_bottom - OPTICAL_CENTER[1], FOCAL_LENGTH_PX) * 1000.0
                        net_dep_mrad = opt_dep_mrad - float(tilt_at_cap) - GROUND_SLOPE_MRAD
                        if net_dep_mrad >= 25.0 and y_bottom < (FRAME_HEIGHT - 8):
                            r_ground = CAMERA_MOUNT_HEIGHT_M / math.sin(net_dep_mrad / 1000.0)
                            est_height_m = (bh_px * r_ground) / FOCAL_LENGTH_PX
                            if est_height_m > self.MAX_ANIMAL_HEIGHT_M:
                                rejected_animals.append({
                                    "reason": f"HEIGHT({est_height_m:.2f}m)",
                                    "box": d["box"],
                                    "probs": (pa, pp, pv),
                                })
                                continue
                            r_stadia = d.get("range_m", DEFAULT_FALLBACK_RANGE_M)
                            r_fused = (0.70 * r_ground) + (0.30 * r_stadia)
                            d["range_m"] = float(np.clip(r_fused, MIN_VALID_RANGE_M, MAX_VALID_RANGE_M))

                        filtered_dets.append(d)
                    dets = filtered_dets

                with self._lock:
                    self._latest_detections = dets
                    self._latest_debug_info = debug_info
                    self._latest_capture_mono = eff_mono
                    self._npu_latency_ms = dt_ms
                    self._npu_fps = cur_fps
                with target_lock:
                    system_health["npu_fps"] = cur_fps
                    system_health["npu_latency_ms"] = dt_ms
            except Exception:
                break

        # 2. Zero-copy write latest camera frame into POSIX shared memory (~0.3ms)
        np.copyto(self._shm_array, frame_bgr)
        self._cap_mono_val.value = capture_mono_time

        with self._lock:
            dets = list(self._latest_detections)
            eff_mono = self._latest_capture_mono if self._latest_capture_mono > 0 else capture_mono_time

        return dets, eff_mono

# =========================================================================
# Robust Serial Worker Thread
# =========================================================================
def find_working_serial_port():
    for port in DEFAULT_SERIAL_PORTS:
        if os.path.exists(port):
            try:
                ser = serial.Serial(port, BAUD_RATE, timeout=0.02)
                ser.close()
                return port
            except (OSError, serial.SerialException):
                continue
    return None

def turret_serial_worker(cmd_queue):
    ser = None
    active_port = None
    reconnect_delay = 2.0
    next_reconnect_time = 0.0
    last_port_missing_log_time = 0.0

    trigger_state = 0
    trigger_start_time = 0.0
    active_pulse_limit = MAX_TRIGGER_DURATION_SEC
    cooldown_until = 0.0

    rx_buf = bytearray()
    last_telemetry_rx_time = 0.0

    set_os_thread_name("TurretSerial", cpu_cores=[3])

    while True:
        now = time.time()
        mono_now = time.monotonic()

        if ser is None or not ser.is_open:
            set_serial_connected(False)
            clear_queue(cmd_queue)

            if now >= next_reconnect_time:
                active_port = find_working_serial_port()
                if active_port:
                    try:
                        ser = serial.Serial(
                            active_port,
                            BAUD_RATE,
                            timeout=0.01,
                            write_timeout=0.1,
                            rtscts=False,
                            dsrdtr=False
                        )
                        try:
                            ser.dtr = False
                            ser.rts = False
                        except Exception:
                            pass

                        rx_buf.clear()
                        last_telemetry_rx_time = mono_now

                        time.sleep(0.3)
                        ser.reset_input_buffer()
                        ser.reset_output_buffer()

                        set_serial_connected(True)
                        log.info(f"[SERIAL] Connected to {active_port} at {BAUD_RATE} baud.")
                        set_alert(f"Connected: {active_port}", duration=2.0)

                    except Exception as e:
                        if ser:
                            try:
                                ser.close()
                            except Exception:
                                pass
                        ser = None
                        next_reconnect_time = now + reconnect_delay
                        set_alert(f"Serial Open Error: {e}", duration=2.0)
                        log.error(f"[SERIAL ERROR] Failed connecting to {active_port}: {e}")
                else:
                    if now - last_port_missing_log_time >= 4.0:
                        last_port_missing_log_time = now
                        log.warning(f"[SERIAL WARN] No hardware ESP32 found on ports {DEFAULT_SERIAL_PORTS}.")
                        set_alert("ESP32 DISCONNECTED", duration=3.0)
                    next_reconnect_time = now + reconnect_delay

        if ser and ser.is_open:
            is_active = (mono_now - last_telemetry_rx_time <= 2.0)
            set_serial_connected(is_active)

        try:
            pending_commands = []
            latest_velocity = None

            while not cmd_queue.empty():
                item = cmd_queue.get_nowait()
                if item.get("cmd") == "VELOCITY":
                    latest_velocity = item
                else:
                    pending_commands.append(item)

            if latest_velocity is not None:
                pending_commands.append(latest_velocity)

            for item in pending_commands:
                cmd = item.get("cmd")

                if ser and ser.is_open:
                    if cmd == "SETPOINT":
                        t_pan = int(item["pan"])
                        t_tilt = int(item["tilt"])
                        ser.write(f"P {t_pan} {t_tilt}\n".encode("ascii"))
                        with target_lock:
                            system_health["serial_tx_count"] += 1

                    elif cmd == "VELOCITY":
                        p_s = int(item["pan_s"])
                        t_s = int(item["tilt_s"])
                        ser.write(f"V {p_s} {t_s}\n".encode("ascii"))
                        with target_lock:
                            system_health["serial_tx_count"] += 1

                    elif cmd == "HALT":
                        ser.write(b"X\n")
                        with target_lock:
                            system_health["serial_tx_count"] += 1

                    elif cmd == "TRIGGER":
                        req_state = item["state"]
                        custom_pulse = item.get("pulse_len")

                        if req_state == 1 and (now - item.get("timestamp", now) > 0.100):
                            continue

                        if req_state == 1:
                            if trigger_state == 0 and now >= cooldown_until:
                                trigger_state = 1
                                trigger_start_time = now
                                active_pulse_limit = custom_pulse if custom_pulse else MAX_TRIGGER_DURATION_SEC
                                set_firing_state(True)
                                ser.write(b"T 1\n")
                                with target_lock:
                                    system_health["serial_tx_count"] += 1
                                log.info(f"[SERIAL] Solenoid trigger pulse FIRED (Limit: {active_pulse_limit*1000:.0f}ms)")
                        elif req_state == 0:
                            if trigger_state == 1:
                                trigger_state = 0
                                set_firing_state(False)
                                cooldown_until = now + TRIGGER_COOLDOWN_SEC
                                ser.write(b"T 0\n")
                                with target_lock:
                                    system_health["serial_tx_count"] += 1
        except Exception as e:
            log.warning(f"[SERIAL TX FAULT] {e}")

        if trigger_state == 1 and (now - trigger_start_time >= active_pulse_limit):
            trigger_state = 0
            set_firing_state(False)
            cooldown_until = now + TRIGGER_COOLDOWN_SEC
            if ser and ser.is_open:
                try:
                    ser.write(b"T 0\n")
                    with target_lock:
                        system_health["serial_tx_count"] += 1
                except Exception:
                    pass

        if ser and ser.is_open:
            try:
                bytes_avail = ser.in_waiting
                if bytes_avail > 0:
                    rx_buf.extend(ser.read(bytes_avail))

                    while True:
                        nl_idx = rx_buf.find(b"\n")
                        if nl_idx == -1:
                            break

                        line_bytes = rx_buf[:nl_idx].strip()
                        del rx_buf[:nl_idx + 1]

                        if not line_bytes or not line_bytes.startswith(b"S "):
                            continue

                        rx_mono_time = time.monotonic()
                        parts = line_bytes.split()

                        try:
                            if len(parts) >= 6:
                                p_mrad = int(parts[2])
                                t_mrad = int(parts[3])
                                moving = (parts[5] == b"1")
                            elif len(parts) >= 5:
                                p_mrad = int(parts[1])
                                t_mrad = int(parts[2])
                                moving = (parts[4] == b"1")
                            else:
                                continue

                            last_telemetry_rx_time = rx_mono_time
                            update_turret_telemetry(p_mrad, t_mrad, moving, rx_mono_time)

                            with target_lock:
                                system_health["serial_rx_count"] += 1

                        except ValueError:
                            continue

            except (serial.SerialException, OSError) as e:
                log.warning(f"[SERIAL HARDWARE ERROR] {e}")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                set_serial_connected(False)

        time.sleep(0.002)

# =========================================================================
# Video Storage Worker
# =========================================================================
def ensure_storage_headroom():
    try:
        _, _, free = shutil.disk_usage(RECORDINGS_DIR)
        calc_free_gb = free / (1024 ** 3)
        with target_lock:
            system_health["free_disk_gb"] = calc_free_gb

        if calc_free_gb < MIN_FREE_SPACE_GB:
            set_alert(f"LOW STORAGE: {calc_free_gb:.1f} GB Free", duration=4.0)

        files = glob.glob(os.path.join(RECORDINGS_DIR, "wildlife_*.*"))
        if not files:
            return

        files.sort(key=os.path.getmtime)
        archive_gb = sum(os.path.getsize(f) for f in files) / (1024 ** 3)

        for fpath in files:
            if calc_free_gb >= MIN_FREE_SPACE_GB and archive_gb <= MAX_RECORDINGS_STORAGE_GB:
                break
            size = os.path.getsize(fpath)
            try:
                os.remove(fpath)
                log.info(f"[STORAGE PURGE] Cleared old recording: {fpath}")
                archive_gb -= size / (1024 ** 3)
                _, _, updated_free = shutil.disk_usage(RECORDINGS_DIR)
                calc_free_gb = updated_free / (1024 ** 3)
                with target_lock:
                    system_health["free_disk_gb"] = calc_free_gb
            except OSError as e:
                log.error(f"[STORAGE ERROR] Purge failed on {fpath}: {e}")
    except Exception as e:
        log.error(f"[STORAGE EXCEPTION] Disk check failed: {e}")

def video_recorder_worker(record_queue):
    writer = None
    recording_active = False
    current_filepath = None
    frames_written = 0

    CODECS = [
        ("mp4v", cv2.VideoWriter_fourcc(*"mp4v"), ".mp4"),
        ("avc1", cv2.VideoWriter_fourcc(*"avc1"), ".mp4"),
        ("MJPG", cv2.VideoWriter_fourcc(*"MJPG"), ".avi"),
        ("XVID", cv2.VideoWriter_fourcc(*"XVID"), ".avi")
    ]

    set_os_thread_name("VidRecorder", cpu_cores=[2])

    while True:
        try:
            item = record_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        cmd = item.get("cmd")

        if cmd == "start":
            if not recording_active:
                ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                writer = None
                frames_written = 0
                current_filepath = None
                clip_fps = float(item.get("fps", TARGET_FPS))

                for tag, fourcc, ext in CODECS:
                    test_filepath = os.path.join(RECORDINGS_DIR, f"wildlife_{ts}{ext}")
                    try:
                        cand = cv2.VideoWriter(test_filepath, fourcc, clip_fps, (FRAME_WIDTH, FRAME_HEIGHT))
                        if cand is not None and cand.isOpened():
                            writer = cand
                            current_filepath = test_filepath
                            log.info(f"[RECORDER] VideoWriter opened ({clip_fps:.1f} FPS, codec '{tag}') -> {current_filepath}")
                            break
                        elif cand is not None:
                            cand.release()
                    except Exception as e:
                        log.debug(f"[RECORDER] Codec {tag} rejected: {e}")

                if writer and writer.isOpened():
                    recording_active = True
                    with target_lock:
                        system_health["recorder_active"] = True
                        system_health["recorder_error"] = None

                    for f in item.get("pre_roll", []):
                        try:
                            writer.write(f)
                            frames_written += 1
                        except Exception as e:
                            log.error(f"[RECORDER ERROR] Pre-roll write failure: {e}")
                else:
                    err_msg = "All VideoWriter codecs failed to open."
                    log.error(f"[RECORDER ERROR] {err_msg} Path: {current_filepath}")
                    with target_lock:
                        system_health["recorder_active"] = False
                        system_health["recorder_error"] = err_msg
                    set_alert("RECORDER FAILED: Check Codecs", duration=5.0)
                    recording_active = False

        elif cmd == "frame" and recording_active:
            frame = item.get("frame")
            if writer and writer.isOpened():
                try:
                    writer.write(frame)
                    frames_written += 1
                except Exception as e:
                    log.error(f"[RECORDER ERROR] Frame write fault: {e}")
                    with target_lock:
                        system_health["recorder_error"] = str(e)

        elif cmd == "stop" and recording_active:
            recording_active = False
            with target_lock:
                system_health["recorder_active"] = False

            if writer:
                try:
                    writer.release()
                    writer = None
                    if os.path.exists(current_filepath):
                        file_sz_kb = os.path.getsize(current_filepath) / 1024.0
                        log.info(f"[RECORDER] Finalized {current_filepath} ({frames_written} frames, {file_sz_kb:.1f} KB)")
                        set_alert(f"Saved: {os.path.basename(current_filepath)}", duration=3.0)
                except Exception as e:
                    log.error(f"[RECORDER ERROR] Failed releasing writer: {e}")
            ensure_storage_headroom()

def get_measured_turret_speed_mrad_s():
    """Computes actual physical turret slew speed (mrad/s) over the last ~100ms of ESP32 telemetry."""
    with telemetry_lock:
        if len(telemetry_history) < 2:
            return 0.0
        t1, p1, tilt1 = telemetry_history[-1]
        # Look back up to 3 samples (~150ms at 20Hz telemetry)
        idx = max(0, len(telemetry_history) - 3)
        t0, p0, tilt0 = telemetry_history[idx]
        dt = t1 - t0
        if dt <= 0.005:
            return 0.0
        return math.hypot(p1 - p0, tilt1 - tilt0) / dt

# =========================================================================
# Asynchronous Real-Time Motion Servoing Loop (50 Hz / 20ms Cadence)
# =========================================================================
def motion_servoing_worker(target_filter, turret_queue):
    last_setpoint_time = 0.0
    last_debug_log_time = 0.0
    is_actively_tracking_velocity = False
    last_v_pan = 0
    last_v_tilt = 0
    last_sent_p_pan = 99999
    last_sent_p_tilt = 99999
    last_mode_seen = "OFF"
    last_fire_gate_ok_time = 0.0

    last_target_time = time.time()
    is_homing = False

    MUZZLE_VELOCITY_M_S = 85.0
    GRAVITY_M_S2 = 9.81
    DRAG_K = 0.0128

    log.info("[MOTION] 50 Hz Real-Time Motion Controller (Drag Ballistics & Auto-Home) started")
    set_os_thread_name("MotionServo", cpu_cores=[3])

    while True:
        loop_start = time.monotonic()
        now = time.time()

        mode, is_primed = get_turret_mode()
        target_data = get_primary_target_snapshot()
        current_pan, current_tilt, hw_moving, _ = get_instantaneous_telemetry()
        serial_ok = get_serial_connected()

        if mode == "OFF":
            if last_mode_seen != "OFF":
                turret_queue.put({"cmd": "HALT"})
                turret_queue.put({"cmd": "TRIGGER", "state": 0})
                is_actively_tracking_velocity = False
                last_v_pan = 0
                last_v_tilt = 0
                last_sent_p_pan = 99999
                last_sent_p_tilt = 99999
                last_fire_gate_ok_time = 0.0
                target_filter.reset()
                log.info("[MODE] Turret disengaged and placed in OFF / STANDBY.")

            last_target_time = now
            is_homing = False
            last_mode_seen = mode

            elapsed = time.monotonic() - loop_start
            sleep_time = MOTION_LOOP_INTERVAL_SEC - elapsed
            if sleep_time > 0.001:
                time.sleep(sleep_time)
            continue

        last_mode_seen = mode

        if serial_ok:
            target_active = False
            if target_data is not None:
                time_since_seen = now - target_data["last_seen"]
                if time_since_seen <= COAST_MAX_HORIZON_SEC:
                    target_active = True

            if target_active:
                last_target_time = now
                is_homing = False

                predicted_state = target_filter.predict_state(now)
                if predicted_state is not None:
                    base_pan, base_tilt, tgt_v_pan, tgt_v_tilt = predicted_state

                    current_range_m = target_data.get("range_m", DEFAULT_FALLBACK_RANGE_M)

                    if current_range_m > 0:
                        tof_sec = (math.exp(DRAG_K * current_range_m) - 1.0) / (DRAG_K * MUZZLE_VELOCITY_M_S)
                    else:
                        tof_sec = 0.0

                    current_tilt_radians = current_tilt / 1000.0
                    drop_m = 0.5 * (GRAVITY_M_S2 * math.cos(current_tilt_radians)) * (tof_sec ** 2)
                    drop_mrad = (drop_m / current_range_m) * 1000.0 if current_range_m > 0 else 0.0

                    lead_pan_mrad = tgt_v_pan * tof_sec
                    lead_tilt_mrad = tgt_v_tilt * tof_sec

                    world_pan = int(round(base_pan + lead_pan_mrad))
                    world_tilt = int(round(base_tilt + lead_tilt_mrad + drop_mrad + TILT_CALIBRATION_TRIM_MRAD))

                    pan_clamped = False
                    tilt_clamped = False

                    if world_pan < PAN_MIN_MRAD or world_pan > PAN_MAX_MRAD:
                        pan_clamped = True
                        world_pan = max(PAN_MIN_MRAD, min(PAN_MAX_MRAD, world_pan))

                    if world_tilt < TILT_MIN_MRAD or world_tilt > TILT_MAX_MRAD:
                        tilt_clamped = True
                        world_tilt = max(TILT_MIN_MRAD, min(TILT_MAX_MRAD, world_tilt))

                    with target_filter.lock:
                        if target_filter.is_initialized:
                            if pan_clamped:
                                target_filter.kf.statePost[0, 0] = float(world_pan)
                                target_filter.kf.statePost[2, 0] = 0.0
                            if tilt_clamped:
                                target_filter.kf.statePost[1, 0] = float(world_tilt)
                                target_filter.kf.statePost[3, 0] = 0.0

                    realtime_err_pan = world_pan - current_pan
                    realtime_err_tilt = world_tilt - current_tilt
                    radial_error_mrad = math.hypot(realtime_err_pan, realtime_err_tilt)
                    measured_speed_mag = get_measured_turret_speed_mrad_s()
                    cmd_speed_mag = math.hypot(last_v_pan, last_v_tilt)
                    turret_speed_mag = max(cmd_speed_mag, measured_speed_mag)

                    production_fire_gate = (
                        time_since_seen <= 0.150
                        and radial_error_mrad <= FIRE_DEADBAND_MRAD
                        and turret_speed_mag < MAX_FIRE_SLEW_SPEED_MRAD_S
                    )

                    if production_fire_gate:
                        last_fire_gate_ok_time = now

                    can_fire = False
                    shot_pulse_duration = None

                    if mode == "ARMED":
                        can_fire = (now - last_fire_gate_ok_time <= 0.120)
                        shot_pulse_duration = None
                    elif mode == "CALIBRATE":
                        if production_fire_gate and is_primed:
                            if consume_calibration_prime():
                                can_fire = True
                                shot_pulse_duration = 0.080
                                log.info(
                                    f"[CALIBRATE] Shot discharged! | "
                                    f"Range: {current_range_m:.2f}m | "
                                    f"ToF: {tof_sec:.3f}s | "
                                    f"Lead: ({lead_pan_mrad:.1f}, {lead_tilt_mrad:.1f})mrad | "
                                    f"Drop: {drop_mrad:.1f}mrad"
                                )

                    turret_queue.put({
                        "cmd": "TRIGGER",
                        "state": 1 if can_fire else 0,
                        "pulse_len": shot_pulse_duration,
                        "timestamp": now
                    })

                    if not is_actively_tracking_velocity:
                        if radial_error_mrad > BALLISTIC_ACQUIRE_MRAD:
                            use_ballistic = True
                        else:
                            use_ballistic = False
                            is_actively_tracking_velocity = True
                    else:
                        if radial_error_mrad > BALLISTIC_ABORT_MRAD:
                            use_ballistic = True
                            is_actively_tracking_velocity = False
                        else:
                            use_ballistic = False

                    time_since_tx = now - last_setpoint_time

                    if use_ballistic:
                        goal_delta = math.hypot(world_pan - last_sent_p_pan, world_tilt - last_sent_p_tilt)
                        if (time_since_tx >= 0.400 and goal_delta > 60.0) or last_v_pan != 0:
                            last_setpoint_time = now
                            last_sent_p_pan = world_pan
                            last_sent_p_tilt = world_tilt
                            turret_queue.put({
                                "cmd": "SETPOINT",
                                "pan": world_pan,
                                "tilt": world_tilt
                            })
                            last_v_pan = 0
                            last_v_tilt = 0
                        cmd_type = f"P({world_pan:+4d},{world_tilt:+4d})"
                    else:
                        if time_since_tx >= SETPOINT_MIN_INTERVAL_SEC:
                            last_setpoint_time = now

                            def compute_velocity(err, tgt_v, deadband):
                                mag = abs(err)
                                if mag < deadband:
                                    return 0
                                if mag <= 25.0:
                                    kp = 1.4
                                    cmd = err * kp + 0.8 * tgt_v
                                else:
                                    kp = 2.8
                                    cmd = err * kp + 1.0 * tgt_v
                                return int(round(cmd))

                            raw_v_pan = compute_velocity(realtime_err_pan, tgt_v_pan, VELOCITY_DEADBAND_MRAD)
                            raw_v_tilt = compute_velocity(realtime_err_tilt, tgt_v_tilt, VELOCITY_DEADBAND_MRAD)

                            if time_since_seen > 0.350:
                                decay_factor = math.exp(-COAST_EXP_DECAY_RATE * (time_since_seen - 0.350))
                                raw_v_pan = int(round(raw_v_pan * decay_factor))
                                raw_v_tilt = int(round(raw_v_tilt * decay_factor))

                            v_pan_cmd = max(-850, min(850, raw_v_pan))
                            v_tilt_cmd = max(-450, min(450, raw_v_tilt))

                            if current_tilt >= TILT_MAX_MRAD and v_tilt_cmd > 0:
                                v_tilt_cmd = 0
                            elif current_tilt <= TILT_MIN_MRAD and v_tilt_cmd < 0:
                                v_tilt_cmd = 0

                            last_v_pan = v_pan_cmd
                            last_v_tilt = v_tilt_cmd

                            turret_queue.put({
                                "cmd": "VELOCITY",
                                "pan_s": v_pan_cmd,
                                "tilt_s": v_tilt_cmd
                            })

                        cmd_type = f"V({last_v_pan:+4d},{last_v_tilt:+4d})"

                    if now - last_debug_log_time >= 0.150:
                        last_debug_log_time = now
                        log.info(
                            f"[{mode}] Cur:({current_pan:+5d},{current_tilt:+4d}) | "
                            f"Err:({realtime_err_pan:+5.1f},{realtime_err_tilt:+4.1f}) | "
                            f"Cmd:{cmd_type} | Mv:{int(hw_moving)}"
                        )
            else:
                if target_data is not None:
                    clear_primary_target()
                    last_fire_gate_ok_time = 0.0
                    turret_queue.put({"cmd": "TRIGGER", "state": 0})
                    if is_actively_tracking_velocity:
                        turret_queue.put({"cmd": "VELOCITY", "pan_s": 0, "tilt_s": 0})
                        is_actively_tracking_velocity = False
                    last_v_pan = 0
                    last_v_tilt = 0
                    last_sent_p_pan = 99999
                    last_sent_p_tilt = 99999
                    target_filter.reset()
                    log.info(f"[{mode}] Target horizon expired (>1.5s). Standing by.")

                idle_duration = now - last_target_time
                if idle_duration >= HOME_TIMEOUT_SEC:
                    off_home = (abs(current_pan) > 2 or abs(current_tilt) > 2)
                    if not is_homing and off_home:
                        log.info(
                            f"[{mode}] Idle for {idle_duration:.1f}s at "
                            f"({current_pan:+d},{current_tilt:+d}). Returning turret to scan home (0,0)."
                        )
                        is_homing = True
                        turret_queue.put({
                            "cmd": "SETPOINT",
                            "pan": 0,
                            "tilt": 0
                        })
                        last_setpoint_time = now
                    elif is_homing:
                        if not off_home:
                            is_homing = False
                            log.info(f"[{mode}] Returned to scan home (0,0).")
                        elif (not hw_moving) and (now - last_setpoint_time >= 2.0):
                            log.info(
                                f"[{mode}] Re-issuing home setpoint (0,0) from "
                                f"({current_pan:+d},{current_tilt:+d})."
                            )
                            turret_queue.put({
                                "cmd": "SETPOINT",
                                "pan": 0,
                                "tilt": 0
                            })
                            last_setpoint_time = now

        elapsed = time.monotonic() - loop_start
        sleep_time = MOTION_LOOP_INTERVAL_SEC - elapsed
        if sleep_time > 0.001:
            time.sleep(sleep_time)

# =========================================================================
# Vision Perception Thread (Asynchronous Ingestion & Polymorphic Dispatch)
# =========================================================================
def vision_thread():
    global latest_jpeg, global_turret_queue, global_target_filter, global_wildlife_detector

    # Pin VisionMain and OpenCV's 2 worker threads strictly to Cores 0 & 1
    set_os_thread_name("VisionMain", cpu_cores=[0, 1])

    ensure_storage_headroom()

    detectors = {
        "ARUCO": ArucoTargetDetector(),
        "WILDLIFE": HailoWildlifeDetector(),
    }
    global_wildlife_detector = detectors["WILDLIFE"]
    detectors["WILDLIFE"].set_active(get_active_detector_type() == "WILDLIFE")

    target_filter = TargetKalmanFilter(process_noise=35.0, measurement_noise=10.0)
    global_target_filter = target_filter

    try:
        # Prioritize wide_noir tuning profile first to match physical IMX708 wide lens
        noir_candidates = [
            "/usr/share/libcamera/ipa/rpi/pisp/imx708_wide_noir.json",
            "/usr/share/libcamera/ipa/rpi/vc4/imx708_wide_noir.json",
            "/usr/share/libcamera/ipa/rpi/pisp/imx708_noir.json",
            "/usr/share/libcamera/ipa/rpi/vc4/imx708_noir.json"
        ]
        active_tuning = next((f for f in noir_candidates if os.path.exists(f)), None)

        picam2 = Picamera2()
        if active_tuning:
            try:
                picam2.load_tuning_file(active_tuning)
                log.info(f"[CAMERA] Loaded IMX708 NoIR ISP tuning profile: {active_tuning}")
            except Exception as te:
                log.warning(f"[CAMERA] Failed to apply tuning file {active_tuning}: {te}; proceeding with default ISP.")
        else:
            log.warning("[CAMERA] Specific NoIR tuning JSON not found in standard paths; proceeding with default ISP.")

        cam_config = picam2.create_video_configuration(
            sensor={"output_size": (2304, 1296)},
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
            lores={"size": (DISPLAY_WIDTH, DISPLAY_HEIGHT), "format": "RGB888"},
            buffer_count=6,
            controls={
                "AeEnable": True,
                "HdrMode": 2,                      # Start with HDR active for daytime dynamic range
                "AeMeteringMode": 0,               # Centre-weighted metering
                "AwbMode": 0,
                "NoiseReductionMode": 2,           # High-Quality PiSP hardware denoise
                "ExposureValue": 0.0,
                "FrameDurationLimits": (33333, 83333),  # 30 FPS day ceiling -> 12 FPS deep-dusk floor
                "Contrast": 1.05,
                "Saturation": 0.85,
                "Sharpness": 1.15,
                "AfMode": 0,
                "LensPosition": 0.0
            },
            transform=Transform(hflip=True, vflip=True)
        )
        picam2.configure(cam_config)
        picam2.start()
        log.info("[CAMERA] Hardware PiSP [B,G,R] stream active with dynamic exposure range")
        time.sleep(0.5)
    except Exception as e:
        log.critical(f"[CAMERA FATAL] Could not initialize Picamera2: {e}\n{traceback.format_exc()}")
        set_alert("FATAL: Camera init failed", duration=10.0)
        return

    turret_queue = queue.Queue(maxsize=25)
    global_turret_queue = turret_queue
    record_queue = queue.Queue(maxsize=90)

    threading.Thread(target=turret_serial_worker, args=(turret_queue,), daemon=True, name="SerialWorker").start()
    threading.Thread(target=video_recorder_worker, args=(record_queue,), daemon=True, name="RecorderWorker").start()
    threading.Thread(target=motion_servoing_worker, args=(target_filter, turret_queue), daemon=True, name="MotionWorker").start()

    fps_time = time.time()
    frame_count = 0
    sensor_fps = 0.0

    last_valid_range_m = DEFAULT_FALLBACK_RANGE_M
    last_seen_connected = False
    last_detector_type = get_active_detector_type()

    tentative_target = None
    tentative_streak = 0
    last_processed_capture_mono = 0.0
    last_veto_audit_time = 0.0

    is_recording = False
    lock_consecutive_frames = 0
    last_detection_time = 0.0
    recording_start_time = 0.0
    last_standby_log_time = 0.0

    isp_eval_counter = 0
    is_monochrome_mode = False
    last_ev = 0.0

    PRE_ROLL_CAPACITY = int(RECORD_PRE_ROLL_SEC * TARGET_FPS)
    pre_roll_pool = np.empty((PRE_ROLL_CAPACITY, FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    pre_roll_idx = 0
    pre_roll_count = 0

    while True:
        now = time.time()
        capture_arrival_mono = time.monotonic()

        # --- Stage 1: Camera Acquisition ---
        try:
            request_frame = picam2.capture_request()
            frame_main = request_frame.make_array("main")
            annotated = request_frame.make_array("lores")
            metadata = request_frame.get_metadata()
            request_frame.release()
        except Exception as e:
            log.error(f"[CAMERA ERROR] Capture dropped: {e}")
            time.sleep(0.01)
            continue

        _, current_tilt_mrad, _, _ = get_instantaneous_telemetry()

        # Never apply negative EV when tilted down at dark ground; boost slightly in B&W twilight
        if current_tilt_mrad > 60:
            dynamic_ev = 1.0
        elif is_monochrome_mode:
            dynamic_ev = 0.25
        else:
            dynamic_ev = 0.0

        if last_ev != dynamic_ev:
            try:
                picam2.set_controls({"ExposureValue": dynamic_ev})
                last_ev = dynamic_ev
            except Exception:
                pass

        frame_count += 1
        isp_eval_counter += 1

        if isp_eval_counter >= 30:
            isp_eval_counter = 0
            lux = metadata.get("Lux", 50.0)
            gain = metadata.get("AnalogueGain", 1.0)
            exp_us = metadata.get("ExposureTime", 33333)
            has_active_tgt = (get_primary_target_snapshot() is not None)

            # Transition to # Transition to Dawn/Dusk B&W Low-Light Profile when light drops
            if (lux < 25.0 or gain > 6.0) and not is_monochrome_mode and not has_active_tgt:
                try:
                    picam2.set_controls({
                        "HdrMode": 0,          # Disable multi-exposure HDR split in low light
                        "Saturation": 0.0,     # Collapse muddy NoIR color cast into clean monochrome
                        "Contrast": 1.15,      # Pop deer silhouettes against twilight grass
                        "Sharpness": 0.90,     # Soften high-gain sensor grain
                        "ExposureValue": 0.25
                    })
                    is_monochrome_mode = True
                    last_ev = 0.25
                    log.info(
                        f"[ISP] Dawn/Dusk Low-Light Mode ACTIVE (HDR:OFF, B&W:ON | "
                        f"Lux:{lux:.1f}, Gain:{gain:.1f}x, Shutter:{exp_us/1000.0:.1f}ms)"
                    )
                except Exception as e:
                    log.warning(f"[ISP] Low-light control update warning: {e}")

            # Hysteresis return to Daylight HDR Color Mode (Relaxed gain ceiling from 3.5x to 6.5x)
            elif (lux >= 40.0 and gain <= 6.5) and is_monochrome_mode and not has_active_tgt:
                try:
                    picam2.set_controls({
                        "HdrMode": 2,          # Re-enable SingleExposure HDR for bright daylight
                        "Saturation": 0.85,
                        "Contrast": 1.05,
                        "Sharpness": 1.15,
                        "ExposureValue": 0.0
                    })
                    is_monochrome_mode = False
                    last_ev = 0.0
                    log.info(
                        f"[ISP] Daylight HDR Mode RESTORED (HDR:ON, Color:ON | "
                        f"Lux:{lux:.1f}, Gain:{gain:.1f}x, Shutter:{exp_us/1000.0:.1f}ms)"
                    )
                except Exception as e:
                    log.warning(f"[ISP] Daylight control update warning: {e}")

        if now - fps_time >= 1.0:
            sensor_fps = frame_count / (now - fps_time)
            frame_count = 0
            fps_time = now

        currently_connected = get_serial_connected()
        if currently_connected != last_seen_connected:
            last_seen_connected = currently_connected
            target_filter.reset()
            if not currently_connected:
                clear_primary_target()

        # --- Stage 2: Polymorphic Target Detection ---
        requested_det_type = get_active_detector_type()
        active_detector = detectors.get(requested_det_type, detectors["ARUCO"])
        if not active_detector.is_available:
            active_detector = detectors["ARUCO"]
            set_active_detector_type("ARUCO")

        if active_detector.name != last_detector_type:
            log.info(f"[PERCEPTION] Switching active detector: {last_detector_type} -> {active_detector.name}")
            detectors["WILDLIFE"].set_active(active_detector.name == "WILDLIFE")
            last_detector_type = active_detector.name
            tentative_target = None
            tentative_streak = 0
            target_filter.reset()
            clear_primary_target()

        detections, effective_capture_mono = active_detector.detect(frame_main, capture_arrival_mono, is_monochrome=is_monochrome_mode)
        debug_info = active_detector.get_last_debug_info()
        is_new_inference_batch = (effective_capture_mono != last_processed_capture_mono)

        best_candidate = detections[0] if len(detections) > 0 else None
        verified_detection = None

        if is_new_inference_batch:
            last_processed_capture_mono = effective_capture_mono

            if active_detector.name == "WILDLIFE":
                veto_boxes = debug_info.get("veto_boxes", [])
                rejected_animals = debug_info.get("rejected_animals", [])

                # Immediate safety abort if a person/vehicle overlaps the currently locked target
                cur_primary = get_primary_target_snapshot()
                if cur_primary is not None and veto_boxes:
                    hit, veto_obj = MDV6Decoder1280._check_veto_overlap(cur_primary["box"], veto_boxes)
                    if hit:
                        log.warning(
                            f"[SAFETY VETO:ACTIVE] Aborting lock — {veto_obj['class']} "
                            f"({veto_obj['score']:.2f}) overlapped active target box {cur_primary['box']}"
                        )
                        clear_primary_target()
                        target_filter.reset()
                        tentative_target = None
                        tentative_streak = 0
                        best_candidate = None
                        try:
                            turret_queue.put_nowait({"cmd": "TRIGGER", "state": 0})
                        except Exception:
                            pass
                        if now - last_veto_audit_time >= AUDIT_VETO_COOLDOWN_SEC:
                            last_veto_audit_time = now
                            save_audit_snapshot(frame_main, "VETO_ACTIVE", [], debug_info)

                # Post-mortem audit logging & throttled snapshot for rejected animal candidates
                if rejected_animals and (now - last_veto_audit_time >= AUDIT_VETO_COOLDOWN_SEC):
                    last_veto_audit_time = now
                    rej_summary = "; ".join(
                        f"{r['reason']} A:{r['probs'][0]:.2f}/P:{r['probs'][1]:.2f}/V:{r['probs'][2]:.2f} box={r['box']}"
                        for r in rejected_animals[:3]
                    )
                    log.info(f"[AUDIT:REJECTED] Blocked {len(rejected_animals)} candidate(s): {rej_summary}")
                    save_audit_snapshot(frame_main, "VETO", detections, debug_info)

            if best_candidate is not None:
                has_active = (get_primary_target_snapshot() is not None)
                if has_active:
                    verified_detection = best_candidate
                else:
                    if tentative_target is not None:
                        prev_cx, prev_cy = tentative_target["center"]
                        cur_cx, cur_cy = best_candidate["center"]
                        if math.hypot(cur_cx - prev_cx, cur_cy - prev_cy) <= CONFIRMATION_MAX_JUMP_PX:
                            tentative_streak += 1
                        else:
                            tentative_streak = 1
                        tentative_target = best_candidate
                        if tentative_streak >= CONFIRMATION_FRAMES:
                            verified_detection = best_candidate
                            if active_detector.name == "WILDLIFE":
                                pa, pp, pv = best_candidate.get("probs", (best_candidate["score"], 0.0, 0.0))
                                log.warning(
                                    f"[ACQUIRE:WILDLIFE] Locked after {tentative_streak} frames | "
                                    f"Probs(A:{pa:.2f} P:{pp:.2f} V:{pv:.2f}) | "
                                    f"Box:{best_candidate['box']} | Range:{best_candidate['range_m']:.1f}m | "
                                    f"VetoZones:{len(debug_info.get('veto_boxes', []))}"
                                )
                                save_audit_snapshot(frame_main, "ACQUIRE", detections, debug_info)
                    else:
                        tentative_target = best_candidate
                        tentative_streak = 1
            else:
                tentative_target = None
                tentative_streak = 0

        # --- Stage 3: Kalman Measurement Correction ---
        if verified_detection is not None:
            raw_cx, raw_cy = verified_detection["center"]
            pan_at_capture, tilt_at_capture = get_turret_pos_at_time(effective_capture_mono - 0.040)

            if "range_m" in verified_detection:
                last_valid_range_m = verified_detection["range_m"]

            current_range_m = last_valid_range_m

            parallax_pan_mrad = (-CAMERA_OFFSET_X_MM / (current_range_m * 1000.0)) * 1000.0
            parallax_tilt_mrad = (CAMERA_OFFSET_Y_MM / (current_range_m * 1000.0)) * 1000.0

            dx = -(raw_cx - OPTICAL_CENTER[0])
            dy = -(raw_cy - OPTICAL_CENTER[1])
            opt_err_pan_mrad = math.atan2(dx, FOCAL_LENGTH_PX) * 1000.0 + parallax_pan_mrad
            opt_err_tilt_mrad = math.atan2(dy, FOCAL_LENGTH_PX) * 1000.0 + parallax_tilt_mrad

            raw_world_pan = pan_at_capture + int(round(opt_err_pan_mrad))
            raw_world_tilt = tilt_at_capture + int(round(opt_err_tilt_mrad))

            target_filter.update_measurement(raw_world_pan, raw_world_tilt, now)

            set_primary_target({
                "center": verified_detection["center"],
                "box": verified_detection["box"],
                "score": verified_detection["score"],
                "area": verified_detection["area"],
                "range_m": current_range_m,
                "last_seen": now,
                "id": verified_detection.get("id", 0),
                "label": verified_detection.get("label", "TGT")
            })
        else:
            if get_primary_target_snapshot() is None and (now - last_standby_log_time >= 2.0):
                last_standby_log_time = now
                p_cur, t_cur, _, _ = get_instantaneous_telemetry()
                cur_lux = metadata.get("Lux", 0.0)
                cur_gain = metadata.get("AnalogueGain", 1.0)
                cur_shutter_ms = metadata.get("ExposureTime", 0) / 1000.0
                isp_tag = "BW" if is_monochrome_mode else "HDR"

                if active_detector.name == "WILDLIFE":
                    tel = active_detector.get_telemetry()
                    npu_info = f" | NPU:{tel.get('npu_fps', 0.0):4.1f}Hz({tel.get('npu_latency_ms', 0.0):3.0f}ms)"
                else:
                    npu_info = ""

                log.info(
                    f"[STANDBY:{active_detector.name}] "
                    f"Cam:{sensor_fps:4.1f}FPS({isp_tag}) | "
                    f"Lux:{cur_lux:5.1f} Gain:{cur_gain:4.1f}x Exp:{cur_shutter_ms:4.1f}ms"
                    f"{npu_info} | Turret:({p_cur:+5d},{t_cur:+4d})"
                )

        # --- Stage 4: Video Archiving ---
        local_target = get_primary_target_snapshot()
        target_locked = (local_target is not None and (now - local_target["last_seen"] <= 0.350))
        if target_locked:
            lock_consecutive_frames += 1
            last_detection_time = now
        else:
            lock_consecutive_frames = 0

        if not is_recording:
            np.copyto(pre_roll_pool[pre_roll_idx], frame_main)
            pre_roll_idx = (pre_roll_idx + 1) % PRE_ROLL_CAPACITY
            if pre_roll_count < PRE_ROLL_CAPACITY:
                pre_roll_count += 1

            if target_locked and lock_consecutive_frames >= MIN_LOCK_FRAMES_TO_RECORD:
                is_recording = True
                recording_start_time = now

                # Match recording header FPS and pre-roll frame count to live sensor cadence
                live_fps = float(np.clip(sensor_fps if sensor_fps >= 5.0 else TARGET_FPS, 10.0, 30.0))
                wanted_pre_frames = min(pre_roll_count, int(round(RECORD_PRE_ROLL_SEC * live_fps)))

                if pre_roll_count < PRE_ROLL_CAPACITY:
                    start_i = max(0, pre_roll_count - wanted_pre_frames)
                    saved_pre_roll = [pre_roll_pool[i].copy() for i in range(start_i, pre_roll_count)]
                else:
                    start_offset = PRE_ROLL_CAPACITY - wanted_pre_frames
                    saved_pre_roll = [
                        pre_roll_pool[(pre_roll_idx + start_offset + i) % PRE_ROLL_CAPACITY].copy()
                        for i in range(wanted_pre_frames)
                    ]
                try:
                    record_queue.put_nowait({
                        "cmd": "start",
                        "fps": round(live_fps, 1),
                        "pre_roll": saved_pre_roll
                    })
                except queue.Full:
                    pass

                pre_roll_idx = 0
                pre_roll_count = 0
        else:
            time_since_last_seen = now - last_detection_time
            clip_duration = now - recording_start_time

            if time_since_last_seen > RECORD_POST_ROLL_SEC or clip_duration > MAX_RECORDING_DURATION_SEC:
                is_recording = False
                pre_roll_idx = 0
                pre_roll_count = 0
                try:
                    record_queue.put_nowait({"cmd": "stop"})
                except queue.Full:
                    pass
            else:
                try:
                    record_queue.put_nowait({"cmd": "frame", "frame": frame_main})
                except queue.Full:
                    pass

        # --- Stage 5: Client-Aware HUD Render & Web Stream ---
        with frame_lock:
            clients_viewing = (active_web_clients > 0)

        if clients_viewing:
            cur_mode, is_primed_hud = get_turret_mode()
            current_range_m = local_target.get("range_m", DEFAULT_FALLBACK_RANGE_M) if local_target else DEFAULT_FALLBACK_RANGE_M
            oc_x = int(OPTICAL_CENTER[0] * SCALE_X)
            oc_y = int(OPTICAL_CENTER[1] * SCALE_Y)
            cv2.drawMarker(annotated, (oc_x, oc_y), (0, 255, 255), cv2.MARKER_CROSS, 16, 1)

            tof_hud = (math.exp(0.0128 * current_range_m) - 1.0) / (0.0128 * 85.0) if current_range_m > 0 else 0.0
            drop_mrad_hud = (0.5 * 9.81 * (tof_hud ** 2) / current_range_m * 1000.0) if current_range_m > 0 else 0.0
            total_tilt_adj_mrad = drop_mrad_hud + TILT_CALIBRATION_TRIM_MRAD

            conv_px_x = OPTICAL_CENTER[0] + int((-CAMERA_OFFSET_X_MM / (current_range_m * 1000.0)) * FOCAL_LENGTH_PX)
            conv_px_y = OPTICAL_CENTER[1] + int(((CAMERA_OFFSET_Y_MM / (current_range_m * 1000.0)) * 1000.0 - total_tilt_adj_mrad) * (FOCAL_LENGTH_PX / 1000.0))
            aim_x = int(conv_px_x * SCALE_X)
            aim_y = int(conv_px_y * SCALE_Y)

            active_deadband = 15.0 if cur_mode == "CALIBRATE" else FIRE_DEADBAND_MRAD
            db_radius = int((active_deadband / MRAD_PER_PIXEL_X) * SCALE_X)

            p_mrad, t_mrad, _, firing = get_instantaneous_telemetry()
            with target_lock:
                ser_ok = system_health["serial_connected"]
                rec_ok = system_health["recorder_active"]
                npu_fps_hud = system_health["npu_fps"]
                npu_lat_hud = system_health["npu_latency_ms"]
                alert_msg = system_health["last_alert"] if now < system_health["alert_time"] else None

            if cur_mode == "ARMED":
                ring_color = (0, 0, 255)
            elif cur_mode == "CALIBRATE":
                ring_color = (0, 165, 255) if is_primed_hud else (100, 100, 100)
            elif cur_mode == "ON":
                ring_color = (255, 200, 0)
            else:
                ring_color = (0, 200, 50)

            if firing:
                ring_color = (0, 255, 255)

            cv2.circle(annotated, (aim_x, aim_y), db_radius, ring_color, 1)
            cv2.drawMarker(annotated, (aim_x, aim_y), ring_color, cv2.MARKER_CROSS, 24, 1)

            for v in debug_info.get("veto_boxes", []):
                vx1 = int(v["box"][0] * SCALE_X)
                vy1 = int(v["box"][1] * SCALE_Y)
                vx2 = int(v["box"][2] * SCALE_X)
                vy2 = int(v["box"][3] * SCALE_Y)
                cv2.rectangle(annotated, (vx1, vy1), (vx2, vy2), (0, 0, 255), 1)
                cv2.putText(
                    annotated,
                    f"VETO:{v['class']} {v['score']*100:.0f}%",
                    (vx1, max(vy1 - 4, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.35,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA,
                )

            for det in detections:
                x1 = int(det["box"][0] * SCALE_X)
                y1 = int(det["box"][1] * SCALE_Y)
                x2 = int(det["box"][2] * SCALE_X)
                y2 = int(det["box"][3] * SCALE_Y)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (180, 180, 180), 1)

            if local_target and (now - local_target["last_seen"] < COAST_MAX_HORIZON_SEC):
                tx1 = int(local_target["box"][0] * SCALE_X)
                ty1 = int(local_target["box"][1] * SCALE_Y)
                tx2 = int(local_target["box"][2] * SCALE_X)
                ty2 = int(local_target["box"][3] * SCALE_Y)
                tcx_d = int(local_target["center"][0] * SCALE_X)
                tcy_d = int(local_target["center"][1] * SCALE_Y)

                box_color = (0, 255, 0) if (now - local_target["last_seen"] <= 0.150) else (0, 165, 255)
                cv2.rectangle(annotated, (tx1, ty1), (tx2, ty2), box_color, 2)
                cv2.circle(annotated, (tcx_d, tcy_d), 4, box_color, -1)

                tgt_lbl = local_target.get("label", f"ID:{local_target.get('id', 0)}")
                status_lbl = f"{tgt_lbl} | {current_range_m:.1f}m"
                cv2.putText(annotated, status_lbl, (tx1, max(ty1 - 8, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_color, 1, cv2.LINE_AA)

            # High-Contrast Mode Watermark
            if cur_mode == "ARMED":
                cv2.rectangle(annotated, (10, 10), (150, 34), (0, 0, 180), -1)
                cv2.circle(annotated, (22, 22), 4, (255, 255, 255), -1)
                cv2.putText(annotated, "ARMED / LIVE", (32, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
            elif cur_mode == "CALIBRATE":
                cal_text = "TEST / PRIMED" if is_primed_hud else "TEST / LOCKED"
                bg_cal = (0, 100, 200) if is_primed_hud else (60, 60, 60)
                cv2.rectangle(annotated, (10, 10), (165, 34), bg_cal, -1)
                cv2.circle(annotated, (22, 22), 4, (255, 255, 255), -1)
                cv2.putText(annotated, cal_text, (32, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            elif cur_mode == "ON":
                cv2.rectangle(annotated, (10, 10), (165, 34), (180, 110, 0), -1)
                cv2.circle(annotated, (22, 22), 4, (255, 255, 255), -1)
                cv2.putText(annotated, "TRACKING / SAFE", (32, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            else:
                cv2.rectangle(annotated, (10, 10), (130, 34), (0, 140, 40), -1)
                cv2.circle(annotated, (22, 22), 4, (255, 255, 255), -1)
                cv2.putText(annotated, "OFF / SAFE", (32, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

            # Persistent Dedicated Target Range Card
            if local_target and (now - local_target["last_seen"] < COAST_MAX_HORIZON_SEC):
                tgt_rng_m = local_target.get("range_m", 0.0)
                tgt_rng_ft = tgt_rng_m * 3.28084
                is_fresh = (now - local_target["last_seen"] <= 0.150)
                hud_card_border = (0, 200, 0) if is_fresh else (0, 140, 255)

                cv2.rectangle(annotated, (DISPLAY_WIDTH - 215, 10), (DISPLAY_WIDTH - 10, 42), (18, 18, 18), -1)
                cv2.rectangle(annotated, (DISPLAY_WIDTH - 215, 10), (DISPLAY_WIDTH - 10, 42), hud_card_border, 1)

                range_str = f"TGT: {tgt_rng_ft:4.1f}ft ({tgt_rng_m:3.1f}m)"
                cv2.putText(annotated, range_str, (DISPLAY_WIDTH - 208, 31),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
            else:
                cv2.rectangle(annotated, (DISPLAY_WIDTH - 140, 10), (DISPLAY_WIDTH - 10, 36), (18, 18, 18), -1)
                cv2.rectangle(annotated, (DISPLAY_WIDTH - 140, 10), (DISPLAY_WIDTH - 10, 36), (60, 60, 60), 1)
                cv2.putText(annotated, "TGT: NONE", (DISPLAY_WIDTH - 130, 27),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 150), 1, cv2.LINE_AA)

            if firing:
                cv2.putText(annotated, "SOLENOID ENGAGED", (aim_x - 75, aim_y - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

            if rec_ok:
                cv2.circle(annotated, (DISPLAY_WIDTH - 25, 60), 8, (0, 0, 255), -1)
                cv2.putText(annotated, "REC", (DISPLAY_WIDTH - 70, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

            badge_color = (0, 255, 0) if ser_ok else (0, 0, 255)
            ser_badge = "SERIAL: OK" if ser_ok else "SERIAL: DISCONNECTED"
            det_telemetry = (
                f"NPU:{npu_fps_hud:.1f}Hz({npu_lat_hud:.0f}ms)"
                if active_detector.name == "WILDLIFE"
                else "DET:ARUCO"
            )
            status_text = f"FPS:{sensor_fps:.1f} | {det_telemetry} | Pan:{p_mrad} Tilt:{t_mrad} | {ser_badge}"
            cv2.putText(annotated, status_text, (10, DISPLAY_HEIGHT - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, badge_color, 1, cv2.LINE_AA)

            if alert_msg:
                cv2.rectangle(annotated, (0, DISPLAY_HEIGHT - 45), (DISPLAY_WIDTH, DISPLAY_HEIGHT - 20), (0, 0, 180), -1)
                cv2.putText(annotated, alert_msg, (12, DISPLAY_HEIGHT - 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

            success, buffer = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if success:
                with frame_lock:
                    latest_jpeg = buffer.tobytes()

# =========================================================================
# Web API & Interface
# =========================================================================

@app.before_request
def require_web_auth():
    """Enforces HTTP Basic Auth across all Web UI, MJPEG stream, and control API routes."""
    if not WEB_AUTH_USER and not WEB_AUTH_PASS:
        return None
    auth = request.authorization
    if (
        auth is None
        or not hmac.compare_digest(auth.username or "", WEB_AUTH_USER)
        or not hmac.compare_digest(auth.password or "", WEB_AUTH_PASS)
    ):
        return Response(
            "Authentication required.",
            401,
            {"WWW-Authenticate": 'Basic realm="Bombardeer Console"'},
        )
    return None

@app.route("/api/mode", methods=["GET", "POST"])
def api_mode():
    global global_turret_queue, global_target_filter
    hailo_ok = is_hailo_npu_ready()

    if request.method == "POST":
        req = request.get_json(silent=True) or {}
        target_mode = req.get("mode", "").upper()
        target_detector = req.get("detector", "").upper()
        reprime_req = req.get("reprime", False)

        cur_mode, cur_primed = get_turret_mode()

        # Handle Runtime Perception Model Selection
        if target_detector:
            if target_detector not in ("ARUCO", "WILDLIFE"):
                return jsonify({"status": "error", "message": "Invalid detector specified"}), 400

            if target_detector == "WILDLIFE" and not hailo_ok:
                set_alert("HAILO NPU UNAVAILABLE", duration=3.0)
                return jsonify({
                    "status": "error",
                    "message": "Hailo-8 NPU or MDV6 HEF model is unavailable"
                }), 400

            set_active_detector_type(target_detector)
            clear_primary_target()
            if global_target_filter is not None:
                global_target_filter.reset()
            set_alert(f"DETECTOR: {target_detector} ACTIVE", duration=2.5)
            log.info(f"[API] Runtime detector switched to {target_detector}")

            c_mode, c_primed = get_turret_mode()
            return jsonify({
                "status": "ok",
                "mode": c_mode,
                "primed": c_primed,
                "detector": get_active_detector_type(),
                "hailo_available": hailo_ok
            })

        if reprime_req:
            if cur_mode == "CALIBRATE":
                set_turret_mode("CALIBRATE", primed=True)
                set_alert("TEST FIRE: RE-PRIMED (READY)", duration=2.5)
                log.info("[CALIBRATE] Single-shot test fire manually RE-PRIMED.")
            c_mode, c_primed = get_turret_mode()
            return jsonify({
                "status": "ok",
                "mode": c_mode,
                "primed": c_primed,
                "detector": get_active_detector_type(),
                "hailo_available": hailo_ok
            })

        if target_mode not in ("OFF", "ON", "ARMED", "CALIBRATE"):
            return jsonify({"status": "error", "message": "Invalid mode specified"}), 400

        if target_mode in ("ARMED", "CALIBRATE") and cur_mode != "ON":
            return jsonify({"status": "error", "message": "Turret must be turned ON before arming or testing"}), 400

        set_turret_mode(target_mode)

        if target_mode == "OFF":
            if global_turret_queue is not None:
                try:
                    global_turret_queue.put_nowait({"cmd": "HALT"})
                    global_turret_queue.put_nowait({"cmd": "TRIGGER", "state": 0})
                except Exception:
                    pass
            set_alert("TURRET DISENGAGED (OFF)", duration=2.5)
        elif target_mode == "ON":
            if global_turret_queue is not None:
                try:
                    global_turret_queue.put_nowait({"cmd": "TRIGGER", "state": 0})
                except Exception:
                    pass
            set_alert("TRACKING ACTIVE (SAFE)", duration=2.5)
        elif target_mode == "CALIBRATE":
            set_alert("CALIBRATION MODE: PRIMED (1-SHOT)", duration=3.0)
        elif target_mode == "ARMED":
            set_alert("WARNING: TURRET ARMED / LIVE FIRE", duration=4.0)

        log.warning(f"[MODE TRANSITION] Turret switched to {target_mode}")
        c_mode, c_primed = get_turret_mode()
        return jsonify({
            "status": "ok",
            "mode": c_mode,
            "primed": c_primed,
            "detector": get_active_detector_type(),
            "hailo_available": hailo_ok
        })

    c_mode, c_primed = get_turret_mode()
    return jsonify({
        "status": "ok",
        "mode": c_mode,
        "primed": c_primed,
        "detector": get_active_detector_type(),
        "hailo_available": hailo_ok
    })

@app.route("/")
def index():
    return render_template_string("""
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
        <meta name="apple-mobile-web-app-capable" content="yes">
        <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
        <title>Bombardeer Console</title>
        <link rel="icon" type="image/x-icon" href="/favicon.ico">
        <style>
            * {
                box-sizing: border-box;
                margin: 0;
                padding: 0;
            }
            body {
                background-color: #0d1117;
                color: #c9d1d9;
                font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
                display: flex;
                flex-direction: column;
                align-items: center;
                justify-content: center;
                min-height: 100vh;
                padding: 12px;
                -webkit-touch-callout: none;
            }
            .hud-card {
                background: #161b22;
                border: 1px solid #30363d;
                border-radius: 12px;
                padding: 14px;
                box-shadow: 0 8px 24px rgba(0, 0, 0, 0.6);
                text-align: center;
                width: 100%;
                max-width: 680px;
                display: flex;
                flex-direction: column;
                align-items: center;
            }
            .header-bar {
                display: flex;
                justify-content: space-between;
                align-items: center;
                width: 100%;
                margin-bottom: 10px;
                flex-wrap: wrap;
                gap: 8px;
            }
            .brand-wrap {
                display: flex;
                align-items: center;
                gap: 8px;
            }
            h1 {
                font-size: 1.05rem;
                letter-spacing: 0.05em;
                text-transform: uppercase;
                color: #39d353;
                text-align: left;
            }
            .status-pill {
                font-size: 0.70rem;
                font-weight: 800;
                letter-spacing: 0.08em;
                padding: 3px 8px;
                border-radius: 12px;
                text-transform: uppercase;
                transition: all 0.2s ease;
            }
            .pill-off {
                background: #23863622;
                color: #3fb950;
                border: 1px solid #238636;
            }
            .pill-on {
                background: #1f6feb22;
                color: #58a6ff;
                border: 1px solid #1f6feb;
            }
            .pill-cal {
                background: #d2992222;
                color: #f0883e;
                border: 1px solid #d29922;
            }
            .pill-armed {
                background: #da363333;
                color: #f85149;
                border: 1px solid #da3633;
                box-shadow: 0 0 10px rgba(218, 54, 51, 0.4);
                animation: pulse-red 1.2s infinite;
            }
            @keyframes pulse-red {
                0% { opacity: 0.85; }
                50% { opacity: 1; }
                100% { opacity: 0.85; }
            }

            /* Runtime Perception Model Selector Bar */
            .detector-bar {
                display: flex;
                align-items: center;
                justify-content: space-between;
                width: 100%;
                background: #0d1117;
                border: 1px solid #30363d;
                border-radius: 8px;
                padding: 6px 10px;
                margin-bottom: 10px;
                gap: 8px;
            }
            .detector-label {
                font-size: 0.75rem;
                font-weight: 700;
                text-transform: uppercase;
                letter-spacing: 0.05em;
                color: #8b949e;
            }
            .detector-toggle-group {
                display: flex;
                gap: 6px;
            }
            .btn-det {
                background: #21262d;
                color: #c9d1d9;
                border: 1px solid #30363d;
                border-radius: 6px;
                padding: 5px 12px;
                font-size: 0.76rem;
                font-weight: 700;
                cursor: pointer;
                transition: all 0.15s ease;
            }
            .btn-det.active {
                background: #8957e5;
                color: #ffffff;
                border-color: #a371f7;
                box-shadow: 0 0 10px rgba(137, 87, 229, 0.4);
            }
            .btn-det.disabled {
                opacity: 0.4;
                cursor: not-allowed;
            }

            .video-container {
                position: relative;
                width: 100%;
                background: #000;
                border-radius: 8px;
                overflow: hidden;
                display: flex;
                align-items: center;
                justify-content: center;
                aspect-ratio: 16 / 9;
                border: 1px solid #30363d;
            }
            .video-container img {
                width: 100%;
                height: 100%;
                object-fit: contain;
                display: block;
            }
            .video-container:fullscreen,
            .video-container:-webkit-full-screen {
                width: 100vw;
                height: 100vh;
                border-radius: 0;
                border: none;
                background-color: #000;
            }
            .video-container:fullscreen img,
            .video-container:-webkit-full-screen img {
                width: 100vw;
                height: 100vh;
                object-fit: contain;
            }

            /* Control Panel Layout */
            .controls-panel {
                width: 100%;
                display: grid;
                grid-template-columns: 1fr 1fr 1.3fr 1.5fr;
                gap: 8px;
                margin-top: 12px;
                align-items: stretch;
            }
            .btn-base {
                border-radius: 8px;
                font-size: 0.82rem;
                font-weight: 700;
                cursor: pointer;
                transition: transform 0.08s ease, background 0.15s ease, filter 0.15s ease;
                display: flex;
                align-items: center;
                justify-content: center;
                gap: 4px;
                border: 1px solid transparent;
                min-height: 48px;
                padding: 4px 6px;
            }
            .btn-base:active {
                transform: scale(0.98);
            }
            .btn-off {
                background: #21262d;
                color: #3fb950;
                border-color: #23863666;
            }
            .btn-off.active {
                background: #238636;
                color: #fff;
                border-color: #2ea043;
                box-shadow: 0 0 14px rgba(46, 160, 67, 0.45);
            }
            .btn-on {
                background: #21262d;
                color: #58a6ff;
                border-color: #1f6feb66;
            }
            .btn-on.active {
                background: #1f6feb;
                color: #fff;
                border-color: #388bfd;
            }
            .btn-cal {
                background: #21262d;
                color: #f0883e;
                border-color: #d2992266;
            }
            .btn-cal.active {
                background: #d29922;
                color: #0d1117;
                border-color: #f0883e;
                font-weight: 800;
            }
            .btn-cal.locked {
                background: #21262d;
                color: #8b949e;
                border-color: #30363d;
            }

            /* Flip-Cover Two-Stage Guarded Switch Assembly */
            .switch-bay {
                position: relative;
                perspective: 600px;
                min-height: 48px;
            }
            .safety-cover {
                position: absolute;
                inset: 0;
                border-radius: 8px;
                background: repeating-linear-gradient(
                    -45deg,
                    #2d2206,
                    #2d2206 10px,
                    #d29922 10px,
                    #d29922 20px
                );
                color: #000;
                font-weight: 800;
                font-size: 0.72rem;
                letter-spacing: 0.04em;
                display: flex;
                align-items: center;
                justify-content: center;
                border: 2px solid #d29922;
                box-shadow: 0 4px 8px rgba(0,0,0,0.4);
                cursor: pointer;
                transform-origin: top center;
                transition: transform 0.28s cubic-bezier(0.4, 0, 0.2, 1), opacity 0.2s ease;
                z-index: 5;
            }
            .cover-text {
                background: rgba(0, 0, 0, 0.85);
                color: #f0883e;
                padding: 4px 8px;
                border-radius: 4px;
                display: flex;
                align-items: center;
                gap: 5px;
            }
            .safety-cover.disabled {
                opacity: 0.35;
                cursor: not-allowed;
                filter: grayscale(0.8);
            }
            .safety-cover.flipped {
                transform: rotateX(85deg) translateY(-8px);
                opacity: 0.15;
                pointer-events: none;
            }

            .armed-trigger {
                width: 100%;
                height: 100%;
                border-radius: 8px;
                background: #490202;
                border: 2px dashed #f85149;
                color: #ff7b72;
                font-weight: 800;
                font-size: 0.76rem;
                letter-spacing: 0.04em;
                display: flex;
                flex-direction: column;
                align-items: center;
                justify-content: center;
                cursor: pointer;
                position: relative;
                overflow: hidden;
            }
            .armed-trigger.live-armed {
                background: #da3633;
                color: #fff;
                border-style: solid;
                border-color: #ff7b72;
                box-shadow: 0 0 16px rgba(218, 54, 51, 0.7);
                animation: pulse-red 1.0s infinite;
            }
            .timeout-bar {
                position: absolute;
                bottom: 0;
                left: 0;
                height: 4px;
                background: #f85149;
                width: 100%;
                transition: width 0.05s linear;
            }

            .btn-fullscreen {
                background: #21262d;
                color: #c9d1d9;
                border: 1px solid #30363d;
                border-radius: 6px;
                padding: 6px 12px;
                font-size: 0.82rem;
                font-weight: 600;
                cursor: pointer;
            }
            .fs-overlay-btn {
                position: absolute;
                bottom: 12px;
                right: 12px;
                background: rgba(22, 27, 34, 0.8);
                border: 1px solid rgba(255, 255, 255, 0.2);
                color: #fff;
                padding: 8px 10px;
                border-radius: 6px;
                cursor: pointer;
                display: flex;
                align-items: center;
                justify-content: center;
                backdrop-filter: blur(4px);
                z-index: 10;
            }
            .fs-kill-btn {
                position: absolute;
                top: 14px;
                left: 14px;
                background: rgba(35, 134, 54, 0.9);
                border: 1px solid #3fb950;
                color: #fff;
                padding: 8px 14px;
                border-radius: 6px;
                font-weight: 800;
                font-size: 0.85rem;
                cursor: pointer;
                backdrop-filter: blur(4px);
                z-index: 10;
                box-shadow: 0 4px 12px rgba(0,0,0,0.5);
                display: none;
            }
            .video-container:fullscreen .fs-kill-btn,
            .video-container:-webkit-full-screen .fs-kill-btn {
                display: block;
            }
            .meta {
                margin-top: 10px;
                font-size: 0.76rem;
                color: #8b949e;
                line-height: 1.4;
            }
        </style>
    </head>
    <body>
        <div class="hud-card">
            <div class="header-bar">
                <div class="brand-wrap">
                    <h1>Bombardeer HUD</h1>
                    <span id="modeBadge" class="status-pill pill-off">OFF / SAFE</span>
                </div>
                <button class="btn-fullscreen" onclick="toggleMaximize()">⛶ Fullscreen</button>
            </div>

            <!-- Runtime Perception Engine Selector -->
            <div class="detector-bar">
                <span class="detector-label">Perception Engine:</span>
                <div class="detector-toggle-group">
                    <button id="btnDetAruco" class="btn-det active" onclick="requestDetector('ARUCO')">
                        🏁 ArUco Target (Test)
                    </button>
                    <button id="btnDetWildlife" class="btn-det" onclick="requestDetector('WILDLIFE')">
                        🦌 Wildlife NPU (Hailo-8)
                    </button>
                </div>
            </div>

            <div class="video-container" id="videoBox" ondblclick="toggleMaximize()">
                <button class="fs-kill-btn" onclick="requestMode('OFF')">🛑 SAFE / DISARM</button>
                <img id="streamImg" src="/video_feed" alt="Live Targeting Feed">
                <button class="fs-overlay-btn" onclick="toggleMaximize()" title="Toggle Fullscreen">
                    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round">
                        <path d="M8 3H5a2 2 0 0 0-2 2v3m18 0V5a2 2 0 0 0-2-2h-3m0 18h3a2 2 0 0 0 2-2v-3M3 16v3a2 2 0 0 0 2 2h3"></path>
                    </svg>
                </button>
            </div>

            <div class="controls-panel">
                <button id="btnOff" class="btn-base btn-off active" onclick="requestMode('OFF')">
                    🛑 OFF
                </button>
                <button id="btnOn" class="btn-base btn-on" onclick="requestMode('ON')">
                    🎯 TRACK
                </button>
                <button id="btnCal" class="btn-base btn-cal" onclick="handleCalibrateClick()">
                    🧪 1-SHOT
                </button>

                <div class="switch-bay" id="switchBay">
                    <button id="armedTrigger" class="armed-trigger" onclick="confirmArmTrigger()">
                        <span id="triggerLabel">⚠️ ARM LIVE</span>
                        <div id="timeoutBar" class="timeout-bar" style="width: 0%;"></div>
                    </button>
                    <div id="safetyCover" class="safety-cover disabled" onclick="flipOpenCover()">
                        <div class="cover-text">
                            <span>🔒</span> <span id="coverLabel">LOCKED</span>
                        </div>
                    </div>
                </div>
            </div>

            <div class="meta">
                Deterministic 50Hz Motion | 921.6k Baud Closed-Loop | Hailo-8 26 TOPS PCIe NPU<br>
                <em>Switch between ArUco Fiducial and MegaDetector v6 Wildlife AI at runtime.</em>
            </div>
        </div>

        <script>
            let currentMode = "OFF";
            let isPrimed = false;
            let currentDetector = "ARUCO";
            let hailoAvailable = true;
            let coverOpen = false;
            let coverTimer = null;
            let timerStart = 0;
            const TIMEOUT_MS = 4000;

            function flipOpenCover() {
                if (currentMode !== "ON") {
                    return;
                }
                coverOpen = true;
                const cover = document.getElementById("safetyCover");
                cover.classList.add("flipped");

                const bar = document.getElementById("timeoutBar");
                timerStart = Date.now();
                clearInterval(coverTimer);

                coverTimer = setInterval(() => {
                    const elapsed = Date.now() - timerStart;
                    const remaining = Math.max(0, 1 - (elapsed / TIMEOUT_MS));
                    bar.style.width = (remaining * 100) + "%";

                    if (elapsed >= TIMEOUT_MS) {
                        snapCoverClosed();
                    }
                }, 40);
            }

            function snapCoverClosed() {
                clearInterval(coverTimer);
                coverOpen = false;
                const cover = document.getElementById("safetyCover");
                const bar = document.getElementById("timeoutBar");
                if (bar) bar.style.width = "0%";
                if (cover) cover.classList.remove("flipped");
            }

            function confirmArmTrigger() {
                if (currentMode === "ON" && coverOpen) {
                    clearInterval(coverTimer);
                    document.getElementById("timeoutBar").style.width = "0%";
                    requestMode("ARMED");
                } else if (currentMode === "ARMED") {
                    snapCoverClosed();
                    requestMode("ON");
                }
            }

            function handleCalibrateClick() {
                if (currentMode === "CALIBRATE") {
                    fetch("/api/mode", {
                        method: "POST",
                        headers: { "Content-Type": "application/json" },
                        body: JSON.stringify({ reprime: true })
                    })
                    .then(r => r.json())
                    .then(data => {
                        if (data.status === "ok") {
                            updateUIState(data.mode, data.primed, data.detector, data.hailo_available);
                        }
                    })
                    .catch(err => console.error("Reprime failed:", err));
                } else if (currentMode === "ON") {
                    requestMode("CALIBRATE");
                }
            }

            function requestDetector(detType) {
                fetch("/api/mode", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ detector: detType })
                })
                .then(r => r.json())
                .then(data => {
                    if (data.status === "ok") {
                        updateUIState(data.mode, data.primed, data.detector, data.hailo_available);
                    }
                })
                .catch(err => console.error("Detector switch failed:", err));
            }

            function updateUIState(mode, primedState, detectorType, isHailoOk) {
                currentMode = mode;
                isPrimed = !!primedState;
                if (detectorType) currentDetector = detectorType;
                if (typeof isHailoOk === "boolean") hailoAvailable = isHailoOk;

                const badge = document.getElementById("modeBadge");
                const btnOff = document.getElementById("btnOff");
                const btnOn = document.getElementById("btnOn");
                const btnCal = document.getElementById("btnCal");
                const cover = document.getElementById("safetyCover");
                const trigger = document.getElementById("armedTrigger");
                const triggerLabel = document.getElementById("triggerLabel");
                const coverLabel = document.getElementById("coverLabel");

                const btnDetAruco = document.getElementById("btnDetAruco");
                const btnDetWildlife = document.getElementById("btnDetWildlife");

                btnDetAruco.classList.toggle("active", currentDetector === "ARUCO");
                btnDetWildlife.classList.toggle("active", currentDetector === "WILDLIFE");
                btnDetWildlife.classList.toggle("disabled", !hailoAvailable);

                btnOff.classList.remove("active");
                btnOn.classList.remove("active");
                btnCal.classList.remove("active", "locked");
                trigger.classList.remove("live-armed");
                badge.className = "status-pill";

                if (mode === "OFF") {
                    badge.textContent = "OFF / SAFE";
                    badge.classList.add("pill-off");
                    btnOff.classList.add("active");

                    btnCal.textContent = "🧪 1-SHOT";
                    btnCal.classList.add("locked");

                    cover.classList.add("disabled");
                    cover.classList.remove("flipped");
                    coverLabel.textContent = "LOCKED";
                    triggerLabel.textContent = "⚠️ ARM LIVE";
                    snapCoverClosed();
                } else if (mode === "ON") {
                    badge.textContent = "TRACKING (SAFE)";
                    badge.classList.add("pill-on");
                    btnOn.classList.add("active");

                    btnCal.textContent = "🧪 1-SHOT";

                    cover.classList.remove("disabled");
                    coverLabel.textContent = "LIFT TO ARM";
                    triggerLabel.textContent = "⚠️ ARM LIVE";
                    if (coverOpen || cover.classList.contains("flipped")) {
                        snapCoverClosed();
                    }
                } else if (mode === "CALIBRATE") {
                    badge.textContent = isPrimed ? "CAL: PRIMED" : "CAL: LOCKED";
                    badge.classList.add("pill-cal");

                    btnCal.classList.add("active");
                    btnCal.textContent = isPrimed ? "🎯 PRIMED" : "🔄 RE-PRIME";

                    cover.classList.add("disabled");
                    coverLabel.textContent = "TESTING";
                    snapCoverClosed();
                } else if (mode === "ARMED") {
                    badge.textContent = "ARMED / LIVE";
                    badge.classList.add("pill-armed");

                    btnCal.textContent = "🧪 1-SHOT";
                    btnCal.classList.add("locked");

                    cover.classList.add("flipped");
                    trigger.classList.add("live-armed");
                    triggerLabel.textContent = "🔥 DISARM";
                }
            }

            function requestMode(targetMode) {
                fetch("/api/mode", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ mode: targetMode })
                })
                .then(r => r.json())
                .then(data => {
                    if (data.status === "ok") {
                        updateUIState(data.mode, data.primed, data.detector, data.hailo_available);
                    }
                })
                .catch(err => {
                    console.error("Mode update failed:", err);
                });
            }

            function syncMode() {
                fetch("/api/mode")
                .then(r => r.json())
                .then(data => {
                    if (data.status === "ok") {
                        if (
                            data.mode !== currentMode ||
                            data.primed !== isPrimed ||
                            data.detector !== currentDetector ||
                            data.hailo_available !== hailoAvailable
                        ) {
                            updateUIState(data.mode, data.primed, data.detector, data.hailo_available);
                        }
                    }
                })
                .catch(() => {});
            }

            setInterval(syncMode, 1000);
            syncMode();

            function toggleMaximize() {
                const box = document.getElementById("videoBox");
                if (!document.fullscreenElement && !document.webkitFullscreenElement) {
                    if (box.requestFullscreen) {
                        box.requestFullscreen().catch(err => console.warn(err));
                    } else if (box.webkitRequestFullscreen) {
                        box.webkitRequestFullscreen();
                    }
                } else {
                    if (document.exitFullscreen) {
                        document.exitFullscreen();
                    } else if (document.webkitExitFullscreen) {
                        document.webkitExitFullscreen();
                    }
                }
            }
        </script>
    </body>
    </html>
    """)

def generate_frames():
    global active_web_clients
    with frame_lock:
        active_web_clients += 1
    try:
        while not app_shutting_down.is_set():
            with frame_lock:
                frame = latest_jpeg

            if frame is not None:
                yield STREAM_PART_HEADER
                yield frame
                yield STREAM_PART_FOOTER

            if app_shutting_down.wait(timeout=0.033):
                break
    finally:
        with frame_lock:
            active_web_clients = max(0, active_web_clients - 1)

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/favicon.ico")
def favicon():
    if os.path.exists(os.path.join(STATIC_DIR, "bombardeer.ico")):
        return send_from_directory(STATIC_DIR, "bombardeer.ico", mimetype="image/vnd.microsoft.icon")
    return ("", 204)

# =========================================================================
# Execution Entry Points & Fast Shutdown Signal Hook
# =========================================================================

if mp.current_process().name == "MainProcess":
    _orig_sigterm = signal.getsignal(signal.SIGTERM)
    _orig_sigint = signal.getsignal(signal.SIGINT)

    def _fast_worker_shutdown(signum, frame):
        app_shutting_down.set()
        if global_wildlife_detector is not None:
            global_wildlife_detector.shutdown()
        if callable(_orig_sigterm) and signum == signal.SIGTERM:
            _orig_sigterm(signum, frame)
        elif callable(_orig_sigint) and signum == signal.SIGINT:
            _orig_sigint(signum, frame)
        else:
            os._exit(0)

    try:
        signal.signal(signal.SIGTERM, _fast_worker_shutdown)
        signal.signal(signal.SIGINT, _fast_worker_shutdown)
    except ValueError:
        pass

    if not any(t.name == "VisionThread" for t in threading.enumerate()):
        t = threading.Thread(target=vision_thread, daemon=True, name="VisionThread")
        t.start()

if __name__ == "__main__":
    log.info("Starting local Flask development server. For production, use Gunicorn/systemd.")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True, use_reloader=False)