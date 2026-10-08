#!/usr/bin/env python3
"""
Bombardeer Turret: Real-Time Dual-Mode Controller (Mission-Critical Edition)
----------------------------------------------------------------------------
- Architecture:
    * Asynchronous Perception & Motion Decoupling:
        - Vision Worker: Processes frames at sensor/inference rate (15-30 Hz).
        - Motion Servoing Loop: Runs deterministically at 50 Hz (20ms ticks),
          predicting target state and feeding the ESP32 continuous velocity
          vectors regardless of optical frame cadence or ML inference lag.
    * Concurrency Architecture (Segregated Lock Planes):
        - telemetry_lock: High-speed motor angles, moving flags, and history deque.
        - target_lock: Perception target state, supervisory health, and storage metrics.
        - frame_lock: Flask JPEG streaming and client tracking.
        - mode_lock: Tri-state engagement model (OFF -> ON -> ARMED).
    * Latency-Compensated Time-Sync Forward Projection:
        - Projects Kalman state to the exact actuation instant, eliminating
          hunting, overshoot, and FastAccelStepper trajectory replanning halts.
    * 1.5-Second Continuous Dead-Reckoning Coasting:
        - Exponential velocity decay through optical dropouts, foliage
          occlusions, and motion blur without resetting filter momentum.
    * Software Travel Clamping & Angular Velocity Fire Gating:
        - Prevents driving into mechanical stops and suppresses firing during
          high-speed slewing.
    * Clamped Multithreading & Zero-Copy Communications:
        - Binary UART protocol at 921,600 baud with non-blocking supervisor.
"""

import os
os.environ["LIBCAMERA_LOG_LEVELS"] = "IPARPI:FATAL"

import glob
import time
import math
import shutil
import datetime
import threading
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

# -------------------------------------------------------------------------
# OpenCV Deterministic Core Clamping
# -------------------------------------------------------------------------
# Clamp OpenCV to 2 worker threads on physical cores 0 & 1, leaving cores
# 2 & 3 dedicated to the 50 Hz motion servo, UART I/O, and PiSP DMA interrupts.
cv2.setNumThreads(2)
cv2.ocl.setUseOpenCL(False)

# =========================================================================
# Structured Logging Setup
# =========================================================================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("Bombardeer")

# =========================================================================
# Configuration & Hardware Calibration
# =========================================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
RECORDINGS_DIR = os.path.join(BASE_DIR, "recordings")

try:
    os.makedirs(RECORDINGS_DIR, exist_ok=True)
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

# Camera Optical Intrinsics
FOCAL_LENGTH_PX = 540.4
MRAD_PER_PIXEL_X = 1000.0 / FOCAL_LENGTH_PX
MRAD_PER_PIXEL_Y = 1000.0 / FOCAL_LENGTH_PX

ARUCO_REAL_WIDTH_MM = 365.0
MIN_VALID_RANGE_M = 3.0
MAX_VALID_RANGE_M = 75.0

# Dynamic Targeting, Hysteresis & Deadbands
BALLISTIC_ACQUIRE_MRAD = 120.0     # Snap-to-target threshold from rest
BALLISTIC_ABORT_MRAD = 260.0       # Only abort velocity mode if target exceeds 260mrad
VELOCITY_DEADBAND_MRAD = 4.0       # Coast to standstill inside this error radius
FIRE_DEADBAND_MRAD = 40.0          # Permissive firing engagement radius
MAX_FIRE_SLEW_SPEED_MRAD_S = 180.0 # Suppress firing if turret is slewing faster than this
MOTION_LOOP_INTERVAL_SEC = 0.020   # 50 Hz deterministic motion servo loop
SETPOINT_MIN_INTERVAL_SEC = 0.040  # 25 Hz maximum UART transmission rate

# Software Travel Limits (Physical Axis Protection)
TILT_MIN_MRAD = -420
TILT_MAX_MRAD = 400

MAX_TRIGGER_DURATION_SEC = 2.0
TRIGGER_COOLDOWN_SEC = 1.5

TARGET_MARKER_ID = 0
CONFIRMATION_FRAMES = 2

# Persistence & Coasting Horizons
COAST_MAX_HORIZON_SEC = 1.500      # Track through occlusions up to 1.5s
COAST_EXP_DECAY_RATE = 2.5         # Exponential velocity decay factor after 350ms

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

# =========================================================================
# Concurrency Architecture: Isolated Lock Planes
# =========================================================================
app = Flask(__name__)

# Lock 0: Tri-State Operational Engagement (OFF, ON, ARMED)
mode_lock = threading.Lock()
turret_mode = "OFF"  # Power-on state is always disarmed and inert

def get_turret_mode():
    with mode_lock:
        return turret_mode

def set_turret_mode(new_mode):
    global turret_mode
    with mode_lock:
        turret_mode = new_mode

# Lock 1: Web Streaming Buffer and Client Counter
frame_lock = threading.Lock()
latest_jpeg = None
active_web_clients = 0

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
}

global_turret_queue = None

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


# --- Atomic Telemetry Accessors (telemetry_lock) ---

def update_turret_telemetry(pan, tilt, moving, rx_mono_time):
    """Atomic update called exclusively by turret_serial_worker."""
    with telemetry_lock:
        turret_telemetry["pan_mrad"] = pan
        turret_telemetry["tilt_mrad"] = tilt
        turret_telemetry["is_moving"] = moving
        telemetry_history.append((rx_mono_time, pan, tilt))

def set_firing_state(firing):
    with telemetry_lock:
        turret_telemetry["is_firing"] = firing

def get_instantaneous_telemetry():
    """Ultra-fast lock acquisition for the 50 Hz motion servo."""
    with telemetry_lock:
        return (
            turret_telemetry["pan_mrad"],
            turret_telemetry["tilt_mrad"],
            turret_telemetry["is_moving"],
            turret_telemetry["is_firing"]
        )

def get_turret_pos_at_time(target_mono_sec):
    """Interpolates historical position under telemetry_lock without blocking target updates."""
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


# --- Atomic Target Accessors (target_lock) ---

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
    """Thread-safe Kalman Filter supporting continuous multi-rate predict and correct."""
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
        """Asynchronous measurement correction step invoked by perception thread."""
        with self.lock:
            if not self.is_initialized:
                self.kf.statePost = np.array([
                    [float(measured_pan)],
                    [float(measured_tilt)],
                    [0.0],
                    [0.0]
                ], dtype=np.float32)
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
        """Continuous state projection step invoked by 50 Hz motion servo thread."""
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
# Perception Layer (ArUco / Drop-in Interface for Future ML Models)
# =========================================================================
class ArucoTargetDetector:
    def __init__(self, target_id=TARGET_MARKER_ID):
        self.target_id = target_id
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.errorCorrectionRate = 0.55

        # Sub-pixel corner refinement & adaptive windowing for small, distant markers
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 4

        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def detect(self, frame_bgr):
        try:
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            enhanced_gray = self.clahe.apply(gray)
            corners, ids, _ = self.detector.detectMarkers(enhanced_gray)
            detections = []

            if ids is not None and len(ids) > 0:
                for marker_corners, marker_id in zip(corners, ids.flatten()):
                    if self.target_id is not None and marker_id != self.target_id:
                        continue

                    pts = marker_corners.reshape((4, 2))
                    d01 = math.hypot(pts[0][0] - pts[1][0], pts[0][1] - pts[1][1])
                    d12 = math.hypot(pts[1][0] - pts[2][0], pts[1][1] - pts[2][1])

                    # Reject markers smaller than 8 pixels (allows tracking out to 30+ yards)
                    if d01 < 8 or d12 < 8:
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
                    estimated_range_m = (ARUCO_REAL_WIDTH_MM * FOCAL_LENGTH_PX) / (marker_px * 1000.0)

                    detections.append({
                        "box": [x1, y1, x2, y2],
                        "center": (cx, cy),
                        "score": 0.99,
                        "area": area,
                        "range_m": float(np.clip(estimated_range_m, MIN_VALID_RANGE_M, MAX_VALID_RANGE_M)),
                        "id": int(marker_id)
                    })

            return detections
        except Exception as e:
            log.error(f"[DETECTOR ERROR] Vision parsing failed: {e}")
            return []

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
    cooldown_until = 0.0

    rx_buf = bytearray()
    last_telemetry_rx_time = 0.0

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
            # Drain the entire queue into discrete action slots
            pending_commands = []
            latest_velocity = None

            while not cmd_queue.empty():
                item = cmd_queue.get_nowait()
                if item.get("cmd") == "VELOCITY":
                    # Coalesce: keep only the newest velocity vector
                    latest_velocity = item
                else:
                    # Preserve chronological order for discrete commands (SETPOINT, TRIGGER, HALT)
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
                        # Drop stale trigger commands older than 100ms
                        if req_state == 1 and (now - item.get("timestamp", now) > 0.100):
                            continue
                        if req_state == 1:
                            if trigger_state == 0 and now >= cooldown_until:
                                trigger_state = 1
                                trigger_start_time = now
                                set_firing_state(True)
                                ser.write(b"T 1\n")
                                with target_lock:
                                    system_health["serial_tx_count"] += 1
                                log.info("[SERIAL] Solenoid trigger pulse FIRED")
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

        if trigger_state == 1 and (now - trigger_start_time >= MAX_TRIGGER_DURATION_SEC):
            trigger_state = 0
            set_firing_state(False)
            cooldown_until = now + TRIGGER_COOLDOWN_SEC
            if ser and ser.is_open:
                try:
                    ser.write(b"T 0\n")
                except Exception:
                    pass

        # Ingest Telemetry Stream
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

        files = glob.glob(os.path.join(RECORDINGS_DIR, "wildlife_*.mp4"))
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
        ("mp4v", cv2.VideoWriter_fourcc(*"mp4v")),
        ("avc1", cv2.VideoWriter_fourcc(*"avc1")),
        ("MJPG", cv2.VideoWriter_fourcc(*"MJPG")),
        ("XVID", cv2.VideoWriter_fourcc(*"XVID"))
    ]

    while True:
        try:
            item = record_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        cmd = item.get("cmd")

        if cmd == "start":
            if not recording_active:
                ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                current_filepath = os.path.join(RECORDINGS_DIR, f"wildlife_{ts}.mp4")
                writer = None
                frames_written = 0

                for tag, fourcc in CODECS:
                    try:
                        cand = cv2.VideoWriter(current_filepath, fourcc, TARGET_FPS, (FRAME_WIDTH, FRAME_HEIGHT))
                        if cand is not None and cand.isOpened():
                            writer = cand
                            log.info(f"[RECORDER] VideoWriter opened with codec '{tag}' -> {current_filepath}")
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

# =========================================================================
# Asynchronous Real-Time Motion Servoing Loop (50 Hz / 20ms Cadence)
# =========================================================================
def motion_servoing_worker(target_filter, turret_queue):
    """
    Decoupled 50 Hz real-time motion control loop.
    Executes Kalman forward projection against current physical position,
    streaming continuous velocity vectors regardless of camera or ML inference cadence.
    """
    last_setpoint_time = 0.0
    last_debug_log_time = 0.0
    is_actively_tracking_velocity = False
    last_v_pan = 0
    last_v_tilt = 0
    last_sent_p_pan = 99999
    last_sent_p_tilt = 99999
    last_mode_seen = "OFF"
    
    log.info("[MOTION] 50 Hz Real-Time Motion Controller started")

    while True:
        loop_start = time.monotonic()
        now = time.time()

        # Atomic, flat snapshot retrieval (No nested locks)
        mode = get_turret_mode()
        target_data = get_primary_target_snapshot()
        current_pan, current_tilt, hw_moving, _ = get_instantaneous_telemetry()
        serial_ok = get_serial_connected()

        # Handle Transition to OFF / Inactive Mode
        if mode == "OFF":
            if last_mode_seen != "OFF":
                turret_queue.put({"cmd": "HALT"})
                turret_queue.put({"cmd": "TRIGGER", "state": 0})
                is_actively_tracking_velocity = False
                last_v_pan = 0
                last_v_tilt = 0
                last_sent_p_pan = 99999
                last_sent_p_tilt = 99999
                target_filter.reset()
                log.info("[MODE] Turret disengaged and placed in OFF / STANDBY.")
            last_mode_seen = mode

            elapsed = time.monotonic() - loop_start
            sleep_time = MOTION_LOOP_INTERVAL_SEC - elapsed
            if sleep_time > 0.001:
                time.sleep(sleep_time)
            continue

        last_mode_seen = mode

        # Turret is ON (Tracking) or ARMED (Tracking + Firing)
        if serial_ok and target_data is not None:
            time_since_seen = now - target_data["last_seen"]

            # -------------------------------------------------------------
            # CASE A: TARGET ACTIVE OR WITHIN 1.5s BALLISTIC COAST HORIZON
            # -------------------------------------------------------------
            if time_since_seen <= COAST_MAX_HORIZON_SEC:
                # Continuous Kalman projection to the exact actuation instant
                predicted_state = target_filter.predict_state(now)
                if predicted_state is not None:
                    world_pan, world_tilt, tgt_v_pan, tgt_v_tilt = predicted_state

                    # LATENCY-FREE INSTANTANEOUS REAL-TIME ERROR
                    realtime_err_pan = world_pan - current_pan
                    realtime_err_tilt = world_tilt - current_tilt
                    radial_error_mrad = math.hypot(realtime_err_pan, realtime_err_tilt)

                    # Dynamic Trigger Gate: Strictly inhibited unless mode == "ARMED"
                    # Require target freshness (<150ms), radial error within deadband,
                    # AND ensure turret is stabilized (speed < 180 mrad/s) so it doesn't
                    # fire while slewing past at maximum velocity.
                    turret_speed_mag = math.hypot(last_v_pan, last_v_tilt)
                    can_fire = (
                        mode == "ARMED"
                        and time_since_seen <= 0.150
                        and radial_error_mrad <= FIRE_DEADBAND_MRAD
                        and turret_speed_mag < MAX_FIRE_SLEW_SPEED_MRAD_S
                    )
                    turret_queue.put({"cmd": "TRIGGER", "state": 1 if can_fire else 0, "timestamp": now})

                    # MODE HYSTERESIS SELECTION
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

                    # MODE 1: BALLISTIC SNAP-TO-TARGET ('P')
                    if use_ballistic:
                        goal_delta = math.hypot(world_pan - last_sent_p_pan, world_tilt - last_sent_p_tilt)
                        # Require at least 400ms between P setpoints, or an extreme shift of >60mrad
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
                        
                    # MODE 2: CONTINUOUS FLUID PURSUIT ('V')
                    else:
                        if time_since_tx >= SETPOINT_MIN_INTERVAL_SEC:
                            last_setpoint_time = now

                            def compute_velocity(err, tgt_v, deadband):
                                mag = abs(err)
                                if mag < deadband:
                                    return 0
                                # Near target: smooth settling
                                if mag <= 25.0:
                                    kp = 1.4
                                    cmd = err * kp + 0.8 * tgt_v
                                else:
                                    # Fast sweep: full 1.0x feed-forward eliminates tracking lag
                                    kp = 2.8
                                    cmd = err * kp + 1.0 * tgt_v
                                return int(round(cmd))
                            
                            raw_v_pan = compute_velocity(realtime_err_pan, tgt_v_pan, VELOCITY_DEADBAND_MRAD)
                            raw_v_tilt = compute_velocity(realtime_err_tilt, tgt_v_tilt, VELOCITY_DEADBAND_MRAD)

                            # COAST DECAY DAMPING: after 350ms of occlusion, smoothly decay velocity
                            if time_since_seen > 0.350:
                                decay_factor = math.exp(-COAST_EXP_DECAY_RATE * (time_since_seen - 0.350))
                                raw_v_pan = int(round(raw_v_pan * decay_factor))
                                raw_v_tilt = int(round(raw_v_tilt * decay_factor))

                            # Slew bounds: up to 850 mrad/s pan, 450 mrad/s tilt
                            v_pan_cmd = max(-850, min(850, raw_v_pan))
                            v_tilt_cmd = max(-450, min(450, raw_v_tilt))

                            # Software Travel Limits: prevent driving into mechanical endstops
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

                    # Diagnostic Telemetry
                    if now - last_debug_log_time >= 0.150:
                        last_debug_log_time = now
                        log.info(
                            f"[{mode}] Cur:({current_pan:+5d},{current_tilt:+4d}) | "
                            f"Err:({realtime_err_pan:+5.1f},{realtime_err_tilt:+4.1f}) | "
                            f"Cmd:{cmd_type} | Mv:{int(hw_moving)}"
                        )

            # -------------------------------------------------------------
            # CASE B: TARGET EXPIRED BEYOND 1.5s COAST HORIZON
            # -------------------------------------------------------------
            else:
                clear_primary_target()
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

        # Precise 50 Hz cycle regulation
        elapsed = time.monotonic() - loop_start
        sleep_time = MOTION_LOOP_INTERVAL_SEC - elapsed
        if sleep_time > 0.001:
            time.sleep(sleep_time)

# =========================================================================
# Vision Perception Thread (Asynchronous Ingestion & Kalman Correction)
# =========================================================================
def vision_thread():
    global latest_jpeg, global_turret_queue

    ensure_storage_headroom()
    detector = ArucoTargetDetector()
    target_filter = TargetKalmanFilter(process_noise=35.0, measurement_noise=10.0)

    try:
        # Detect candidate IMX708 NoIR ISP tuning profiles
        noir_candidates = [
            "/usr/share/libcamera/ipa/rpi/pisp/imx708_noir.json",
            "/usr/share/libcamera/ipa/rpi/vc4/imx708_noir.json",
            "/usr/share/libcamera/ipa/rpi/pisp/imx708_wide_noir.json",
            "/usr/share/libcamera/ipa/rpi/vc4/imx708_wide_noir.json"
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
                # 1: Highlight Priority - prevents clipping brightly lit grass/sky
                "AeConstraintMode": 1,
                # 1: Spot Metering - meters strictly the center landscape aperture,
                # ignoring dark blind fabric and barrel perimeter
                "AeMeteringMode": 1,
                # Dynamic Auto White Balance
                "AwbMode": 0,
                # Negative EV bias pulls down direct sun luminance
                "ExposureValue": -0.8,
                # Allows sub-millisecond daylight shutter, up to 33.3ms for dusk
                "FrameDurationLimits": (100, 33333),
                # Increase contrast to cut through atmospheric and NoIR haze
                "Contrast": 1.25,
                "Saturation": 0.80,
                "Sharpness": 1.25,
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

    last_valid_range_m = 25.0
    last_seen_connected = False

    tentative_target = None
    tentative_streak = 0

    is_recording = False
    lock_consecutive_frames = 0
    last_detection_time = 0.0
    recording_start_time = 0.0
    last_standby_log_time = 0.0

    # ISP Adaptive Day/Night State
    isp_eval_counter = 0
    is_monochrome_mode = False

    # Pre-allocated fixed memory ring buffer for 720p pre-roll (Zero heap churn)
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

        frame_count += 1
        isp_eval_counter += 1

        # Adaptive Dusk/Day Chroma & Gain Management
        if isp_eval_counter >= 30:
            isp_eval_counter = 0
            lux = metadata.get("Lux", 50.0)
            gain = metadata.get("AnalogueGain", 1.0)

            # Priority 1: Deep Dusk / Low-Light (< 12 Lux or Gain > 5.0)
            # Switch to pure monochrome to eliminate IR sensor noise and maximize tracking contrast
            if (gain > 5.0 or lux < 12.0) and not is_monochrome_mode:
                try:
                    picam2.set_controls({
                        "Saturation": 0.0,
                        "Sharpness": 1.3  # Extra sharpness on ArUco / target edges at dusk
                    })
                    is_monochrome_mode = True
                    log.info(f"[ISP] Priority 1 (Dusk/Low-Light): B&W Enhanced (Lux:{lux:.1f}, Gain:{gain:.2f})")
                except Exception:
                    pass

            # Priority 2 & 3: Daylight & Indoor (> 18 Lux and Gain <= 4.0)
            elif (gain <= 4.0 and lux >= 18.0) and is_monochrome_mode:
                try:
                    picam2.set_controls({
                        "Saturation": 0.85,
                        "Sharpness": 1.15
                    })
                    is_monochrome_mode = False
                    log.info(f"[ISP] Priority 2/3 (Day/Indoor): Color Restored (Lux:{lux:.1f}, Gain:{gain:.2f})")
                except Exception:
                    pass

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

        # --- Stage 2: Target Detection ---
        detections = detector.detect(frame_main)

        # ArUco has built-in CRC error checking; accept candidate on frame 1 without delay
        best_candidate = detections[0] if len(detections) > 0 else None
        verified_detection = best_candidate
        
        if best_candidate is not None:
            has_active = (get_primary_target_snapshot() is not None)
            if has_active:
                verified_detection = best_candidate
            else:
                if tentative_target is not None:
                    tentative_streak += 1
                    tentative_target = best_candidate
                    if tentative_streak >= CONFIRMATION_FRAMES:
                        verified_detection = best_candidate
                else:
                    tentative_target = best_candidate
                    tentative_streak = 1
        else:
            tentative_target = None
            tentative_streak = 0

        # --- Stage 3: Kalman Measurement Correction ---
        if verified_detection is not None:
            raw_cx, raw_cy = verified_detection["center"]

            if "range_m" in verified_detection:
                last_valid_range_m = verified_detection["range_m"]
            current_range_m = last_valid_range_m

            # Look up physical position matching the exact instant of sensor exposure (~40ms latency)
            pan_at_capture, tilt_at_capture = get_turret_pos_at_time(capture_arrival_mono - 0.040)

            # Optical parallax corrections
            parallax_pan_mrad = (-CAMERA_OFFSET_X_MM / (current_range_m * 1000.0)) * 1000.0
            parallax_tilt_mrad = (CAMERA_OFFSET_Y_MM / (current_range_m * 1000.0)) * 1000.0

            # True trigonometric mapping
            dx = -(raw_cx - OPTICAL_CENTER[0])
            dy = -(raw_cy - OPTICAL_CENTER[1])
            opt_err_pan_mrad = math.atan2(dx, FOCAL_LENGTH_PX) * 1000.0 + parallax_pan_mrad
            opt_err_tilt_mrad = math.atan2(dy, FOCAL_LENGTH_PX) * 1000.0 + parallax_tilt_mrad

            raw_world_pan = pan_at_capture + int(round(opt_err_pan_mrad))
            raw_world_tilt = tilt_at_capture + int(round(opt_err_tilt_mrad))

            # Update Kalman measurement model
            target_filter.update_measurement(raw_world_pan, raw_world_tilt, now)

            set_primary_target({
                "center": verified_detection["center"],
                "box": verified_detection["box"],
                "score": verified_detection["score"],
                "area": verified_detection["area"],
                "range_m": current_range_m,
                "last_seen": now,
                "id": verified_detection.get("id", 0)
            })
        else:
            # Standby heartbeats: Log when idling with no active targets in view
            if get_primary_target_snapshot() is None and (now - last_standby_log_time >= 2.0):
                last_standby_log_time = now
                p_cur, t_cur, _, _ = get_instantaneous_telemetry()
                log.info(f"[STANDBY] Scanning... | FPS:{sensor_fps:4.1f} | Turret:({p_cur:+5d},{t_cur:+4d})")

        # --- Stage 4: Video Archiving ---
        local_target = get_primary_target_snapshot()
        target_locked = (local_target is not None and (now - local_target["last_seen"] <= 0.350))
        if target_locked:
            lock_consecutive_frames += 1
            last_detection_time = now
        else:
            lock_consecutive_frames = 0

        if not is_recording:
            # 1. Update circular memory slot in-place (No new heap allocations)
            np.copyto(pre_roll_pool[pre_roll_idx], frame_main)
            pre_roll_idx = (pre_roll_idx + 1) % PRE_ROLL_CAPACITY
            if pre_roll_count < PRE_ROLL_CAPACITY:
                pre_roll_count += 1

            # 2. Trigger recording when lock threshold is met
            if target_locked and lock_consecutive_frames >= MIN_LOCK_FRAMES_TO_RECORD:
                is_recording = True
                recording_start_time = now

                # Extract frames in strict chronological order from the ring buffer
                if pre_roll_count < PRE_ROLL_CAPACITY:
                    saved_pre_roll = [pre_roll_pool[i].copy() for i in range(pre_roll_count)]
                else:
                    saved_pre_roll = [
                        pre_roll_pool[(pre_roll_idx + i) % PRE_ROLL_CAPACITY].copy()
                        for i in range(PRE_ROLL_CAPACITY)
                    ]
                try:
                    record_queue.put_nowait({"cmd": "start", "pre_roll": saved_pre_roll})
                except queue.Full:
                    pass

                # Reset ring buffer metrics immediately for future clips
                pre_roll_idx = 0
                pre_roll_count = 0
        else:
            # While actively recording: zero-copy pass directly to the recorder queue
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
                    # Direct reference handoff: No .copy() during recording
                    record_queue.put_nowait({"cmd": "frame", "frame": frame_main})
                except queue.Full:
                    pass


        # --- Stage 5: Client-Aware HUD Render & Web Stream ---
        with frame_lock:
            clients_viewing = (active_web_clients > 0)

        if clients_viewing:
            cur_mode = get_turret_mode()
            current_range_m = local_target.get("range_m", 25.0) if local_target else 25.0
            oc_x = int(OPTICAL_CENTER[0] * SCALE_X)
            oc_y = int(OPTICAL_CENTER[1] * SCALE_Y)
            cv2.drawMarker(annotated, (oc_x, oc_y), (0, 255, 255), cv2.MARKER_CROSS, 16, 1)

            conv_px_x = OPTICAL_CENTER[0] + int((-CAMERA_OFFSET_X_MM / (current_range_m * 1000.0)) * FOCAL_LENGTH_PX)
            conv_px_y = OPTICAL_CENTER[1] + int((CAMERA_OFFSET_Y_MM / (current_range_m * 1000.0)) * FOCAL_LENGTH_PX)
            aim_x = int(conv_px_x * SCALE_X)
            aim_y = int(conv_px_y * SCALE_Y)
            db_radius = int((FIRE_DEADBAND_MRAD / MRAD_PER_PIXEL_X) * SCALE_X)

            p_mrad, t_mrad, _, firing = get_instantaneous_telemetry()
            with target_lock:
                ser_ok = system_health["serial_connected"]
                rec_ok = system_health["recorder_active"]
                alert_msg = system_health["last_alert"] if now < system_health["alert_time"] else None

            ring_color = (0, 0, 255) if cur_mode == "ARMED" else ((255, 200, 0) if cur_mode == "ON" else (0, 200, 50))
            if firing:
                ring_color = (0, 255, 255)

            cv2.circle(annotated, (aim_x, aim_y), db_radius, ring_color, 1)
            cv2.drawMarker(annotated, (aim_x, aim_y), ring_color, cv2.MARKER_CROSS, 24, 1)

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

                status_lbl = f"ID:{local_target.get('id', 0)} | {current_range_m:.1f}m"
                cv2.putText(annotated, status_lbl, (tx1, max(ty1 - 8, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_color, 1, cv2.LINE_AA)

            # High-Contrast Mode Watermark with Native Geometry Dot
            if cur_mode == "ARMED":
                cv2.rectangle(annotated, (10, 10), (150, 34), (0, 0, 180), -1)
                cv2.circle(annotated, (22, 22), 4, (255, 255, 255), -1)
                cv2.putText(annotated, "ARMED / LIVE", (32, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
            elif cur_mode == "ON":
                cv2.rectangle(annotated, (10, 10), (165, 34), (180, 110, 0), -1)
                cv2.circle(annotated, (22, 22), 4, (255, 255, 255), -1)
                cv2.putText(annotated, "TRACKING / SAFE", (32, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            else:
                cv2.rectangle(annotated, (10, 10), (130, 34), (0, 140, 40), -1)
                cv2.circle(annotated, (22, 22), 4, (255, 255, 255), -1)
                cv2.putText(annotated, "OFF / SAFE", (32, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

            # Persistent Dedicated Target Range Card (Upper Right, Always Visible in Fullscreen)
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
                cv2.circle(annotated, (DISPLAY_WIDTH - 25, 25), 8, (0, 0, 255), -1)
                cv2.putText(annotated, "REC", (DISPLAY_WIDTH - 70, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

            badge_color = (0, 255, 0) if ser_ok else (0, 0, 255)
            ser_badge = "SERIAL: OK" if ser_ok else "SERIAL: DISCONNECTED"
            status_text = f"FPS:{sensor_fps:.1f} | Pan:{p_mrad}mrad Tilt:{t_mrad}mrad | {ser_badge}"
            cv2.putText(annotated, status_text, (10, DISPLAY_HEIGHT - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, badge_color, 1, cv2.LINE_AA)

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

@app.route("/api/mode", methods=["GET", "POST"])
def api_mode():
    global global_turret_queue
    if request.method == "POST":
        req = request.get_json(silent=True) or {}
        target_mode = req.get("mode", "").upper()

        if target_mode not in ("OFF", "ON", "ARMED"):
            return jsonify({"status": "error", "message": "Invalid mode specified"}), 400

        current_mode = get_turret_mode()

        # Strict Progression Guard: Can only arm when already turned "ON"
        if target_mode == "ARMED" and current_mode != "ON":
            return jsonify({"status": "error", "message": "Turret must be turned ON before arming"}), 400

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
        elif target_mode == "ARMED":
            set_alert("WARNING: TURRET ARMED / LIVE FIRE", duration=4.0)

        log.warning(f"[MODE TRANSITION] Turret switched to {target_mode}")
        return jsonify({"status": "ok", "mode": target_mode})

    return jsonify({"status": "ok", "mode": get_turret_mode()})


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
                margin-bottom: 12px;
            }
            .brand-wrap {
                display: flex;
                align-items: center;
                gap: 10px;
            }
            h1 {
                font-size: 1.1rem;
                letter-spacing: 0.05em;
                text-transform: uppercase;
                color: #39d353;
                text-align: left;
            }
            .status-pill {
                font-size: 0.72rem;
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
                grid-template-columns: 1.5fr 1.2fr 2.3fr;
                gap: 10px;
                margin-top: 12px;
                align-items: stretch;
            }
            .btn-base {
                border-radius: 8px;
                font-size: 0.88rem;
                font-weight: 700;
                cursor: pointer;
                transition: transform 0.08s ease, background 0.15s ease, filter 0.15s ease;
                display: flex;
                align-items: center;
                justify-content: center;
                gap: 6px;
                border: 1px solid transparent;
                min-height: 48px;
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

            /* Option 3: Flip-Cover Two-Stage Guarded Switch Assembly */
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
                font-size: 0.76rem;
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
                padding: 4px 10px;
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

            /* Exposed Arm Trigger & Timeout Meter */
            .armed-trigger {
                width: 100%;
                height: 100%;
                border-radius: 8px;
                background: #490202;
                border: 2px dashed #f85149;
                color: #ff7b72;
                font-weight: 800;
                font-size: 0.82rem;
                letter-spacing: 0.05em;
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
                    🛑 OFF / SAFE
                </button>
                <button id="btnOn" class="btn-base btn-on" onclick="requestMode('ON')">
                    🎯 TRACK (ON)
                </button>

                <!-- Two-Stage Guarded Switch Assembly -->
                <div class="switch-bay" id="switchBay">
                    <button id="armedTrigger" class="armed-trigger" onclick="confirmArmTrigger()">
                        <span id="triggerLabel">⚠️ CONFIRM LIVE FIRE</span>
                        <div id="timeoutBar" class="timeout-bar" style="width: 0%;"></div>
                    </button>
                    <div id="safetyCover" class="safety-cover disabled" onclick="flipOpenCover()">
                        <div class="cover-text">
                            <span>🔒</span> <span id="coverLabel">OFF / SAFE</span>
                        </div>
                    </div>
                </div>
            </div>

            <div class="meta">
                Deterministic 50Hz Motion | 921.6k Baud Closed-Loop<br>
                <em>Guarded flip-cover arms the turret. Cover auto-closes after 4s without confirmation.</em>
            </div>
        </div>

        <script>
            let currentMode = "OFF";
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
                    // Clicking the active live-fire button safely toggles back to tracking
                    snapCoverClosed();
                    requestMode("ON");
                }
            }

            function updateUIState(mode) {
                currentMode = mode;
                const badge = document.getElementById("modeBadge");
                const btnOff = document.getElementById("btnOff");
                const btnOn = document.getElementById("btnOn");
                const cover = document.getElementById("safetyCover");
                const trigger = document.getElementById("armedTrigger");
                const triggerLabel = document.getElementById("triggerLabel");
                const coverLabel = document.getElementById("coverLabel");

                btnOff.classList.remove("active");
                btnOn.classList.remove("active");
                trigger.classList.remove("live-armed");
                badge.className = "status-pill";

                if (mode === "OFF") {
                    badge.textContent = "OFF / SAFE";
                    badge.classList.add("pill-off");
                    btnOff.classList.add("active");

                    cover.classList.add("disabled");
                    cover.classList.remove("flipped");
                    coverLabel.textContent = "OFF / SAFE";
                    triggerLabel.textContent = "⚠️ CONFIRM LIVE FIRE";
                    snapCoverClosed();
                } else if (mode === "ON") {
                    badge.textContent = "TRACKING (SAFE)";
                    badge.classList.add("pill-on");
                    btnOn.classList.add("active");

                    cover.classList.remove("disabled");
                    coverLabel.textContent = "LIFT TO ARM";
                    triggerLabel.textContent = "⚠️ CONFIRM LIVE FIRE";
                    // If coming out of ARMED, close the cover and restore the yellow hatched safety latch
                    if (coverOpen || cover.classList.contains("flipped")) {
                        snapCoverClosed();
                    }
                } else if (mode === "ARMED") {
                    badge.textContent = "ARMED / LIVE";
                    badge.classList.add("pill-armed");

                    cover.classList.add("flipped");
                    trigger.classList.add("live-armed");
                    triggerLabel.textContent = "🔥 LIVE FIRE (CLICK TO SAFE)";
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
                        updateUIState(data.mode);
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
                    if (data.status === "ok" && data.mode !== currentMode) {
                        updateUIState(data.mode);
                    }
                })
                .catch(() => {});
            }

            setInterval(syncMode, 1000);

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
        while True:
            with frame_lock:
                frame = latest_jpeg

            if frame is not None:
                yield STREAM_PART_HEADER
                yield frame
                yield STREAM_PART_FOOTER

            time.sleep(0.033)
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

if __name__ == "__main__":
    t = threading.Thread(target=vision_thread, daemon=True, name="VisionThread")
    t.start()
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)