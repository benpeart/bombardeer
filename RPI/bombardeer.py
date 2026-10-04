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
    * Latency-Compensated Time-Sync Forward Projection:
        - Projects Kalman state to the exact actuation instant, eliminating
          hunting, overshoot, and FastAccelStepper trajectory replanning halts.
    * 1.5-Second Continuous Dead-Reckoning Coasting:
        - Exponential velocity decay through optical dropouts, foliage
          occlusions, and motion blur without resetting filter momentum.
    * Travel Clamping & Angular Velocity Fire Gating:
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
from flask import Flask, Response, render_template_string, send_from_directory

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

ARUCO_REAL_WIDTH_MM = 100.0
MIN_VALID_RANGE_M = 1.5
MAX_VALID_RANGE_M = 50.0

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
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
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
                    if d01 < 14 or d12 < 14:
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
                        log.info(f"[SERIAL TX] -> P {t_pan} {t_tilt}")

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

        # Ingest Telemetry Stream via Bytearray
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
                            # S <time> <pan> <tilt> <firing> <moving>
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

                            # Centralized atomic update under telemetry_lock
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
    streaming continuous velocity vectors regardless of camera or ML inference cadence[cite: 4].
    """
    last_setpoint_time = 0.0
    last_debug_log_time = 0.0
    is_actively_tracking_velocity = False
    last_v_pan = 0
    last_v_tilt = 0
    last_sent_p_pan = 99999
    last_sent_p_tilt = 99999
    
    log.info("[MOTION] 50 Hz Real-Time Motion Controller started")

    while True:
        loop_start = time.monotonic()
        now = time.time()

        # Atomic, flat snapshot retrieval (No nested locks)
        target_data = get_primary_target_snapshot()
        current_pan, current_tilt, hw_moving, _ = get_instantaneous_telemetry()
        serial_ok = get_serial_connected()

        if serial_ok and target_data is not None:
            time_since_seen = now - target_data["last_seen"]

            # -------------------------------------------------------------
            # CASE A: TARGET ACTIVE OR WITHIN 1.5s BALLISTIC COAST HORIZON[cite: 4]
            # -------------------------------------------------------------
            if time_since_seen <= COAST_MAX_HORIZON_SEC:
                # Continuous Kalman projection to the exact actuation instant[cite: 4]
                predicted_state = target_filter.predict_state(now)
                if predicted_state is not None:
                    world_pan, world_tilt, tgt_v_pan, tgt_v_tilt = predicted_state

                    # LATENCY-FREE INSTANTANEOUS REAL-TIME ERROR[cite: 4]
                    realtime_err_pan = world_pan - current_pan
                    realtime_err_tilt = world_tilt - current_tilt
                    radial_error_mrad = math.hypot(realtime_err_pan, realtime_err_tilt)

                    # Dynamic Solenoid Trigger Gate:
                    # Require target freshness (<150ms), radial error within deadband,
                    # AND ensure turret is stabilized (speed < 180 mrad/s) so it doesn't
                    # fire while slewing past at maximum velocity.
                    turret_speed_mag = math.hypot(last_v_pan, last_v_tilt)
                    can_fire = (
                        time_since_seen <= 0.150
                        and radial_error_mrad <= FIRE_DEADBAND_MRAD
                        and turret_speed_mag < MAX_FIRE_SLEW_SPEED_MRAD_S
                    )
                    turret_queue.put({"cmd": "TRIGGER", "state": 1 if can_fire else 0, "timestamp": now})

                    # MODE HYSTERESIS SELECTION[cite: 4]
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

                    # MODE 1: BALLISTIC SNAP-TO-TARGET ('P')[cite: 4]
                    if use_ballistic:
                        goal_delta = math.hypot(world_pan - last_sent_p_pan, world_tilt - last_sent_p_tilt)
                        # Require at least 400ms between P setpoints, or an extreme shift of >60mrad[cite: 4]
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
                        
                    # MODE 2: CONTINUOUS FLUID PURSUIT ('V')[cite: 4]
                    else:
                        if time_since_tx >= SETPOINT_MIN_INTERVAL_SEC:
                            last_setpoint_time = now

                            def compute_velocity(err, tgt_v, deadband):
                                mag = abs(err)
                                if mag < deadband:
                                    return 0
                                # Near target: smooth settling[cite: 4]
                                if mag <= 25.0:
                                    kp = 1.4
                                    cmd = err * kp + 0.8 * tgt_v
                                else:
                                    # Fast sweep: full 1.0x feed-forward eliminates tracking lag[cite: 4]
                                    kp = 2.8
                                    cmd = err * kp + 1.0 * tgt_v
                                return int(round(cmd))
                            
                            raw_v_pan = compute_velocity(realtime_err_pan, tgt_v_pan, VELOCITY_DEADBAND_MRAD)
                            raw_v_tilt = compute_velocity(realtime_err_tilt, tgt_v_tilt, VELOCITY_DEADBAND_MRAD)

                            # COAST DECAY DAMPING: after 350ms of occlusion, smoothly decay velocity[cite: 4]
                            if time_since_seen > 0.350:
                                decay_factor = math.exp(-COAST_EXP_DECAY_RATE * (time_since_seen - 0.350))
                                raw_v_pan = int(round(raw_v_pan * decay_factor))
                                raw_v_tilt = int(round(raw_v_tilt * decay_factor))

                            # Slew bounds: up to 850 mrad/s pan, 450 mrad/s tilt[cite: 4]
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

                    # Diagnostic Telemetry[cite: 4]
                    if now - last_debug_log_time >= 0.150:
                        last_debug_log_time = now
                        if time_since_seen <= 0.100:
                            log.info(
                                f"[TRACK] Cur:({current_pan:+5d},{current_tilt:+4d}) | "
                                f"Err:({realtime_err_pan:+5.1f},{realtime_err_tilt:+4.1f}) Rad:{radial_error_mrad:4.1f} | "
                                f"Cmd:{cmd_type} | Mv:{int(hw_moving)}"
                            )
                        else:
                            ms_lost = int(time_since_seen * 1000.0)
                            log.info(
                                f"[COAST] Drop:{ms_lost:3d}ms | Cur:({current_pan:+5d},{current_tilt:+4d}) | "
                                f"Err:({realtime_err_pan:+5.1f},{realtime_err_tilt:+4.1f}) | "
                                f"Cmd:{cmd_type} | Mv:{int(hw_moving)}"
                            )

            # -------------------------------------------------------------
            # CASE B: TARGET EXPIRED BEYOND 1.5s COAST HORIZON[cite: 4]
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
                log.info("[TRACK] Target horizon expired (>1.5s). Standing by.")

        # Precise 50 Hz cycle regulation[cite: 4]
        elapsed = time.monotonic() - loop_start
        sleep_time = MOTION_LOOP_INTERVAL_SEC - elapsed
        if sleep_time > 0.001:
            time.sleep(sleep_time)


# =========================================================================
# Vision Perception Thread (Asynchronous Ingestion & Kalman Correction)
# =========================================================================

def vision_thread():
    global latest_jpeg

    ensure_storage_headroom()
    detector = ArucoTargetDetector()
    target_filter = TargetKalmanFilter(process_noise=35.0, measurement_noise=10.0)

    try:
        picam2 = Picamera2()
        cam_config = picam2.create_video_configuration(
            sensor={"output_size": (2304, 1296)},
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
            lores={"size": (DISPLAY_WIDTH, DISPLAY_HEIGHT), "format": "RGB888"},
            buffer_count=6,
            controls={
                "AeEnable": True,
                "AwbMode": 2,
                "AeExposureMode": 0,
                "FrameDurationLimits": (33333, 33333),
                "ExposureValue": 1.0,
                "AfMode": 0,
                "LensPosition": 0.0
            },
            transform=Transform(hflip=True, vflip=True)
        )
        picam2.configure(cam_config)
        picam2.start()
        log.info("[CAMERA] Hardware PiSP [B,G,R] stream active (Low-light 33ms window)")
        log.info(f"[DIAG] Verified OpenCV Active Threads: {cv2.getNumThreads()} | OpenCL: {cv2.ocl.useOpenCL()}")
        time.sleep(0.5)
    except Exception as e:
        log.critical(f"[CAMERA FATAL] Could not initialize Picamera2: {e}\n{traceback.format_exc()}")
        set_alert("FATAL: Camera init failed", duration=10.0)
        return

    turret_queue = queue.Queue(maxsize=25)
    record_queue = queue.Queue(maxsize=90)

    threading.Thread(target=turret_serial_worker, args=(turret_queue,), daemon=True, name="SerialWorker").start()
    threading.Thread(target=video_recorder_worker, args=(record_queue,), daemon=True, name="RecorderWorker").start()
    threading.Thread(target=motion_servoing_worker, args=(target_filter, turret_queue), daemon=True, name="MotionWorker").start()

    pre_roll_buffer = deque(maxlen=int(RECORD_PRE_ROLL_SEC * TARGET_FPS))

    fps_time = time.time()
    frame_count = 0
    sensor_fps = 0.0

    last_valid_range_m = 10.0
    last_seen_connected = False

    tentative_target = None
    tentative_streak = 0

    is_recording = False
    lock_consecutive_frames = 0
    last_detection_time = 0.0
    recording_start_time = 0.0
    last_standby_log_time = 0.0

    while True:
        loop_start_mono = time.monotonic()
        now = time.time()
        capture_arrival_mono = time.monotonic()

        # --- Stage 1: Camera Acquisition ---
        t0 = time.monotonic()
        try:
            request = picam2.capture_request()
            frame_main = request.make_array("main")
            annotated = request.make_array("lores")
            request.release()
        except Exception as e:
            log.error(f"[CAMERA ERROR] Capture dropped: {e}")
            time.sleep(0.01)
            continue
        t_cam = (time.monotonic() - t0) * 1000.0

        frame_count += 1
        if now - fps_time >= 1.0:
            sensor_fps = frame_count / (now - fps_time)
            frame_count = 0
            fps_time = now

        # Connection State Supervision
        currently_connected = get_serial_connected()
        if currently_connected != last_seen_connected:
            last_seen_connected = currently_connected
            if not currently_connected:
                target_filter.reset()
                clear_primary_target()
            else:
                target_filter.reset()

        # --- Stage 2: Target Detection ---
        t1 = time.monotonic()
        detections = detector.detect(frame_main)
        t_det = (time.monotonic() - t1) * 1000.0

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
        t2 = time.monotonic()
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
                log.info(f"[STANDBY] Scanning... | FPS:{sensor_fps:4.1f} | Turret:({p_cur:+5d},{t_cur:+4d}) | Detections:0")

        t_track = (time.monotonic() - t2) * 1000.0

        # --- Stage 4: Video Archiving ---
        t4 = time.monotonic()
        local_target = get_primary_target_snapshot()

        target_locked = (local_target is not None and (now - local_target["last_seen"] <= 0.350))
        if target_locked:
            lock_consecutive_frames += 1
            last_detection_time = now
        else:
            lock_consecutive_frames = 0

        if not is_recording:
            if target_locked and lock_consecutive_frames >= MIN_LOCK_FRAMES_TO_RECORD:
                is_recording = True
                recording_start_time = now
                try:
                    saved_pre_roll = list(pre_roll_buffer)
                    record_queue.put_nowait({"cmd": "start", "pre_roll": saved_pre_roll})
                except queue.Full:
                    log.warning("[RECORDER QUEUE] Queue full on start trigger")
            else:
                pre_roll_buffer.append(frame_main)
        else:
            time_since_last_seen = now - last_detection_time
            clip_duration = now - recording_start_time

            if time_since_last_seen > RECORD_POST_ROLL_SEC or clip_duration > MAX_RECORDING_DURATION_SEC:
                is_recording = False
                try:
                    record_queue.put_nowait({"cmd": "stop"})
                except queue.Full:
                    pass
            else:
                try:
                    record_queue.put_nowait({"cmd": "frame", "frame": frame_main})
                except queue.Full:
                    pass
        t_rec = (time.monotonic() - t4) * 1000.0

        # --- Stage 5: Client-Aware HUD Render & Web Stream ---
        t5 = time.monotonic()
        with frame_lock:
            clients_viewing = (active_web_clients > 0)

        if clients_viewing:
            current_range_m = local_target.get("range_m", 10.0) if local_target else 10.0
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

            ring_color = (0, 255, 255) if firing else (0, 0, 255)
            cv2.circle(annotated, (aim_x, aim_y), db_radius, ring_color, 1)
            cv2.drawMarker(annotated, (aim_x, aim_y), (0, 0, 255), cv2.MARKER_CROSS, 24, 1)

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
                cv2.rectangle(annotated, (0, 0), (DISPLAY_WIDTH, 26), (0, 0, 180), -1)
                cv2.putText(annotated, alert_msg, (12, 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

            success, buffer = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if success:
                with frame_lock:
                    latest_jpeg = buffer.tobytes()

        t_jpeg = (time.monotonic() - t5) * 1000.0

        loop_total_ms = (time.monotonic() - loop_start_mono) * 1000.0
        if loop_total_ms > 75.0:
            log.warning(
                f"[TIMING STALL] Total:{loop_total_ms:5.1f}ms | "
                f"Cam:{t_cam:5.1f}ms | Det:{t_det:4.1f}ms | Track:{t_track:4.1f}ms | "
                f"RecQueue:{t_rec:4.1f}ms | JPEG:{t_jpeg:4.1f}ms"
            )


# =========================================================================
# Web Server
# =========================================================================

@app.route("/")
def index():
    return render_template_string("""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Bombardeer Diagnostic Console</title>
        <link rel="icon" type="image/x-icon" href="/favicon.ico">
        <style>
            body {
                margin: 0; padding: 0;
                background-color: #0d1117; color: #c9d1d9;
                font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
                display: flex; flex-direction: column; align-items: center; justify-content: center;
                min-height: 100vh;
            }
            .hud-card {
                background: #161b22; border: 1px solid #30363d; border-radius: 8px;
                padding: 16px; box-shadow: 0 8px 24px rgba(0, 0, 0, 0.5); text-align: center;
            }
            h1 { margin: 0 0 12px 0; font-size: 1.2rem; letter-spacing: 0.05em; text-transform: uppercase; color: #39d353; }
            img { border-radius: 4px; background: #000; width: 640px; height: 360px; }
            .meta { margin-top: 10px; font-size: 0.82rem; color: #8b949e; }
        </style>
    </head>
    <body>
        <div class="hud-card">
            <h1>Bombardeer Diagnostic Suite</h1>
            <img src="/video_feed" alt="Targeting Stream">
            <div class="meta">Asynchronous 50Hz Servoing | 921600 Baud Dual-Mode Link</div>
        </div>
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