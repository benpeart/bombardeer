#!/usr/bin/env python3
"""
Bombardeer Turret: Fault-Tolerant Waypoint Servoing Controller
--------------------------------------------------------------
- High-Speed 921600 Baud UART Synchronization:
    * Perfectly matched to ESP32 Serial.begin(921600) for sub-millisecond latency.
    * Startup boot-settle discards initial boot noise without triggering false resets.
    * 2.5s watchdog timeout protects against real disconnections while tolerating boot delays.
- Pinhole Ray Projection (math.atan2):
    * True trigonometric mapping eliminates 60+ mrad wide-angle edge error.
- World-Space Kalman Filter:
    * Smooths high-frequency optical pixel noise while preserving responsiveness.
- Continuous 20 Hz Setpoint Streaming:
    * Avoids startup threshold deadlocks; streams setpoints cleanly outside deadband.
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
from collections import deque
import numpy as np
import cv2
import serial
from flask import Flask, Response, render_template_string, send_from_directory

from picamera2 import Picamera2
from libcamera import Transform

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

STATIC_DIR = "/home/ben/Bombardeer/static"
RECORDINGS_DIR = "/home/ben/Bombardeer/recordings"

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
CAMERA_OFFSET_X_MM = -170.0  # mm (Camera 170mm Left of Bore)
CAMERA_OFFSET_Y_MM = -60.0   # mm (Camera 60mm Below Bore)

# Camera Optical Intrinsics
FOCAL_LENGTH_PX = 540.4
MRAD_PER_PIXEL_X = 1000.0 / FOCAL_LENGTH_PX
MRAD_PER_PIXEL_Y = 1000.0 / FOCAL_LENGTH_PX

ARUCO_REAL_WIDTH_MM = 100.0
MIN_VALID_RANGE_M = 1.5
MAX_VALID_RANGE_M = 50.0

# Targeting & Deadbands
DEADBAND_MRAD = 30.0              # Inner deadband: target considered centered
FIRE_DEADBAND_MRAD = 40.0         # Allow firing inside this radius
SETPOINT_MIN_INTERVAL_SEC = 0.050 # Limit 'P' updates to 20 Hz to match ESP32 loop

MAX_TRIGGER_DURATION_SEC = 2.0
TRIGGER_COOLDOWN_SEC = 1.5

TARGET_MARKER_ID = 0
CONFIRMATION_FRAMES = 2

DEFAULT_SERIAL_PORTS = ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1"]
BAUD_RATE = 921600                 # Matches ESP32 setup()
SERIAL_HEARTBEAT_TIMEOUT_SEC = 2.5 # Allows safe headroom for bootloader and setup()

TARGET_FPS = 30.0
RECORD_PRE_ROLL_SEC = 2.0
RECORD_POST_ROLL_SEC = 3.0
MIN_LOCK_FRAMES_TO_RECORD = 3
MAX_RECORDING_DURATION_SEC = 30.0

MIN_FREE_SPACE_GB = 5.0
MAX_RECORDINGS_STORAGE_GB = 10.0

app = Flask(__name__)
frame_lock = threading.Lock()
latest_jpeg = None

state_lock = threading.Lock()
active_detections = []
primary_target = None
turret_pan_mrad = 0
turret_tilt_mrad = 0
is_turret_moving = False
is_firing = False
free_disk_gb = 0.0

telemetry_history = deque(maxlen=60)

system_health = {
    "serial_connected": False,
    "serial_tx_count": 0,
    "serial_rx_count": 0,
    "recorder_active": False,
    "recorder_error": None,
    "last_alert": None,
    "alert_time": 0.0
}


def set_alert(message, duration=3.0):
    with state_lock:
        system_health["last_alert"] = message
        system_health["alert_time"] = time.time() + duration
    log.warning(f"[ALERT] {message}")


def clear_queue(q):
    """Purges all pending commands from a queue to prevent backlogs on reconnect."""
    try:
        while not q.empty():
            q.get_nowait()
    except Exception:
        pass


def get_turret_pos_at_time(target_mono_sec):
    with state_lock:
        if not telemetry_history:
            return turret_pan_mrad, turret_tilt_mrad

        if target_mono_sec >= telemetry_history[-1][0]:
            return telemetry_history[-1][1], telemetry_history[-1][2]

        if target_mono_sec <= telemetry_history[0][0]:
            return telemetry_history[0][1], telemetry_history[0][2]

        for i in range(len(telemetry_history) - 1):
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

        return turret_pan_mrad, turret_tilt_mrad


# =========================================================================
# World-Space Kalman Filter
# =========================================================================

class TargetKalmanFilter:
    def __init__(self, process_noise=30.0, measurement_noise=10.0):
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
        self.is_initialized = False
        self.last_update_time = None

    def update(self, measured_pan, measured_tilt, current_time):
        if not self.is_initialized:
            self.kf.statePost = np.array([
                [float(measured_pan)],
                [float(measured_tilt)],
                [0.0],
                [0.0]
            ], dtype=np.float32)
            self.last_update_time = current_time
            self.is_initialized = True
            return int(measured_pan), int(measured_tilt)

        dt = max(0.001, min(0.2, current_time - self.last_update_time))
        self.last_update_time = current_time

        self.kf.transitionMatrix[0, 2] = dt
        self.kf.transitionMatrix[1, 3] = dt

        self.kf.predict()
        measurement = np.array([[float(measured_pan)], [float(measured_tilt)]], dtype=np.float32)
        estimated = self.kf.correct(measurement)

        return int(round(float(estimated[0, 0]))), int(round(float(estimated[1, 0])))


# =========================================================================
# Perception Layer
# =========================================================================

class ArucoTargetDetector:
    def __init__(self, target_id=TARGET_MARKER_ID):
        self.target_id = target_id
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.errorCorrectionRate = 0.55
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.clahe = cv2.createCLAHE(clipLimit=2.2, tileGridSize=(8, 8))

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
# Robust Serial Worker Thread with Heartbeat Supervision
# =========================================================================

def find_working_serial_port():
    for port in DEFAULT_SERIAL_PORTS:
        if os.path.exists(port):
            try:
                ser = serial.Serial(port, BAUD_RATE, timeout=0.05)
                ser.close()
                return port
            except (OSError, serial.SerialException):
                continue
    return None

def turret_serial_worker(cmd_queue):
    global turret_pan_mrad, turret_tilt_mrad, is_turret_moving, is_firing

    ser = None
    active_port = None
    reconnect_delay = 2.0
    next_reconnect_time = 0.0

    trigger_state = 0
    trigger_start_time = 0.0
    cooldown_until = 0.0

    rx_buffer = ""
    last_telemetry_rx_time = 0.0

    while True:
        now = time.time()
        mono_now = time.monotonic()

        # Connect only if not already open
        if ser is None or not ser.is_open:
            with state_lock:
                system_health["serial_connected"] = False
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
                        # Suppress DTR/RTS to avoid holding ESP32 in reset
                        try:
                            ser.dtr = False
                            ser.rts = False
                        except Exception:
                            pass

                        rx_buffer = ""
                        last_telemetry_rx_time = mono_now

                        # Brief settle window
                        time.sleep(0.5)
                        ser.reset_input_buffer()
                        ser.reset_output_buffer()

                        with state_lock:
                            system_health["serial_connected"] = True
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

        # Soft Watchdog: Mark status disconnected, but DO NOT close/reset the port!
        if ser and ser.is_open:
            is_active = (mono_now - last_telemetry_rx_time <= 2.0)
            with state_lock:
                system_health["serial_connected"] = is_active

        # Process outgoing command queue
        try:
            while not cmd_queue.empty():
                item = cmd_queue.get_nowait()
                cmd = item.get("cmd")

                if ser and ser.is_open:
                    if cmd == "SETPOINT":
                        t_pan = int(item["pan"])
                        t_tilt = int(item["tilt"])
                        ser.write(f"P {t_pan} {t_tilt}\n".encode())
                        with state_lock:
                            system_health["serial_tx_count"] += 1
                        log.info(f"[SERIAL TX] -> P {t_pan} {t_tilt}")

                    elif cmd == "HALT":
                        ser.write(b"X\n")
                        with state_lock:
                            system_health["serial_tx_count"] += 1

                    elif cmd == "TRIGGER":
                        req_state = item["state"]
                        if req_state == 1:
                            if trigger_state == 0 and now >= cooldown_until:
                                trigger_state = 1
                                trigger_start_time = now
                                with state_lock:
                                    is_firing = True
                                ser.write(b"T 1\n")
                                with state_lock:
                                    system_health["serial_tx_count"] += 1
                                log.info("[SERIAL] Solenoid trigger pulse FIRED")
                        elif req_state == 0:
                            if trigger_state == 1:
                                trigger_state = 0
                                with state_lock:
                                    is_firing = False
                                cooldown_until = now + TRIGGER_COOLDOWN_SEC
                                ser.write(b"T 0\n")
                                with state_lock:
                                    system_health["serial_tx_count"] += 1
        except Exception as e:
            log.warning(f"[SERIAL TX FAULT] {e}")

        # Solenoid Safety Cutoff
        if trigger_state == 1 and (now - trigger_start_time >= MAX_TRIGGER_DURATION_SEC):
            trigger_state = 0
            with state_lock:
                is_firing = False
            cooldown_until = now + TRIGGER_COOLDOWN_SEC
            if ser and ser.is_open:
                try:
                    ser.write(b"T 0\n")
                except Exception:
                    pass

        # Ingest Telemetry Stream (Non-destructive)
        if ser and ser.is_open:
            try:
                bytes_avail = ser.in_waiting
                if bytes_avail > 0:
                    raw_chunk = ser.read(bytes_avail).decode(errors="ignore")
                    rx_buffer += raw_chunk

                    while "\n" in rx_buffer:
                        line, rx_buffer = rx_buffer.split("\n", 1)
                        line = line.strip()
                        if not line or not line.startswith("S "):
                            continue

                        rx_mono_time = time.monotonic()
                        parts = line.split()

                        try:
                            # Matches "S <time> <pan> <tilt> <firing> <moving>"
                            if len(parts) >= 6:
                                p_mrad = int(parts[2])
                                t_mrad = int(parts[3])
                                moving = (int(parts[5]) == 1)
                            elif len(parts) >= 5:
                                p_mrad = int(parts[1])
                                t_mrad = int(parts[2])
                                moving = (int(parts[4]) == 1)
                            else:
                                continue

                            last_telemetry_rx_time = rx_mono_time

                            with state_lock:
                                turret_pan_mrad = p_mrad
                                turret_tilt_mrad = t_mrad
                                is_turret_moving = moving
                                system_health["serial_rx_count"] += 1
                                telemetry_history.append((rx_mono_time, p_mrad, t_mrad))

                        except ValueError:
                            continue

            except (serial.SerialException, OSError) as e:
                log.warning(f"[SERIAL HARDWARE ERROR] {e}")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                with state_lock:
                    system_health["serial_connected"] = False

        time.sleep(0.002)

# =========================================================================
# Video Storage Worker
# =========================================================================

def ensure_storage_headroom():
    global free_disk_gb
    try:
        _, _, free = shutil.disk_usage(RECORDINGS_DIR)
        free_disk_gb = free / (1024 ** 3)

        if free_disk_gb < MIN_FREE_SPACE_GB:
            set_alert(f"LOW STORAGE: {free_disk_gb:.1f} GB Free", duration=4.0)

        files = glob.glob(os.path.join(RECORDINGS_DIR, "test_*.mp4"))
        if not files:
            return

        files.sort(key=os.path.getmtime)
        archive_gb = sum(os.path.getsize(f) for f in files) / (1024 ** 3)

        for fpath in files:
            if free_disk_gb >= MIN_FREE_SPACE_GB and archive_gb <= MAX_RECORDINGS_STORAGE_GB:
                break
            size = os.path.getsize(fpath)
            try:
                os.remove(fpath)
                log.info(f"[STORAGE PURGE] Cleared old recording: {fpath}")
                archive_gb -= size / (1024 ** 3)
                _, _, updated_free = shutil.disk_usage(RECORDINGS_DIR)
                free_disk_gb = updated_free / (1024 ** 3)
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
                ensure_storage_headroom()
                ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                current_filepath = os.path.join(RECORDINGS_DIR, f"test_{ts}.mp4")
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
                    with state_lock:
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
                    with state_lock:
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
                    with state_lock:
                        system_health["recorder_error"] = str(e)

        elif cmd == "stop" and recording_active:
            recording_active = False
            with state_lock:
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
# Vision & Servoing Loop
# =========================================================================

def vision_thread():
    global latest_jpeg, primary_target, active_detections

    ensure_storage_headroom()
    detector = ArucoTargetDetector()
    target_filter = TargetKalmanFilter(process_noise=30.0, measurement_noise=10.0)

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
        time.sleep(0.5)
    except Exception as e:
        log.critical(f"[CAMERA FATAL] Could not initialize Picamera2: {e}\n{traceback.format_exc()}")
        set_alert("FATAL: Camera init failed", duration=10.0)
        return

    turret_queue = queue.Queue(maxsize=25)
    record_queue = queue.Queue(maxsize=90)

    threading.Thread(target=turret_serial_worker, args=(turret_queue,), daemon=True, name="SerialWorker").start()
    threading.Thread(target=video_recorder_worker, args=(record_queue,), daemon=True, name="RecorderWorker").start()

    pre_roll_buffer = deque(maxlen=int(RECORD_PRE_ROLL_SEC * TARGET_FPS))

    fps_time = time.time()
    frame_count = 0
    sensor_fps = 0.0

    last_setpoint_time = 0.0
    last_debug_log_time = 0.0
    last_valid_range_m = 10.0
    last_seen_connected = False

    tentative_target = None
    tentative_streak = 0

    is_recording = False
    lock_consecutive_frames = 0
    last_detection_time = 0.0
    recording_start_time = 0.0

    while True:
        now = time.time()
        capture_arrival_mono = time.monotonic()

        try:
            request = picam2.capture_request()
            frame_main = request.make_array("main")
            annotated = request.make_array("lores")
            request.release()
        except Exception as e:
            log.error(f"[CAMERA ERROR] Capture dropped: {e}")
            time.sleep(0.01)
            continue

        frame_count += 1
        if now - fps_time >= 1.0:
            sensor_fps = frame_count / (now - fps_time)
            frame_count = 0
            fps_time = now

        # Connection State Change Handling
        with state_lock:
            currently_connected = system_health["serial_connected"]

        if currently_connected != last_seen_connected:
            last_seen_connected = currently_connected
            if not currently_connected:
                target_filter.reset()
                primary_target = None
            else:
                target_filter.reset()

        detections = detector.detect(frame_main)
        best_candidate = detections[0] if len(detections) > 0 else None

        verified_detection = None
        if best_candidate is not None:
            if primary_target is not None:
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

        with state_lock:
            active_detections = detections

            if verified_detection is not None:
                primary_target = {
                    "center": verified_detection["center"],
                    "box": verified_detection["box"],
                    "score": verified_detection["score"],
                    "area": verified_detection["area"],
                    "range_m": verified_detection.get("range_m", 10.0),
                    "last_seen": now,
                    "id": verified_detection.get("id", 0)
                }
            elif primary_target and (now - primary_target["last_seen"] >= 0.40):
                primary_target = None

            local_target = dict(primary_target) if primary_target else None

        target_locked = False
        target_in_fire_zone = False
        current_range_m = last_valid_range_m

        # =================================================================
        # WAYPOINT SERVOING ('P')
        # =================================================================
        if verified_detection is not None:
            target_locked = True
            raw_cx, raw_cy = verified_detection["center"]

            if "range_m" in verified_detection:
                last_valid_range_m = verified_detection["range_m"]
                current_range_m = last_valid_range_m

            # Look up historical position matching capture instant (~40ms pipeline latency)
            pan_at_capture, tilt_at_capture = get_turret_pos_at_time(capture_arrival_mono - 0.040)

            # Optical parallax corrections
            parallax_pan_mrad = (-CAMERA_OFFSET_X_MM / (current_range_m * 1000.0)) * 1000.0
            parallax_tilt_mrad = (CAMERA_OFFSET_Y_MM / (current_range_m * 1000.0)) * 1000.0

            # TRUE PINHOLE PROJECTION
            dx = -(raw_cx - OPTICAL_CENTER[0])
            dy = -(raw_cy - OPTICAL_CENTER[1])
            err_pan_mrad = math.atan2(dx, FOCAL_LENGTH_PX) * 1000.0 + parallax_pan_mrad
            err_tilt_mrad = math.atan2(dy, FOCAL_LENGTH_PX) * 1000.0 + parallax_tilt_mrad
            radial_error_mrad = math.hypot(err_pan_mrad, err_tilt_mrad)

            # Raw global coordinate
            raw_goal_pan = pan_at_capture + int(round(err_pan_mrad))
            raw_goal_tilt = tilt_at_capture + int(round(err_tilt_mrad))

            # KALMAN FILTER: smooths optical noise
            goal_pan_mrad, goal_tilt_mrad = target_filter.update(raw_goal_pan, raw_goal_tilt, now)

            # Solenoid Trigger Check
            if radial_error_mrad <= FIRE_DEADBAND_MRAD:
                target_in_fire_zone = True
                turret_queue.put({"cmd": "TRIGGER", "state": 1})
            else:
                turret_queue.put({"cmd": "TRIGGER", "state": 0})

            # Continuous 20 Hz setpoint streaming whenever outside deadband
            time_since_setpoint = now - last_setpoint_time
            if (radial_error_mrad > DEADBAND_MRAD) and (time_since_setpoint >= SETPOINT_MIN_INTERVAL_SEC):
                last_setpoint_time = now
                turret_queue.put({
                    "cmd": "SETPOINT",
                    "pan": goal_pan_mrad,
                    "tilt": goal_tilt_mrad
                })

            # Real-Time Telemetry Logging
            if now - last_debug_log_time >= 0.150:
                last_debug_log_time = now
                with state_lock:
                    tp = turret_pan_mrad
                    tt = turret_tilt_mrad
                    hw_mov = is_turret_moving
                log.info(
                    f"[TRACK] Cur:({tp:+5d},{tt:+4d}) | "
                    f"Err:({err_pan_mrad:+5.1f},{err_tilt_mrad:+4.1f}) Rad:{radial_error_mrad:4.1f} | "
                    f"Goal:({goal_pan_mrad:+5d},{goal_tilt_mrad:+4d}) | Mv:{int(hw_mov)}"
                )

        elif local_target and (now - local_target["last_seen"] < 0.35):
            target_locked = True
            turret_queue.put({"cmd": "TRIGGER", "state": 0})
        else:
            turret_queue.put({"cmd": "TRIGGER", "state": 0})
            target_filter.reset()

        # Video Archiving
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
                    saved_pre_roll = [f.copy() for f in pre_roll_buffer]
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
                    record_queue.put_nowait({"cmd": "frame", "frame": frame_main.copy()})
                except queue.Full:
                    pass

        # -----------------------------------------------------------------
        # HUD Render
        # -----------------------------------------------------------------
        oc_x = int(OPTICAL_CENTER[0] * SCALE_X)
        oc_y = int(OPTICAL_CENTER[1] * SCALE_Y)
        cv2.drawMarker(annotated, (oc_x, oc_y), (0, 255, 255), cv2.MARKER_CROSS, 16, 1)

        conv_px_x = OPTICAL_CENTER[0] + int((-CAMERA_OFFSET_X_MM / (current_range_m * 1000.0)) * FOCAL_LENGTH_PX)
        conv_px_y = OPTICAL_CENTER[1] + int((CAMERA_OFFSET_Y_MM / (current_range_m * 1000.0)) * FOCAL_LENGTH_PX)
        aim_x = int(conv_px_x * SCALE_X)
        aim_y = int(conv_px_y * SCALE_Y)
        db_radius = int((FIRE_DEADBAND_MRAD / MRAD_PER_PIXEL_X) * SCALE_X)

        ring_color = (0, 255, 255) if target_in_fire_zone else (0, 0, 255)
        cv2.circle(annotated, (aim_x, aim_y), db_radius, ring_color, 1)
        cv2.drawMarker(annotated, (aim_x, aim_y), (0, 0, 255), cv2.MARKER_CROSS, 24, 1)

        for det in detections:
            x1 = int(det["box"][0] * SCALE_X)
            y1 = int(det["box"][1] * SCALE_Y)
            x2 = int(det["box"][2] * SCALE_X)
            y2 = int(det["box"][3] * SCALE_Y)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (180, 180, 180), 1)

        if local_target and (now - local_target["last_seen"] < 0.35):
            tx1 = int(local_target["box"][0] * SCALE_X)
            ty1 = int(local_target["box"][1] * SCALE_Y)
            tx2 = int(local_target["box"][2] * SCALE_X)
            ty2 = int(local_target["box"][3] * SCALE_Y)
            tcx_d = int(local_target["center"][0] * SCALE_X)
            tcy_d = int(local_target["center"][1] * SCALE_Y)

            cv2.rectangle(annotated, (tx1, ty1), (tx2, ty2), (0, 255, 0), 2)
            cv2.circle(annotated, (tcx_d, tcy_d), 4, (0, 255, 0), -1)

            status_lbl = f"ID:{local_target.get('id', 0)} | {current_range_m:.1f}m"
            cv2.putText(annotated, status_lbl, (tx1, max(ty1 - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

        with state_lock:
            p_mrad = turret_pan_mrad
            t_mrad = turret_tilt_mrad
            firing = is_firing
            ser_ok = system_health["serial_connected"]
            rec_ok = system_health["recorder_active"]
            alert_msg = system_health["last_alert"] if now < system_health["alert_time"] else None

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
            <div class="meta">921600 Baud Link | Kalman Tracking | Pinhole Linearization</div>
        </div>
    </body>
    </html>
    """)


def generate_frames():
    while True:
        with frame_lock:
            if latest_jpeg is None:
                time.sleep(0.01)
                continue
            frame = latest_jpeg

        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(0.033)


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