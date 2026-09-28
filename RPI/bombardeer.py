#!/usr/bin/env python3
"""
Bombardeer Turret Hybrid Visual Servoing Controller
---------------------------------------------------
- Real-Time Predictive Visual Servoing Architecture:
    * Native milliradian (mrad) / (mrad/s) kinematic control loop.
    * Dynamic pinhole range estimation (ArUco test, BBox production).
    * Dynamic parallax compensation for Camera-Bore physical baseline.
    * Constant-Velocity Kalman Filter with Lead Horizon Prediction.
    * Dynamic Slew-Rate Limiter (Prevents torque spikes & step skips).
- High-Performance Vision Pipeline:
    * Native hardware [B,G,R] stream generation via PiSP ("RGB888" V4L2 format).
    * Dual-stream zero-copy hardware downscaling (1280x720 main, 640x360 lores).
    * Zero CPU color-space conversions: Web HUD and MP4 writer share native format.
"""

import os
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
RECORDINGS_DIR = "/home/ben/Bombardeer/test_recordings"

try:
    os.makedirs(RECORDINGS_DIR, exist_ok=True)
    test_touch = os.path.join(RECORDINGS_DIR, ".write_test")
    with open(test_touch, "w") as f:
        f.write("ok")
    os.remove(test_touch)
    log.info(f"[STORAGE] Storage directory verified writable: {RECORDINGS_DIR}")
except Exception as e:
    log.error(f"[STORAGE ERROR] Cannot write to storage path {RECORDINGS_DIR}: {e}")

# Native 16:9 Vision Geometry
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

# Hardware HUD Resolution (PiSP hardware-scaled)
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 360

SCALE_X = DISPLAY_WIDTH / FRAME_WIDTH
SCALE_Y = DISPLAY_HEIGHT / FRAME_HEIGHT

OPTICAL_CENTER = (FRAME_WIDTH // 2, FRAME_HEIGHT // 2)

# --- PHYSICAL PARALLAX OFFSETS (Camera relative to Barrel) ---
CAMERA_OFFSET_X_MM = -170.0  # mm (Camera 170mm Left of Bore)
CAMERA_OFFSET_Y_MM = -60.0   # mm (Camera 60mm Below Bore)

# Camera Optical Intrinsics (RPi Cam Module 3 Wide NoIR @ 1280x720 native crop)
# Calibrated: 100mm target at 2000mm produces 27.02 px -> f_px = 540.4 px
FOCAL_LENGTH_PX = 540.4
MRAD_PER_PIXEL_X = 1000.0 / FOCAL_LENGTH_PX  # ~1.8505 mrad/px
MRAD_PER_PIXEL_Y = 1000.0 / FOCAL_LENGTH_PX  # ~1.8505 mrad/px

ARUCO_REAL_WIDTH_MM = 100.0   # Test: Printed 100 mm marker edge
DEER_REAL_HEIGHT_MM = 1000.0  # Prod: Average standing deer shoulder height (~1.0 m)

MIN_VALID_RANGE_M = 2.0
MAX_VALID_RANGE_M = 50.0

# --- CONTROL GAINS (mrad/s per mrad error) ---
# Scaled down to match the calibrated ~1.85 mrad/px optical scale factor
KP_PAN = 1.6
KD_PAN = 0.12
KP_TILT = 1.4
KD_TILT = 0.10

# Physical Motor Limits (mrad/s)
MAX_PAN_SPEED = 2400   # Lowered ceiling during active servoing to prevent overshoot
MAX_TILT_SPEED = 500
MIN_RUN_SPEED = 20

# Slew-Rate Limiter (mrad/s per ~33ms frame)
MAX_ACCEL_PAN_PER_FRAME = 180
MAX_ACCEL_TILT_PER_FRAME = 50

# Disable forward lead prediction until stationary stability is verified
PREDICTION_LEAD_SEC = 0.0

# Derivative & Velocity Safety Clamps
MAX_ESTIMATED_VEL_PX_PER_SEC = 3500.0
MAX_DERIVATIVE_RATE = 2000.0

# Deadbands in Milliradians (~13.5 px at 1.85 mrad/px)
MOTION_DEADBAND_MRAD = 25.0
FIRE_DEADBAND_MRAD = 35.0
MAX_TRIGGER_DURATION_SEC = 2.0
TRIGGER_COOLDOWN_SEC = 1.5

TARGET_MARKER_ID = 0
CONFIRMATION_FRAMES = 2

DEFAULT_SERIAL_PORTS = ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1"]
BAUD_RATE = 115200

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
predicted_target = None
turret_pan_mrad = 0
turret_tilt_mrad = 0
is_firing = False
free_disk_gb = 0.0

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


# =========================================================================
# Predictive State Estimator: Constant Velocity Kalman Filter
# =========================================================================

class TargetKalmanFilter:
    def __init__(self):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)

        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        self.kf.processNoiseCov[2, 2] = 2.0
        self.kf.processNoiseCov[3, 3] = 2.0

        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1.5
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 10.0

        self.last_time = None
        self.initialized = False

    def reset(self):
        self.initialized = False
        self.last_time = None

    def update(self, z_x, z_y, current_time):
        try:
            if not self.initialized:
                self.kf.statePost = np.array([[z_x], [z_y], [0.0], [0.0]], dtype=np.float32)
                self.last_time = current_time
                self.initialized = True
                return z_x, z_y, 0.0, 0.0

            dt = max(0.001, min(0.20, current_time - self.last_time))
            self.last_time = current_time

            self.kf.transitionMatrix = np.array([
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1,  0],
                [0, 0, 0,  1]
            ], dtype=np.float32)

            self.kf.predict()
            measurement = np.array([[np.float32(z_x)], [np.float32(z_y)]])
            estimate = self.kf.correct(measurement)

            pos_x = float(estimate[0, 0])
            pos_y = float(estimate[1, 0])
            vel_x = float(np.clip(estimate[2, 0], -MAX_ESTIMATED_VEL_PX_PER_SEC, MAX_ESTIMATED_VEL_PX_PER_SEC))
            vel_y = float(np.clip(estimate[3, 0], -MAX_ESTIMATED_VEL_PX_PER_SEC, MAX_ESTIMATED_VEL_PX_PER_SEC))

            self.kf.statePost[2, 0] = vel_x
            self.kf.statePost[3, 0] = vel_y

            return pos_x, pos_y, vel_x, vel_y
        except Exception as e:
            log.error(f"[KALMAN ERROR] State update exception: {e}")
            return z_x, z_y, 0.0, 0.0

    def predict_future(self, lead_time_sec):
        if not self.initialized:
            return 0.0, 0.0
        state = self.kf.statePost
        pred_x = float(state[0, 0] + state[2, 0] * lead_time_sec)
        pred_y = float(state[1, 0] + state[3, 0] * lead_time_sec)
        return pred_x, pred_y


# =========================================================================
# Perception Layer: Hardened Low-Light ArUco Detector
# =========================================================================

class ArucoTargetDetector:
    def __init__(self, target_id=TARGET_MARKER_ID):
        self.target_id = target_id
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        
        self.parameters.errorCorrectionRate = 0.55
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.perspectiveRemovePixelPerCell = 8
        self.parameters.perspectiveRemoveIgnoredMarginPerCell = 0.18
        self.parameters.minMarkerPerimeterRate = 0.02
        self.parameters.maxMarkerPerimeterRate = 4.0
        
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

                    # Test Range Estimation from ArUco marker edge length in pixels
                    marker_px = max(1.0, (d01 + d12) / 2.0)
                    estimated_range_m = (ARUCO_REAL_WIDTH_MM * FOCAL_LENGTH_PX) / (marker_px * 1000.0)

                    # In Production: Replace with bounding-box height calculation for deer:
                    # bbox_h = max(1.0, float(y2 - y1))
                    # estimated_range_m = (DEER_REAL_HEIGHT_MM * FOCAL_LENGTH_PX) / (bbox_h * 1000.0)

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
# Serial Worker Thread with Auto-Reconnect & Heartbeat Watchdog
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
    global turret_pan_mrad, turret_tilt_mrad, is_firing

    ser = None
    active_port = None
    reconnect_delay = 2.0
    next_reconnect_time = 0.0

    last_pan_spd = 0
    last_tilt_spd = 0
    trigger_state = 0
    trigger_start_time = 0.0
    cooldown_until = 0.0

    while True:
        now = time.time()

        if ser is None or not ser.is_open:
            with state_lock:
                system_health["serial_connected"] = False

            if now >= next_reconnect_time:
                active_port = find_working_serial_port()
                if active_port:
                    try:
                        ser = serial.Serial(active_port, BAUD_RATE, timeout=0.02)
                        with state_lock:
                            system_health["serial_connected"] = True
                        log.info(f"[SERIAL] Successfully opened ESP32 link on {active_port}")
                        set_alert(f"Connected: {active_port}", duration=2.0)
                    except Exception as e:
                        ser = None
                        next_reconnect_time = now + reconnect_delay
                        set_alert(f"Serial Open Error: {e}", duration=2.0)
                        log.error(f"[SERIAL ERROR] Failed connecting to {active_port}: {e}")
                else:
                    next_reconnect_time = now + reconnect_delay

        try:
            while not cmd_queue.empty():
                item = cmd_queue.get_nowait()
                cmd = item.get("cmd")

                if cmd == "VELOCITY":
                    p_spd = item["pan_spd"]
                    t_spd = item["tilt_spd"]

                    if abs(p_spd - last_pan_spd) >= 15 or abs(t_spd - last_tilt_spd) >= 15:
                        last_pan_spd = p_spd
                        last_tilt_spd = t_spd
                        if ser and ser.is_open:
                            try:
                                ser.write(f"V {p_spd} {t_spd}\n".encode())
                                with state_lock:
                                    system_health["serial_tx_count"] += 1
                            except Exception as e:
                                log.error(f"[SERIAL TX ERROR] Write failed: {e}")
                                ser.close()
                                ser = None

                elif cmd == "TRIGGER":
                    req_state = item["state"]
                    if req_state == 1:
                        if trigger_state == 0 and now >= cooldown_until:
                            trigger_state = 1
                            trigger_start_time = now
                            with state_lock:
                                is_firing = True
                            if ser and ser.is_open:
                                try:
                                    ser.write(b"T 1\n")
                                    with state_lock:
                                        system_health["serial_tx_count"] += 1
                                    log.info("[SERIAL] Solenoid trigger pulse FIRED")
                                except Exception as e:
                                    log.error(f"[SERIAL TX ERROR] Trigger write failed: {e}")
                    elif req_state == 0:
                        if trigger_state == 1:
                            trigger_state = 0
                            with state_lock:
                                is_firing = False
                            cooldown_until = now + TRIGGER_COOLDOWN_SEC
                            if ser and ser.is_open:
                                try:
                                    ser.write(b"T 0\n")
                                    with state_lock:
                                        system_health["serial_tx_count"] += 1
                                except Exception as e:
                                    log.error(f"[SERIAL TX ERROR] Trigger release failed: {e}")
        except queue.Empty:
            pass

        if trigger_state == 1 and (now - trigger_start_time >= MAX_TRIGGER_DURATION_SEC):
            trigger_state = 0
            with state_lock:
                is_firing = False
            cooldown_until = now + TRIGGER_COOLDOWN_SEC
            if ser and ser.is_open:
                try:
                    ser.write(b"T 0\n")
                    log.warning("[SERIAL FAILSAFE] Max trigger duration exceeded. Disengaged.")
                except Exception as e:
                    log.error(f"[SERIAL ERROR] Failsafe write failed: {e}")

        if ser and ser.is_open and ser.in_waiting:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if line.startswith("S "):
                    parts = line.split()
                    if len(parts) >= 3:
                        with state_lock:
                            turret_pan_mrad = int(parts[1])
                            turret_tilt_mrad = int(parts[2])
                            system_health["serial_rx_count"] += 1
            except Exception as e:
                log.warning(f"[SERIAL RX ERROR] Malformed telemetry line: {e}")

        time.sleep(0.005)


# =========================================================================
# Video Storage Worker (Zero-Copy Native BGR Pipeline)
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

                    # Write pre-roll buffer directly with zero software color manipulation
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
# Predictive Visual Servoing Tracking Loop
# =========================================================================

def vision_thread():
    global latest_jpeg, primary_target, predicted_target, active_detections

    ensure_storage_headroom()
    detector = ArucoTargetDetector()
    tracker = TargetKalmanFilter()

    try:
        picam2 = Picamera2()
        # V4L2 "RGB888" outputs native [B, G, R] byte ordering directly via PiSP hardware!
        cam_config = picam2.create_video_configuration(
            sensor={"output_size": (2304, 1296)},
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
            lores={"size": (DISPLAY_WIDTH, DISPLAY_HEIGHT), "format": "RGB888"},
            buffer_count=6,
            controls={
                "AeEnable": True,
                "AwbMode": 2,
                "AeExposureMode": 0,
                "FrameDurationLimits": (33333, 33333),  # Lock to rock-solid 30 FPS
                "ExposureValue": 1.0
            },
            transform=Transform(hflip=True, vflip=True)
        )
        picam2.configure(cam_config)
        picam2.start()
        log.info("[CAMERA] Hardware PiSP [B,G,R] stream active via V4L2 RGB888 profile")
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

    last_err_pan_mrad = 0.0
    last_err_tilt_mrad = 0.0
    last_control_time = time.time()

    active_cmd_pan_spd = 0
    active_cmd_tilt_spd = 0
    last_valid_range_m = 10.0

    tentative_target = None
    tentative_streak = 0

    is_recording = False
    lock_consecutive_frames = 0
    last_detection_time = 0.0
    recording_start_time = 0.0

    while True:
        now = time.time()

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
                raw_cx, raw_cy = verified_detection["center"]
                filt_x, filt_y, vel_x, vel_y = tracker.update(raw_cx, raw_cy, now)
                pred_x, pred_y = tracker.predict_future(PREDICTION_LEAD_SEC)

                primary_target = {
                    "center": (filt_x, filt_y),
                    "box": verified_detection["box"],
                    "score": verified_detection["score"],
                    "area": verified_detection["area"],
                    "range_m": verified_detection.get("range_m", 10.0),
                    "vel": (vel_x, vel_y),
                    "last_seen": now,
                    "id": verified_detection.get("id", 0)
                }
                predicted_target = (pred_x, pred_y)
            elif primary_target and (now - primary_target["last_seen"] >= 0.45):
                primary_target = None
                predicted_target = None
                tracker.reset()

            local_target = dict(primary_target) if primary_target else None
            local_pred = tuple(predicted_target) if predicted_target else None

        target_locked = False
        target_in_deadband = False
        current_range_m = last_valid_range_m

        if local_target and (now - local_target["last_seen"] < 0.40):
            target_locked = True
            filt_x, filt_y = local_target["center"]

            if "range_m" in local_target:
                last_valid_range_m = local_target["range_m"]
                current_range_m = last_valid_range_m

            parallax_pan_mrad = (-CAMERA_OFFSET_X_MM / (current_range_m * 1000.0)) * 1000.0
            parallax_tilt_mrad = (CAMERA_OFFSET_Y_MM / (current_range_m * 1000.0)) * 1000.0

            err_pan_mrad = (-(filt_x - OPTICAL_CENTER[0]) * MRAD_PER_PIXEL_X) + parallax_pan_mrad
            err_tilt_mrad = (-(filt_y - OPTICAL_CENTER[1]) * MRAD_PER_PIXEL_Y) + parallax_tilt_mrad
            radial_error_mrad = math.hypot(err_pan_mrad, err_tilt_mrad)

            if radial_error_mrad <= FIRE_DEADBAND_MRAD:
                target_in_deadband = True
                turret_queue.put({"cmd": "TRIGGER", "state": 1})
            else:
                turret_queue.put({"cmd": "TRIGGER", "state": 0})

            dt_ctrl = max(0.005, min(0.15, now - last_control_time))
            
            # Derivative on error (rate of error change in mrad/s)
            d_err_pan = float(np.clip((err_pan_mrad - last_err_pan_mrad) / dt_ctrl, -MAX_DERIVATIVE_RATE, MAX_DERIVATIVE_RATE))
            d_err_tilt = float(np.clip((err_tilt_mrad - last_err_tilt_mrad) / dt_ctrl, -MAX_DERIVATIVE_RATE, MAX_DERIVATIVE_RATE))

            last_err_pan_mrad = err_pan_mrad
            last_err_tilt_mrad = err_tilt_mrad
            last_control_time = now

            if radial_error_mrad > MOTION_DEADBAND_MRAD:
                # PD controller: KD opposes rapid error collapse to brake before overshoot
                raw_pan_spd = int((err_pan_mrad * KP_PAN) + (d_err_pan * KD_PAN))
                raw_tilt_spd = int((err_tilt_mrad * KP_TILT) + (d_err_tilt * KD_TILT))

                # Hard clamp to calibrated speed limits
                raw_pan_spd = max(-MAX_PAN_SPEED, min(MAX_PAN_SPEED, raw_pan_spd))
                raw_tilt_spd = max(-MAX_TILT_SPEED, min(MAX_TILT_SPEED, raw_tilt_spd))

                # Friction overcome / min motor run threshold
                if 0 < abs(raw_pan_spd) < MIN_RUN_SPEED:
                    raw_pan_spd = MIN_RUN_SPEED if raw_pan_spd > 0 else -MIN_RUN_SPEED
                if 0 < abs(raw_tilt_spd) < MIN_RUN_SPEED:
                    raw_tilt_spd = MIN_RUN_SPEED if raw_tilt_spd > 0 else -MIN_RUN_SPEED
            else:
                raw_pan_spd = 0
                raw_tilt_spd = 0

            delta_pan = np.clip(raw_pan_spd - active_cmd_pan_spd, -MAX_ACCEL_PAN_PER_FRAME, MAX_ACCEL_PAN_PER_FRAME)
            delta_tilt = np.clip(raw_tilt_spd - active_cmd_tilt_spd, -MAX_ACCEL_TILT_PER_FRAME, MAX_ACCEL_TILT_PER_FRAME)

            active_cmd_pan_spd += int(delta_pan)
            active_cmd_tilt_spd += int(delta_tilt)

            turret_queue.put({
                "cmd": "VELOCITY",
                "pan_spd": active_cmd_pan_spd,
                "tilt_spd": active_cmd_tilt_spd
            })
        else:
            turret_queue.put({"cmd": "TRIGGER", "state": 0})

            if abs(active_cmd_pan_spd) > 0 or abs(active_cmd_tilt_spd) > 0:
                delta_p = np.clip(-active_cmd_pan_spd, -MAX_ACCEL_PAN_PER_FRAME, MAX_ACCEL_PAN_PER_FRAME)
                delta_t = np.clip(-active_cmd_tilt_spd, -MAX_ACCEL_TILT_PER_FRAME, MAX_ACCEL_TILT_PER_FRAME)
                active_cmd_pan_spd += int(delta_p)
                active_cmd_tilt_spd += int(delta_t)
                turret_queue.put({"cmd": "VELOCITY", "pan_spd": active_cmd_pan_spd, "tilt_spd": active_cmd_tilt_spd})
            else:
                turret_queue.put({"cmd": "VELOCITY", "pan_spd": 0, "tilt_spd": 0})

            last_err_pan_mrad = 0.0
            last_err_tilt_mrad = 0.0
            last_control_time = now

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
        # HUD Render & Diagnostic Visual Overlay (Native BGR Color Space)
        # -----------------------------------------------------------------
        oc_x = int(OPTICAL_CENTER[0] * SCALE_X)
        oc_y = int(OPTICAL_CENTER[1] * SCALE_Y)
        # Yellow crosshair at Optical Center: (Blue=0, Green=255, Red=255)
        cv2.drawMarker(annotated, (oc_x, oc_y), (0, 255, 255), cv2.MARKER_CROSS, 16, 1)

        # Bore Sight convergence marker at estimated target distance
        conv_px_x = OPTICAL_CENTER[0] + int((-CAMERA_OFFSET_X_MM / (current_range_m * 1000.0)) * FOCAL_LENGTH_PX)
        conv_px_y = OPTICAL_CENTER[1] + int((CAMERA_OFFSET_Y_MM / (current_range_m * 1000.0)) * FOCAL_LENGTH_PX)
        aim_x = int(conv_px_x * SCALE_X)
        aim_y = int(conv_px_y * SCALE_Y)
        db_radius = int((FIRE_DEADBAND_MRAD / MRAD_PER_PIXEL_X) * SCALE_X)

        # Ring: Yellow when in firing deadband, Red when acquiring
        ring_color = (0, 255, 255) if target_in_deadband else (0, 0, 255)
        cv2.circle(annotated, (aim_x, aim_y), db_radius, ring_color, 1)
        # Red crosshair at dynamic aim point: (0, 0, 255)
        cv2.drawMarker(annotated, (aim_x, aim_y), (0, 0, 255), cv2.MARKER_CROSS, 24, 1)

        for det in detections:
            x1 = int(det["box"][0] * SCALE_X)
            y1 = int(det["box"][1] * SCALE_Y)
            x2 = int(det["box"][2] * SCALE_X)
            y2 = int(det["box"][3] * SCALE_Y)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (180, 180, 180), 1)

        if local_target and (now - local_target["last_seen"] < 0.40):
            tx1 = int(local_target["box"][0] * SCALE_X)
            ty1 = int(local_target["box"][1] * SCALE_Y)
            tx2 = int(local_target["box"][2] * SCALE_X)
            ty2 = int(local_target["box"][3] * SCALE_Y)
            tcx_d = int(local_target["center"][0] * SCALE_X)
            tcy_d = int(local_target["center"][1] * SCALE_Y)

            # Green box for locked target: (0, 255, 0)
            cv2.rectangle(annotated, (tx1, ty1), (tx2, ty2), (0, 255, 0), 2)
            cv2.circle(annotated, (tcx_d, tcy_d), 4, (0, 255, 0), -1)

            if local_pred:
                px_d = int(local_pred[0] * SCALE_X)
                py_d = int(local_pred[1] * SCALE_Y)
                # Cyan predictive lead vector: (255, 255, 0)
                cv2.arrowedLine(annotated, (tcx_d, tcy_d), (px_d, py_d), (255, 255, 0), 2, tipLength=0.3)
                cv2.circle(annotated, (px_d, py_d), 3, (255, 255, 0), -1)

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

        badge_color = (0, 0, 0) if ser_ok else (0, 0, 255)
        ser_badge = "SERIAL: OK" if ser_ok else "SERIAL: DISCONNECTED"
        status_text = f"FPS:{sensor_fps:.1f} | Pan:{p_mrad}mrad Tilt:{t_mrad}mrad | Range:{current_range_m:.1f}m | {ser_badge}"
        cv2.putText(annotated, status_text, (10, DISPLAY_HEIGHT - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, badge_color, 1, cv2.LINE_AA)

        if alert_msg:
            cv2.rectangle(annotated, (0, 0), (DISPLAY_WIDTH, 26), (0, 0, 180), -1)
            cv2.putText(annotated, alert_msg, (12, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        # Zero conversion: pass annotated directly as BGR to OpenCV's native JPEG compressor
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
            <div class="meta">Live Telemetry | Native mrad Kinematics | Auto-Recovery Watchdogs</div>
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