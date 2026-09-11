#!/usr/bin/env python3
"""
Bombardeer Turret Hybrid Visual Servoing Controller
--------------------------------------------------
- Phase 1: Coarse Saccadic Step (> 50 mrad):
    * Dispatches single waypoint displacement: M <d_pan> <d_tilt>
    * Enters settling blanking window until ESP32 reports is_moving == 0.
- Phase 2: Continuous Fine Trim (< 50 mrad):
    * Dispatches gentle proportional velocity commands: V <p_spd> <t_spd>
    * Tracks smooth target drift right into the crosshair deadband.
- Hardened ArUco perception with BGR MP4 recording and live diagnostic console.
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
from collections import deque
import numpy as np
import cv2
import serial
from flask import Flask, Response, render_template_string, send_from_directory

from picamera2 import Picamera2
from libcamera import Transform

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("Bombardeer")

STATIC_DIR = "/home/ben/Bombardeer/static"
RECORDINGS_DIR = "/home/ben/Bombardeer/test_recordings"
os.makedirs(RECORDINGS_DIR, exist_ok=True)

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 360

SCALE_X = DISPLAY_WIDTH / FRAME_WIDTH
SCALE_Y = DISPLAY_HEIGHT / FRAME_HEIGHT

OPTICAL_CENTER = (FRAME_WIDTH // 2, FRAME_HEIGHT // 2)

GUN_OFFSET_PAN_PX = 0
GUN_OFFSET_TILT_PX = 20

FOCAL_LENGTH_X_PX = 509
FOCAL_LENGTH_Y_PX = 477

# =========================================================================
# Hybrid Dual-Phase Tuning Thresholds
# =========================================================================
SACCADE_ENTRY_THRESHOLD_MRAD = 50.0  # Above this, fire discrete waypoint move
SACCADE_TIMEOUT_SEC = 0.85  # Hard limit to exit saccade blanking if packet lost

# =========================================================================
# Phase 2 Fine Trim Proportional Gains (Unit: (mrad/s) / mrad error = 1/s)
# -------------------------------------------------------------------------
# HOW TO DETERMINE THESE VALUES:
# 1. Physical Meaning: Kp sets how fast error decays (Time to settle ~= 3 / Kp).
#    * Kp = 4.0  -> Settles in ~0.75s (sluggish, feels like a crawl).
#    * Kp = 8.5  -> Settles in ~0.35s (brisk, smooth closure).
#    * Kp > 15.0 -> Unstable. Exceeds loop latency (dt ~= 65ms), causing oscillation.
# 2. Tuning Method:
#    * Start low (e.g., 5.0) and increase in increments of 1.0.
#    * If the axis crawls into the deadband, INCREASE Kp.
#    * If the axis overshoots the crosshair reticle during Phase 2, DECREASE Kp.
#    * Pan requires slightly higher gain (8.0 - 9.0) due to taller gearing (6.67:1).
#    * Tilt requires lower gain (4.5 - 6.0) due to 50:1 worm reduction mechanical damping.
# =========================================================================
KP_TRIM_PAN = 4.25
KP_TRIM_TILT = 2.5

MAX_TRIM_SPEED_PAN = 900.0  # Cap to prevent blur (mrad/s, ~3000 steps/s)
MAX_TRIM_SPEED_TILT = 350.0  # Cap to prevent worm gear whip (mrad/s)

# Settling Deadbands
PAN_DEADBAND_MRAD = 14.0
TILT_DEADBAND_MRAD = 10.0
FIRE_DEADBAND_MRAD = 38.0

MAX_TRIGGER_DURATION_SEC = 2.0
TRIGGER_COOLDOWN_SEC = 1.5

TARGET_MARKER_ID = 0
CONFIRMATION_FRAMES = 2

DEFAULT_SERIAL_PORTS = ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1"]
BAUD_RATE = 115200

TARGET_FPS = 15.0
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
turret_is_moving = False
is_firing = False
free_disk_gb = 0.0

system_health = {
    "serial_connected": False,
    "serial_tx_count": 0,
    "serial_rx_count": 0,
    "active_phase": "IDLE",
    "recorder_active": False,
    "recorder_error": None,
    "last_alert": None,
    "alert_time": 0.0,
}


def set_alert(message, duration=3.0):
    with state_lock:
        system_health["last_alert"] = message
        system_health["alert_time"] = time.time() + duration
    log.warning(f"[ALERT] {message}")


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

    def detect(self, frame_raw):
        try:
            if isinstance(frame_raw, list):
                frame_raw = np.asarray(
                    frame_raw[0]
                    if len(frame_raw) > 0
                    and isinstance(frame_raw[0], (np.ndarray, list))
                    else frame_raw
                )

            gray = (
                cv2.cvtColor(frame_raw, cv2.COLOR_RGB2GRAY)
                if len(frame_raw.shape) == 3
                else frame_raw
            )
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

                    detections.append(
                        {
                            "box": [x1, y1, x2, y2],
                            "center": (cx, cy),
                            "score": 0.99,
                            "area": area,
                            "id": int(marker_id),
                        }
                    )
            return detections
        except Exception as e:
            log.error(f"[DETECTOR ERROR] Vision parsing failed: {e}")
            return []


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
    global turret_pan_mrad, turret_tilt_mrad, turret_is_moving, is_firing

    ser = None
    active_port = None
    next_reconnect_time = 0.0

    last_p_spd = None
    last_t_spd = None
    trigger_state = 0
    trigger_start_time = 0.0
    cooldown_until = 0.0
    pending_velocity = None

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
                        log.info(f"[SERIAL] Connected on {active_port}")
                        set_alert(f"Connected: {active_port}", duration=2.0)
                    except Exception as e:
                        ser = None
                        next_reconnect_time = now + 2.0
                        log.error(f"[SERIAL ERROR] Link failed: {e}")
                else:
                    next_reconnect_time = now + 2.0

        try:
            while not cmd_queue.empty():
                item = cmd_queue.get_nowait()
                cmd = item.get("cmd")

                if cmd == "SACCADE":
                    d_p = item["d_pan"]
                    d_t = item["d_tilt"]
                    pending_velocity = None
                    last_p_spd = 0
                    last_t_spd = 0
                    if ser and ser.is_open:
                        try:
                            ser.write(f"M {d_p} {d_t}\n".encode())
                            with state_lock:
                                system_health["serial_tx_count"] += 1
                        except Exception as e:
                            log.error(f"[SERIAL TX ERROR] Saccade write failed: {e}")

                elif cmd == "VELOCITY":
                    pending_velocity = item

                elif cmd == "HALT":
                    pending_velocity = None
                    if ser and ser.is_open:
                        try:
                            ser.write(b"X\n")
                            with state_lock:
                                system_health["serial_tx_count"] += 1
                        except Exception:
                            pass

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
                                    log.info("[SERIAL] Solenoid FIRED")
                                except Exception:
                                    pass
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
                                except Exception:
                                    pass
        except queue.Empty:
            pass

        if pending_velocity and ser and ser.is_open:
            p_spd = pending_velocity["pan_spd"]
            t_spd = pending_velocity["tilt_spd"]
            if p_spd != last_p_spd or t_spd != last_t_spd:
                try:
                    ser.write(f"V {p_spd} {t_spd}\n".encode())
                    last_p_spd = p_spd
                    last_t_spd = t_spd
                    with state_lock:
                        system_health["serial_tx_count"] += 1
                except Exception as e:
                    log.error(f"[SERIAL TX ERROR] Velocity write failed: {e}")
            pending_velocity = None

        if trigger_state == 1 and (
            now - trigger_start_time >= MAX_TRIGGER_DURATION_SEC
        ):
            trigger_state = 0
            with state_lock:
                is_firing = False
            cooldown_until = now + TRIGGER_COOLDOWN_SEC
            if ser and ser.is_open:
                try:
                    ser.write(b"T 0\n")
                except Exception:
                    pass

        if ser and ser.is_open and ser.in_waiting:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if line.startswith("S "):
                    parts = line.split()
                    if len(parts) >= 5:
                        with state_lock:
                            turret_pan_mrad = int(parts[1])
                            turret_tilt_mrad = int(parts[2])
                            turret_is_moving = int(parts[4]) == 1
                            system_health["serial_rx_count"] += 1
            except Exception:
                pass

        time.sleep(0.005)


def ensure_storage_headroom():
    global free_disk_gb
    try:
        _, _, free = shutil.disk_usage(RECORDINGS_DIR)
        free_disk_gb = free / (1024**3)
        files = glob.glob(os.path.join(RECORDINGS_DIR, "test_*.mp4"))
        if not files:
            return
        files.sort(key=os.path.getmtime)
        archive_gb = sum(os.path.getsize(f) for f in files) / (1024**3)
        for fpath in files:
            if (
                free_disk_gb >= MIN_FREE_SPACE_GB
                and archive_gb <= MAX_RECORDINGS_STORAGE_GB
            ):
                break
            size = os.path.getsize(fpath)
            try:
                os.remove(fpath)
                archive_gb -= size / (1024**3)
                _, _, updated_free = shutil.disk_usage(RECORDINGS_DIR)
                free_disk_gb = updated_free / (1024**3)
            except OSError:
                pass
    except Exception:
        pass


def video_recorder_worker(record_queue):
    writer = None
    recording_active = False
    current_filepath = None
    CODECS = [
        ("mp4v", cv2.VideoWriter_fourcc(*"mp4v")),
        ("avc1", cv2.VideoWriter_fourcc(*"avc1")),
    ]

    while True:
        try:
            item = record_queue.get(timeout=0.5)
        except queue.Empty:
            continue
        cmd = item.get("cmd")

        if cmd == "start" and not recording_active:
            ensure_storage_headroom()
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            current_filepath = os.path.join(RECORDINGS_DIR, f"test_{ts}.mp4")
            writer = None
            for tag, fourcc in CODECS:
                try:
                    c = cv2.VideoWriter(
                        current_filepath,
                        fourcc,
                        TARGET_FPS,
                        (FRAME_WIDTH, FRAME_HEIGHT),
                    )
                    if c and c.isOpened():
                        writer = c
                        break
                    elif c:
                        c.release()
                except Exception:
                    continue

            if writer and writer.isOpened():
                recording_active = True
                with state_lock:
                    system_health["recorder_active"] = True
                for f in item.get("pre_roll", []):
                    writer.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
        elif cmd == "frame" and recording_active and writer and writer.isOpened():
            writer.write(cv2.cvtColor(item.get("frame"), cv2.COLOR_RGB2BGR))
        elif cmd == "stop" and recording_active:
            recording_active = False
            with state_lock:
                system_health["recorder_active"] = False
            if writer:
                writer.release()
                writer = None


def vision_thread():
    global latest_jpeg, primary_target, active_detections

    ensure_storage_headroom()
    detector = ArucoTargetDetector()

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
            "ExposureValue": 1.0,
        },
        transform=Transform(hflip=True, vflip=True),
    )
    picam2.configure(cam_config)
    picam2.start()
    time.sleep(0.5)

    turret_queue = queue.Queue(maxsize=15)
    record_queue = queue.Queue(maxsize=90)

    threading.Thread(
        target=turret_serial_worker,
        args=(turret_queue,),
        daemon=True,
        name="SerialWorker",
    ).start()
    threading.Thread(
        target=video_recorder_worker,
        args=(record_queue,),
        daemon=True,
        name="RecorderWorker",
    ).start()

    pre_roll_buffer = deque(maxlen=int(RECORD_PRE_ROLL_SEC * TARGET_FPS))

    gun_aim_x = OPTICAL_CENTER[0] + GUN_OFFSET_PAN_PX
    gun_aim_y = OPTICAL_CENTER[1] + GUN_OFFSET_TILT_PX

    fps_time = time.time()
    frame_count = 0
    sensor_fps = 0.0

    tentative_target = None
    tentative_streak = 0

    is_recording = False
    lock_consecutive_frames = 0
    last_detection_time = 0.0
    recording_start_time = 0.0

    # Hybrid Tracking State Machine
    in_saccade_wait = False
    saccade_start_time = 0.0

    while True:
        now = time.time()

        try:
            request = picam2.capture_request()
            frame_main = request.make_array("main")
            annotated = request.make_array("lores")
            request.release()
        except Exception:
            time.sleep(0.02)
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
                primary_target = {
                    "center": verified_detection["center"],
                    "box": verified_detection["box"],
                    "last_seen": now,
                    "id": verified_detection.get("id", 0),
                }
            elif primary_target and (now - primary_target["last_seen"] >= 0.40):
                primary_target = None

            local_target = dict(primary_target) if primary_target else None
            is_moving = turret_is_moving

        # Check if active saccade move has completed
        if in_saccade_wait:
            time_in_saccade = now - saccade_start_time
            if (
                not is_moving and time_in_saccade >= 0.12
            ) or time_in_saccade >= SACCADE_TIMEOUT_SEC:
                in_saccade_wait = False

        target_locked = False
        target_in_deadband = False

        if local_target and (now - local_target["last_seen"] < 0.35):
            target_locked = True
            tx, ty = local_target["center"]

            err_px_x = tx - gun_aim_x
            err_px_y = ty - gun_aim_y

            err_pan_mrad = math.atan2(err_px_x, FOCAL_LENGTH_X_PX) * 1000.0
            err_tilt_mrad = math.atan2(-err_px_y, FOCAL_LENGTH_Y_PX) * 1000.0

            radial_err = math.hypot(err_pan_mrad, err_tilt_mrad)

            # Autonomous Trigger Authorization Gate
            if radial_err <= FIRE_DEADBAND_MRAD:
                target_in_deadband = True
                turret_queue.put({"cmd": "TRIGGER", "state": 1})
            else:
                turret_queue.put({"cmd": "TRIGGER", "state": 0})

            # =============================================================
            # HYBRID DUAL-PHASE DECISION ENGINE
            # =============================================================
            if in_saccade_wait:
                # Motor is actively slewing in hardware: DO NOT issue steering commands,
                # but DO NOT call continue so video recording and HUD continue running!
                with state_lock:
                    system_health["active_phase"] = "SACCADE"

            elif radial_err >= SACCADE_ENTRY_THRESHOLD_MRAD:
                # PHASE 1: DISPATCH NEW SACCADIC DISPLACEMENT
                with state_lock:
                    system_health["active_phase"] = "SACCADE"

                d_pan_cmd = int(round(err_pan_mrad))
                d_tilt_cmd = int(round(err_tilt_mrad))

                turret_queue.put(
                    {"cmd": "SACCADE", "d_pan": d_pan_cmd, "d_tilt": d_tilt_cmd}
                )
                in_saccade_wait = True
                saccade_start_time = now

            else:
                # PHASE 2: CONTINUOUS FINE TRIM (< 50 mrad)
                with state_lock:
                    system_health["active_phase"] = "FINE_TRIM"

                if abs(err_pan_mrad) > PAN_DEADBAND_MRAD:
                    v_pan = err_pan_mrad * KP_TRIM_PAN
                    v_pan = max(-MAX_TRIM_SPEED_PAN, min(MAX_TRIM_SPEED_PAN, v_pan))
                    cmd_p_spd = int(round(v_pan))
                else:
                    cmd_p_spd = 0

                if abs(err_tilt_mrad) > TILT_DEADBAND_MRAD:
                    v_tilt = err_tilt_mrad * KP_TRIM_TILT
                    v_tilt = max(-MAX_TRIM_SPEED_TILT, min(MAX_TRIM_SPEED_TILT, v_tilt))
                    cmd_t_spd = int(round(v_tilt))
                else:
                    cmd_t_spd = 0

                turret_queue.put(
                    {"cmd": "VELOCITY", "pan_spd": cmd_p_spd, "tilt_spd": cmd_t_spd}
                )

        else:
            with state_lock:
                system_health["active_phase"] = "IDLE"
            turret_queue.put({"cmd": "TRIGGER", "state": 0})
            turret_queue.put({"cmd": "HALT"})

        # Video Recorder Management
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
                    record_queue.put_nowait(
                        {"cmd": "start", "pre_roll": list(pre_roll_buffer)}
                    )
                except queue.Full:
                    pass
            else:
                pre_roll_buffer.append(frame_main.copy())
        else:
            time_since_last_seen = now - last_detection_time
            clip_duration = now - recording_start_time
            if (
                time_since_last_seen > RECORD_POST_ROLL_SEC
                or clip_duration > MAX_RECORDING_DURATION_SEC
            ):
                is_recording = False
                try:
                    record_queue.put_nowait({"cmd": "stop"})
                except queue.Full:
                    pass
            else:
                try:
                    record_queue.put_nowait(
                        {"cmd": "frame", "frame": frame_main.copy()}
                    )
                except queue.Full:
                    pass

        # HUD Rendering
        aim_x = int(gun_aim_x * SCALE_X)
        aim_y = int(gun_aim_y * SCALE_Y)
        db_radius_px = int(
            math.tan(FIRE_DEADBAND_MRAD / 1000.0) * FOCAL_LENGTH_X_PX * SCALE_X
        )
        saccade_radius_px = int(
            math.tan(SACCADE_ENTRY_THRESHOLD_MRAD / 1000.0)
            * FOCAL_LENGTH_X_PX
            * SCALE_X
        )

        cv2.circle(
            annotated,
            (aim_x, aim_y),
            max(db_radius_px, 8),
            (0, 255, 255) if target_in_deadband else (0, 0, 255),
            1,
        )
        cv2.circle(
            annotated, (aim_x, aim_y), max(saccade_radius_px, 12), (80, 80, 80), 1
        )
        cv2.drawMarker(annotated, (aim_x, aim_y), (0, 0, 255), cv2.MARKER_CROSS, 20, 1)

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
            tcx = int(local_target["center"][0] * SCALE_X)
            tcy = int(local_target["center"][1] * SCALE_Y)
            cv2.rectangle(annotated, (tx1, ty1), (tx2, ty2), (0, 255, 0), 2)
            cv2.circle(annotated, (tcx, tcy), 4, (0, 255, 0), -1)
            cv2.putText(
                annotated,
                f"TARGET ID:{local_target['id']}",
                (tx1, max(ty1 - 8, 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                1,
            )

        with state_lock:
            p_mrad = turret_pan_mrad
            t_mrad = turret_tilt_mrad
            firing = is_firing
            ser_ok = system_health["serial_connected"]
            phase = system_health["active_phase"]

        if firing:
            cv2.putText(
                annotated,
                "SOLENOID ENGAGED",
                (aim_x - 70, aim_y - 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 255),
                2,
            )

        status_text = f"FPS:{sensor_fps:.1f} | POS: P:{p_mrad} T:{t_mrad} | {phase} | {'SERIAL:OK' if ser_ok else 'SERIAL:OFF'}"
        cv2.putText(
            annotated,
            status_text,
            (10, DISPLAY_HEIGHT - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (0, 255, 0) if ser_ok else (0, 0, 255),
            1,
        )

        success, buffer = cv2.imencode(
            ".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 75]
        )
        if success:
            with frame_lock:
                latest_jpeg = buffer.tobytes()


@app.route("/")
def index():
    return render_template_string("""
    <!DOCTYPE html><html><body style="background:#0d1117;color:#c9d1d9;display:flex;flex-direction:column;align-items:center;justify-content:center;min-height:100vh;margin:0;">
    <div style="background:#161b22;padding:16px;border-radius:8px;border:1px solid #30363d;text-align:center;">
        <h2 style="color:#39d353;margin:0 0 10px 0;">Bombardeer Hybrid Servoing Suite</h2>
        <img src="/video_feed" style="border-radius:4px;width:640px;height:360px;background:#000;">
        <p style="color:#8b949e;font-size:0.85rem;margin:10px 0 0 0;">Phase 1: Saccade (>50 mrad) | Phase 2: Fine Trim (<50 mrad)</p>
    </div>
    </body></html>""")


def generate_frames():
    while True:
        with frame_lock:
            if latest_jpeg is None:
                time.sleep(0.01)
                continue
            frame = latest_jpeg
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(0.04)


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/favicon.ico")
def favicon():
    if os.path.exists(os.path.join(STATIC_DIR, "bombardeer.ico")):
        return send_from_directory(
            STATIC_DIR, "bombardeer.ico", mimetype="image/vnd.microsoft.icon"
        )
    return ("", 204)


if __name__ == "__main__":
    t = threading.Thread(target=vision_thread, daemon=True, name="VisionThread")
    t.start()
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
