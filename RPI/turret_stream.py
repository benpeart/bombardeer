#!/usr/bin/env python3
"""
Bombardeer Wildlife Targeting & Archival Vision System
------------------------------------------------------
- Auto low-light exposure with pre-roll circular video archival.
- Sticky wildlife tracker (nearest-centroid + IoU hysteresis).
- FastAccelStepper serial interface with physical boundary clamping.
- Deadband-gated 24V solenoid trigger with hard safety duration cap.
"""

import os
import glob
import time
import math
import shutil
import datetime
import threading
import queue
from collections import deque
import numpy as np
import cv2
import serial
from flask import Flask, Response, render_template_string, send_from_directory

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
from picamera2 import Picamera2
from libcamera import Transform

# -------------------------------------------------------------------------
# Configuration & Calibration
# -------------------------------------------------------------------------
MODEL_PATH = "/home/ben/Bombardeer/MDV6-yolov9-c-1280.hef"
if not os.path.exists(MODEL_PATH):
    MODEL_PATH = "/home/ben/Bombardeer/models/MDV6-yolov9-c-1280.hef"

STATIC_DIR = "/home/ben/Bombardeer/static"
RECORDINGS_DIR = "/home/ben/Bombardeer/recordings"
os.makedirs(RECORDINGS_DIR, exist_ok=True)

# Storage Management Constraints
MIN_FREE_SPACE_GB = 5.0           # Minimum free space to preserve on drive
MAX_RECORDINGS_STORAGE_GB = 16.0  # Max capacity allocated for wildlife archive

# Vision Dimensions
INFER_SIZE = 1280
DISPLAY_SIZE = 640
SCALE_FACTOR = DISPLAY_SIZE / INFER_SIZE
OPTICAL_CENTER = (INFER_SIZE // 2, INFER_SIZE // 2)

# --- BORESIGHT CALIBRATION (Offset in Pixels from Camera Center to Gun POI) ---
GUN_OFFSET_PAN_PX = 0    # Positive shifts target crosshair right
GUN_OFFSET_TILT_PX = 20  # Positive shifts target crosshair down (e.g. gun mounted below camera)

# --- TURRET PHYSICAL BOUNDARIES (FastAccelStepper Steps) ---
# 0 is centered. Adjust to match your mechanical endstops.
PAN_MIN_STEPS = -4000   # Max Left
PAN_MAX_STEPS = 4000    # Max Right
TILT_MIN_STEPS = -1200  # Max Downward Depression
TILT_MAX_STEPS = 1800   # Max Upward Elevation

# Calibration: Pixel error to motor step conversion factor
# Tune based on FOV and gearing (steps per pixel of image error)
STEPS_PER_PIXEL_PAN = 1.85
STEPS_PER_PIXEL_TILT = 1.85

# --- TRACKING & HYSTERESIS PARAMETERS ---
CONF_THRESH = 0.28
IOU_THRESH = 0.45
TARGET_TIMEOUT_SEC = 2.5       # Seconds to retain track through foliage/occlusion
MAX_CENTROID_JUMP_PX = 350     # Max pixel movement per frame for identity match

# --- TRIGGER & FIRING CONTROLS ---
FIRE_DEADBAND_PX = 35          # Target must be within +/- 35px radius to fire
MAX_TRIGGER_DURATION_SEC = 2.0 # Maximum continuous firing time
TRIGGER_COOLDOWN_SEC = 1.5     # Mandatory rest time before re-engaging solenoid

# Serial Port to ESP32
SERIAL_PORT = "/dev/ttyUSB0"
if not os.path.exists(SERIAL_PORT):
    SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

RECORD_PRE_ROLL_SEC = 3.0    # Seconds preserved before trigger
RECORD_POST_ROLL_SEC = 5.0   # Seconds preserved after animal leaves
TARGET_FPS = 12.0            # Nominal target recording rate

app = Flask(__name__)
frame_lock = threading.Lock()
latest_jpeg = None

# Telemetry & Target Tracking
detections_lock = threading.Lock()
active_detections = []
primary_target = None
npu_latency_ms = 0.0
npu_fps = 0.0
actual_sensor_fps = 0.0
free_disk_gb = 0.0

# Turret Coordinate State
turret_pan_step = 0
turret_tilt_step = 0
is_firing = False


# -------------------------------------------------------------------------
# Storage Manager: Prune Oldest Files
# -------------------------------------------------------------------------
def ensure_storage_headroom():
    """
    Checks storage usage and purges the oldest MP4 recordings in FIFO order
    if free drive space drops below MIN_FREE_SPACE_GB or total archive
    size exceeds MAX_RECORDINGS_STORAGE_GB.
    """
    global free_disk_gb
    try:
        total, used, free = shutil.disk_usage(RECORDINGS_DIR)
        free_disk_gb = free / (1024 ** 3)

        recordings = glob.glob(os.path.join(RECORDINGS_DIR, "wildlife_*.mp4"))
        if not recordings:
            return

        # Sort files chronologically by modification time (oldest first)
        recordings.sort(key=os.path.getmtime)
        total_archive_gb = sum(os.path.getsize(f) for f in recordings) / (1024 ** 3)

        # Remove oldest files while constraints are violated
        for fpath in recordings:
            if free_disk_gb >= MIN_FREE_SPACE_GB and total_archive_gb <= MAX_RECORDINGS_STORAGE_GB:
                break
            fsize = os.path.getsize(fpath)
            try:
                os.remove(fpath)
                total_archive_gb -= fsize / (1024 ** 3)
                # Re-check actual disk free space
                _, _, updated_free = shutil.disk_usage(RECORDINGS_DIR)
                free_disk_gb = updated_free / (1024 ** 3)
            except OSError:
                pass
    except Exception as e:
        print(f"[Storage Manager] Audit error: {e}")


# -------------------------------------------------------------------------
# Serial Interface Worker (Pi <-> ESP32)
# -------------------------------------------------------------------------
def turret_serial_worker(cmd_queue):
    global turret_pan_step, turret_tilt_step, is_firing

    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        print(f"[Serial] Connected to ESP32 on {SERIAL_PORT}")
    except Exception as e:
        print(f"[Serial] Warning: Could not open {SERIAL_PORT}: {e}")

    last_pan_sent = 0
    last_tilt_sent = 0
    trigger_state = 0
    trigger_start_time = 0.0
    cooldown_until = 0.0

    while True:
        now = time.time()

        # 1. Process pending movement and trigger requests from queue
        try:
            while not cmd_queue.empty():
                item = cmd_queue.get_nowait()
                cmd = item.get("cmd")

                if cmd == "MOVE":
                    # Clamp to physical limits
                    target_pan = max(PAN_MIN_STEPS, min(PAN_MAX_STEPS, item["pan"]))
                    target_tilt = max(TILT_MIN_STEPS, min(TILT_MAX_STEPS, item["tilt"]))

                    # Only transmit if moved by at least 2 steps (hysteresis filter)
                    if abs(target_pan - last_pan_sent) >= 2 or abs(target_tilt - last_tilt_sent) >= 2:
                        last_pan_sent = target_pan
                        last_tilt_sent = target_tilt
                        turret_pan_step = target_pan
                        turret_tilt_step = target_tilt
                        if ser and ser.is_open:
                            ser.write(f"M {target_pan} {target_tilt}\n".encode())

                elif cmd == "TRIGGER":
                    req_state = item["state"]
                    if req_state == 1:
                        # Safety check: allow fire only if cooldown expired
                        if trigger_state == 0 and now >= cooldown_until:
                            trigger_state = 1
                            trigger_start_time = now
                            is_firing = True
                            if ser and ser.is_open:
                                ser.write(b"T 1\n")
                    elif req_state == 0:
                        if trigger_state == 1:
                            trigger_state = 0
                            is_firing = False
                            cooldown_until = now + TRIGGER_COOLDOWN_SEC
                            if ser and ser.is_open:
                                ser.write(b"T 0\n")
        except queue.Empty:
            pass

        # 2. Hard Watchdog Cap: Force shutoff if open longer than MAX_TRIGGER_DURATION_SEC
        if trigger_state == 1 and (now - trigger_start_time >= MAX_TRIGGER_DURATION_SEC):
            trigger_state = 0
            is_firing = False
            cooldown_until = now + TRIGGER_COOLDOWN_SEC
            if ser and ser.is_open:
                ser.write(b"T 0\n")
            print("[Safety] Hard trigger duration limit reached. Disengaging solenoid.")

        # 3. Read status packets from ESP32: S <pan> <tilt> <trig>\n
        if ser and ser.is_open and ser.in_waiting:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if line.startswith("S "):
                    parts = line.split()
                    if len(parts) >= 4:
                        turret_pan_step = int(parts[1])
                        turret_tilt_step = int(parts[2])
            except Exception:
                pass

        time.sleep(0.01)


# -------------------------------------------------------------------------
# Vectorized MDV6 YOLOv9-c Post-Processor (1280p)
# -------------------------------------------------------------------------
class MDV6Decoder1280:
    def __init__(self, input_size=1280):
        self.input_size = input_size
        self.strides = [8, 16, 32]
        self.reg_max = 16
        self.project = np.arange(self.reg_max, dtype=np.float32)
        self.anchors, self.stride_scales = self._generate_anchors_and_scales()

    def _generate_anchors_and_scales(self):
        anchors = []
        scales = []
        for stride in self.strides:
            grid_size = self.input_size // stride
            grid_y, grid_x = np.meshgrid(
                np.arange(grid_size, dtype=np.float32),
                np.arange(grid_size, dtype=np.float32),
                indexing="ij",
            )
            anchor = np.stack((grid_x + 0.5, grid_y + 0.5), axis=-1) * stride
            flat_anchors = anchor.reshape(-1, 2)
            anchors.append(flat_anchors)
            scales.append(np.full(len(flat_anchors), stride, dtype=np.float32))
        return np.vstack(anchors), np.concatenate(scales)

    def _softmax(self, x, axis=-1):
        e_x = np.exp(x - np.max(x, axis=axis, keepdims=True))
        return e_x / np.sum(e_x, axis=axis, keepdims=True)

    def decode(self, raw_outputs):
        c8 = raw_outputs["MDV6-yolov9-c/conv100"].reshape(-1, 3)
        c16 = raw_outputs["MDV6-yolov9-c/conv122"].reshape(-1, 3)
        c32 = raw_outputs["MDV6-yolov9-c/conv144"].reshape(-1, 3)
        raw_classes = np.vstack([c8, c16, c32])

        scores = 1.0 / (1.0 + np.exp(-raw_classes[:, 0]))

        mask = scores > CONF_THRESH
        if not np.any(mask):
            return []

        cand_scores = scores[mask]
        cand_anchors = self.anchors[mask]
        cand_scales = self.stride_scales[mask]

        b8 = raw_outputs["MDV6-yolov9-c/conv98"].reshape(-1, 4, self.reg_max)
        b16 = raw_outputs["MDV6-yolov9-c/conv121"].reshape(-1, 4, self.reg_max)
        b32 = raw_outputs["MDV6-yolov9-c/conv143"].reshape(-1, 4, self.reg_max)
        raw_boxes = np.vstack([b8, b16, b32])[mask]

        dist = self._softmax(raw_boxes, axis=-1)
        ltrb = np.dot(dist, self.project) * cand_scales[:, None]

        x1 = np.clip(cand_anchors[:, 0] - ltrb[:, 0], 0, self.input_size)
        y1 = np.clip(cand_anchors[:, 1] - ltrb[:, 1], 0, self.input_size)
        x2 = np.clip(cand_anchors[:, 0] + ltrb[:, 2], 0, self.input_size)
        y2 = np.clip(cand_anchors[:, 1] + ltrb[:, 3], 0, self.input_size)

        w = x2 - x1
        h = y2 - y1

        cv_boxes = [[int(x1[i]), int(y1[i]), int(w[i]), int(h[i])] for i in range(len(cand_scores))]
        indices = cv2.dnn.NMSBoxes(cv_boxes, cand_scores.tolist(), CONF_THRESH, IOU_THRESH)

        detections = []
        if len(indices) > 0:
            for idx in indices.flatten():
                bx, by, bw, bh = cv_boxes[idx]
                score = float(cand_scores[idx])
                cx = bx + (bw / 2.0)
                cy = by + (bh / 2.0)
                detections.append({
                    "box_1280": [bx, by, bx + bw, by + bh],
                    "score": score,
                    "center": (cx, cy)
                })
        return detections


# -------------------------------------------------------------------------
# Worker Thread: Hailo-8 NPU Inference Engine
# -------------------------------------------------------------------------
def hailo_worker(infer_queue):
    global active_detections, npu_latency_ms, npu_fps

    decoder = MDV6Decoder1280(input_size=INFER_SIZE)
    hef = HEF(MODEL_PATH)
    params = VDevice.create_params()
    with VDevice(params) as target:
        configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
        network_group = target.configure(hef, configure_params)[0]
        network_group_params = network_group.create_params()

        input_vparams = InputVStreamParams.make(network_group, quantized=False, format_type=FormatType.UINT8)
        output_vparams = OutputVStreamParams.make(network_group, quantized=False, format_type=FormatType.FLOAT32)
        input_name = hef.get_input_vstream_infos()[0].name

        with network_group.activate(network_group_params):
            with InferVStreams(network_group, input_vparams, output_vparams) as pipeline:
                last_time = time.time()
                frames_processed = 0

                while True:
                    try:
                        frame_raw = infer_queue.get(timeout=1.0)
                    except queue.Empty:
                        continue

                    tensor = np.expand_dims(frame_raw, axis=0)

                    t0 = time.perf_counter()
                    raw_outputs = pipeline.infer({input_name: tensor})
                    dt = (time.perf_counter() - t0) * 1000.0

                    dets = decoder.decode(raw_outputs)

                    with detections_lock:
                        active_detections = dets
                        npu_latency_ms = dt

                    frames_processed += 1
                    now = time.time()
                    if now - last_time >= 1.0:
                        npu_fps = frames_processed / (now - last_time)
                        frames_processed = 0
                        last_time = now


# -------------------------------------------------------------------------
# High-Quality Video Archiver Thread
# -------------------------------------------------------------------------
def video_recorder_worker(record_queue):
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = None
    recording_active = False

    while True:
        try:
            item = record_queue.get(timeout=1.0)
        except queue.Empty:
            continue

        cmd = item.get("cmd")

        if cmd == "start":
            if not recording_active:
                ensure_storage_headroom()
                ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(RECORDINGS_DIR, f"wildlife_{ts}.mp4")
                writer = cv2.VideoWriter(filename, fourcc, TARGET_FPS, (INFER_SIZE, INFER_SIZE))
                recording_active = True
                print(f"[Recorder] Wildlife event started: {filename}")

                for f in item.get("pre_roll", []):
                    writer.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))

        elif cmd == "frame" and recording_active:
            frame = item.get("frame")
            if writer:
                writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

        elif cmd == "stop" and recording_active:
            if writer:
                writer.release()
                writer = None
            recording_active = False
            print("[Recorder] Wildlife event ended. Video saved.")
            ensure_storage_headroom()


# -------------------------------------------------------------------------
# Vision, Tracking & Turret Control Loop
# -------------------------------------------------------------------------
def vision_thread():
    global latest_jpeg, actual_sensor_fps, primary_target

    ensure_storage_headroom()

    picam2 = Picamera2()

    # Hardware Tuning for Dynamic Lighting:
    # 1. Output binned 2304x1296 mode for superior SNR without sensor cropping.
    # 2. Re-enable Auto Exposure (AeEnable=True) to prevent washing out in daylight.
    # 3. Allow exposure to scale automatically down to 10 FPS (100ms) in twilight.
    cam_config = picam2.create_video_configuration(
        sensor={"output_size": (2304, 1296)},
        main={"size": (INFER_SIZE, INFER_SIZE), "format": "RGB888"},
        buffer_count=6,
        controls={
            "AeEnable": True,
            "AwbMode": 1,
            "AeExposureMode": 0,
            "FrameDurationLimits": (33333, 100000)
        },
        transform=Transform(vflip=True)
    )
    picam2.configure(cam_config)
    picam2.start()
    time.sleep(1.0)

    infer_queue = queue.Queue(maxsize=1)
    record_queue = queue.Queue(maxsize=120)
    turret_queue = queue.Queue(maxsize=10)

    threading.Thread(target=hailo_worker, args=(infer_queue,), daemon=True).start()
    threading.Thread(target=video_recorder_worker, args=(record_queue,), daemon=True).start()
    threading.Thread(target=turret_serial_worker, args=(turret_queue,), daemon=True).start()

    buffer_capacity = int(RECORD_PRE_ROLL_SEC * TARGET_FPS)
    pre_roll_buffer = deque(maxlen=buffer_capacity)

    fps_time = time.time()
    frame_count = 0
    display_fps = 0.0

    last_detection_time = 0.0
    is_recording = False

    # Virtual targeting zero (Camera optical center + Boresight offset)
    gun_aim_x = OPTICAL_CENTER[0] + GUN_OFFSET_PAN_PX
    gun_aim_y = OPTICAL_CENTER[1] + GUN_OFFSET_TILT_PX

    # Internal integrated step targets
    current_target_pan_step = 0
    current_target_tilt_step = 0

    while True:
        frame_raw = picam2.capture_array("main")

        # Hand off to Hailo-8 NPU if ready
        if not infer_queue.full():
            infer_queue.put_nowait(frame_raw.copy())

        # FPS Telemetry
        frame_count += 1
        now = time.time()
        if now - fps_time >= 1.0:
            display_fps = frame_count / (now - fps_time)
            actual_sensor_fps = display_fps
            frame_count = 0
            fps_time = now

        # -------------------------------------------------------------
        # Target Selection & Tracking Hysteresis
        # -------------------------------------------------------------
        with detections_lock:
            current_dets = list(active_detections)
            lat = npu_latency_ms
            n_fps = npu_fps

        matched_target = None

        if primary_target is not None:
            # Check if previous target is still within timeout
            if (now - primary_target["last_seen"]) < TARGET_TIMEOUT_SEC:
                # Find nearest detection to previous target centroid
                best_dist = float("inf")
                prev_cx, prev_cy = primary_target["center"]

                for det in current_dets:
                    dcx, dcy = det["center"]
                    dist = math.hypot(dcx - prev_cx, dcy - prev_cy)
                    if dist < MAX_CENTROID_JUMP_PX and dist < best_dist:
                        best_dist = dist
                        matched_target = det

                if matched_target:
                    primary_target["center"] = matched_target["center"]
                    primary_target["box_1280"] = matched_target["box_1280"]
                    primary_target["score"] = matched_target["score"]
                    primary_target["last_seen"] = now
            else:
                primary_target = None  # Lost target timed out

        # Acquire new target if none actively tracked
        if primary_target is None and len(current_dets) > 0:
            # Select highest confidence detection
            best_det = max(current_dets, key=lambda d: d["score"])
            primary_target = {
                "center": best_det["center"],
                "box_1280": best_det["box_1280"],
                "score": best_det["score"],
                "last_seen": now
            }
            matched_target = best_det

        # -------------------------------------------------------------
        # Turret Tracking & Firing Gating
        # -------------------------------------------------------------
        target_locked = False
        target_in_deadband = False

        if primary_target and (now - primary_target["last_seen"] < 0.4):
            target_locked = True
            last_detection_time = now
            tcx, tcy = primary_target["center"]

            # Error relative to gun boresight zero
            error_x = tcx - gun_aim_x
            error_y = tcy - gun_aim_y

            # Convert pixel error to stepper increments
            pan_delta_steps = int(error_x * STEPS_PER_PIXEL_PAN)
            tilt_delta_steps = int(error_y * STEPS_PER_PIXEL_TILT)

            current_target_pan_step += pan_delta_steps
            current_target_tilt_step += tilt_delta_steps

            # Clamp to physical envelopes
            current_target_pan_step = max(PAN_MIN_STEPS, min(PAN_MAX_STEPS, current_target_pan_step))
            current_target_tilt_step = max(TILT_MIN_STEPS, min(TILT_MAX_STEPS, current_target_tilt_step))

            # Send movement command to ESP32
            turret_queue.put({
                "cmd": "MOVE",
                "pan": current_target_pan_step,
                "tilt": current_target_tilt_step
            })

            # Evaluate centering for trigger
            radial_error = math.hypot(error_x, error_y)
            if radial_error <= FIRE_DEADBAND_PX:
                target_in_deadband = True
                turret_queue.put({"cmd": "TRIGGER", "state": 1})
            else:
                turret_queue.put({"cmd": "TRIGGER", "state": 0})

        else:
            # Disengage trigger when target is lost
            turret_queue.put({"cmd": "TRIGGER", "state": 0})

        # -------------------------------------------------------------
        # Video Recording State
        # -------------------------------------------------------------
        if target_locked:
            if not is_recording:
                is_recording = True
                record_queue.put({"cmd": "start", "pre_roll": list(pre_roll_buffer)})

        if is_recording:
            if not record_queue.full():
                record_queue.put({"cmd": "frame", "frame": frame_raw.copy()})

            if not target_locked and (now - last_detection_time > RECORD_POST_ROLL_SEC):
                is_recording = False
                record_queue.put({"cmd": "stop"})
        else:
            pre_roll_buffer.append(frame_raw.copy())

        # -------------------------------------------------------------
        # HUD Display Construction
        # -------------------------------------------------------------
        annotated = cv2.resize(frame_raw, (DISPLAY_SIZE, DISPLAY_SIZE), interpolation=cv2.INTER_NEAREST)

        # Draw Optical Center (Cyan Crosshair)
        cx_disp = int(OPTICAL_CENTER[0] * SCALE_FACTOR)
        cy_disp = int(OPTICAL_CENTER[1] * SCALE_FACTOR)
        cv2.drawMarker(annotated, (cx_disp, cy_disp), (255, 255, 0), cv2.MARKER_CROSS, 16, 1)

        # Draw Calibrated Gun Boresight (Red Crosshair + Deadband Ring)
        aim_disp_x = int(gun_aim_x * SCALE_FACTOR)
        aim_disp_y = int(gun_aim_y * SCALE_FACTOR)
        deadband_disp_r = int(FIRE_DEADBAND_PX * SCALE_FACTOR)

        ring_color = (0, 0, 255) if not target_in_deadband else (0, 255, 255)
        cv2.circle(annotated, (aim_disp_x, aim_disp_y), deadband_disp_r, ring_color, 1)
        cv2.drawMarker(annotated, (aim_disp_x, aim_disp_y), (0, 0, 255), cv2.MARKER_CROSS, 24, 1)

        # Render all detected animals
        for det in current_dets:
            x1, y1, x2, y2 = [int(v * SCALE_FACTOR) for v in det["box_1280"]]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (180, 180, 180), 1)

        # Render Active Locked Target in Vivid Green
        if primary_target:
            tx1, ty1, tx2, ty2 = [int(v * SCALE_FACTOR) for v in primary_target["box_1280"]]
            tcx_d = int(primary_target["center"][0] * SCALE_FACTOR)
            tcy_d = int(primary_target["center"][1] * SCALE_FACTOR)

            cv2.rectangle(annotated, (tx1, ty1), (tx2, ty2), (0, 255, 0), 2)
            cv2.line(annotated, (aim_disp_x, aim_disp_y), (tcx_d, tcy_d), (0, 255, 0), 1)
            cv2.circle(annotated, (tcx_d, tcy_d), 4, (0, 255, 0), -1)

            status_lbl = f"LOCKED {primary_target['score']*100:.0f}%"
            cv2.putText(annotated, status_lbl, (tx1, max(ty1 - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

        # Firing Indicator
        if is_firing:
            cv2.putText(annotated, "SOLENOID ENGAGED", (aim_disp_x - 70, aim_disp_y - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

        if is_recording:
            cv2.circle(annotated, (DISPLAY_SIZE - 25, 25), 10, (0, 0, 255), -1)
            cv2.putText(annotated, "REC", (DISPLAY_SIZE - 65, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

        # Telemetry Text
        status_text = f"SENSOR: {display_fps:.1f} FPS | NPU: {n_fps:.1f} FPS | STEPS: P:{turret_pan_step} T:{turret_tilt_step}"
        cv2.putText(annotated, status_text, (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        success, buffer = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if success:
            with frame_lock:
                latest_jpeg = buffer.tobytes()


# -------------------------------------------------------------------------
# Flask Server & Routes
# -------------------------------------------------------------------------
@app.route("/")
def index():
    return render_template_string("""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Bombardeer Active Targeting HUD</title>
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
            h1 { margin: 0 0 12px 0; font-size: 1.2rem; letter-spacing: 0.05em; text-transform: uppercase; color: #58a6ff; }
            img { border-radius: 4px; background: #000; width: 640px; height: 640px; }
            .meta { margin-top: 10px; font-size: 0.82rem; color: #8b949e; }
        </style>
    </head>
    <body>
        <div class="hud-card">
            <h1>Bombardeer Active Wildlife Turret</h1>
            <img src="/video_feed" alt="Targeting Stream">
            <div class="meta">FastAccelStepper Control | Hysteresis Lock | 24V Solenoid Gating</div>
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
        time.sleep(0.05)


@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/favicon.ico")
def favicon():
    if os.path.exists(os.path.join(STATIC_DIR, "bombardeer.ico")):
        return send_from_directory(STATIC_DIR, "bombardeer.ico", mimetype="image/vnd.microsoft.icon")
    return ("", 204)


if __name__ == "__main__":
    t = threading.Thread(target=vision_thread, daemon=True)
    t.start()
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)