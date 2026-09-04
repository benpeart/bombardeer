#!/usr/bin/env python3
"""
Bombardeer Wildlife Targeting & Archival Vision System
------------------------------------------------------
Priorities:
1. Low-Light Dawn/Dusk Detection Accuracy (Dynamic AEC with deep exposure ceiling).
2. High-Quality Event-Based Hardware Video Recording (Pre-roll circular buffer).
3. Balanced hardware utilization (IMX708 ISP + Hailo-8 NPU + Pi5 DMA).
"""

import os
import time
import datetime
import threading
import queue
from collections import deque
import numpy as np
import cv2
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
# Configuration
# -------------------------------------------------------------------------
MODEL_PATH = "/home/ben/Bombardeer/MDV6-yolov9-c-1280.hef"
if not os.path.exists(MODEL_PATH):
    MODEL_PATH = "/home/ben/Bombardeer/models/MDV6-yolov9-c-1280.hef"

STATIC_DIR = "/home/ben/Bombardeer/static"
RECORDINGS_DIR = "/home/ben/Bombardeer/recordings"
os.makedirs(RECORDINGS_DIR, exist_ok=True)

INFER_SIZE = 1280
OPTICAL_CENTER_INFER = (INFER_SIZE // 2, INFER_SIZE // 2)

DISPLAY_SIZE = 640
SCALE_FACTOR = DISPLAY_SIZE / INFER_SIZE
OPTICAL_CENTER_DISP = (DISPLAY_SIZE // 2, DISPLAY_SIZE // 2)

# Detection & Recording Parameters
CONF_THRESH = 0.28
IOU_THRESH = 0.45
RECORD_PRE_ROLL_SEC = 3.0    # Seconds preserved before trigger
RECORD_POST_ROLL_SEC = 5.0   # Seconds preserved after animal leaves
TARGET_FPS = 12.0            # Nominal target recording rate

app = Flask(__name__)
frame_lock = threading.Lock()
latest_jpeg = None

# Telemetry & Target Tracking
detections_lock = threading.Lock()
active_detections = []
npu_latency_ms = 0.0
npu_fps = 0.0
actual_sensor_fps = 0.0


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
                    "center": (cx, cy),
                    "error": (cx - OPTICAL_CENTER_INFER[0], cy - OPTICAL_CENTER_INFER[1]),
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
                ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(RECORDINGS_DIR, f"wildlife_{ts}.mp4")
                writer = cv2.VideoWriter(filename, fourcc, TARGET_FPS, (INFER_SIZE, INFER_SIZE))
                recording_active = True
                print(f"[Recorder] Wildlife event started. Writing: {filename}")

                # Flush pre-roll buffer to disk
                pre_roll = item.get("pre_roll", [])
                for f in pre_roll:
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


# -------------------------------------------------------------------------
# Vision Loop: Dynamic Low-Light Auto-Exposure Pipeline
# -------------------------------------------------------------------------
def vision_thread():
    global latest_jpeg, actual_sensor_fps

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

    threading.Thread(target=hailo_worker, args=(infer_queue,), daemon=True).start()
    threading.Thread(target=video_recorder_worker, args=(record_queue,), daemon=True).start()

    buffer_capacity = int(RECORD_PRE_ROLL_SEC * TARGET_FPS)
    pre_roll_buffer = deque(maxlen=buffer_capacity)

    fps_time = time.time()
    frame_count = 0
    display_fps = 0.0

    last_detection_time = 0
    is_recording = False

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

        # Target Assessment & Recording Trigger
        with detections_lock:
            current_dets = list(active_detections)
            lat = npu_latency_ms
            n_fps = npu_fps

        has_target = len(current_dets) > 0

        if has_target:
            last_detection_time = now
            if not is_recording:
                is_recording = True
                record_queue.put({
                    "cmd": "start",
                    "pre_roll": list(pre_roll_buffer)
                })

        if is_recording:
            if not record_queue.full():
                record_queue.put({"cmd": "frame", "frame": frame_raw.copy()})

            if not has_target and (now - last_detection_time > RECORD_POST_ROLL_SEC):
                is_recording = False
                record_queue.put({"cmd": "stop"})
        else:
            pre_roll_buffer.append(frame_raw.copy())

        # Subsample for Web HUD Display (640x640)
        annotated = cv2.resize(frame_raw, (DISPLAY_SIZE, DISPLAY_SIZE), interpolation=cv2.INTER_NEAREST)

        # Reticle
        cx_disp, cy_disp = OPTICAL_CENTER_DISP
        cv2.drawMarker(annotated, (cx_disp, cy_disp), (0, 0, 255), cv2.MARKER_CROSS, 24, 1)
        cv2.circle(annotated, (cx_disp, cy_disp), 40, (0, 0, 255), 1)

        # Draw Target Vectors
        for det in current_dets:
            x1, y1, x2, y2 = [int(v * SCALE_FACTOR) for v in det["box_1280"]]
            score = det["score"]
            tcx = int(det["center"][0] * SCALE_FACTOR)
            tcy = int(det["center"][1] * SCALE_FACTOR)
            dx, dy = det["error"]

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.line(annotated, (cx_disp, cy_disp), (tcx, tcy), (0, 255, 255), 1)
            cv2.circle(annotated, (tcx, tcy), 4, (0, 255, 0), -1)

            label = f"ANIMAL {score*100:.0f}% dx:{dx:+.0f} dy:{dy:+.0f}"
            cv2.putText(annotated, label, (x1, max(y1 - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

        # Recording Status Indicator
        if is_recording:
            cv2.circle(annotated, (DISPLAY_SIZE - 25, 25), 10, (0, 0, 255), -1)
            cv2.putText(annotated, "REC", (DISPLAY_SIZE - 65, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

        status_text = f"SENSOR: {display_fps:.1f} FPS | NPU: {n_fps:.1f} FPS ({lat:.1f}ms) | 1280p | Targets: {len(current_dets)}"
        cv2.putText(annotated, status_text, (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)

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
        <title>Bombardeer Wildlife Targeting HUD</title>
        <link rel="icon" type="image/x-icon" href="/favicon.ico">
        <style>
            body {
                margin: 0;
                padding: 0;
                background-color: #0d1117;
                color: #c9d1d9;
                font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
                display: flex;
                flex-direction: column;
                align-items: center;
                justify-content: center;
                min-height: 100vh;
            }
            .hud-card {
                background: #161b22;
                border: 1px solid #30363d;
                border-radius: 8px;
                padding: 16px;
                box-shadow: 0 8px 24px rgba(0, 0, 0, 0.5);
                text-align: center;
            }
            h1 {
                margin: 0 0 12px 0;
                font-size: 1.2rem;
                letter-spacing: 0.05em;
                text-transform: uppercase;
                color: #58a6ff;
            }
            img {
                border-radius: 4px;
                background: #000;
                width: 640px;
                height: 640px;
            }
            .meta {
                margin-top: 10px;
                font-size: 0.82rem;
                color: #8b949e;
            }
        </style>
    </head>
    <body>
        <div class="hud-card">
            <h1>Bombardeer Active Targeting HUD</h1>
            <img src="/video_feed" alt="Targeting Stream">
            <div class="meta">MegaDetector v6 1280p | Auto Exposure | Auto Event Recording</div>
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

        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
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