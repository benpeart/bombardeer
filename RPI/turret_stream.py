import io
import os
import threading
import time
import cv2
import numpy as np
from flask import Flask, Response, send_from_directory
from picamera2 import Picamera2
from hailo_platform import (
    HEF,
    ConfigureParams,
    FormatType,
    HailoStreamInterface,
    InferVStreams,
    InputVStreamParams,
    OutputVStreamParams,
    VDevice,
)

# Target configuration: Class 0 = Animal (Deer) in MegaDetector v6
TARGET_CLASS_ID = 0
CONF_THRESH = 0.40
IOU_THRESH = 0.50
MODEL_PATH = "/home/ben/Bombardeer/MDV6-yolov9-c.hef"
BASE_DIR = "/home/ben/Bombardeer"

app = Flask(__name__)
output_frame = None


class MDV6Decoder:
    """Decodes MegaDetector v6 multi-scale DFL bounding boxes and classification heads."""
    def __init__(self, input_size=640, reg_max=16):
        self.input_size = input_size
        self.reg_max = reg_max
        self.strides = [8, 16, 32]
        self.project = np.arange(reg_max, dtype=np.float32)
        self.grids, self.stride_factors = self._generate_anchors()

    def _generate_anchors(self):
        grids, strides = [], []
        for s in self.strides:
            dim = self.input_size // s
            gx, gy = np.meshgrid(np.arange(dim), np.arange(dim))
            grid = np.stack([gx, gy], axis=-1).reshape(-1, 2) + 0.5
            grids.append(grid)
            strides.append(np.full((dim * dim, 1), s, dtype=np.float32))
        return np.vstack(grids), np.vstack(strides)

    def decode(self, raw_outputs):
        # Stride 8 (80x80)
        b8 = raw_outputs["MDV6-yolov9-c/conv98"].reshape(-1, 4, self.reg_max)
        c8 = raw_outputs["MDV6-yolov9-c/conv100"].reshape(-1, 3)

        # Stride 16 (40x40)
        b16 = raw_outputs["MDV6-yolov9-c/conv121"].reshape(-1, 4, self.reg_max)
        c16 = raw_outputs["MDV6-yolov9-c/conv122"].reshape(-1, 3)

        # Stride 32 (20x20)
        b32 = raw_outputs["MDV6-yolov9-c/conv143"].reshape(-1, 4, self.reg_max)
        c32 = raw_outputs["MDV6-yolov9-c/conv144"].reshape(-1, 3)

        raw_boxes = np.vstack([b8, b16, b32])
        raw_scores = np.vstack([c8, c16, c32])

        # Softmax over regression distribution bins
        exp_boxes = np.exp(raw_boxes - np.max(raw_boxes, axis=-1, keepdims=True))
        dist = np.dot(exp_boxes / np.sum(exp_boxes, axis=-1, keepdims=True), self.project)

        # Reconstruct coordinates [x1, y1, x2, y2]
        x1 = (self.grids[:, 0] - dist[:, 0]) * self.stride_factors[:, 0]
        y1 = (self.grids[:, 1] - dist[:, 1]) * self.stride_factors[:, 0]
        x2 = (self.grids[:, 0] + dist[:, 2]) * self.stride_factors[:, 0]
        y2 = (self.grids[:, 1] + dist[:, 3]) * self.stride_factors[:, 0]
        boxes = np.stack([x1, y1, x2, y2], axis=-1)

        # Sigmoid on classification logits
        scores = 1.0 / (1.0 + np.exp(-raw_scores))
        animal_scores = scores[:, TARGET_CLASS_ID]
        mask = animal_scores > CONF_THRESH

        filtered_boxes = boxes[mask]
        filtered_scores = animal_scores[mask]

        if len(filtered_boxes) == 0:
            return []

        # Convert to [x, y, w, h] for NMS
        xywh = filtered_boxes.copy()
        xywh[:, 2] -= xywh[:, 0]
        xywh[:, 3] -= xywh[:, 1]

        indices = cv2.dnn.NMSBoxes(xywh.tolist(), filtered_scores.tolist(), CONF_THRESH, IOU_THRESH)

        detections = []
        for idx in indices:
            i = idx[0] if isinstance(idx, (list, tuple, np.ndarray)) else idx
            bx = filtered_boxes[i]
            cx = (bx[0] + bx[2]) / 2.0
            cy = (bx[1] + bx[3]) / 2.0
            detections.append({
                "box": bx.astype(int),
                "center": (cx, cy),
                "score": float(filtered_scores[i])
            })
        return detections


def tracking_worker():
    global output_frame
    decoder = MDV6Decoder(input_size=640)

    # 1. Start Picamera2 with 640x640 RGB hardware ISP stream
    picam2 = Picamera2()
    cam_config = picam2.create_video_configuration(
        main={"size": (640, 640), "format": "RGB888"},
        controls={"FrameRate": 25}
    )
    picam2.configure(cam_config)
    picam2.start()
    time.sleep(1.0)
    print("[Camera] Picamera2 ISP active at 640x640 RGB @ 25 FPS")

    # 2. Configure Hailo-8 Device & Pipelines
    hef = HEF(MODEL_PATH)
    params = VDevice.create_params()
    with VDevice(params) as target:
        configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
        network_group = target.configure(hef, configure_params)[0]
        network_group_params = network_group.create_params()

        input_vparams = InputVStreamParams.make(network_group, quantized=False, format_type=FormatType.UINT8)
        output_vparams = OutputVStreamParams.make(network_group, quantized=False, format_type=FormatType.FLOAT32)
        input_name = hef.get_input_vstream_infos()[0].name

        # Explicitly activate network group for 4-context model
        with network_group.activate(network_group_params):
            with InferVStreams(network_group, input_vparams, output_vparams) as pipeline:
                print("[Hailo-8] All 4 contexts activated. Running inference...")
                fps_start = time.perf_counter()
                frames = 0
                fps_display = "0.0 FPS"

                while True:
                    # Capture unbatched (640, 640, 3) frame
                    frame_rgb = picam2.capture_array("main")

                    # Add batch dimension: shape becomes (1, 640, 640, 3)
                    input_tensor = np.expand_dims(frame_rgb, axis=0)

                    # Hardware inference
                    raw_outputs = pipeline.infer({input_name: input_tensor})
                    detections = decoder.decode(raw_outputs)

                    # Visual HUD rendering
                    vis_frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

                    # Optical center reticle (320, 320)
                    cv2.drawMarker(vis_frame, (320, 320), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)

                    for det in detections:
                        x1, y1, x2, y2 = det["box"]
                        cx, cy = int(det["center"][0]), int(det["center"][1])
                        score = det["score"]

                        # Green target bounding box
                        cv2.rectangle(vis_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        # Red target center dot
                        cv2.circle(vis_frame, (cx, cy), 4, (0, 0, 255), -1)
                        # Yellow tracking displacement vector
                        cv2.line(vis_frame, (320, 320), (cx, cy), (0, 255, 255), 1)

                        dx = cx - 320
                        dy = cy - 320
                        label = f"Deer: {score:.2f} (dx:{dx:+d}, dy:{dy:+d})"
                        cv2.putText(vis_frame, label, (x1, max(y1 - 8, 15)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

                    frames += 1
                    if frames % 15 == 0:
                        fps_val = 15.0 / (time.perf_counter() - fps_start)
                        fps_display = f"{fps_val:.1f} FPS"
                        fps_start = time.perf_counter()

                    # System telemetry overlay
                    cv2.putText(vis_frame, f"BOMBARDEER | {fps_display}", (15, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 204), 2)

                    ret, buffer = cv2.imencode(".jpg", vis_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if ret:
                        output_frame = buffer.tobytes()


def generate_mjpeg():
    global output_frame
    while True:
        if output_frame is not None:
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + output_frame + b"\r\n")
        time.sleep(0.02)


@app.route("/favicon.ico")
def favicon():
    icon_path = os.path.join(BASE_DIR, "bombardeer.ico")
    if os.path.exists(icon_path):
        return send_from_directory(BASE_DIR, "bombardeer.ico", mimetype="image/vnd.microsoft.icon")
    return ("", 204)


@app.route("/")
def index():
    return """
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <title>Bombardeer HUD</title>
        <link rel="shortcut icon" href="/favicon.ico">
        <style>
            body {
                background: #0a0d14;
                color: #e2e8f0;
                font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
                margin: 0;
                display: flex;
                flex-direction: column;
                align-items: center;
                justify-content: center;
                min-height: 100vh;
            }
            .hud-container {
                text-align: center;
                padding: 20px;
            }
            h1 {
                color: #00ffcc;
                letter-spacing: 5px;
                font-size: 24px;
                margin-bottom: 15px;
                text-shadow: 0 0 10px rgba(0, 255, 204, 0.4);
            }
            .video-box {
                border: 2px solid #253346;
                border-radius: 12px;
                box-shadow: 0 0 30px rgba(0, 255, 204, 0.15);
                max-width: 90vw;
                height: auto;
            }
            .status {
                margin-top: 12px;
                font-size: 13px;
                color: #718096;
                letter-spacing: 2px;
            }
        </style>
    </head>
    <body>
        <div class="hud-container">
            <h1>BOMBARDEER TURRET HUD</h1>
            <img class="video-box" src="/video_feed" alt="Turret Video Stream" />
            <div class="status">HAILO-8 AI ACCELERATED | 640x640 RGB</div>
        </div>
    </body>
    </html>
    """


@app.route("/video_feed")
def video_feed():
    return Response(generate_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    t = threading.Thread(target=tracking_worker, daemon=True)
    t.start()
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
