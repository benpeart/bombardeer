"""
VisionRuntime orchestrator + Core-0 VisionMain loop + Core-1 HudEncoder thread.
- Spawns HailoNPUProc BEFORE starting worker threads (deadlock-free spawn).
- MappedArray minimal-ROI capture into a reusable scratch buffer (zero per-frame allocs, DMA-safe).
- PiSP hardware BGR888 (RGB in memory) skips CPU BGR2RGB conversion.
- Projects Kalman world state back into exact capture_mono image coordinates for TRK-1:1 crops,
  safety vetoes, tentative confirmation, and real-time HUD overlays.
- Drives systemd watchdog (sd_notify) and fenced monotonic Day/Dusk sensor switching.
"""

from collections import deque
from dataclasses import replace
import gc
import logging
import math
import os
import queue
import threading
import time
import traceback

import cv2
import numpy as np
from libcamera import Transform
from picamera2 import MappedArray, Picamera2

from bombardeer_pkg.config import BombardeerConfig
from bombardeer_pkg.control.ballistics import (
    compute_drag_ballistics,
    compute_deer_head_ceiling_mrad,
    compute_horizon_y_720,
    compute_target_kinematics,
    image_px_to_world_mrad,
    project_world_to_image_px,
)
from bombardeer_pkg.control.motion_servo import motion_servoing_worker
from bombardeer_pkg.hardware.recorder import ensure_storage_headroom, video_recorder_worker
from bombardeer_pkg.hardware.serial_link import turret_serial_worker
from bombardeer_pkg.perception.aruco import ArucoTargetDetector
from bombardeer_pkg.perception.hailo_detector import HailoWildlifeDetector
from bombardeer_pkg.perception.mdv6_decoder import MDV6Decoder1280
from bombardeer_pkg.state import SystemState
from bombardeer_pkg.utils import (
    RateLimitedLogger,
    ensure_storage_dirs,
    save_audit_snapshot,
    sd_notify,
    set_os_thread_name,
)

log = logging.getLogger("Bombardeer")

NOIR_TUNING_CANDIDATES = (
    "/usr/share/libcamera/ipa/rpi/pisp/imx708_wide_noir.json",
    "/usr/share/libcamera/ipa/rpi/vc4/imx708_wide_noir.json",
    "/usr/share/libcamera/ipa/rpi/pisp/imx708_noir.json",
    "/usr/share/libcamera/ipa/rpi/vc4/imx708_noir.json",
)


def _build_camera_config(picam2, cfg: BombardeerConfig, sensor_mode: str, is_monochrome: bool, ev: float):
    cam = cfg.camera
    if sensor_mode == "DAY_12MP":
        sensor_size = cam.sensor_day_size
        buf_count = 4
        frame_limits = (69686, 100000)
    else:
        sensor_size = cam.sensor_dusk_size
        buf_count = 6
        frame_limits = (33333, 83333)

    main_fmt = "BGR888" if cam.main_stream_is_rgb else "RGB888"
    controls = {
        "AeEnable": True,
        "HdrMode": 0 if is_monochrome else 2,
        "AeMeteringMode": 0,
        "AwbMode": 0,
        "NoiseReductionMode": 2,
        "ExposureValue": ev,
        "FrameDurationLimits": frame_limits,
        "Contrast": 1.15 if is_monochrome else 1.05,
        "Saturation": 0.0 if is_monochrome else 0.85,
        "Sharpness": 0.90 if is_monochrome else 1.15,
        "AfMode": 0,
        "LensPosition": cam.manual_lens_position,
    }
    return picam2.create_video_configuration(
        sensor={"output_size": sensor_size},
        main={"size": sensor_size, "format": main_fmt},
        lores={"size": (cam.frame_width, cam.frame_height), "format": "RGB888"},
        buffer_count=buf_count,
        controls=controls,
        transform=Transform(hflip=True, vflip=True),
    )


def _estimate_capture_mono(metadata: dict, arrival_mono: float) -> float:
    exp_s = float(metadata.get("ExposureTime", 33333)) * 1e-6
    sensor_ts_ns = metadata.get("SensorTimestamp")
    if sensor_ts_ns is not None and hasattr(time, "CLOCK_BOOTTIME"):
        try:
            boot_now = time.clock_gettime(time.CLOCK_BOOTTIME)
            frame_age = boot_now - (float(sensor_ts_ns) * 1e-9)
            if 0.0 <= frame_age <= 0.500:
                return arrival_mono - frame_age - (0.5 * exp_s)
        except Exception:
            pass
    return arrival_mono - 0.020 - (0.5 * exp_s)


def _reconstruct_box_at_center(px: float, py: float, ref_box, ref_center=None):
    """Rebuilds a bounding box around projected (px, py), preserving aim-point vertical offset."""
    bx1, by1, bx2, by2 = ref_box
    bw = max(20.0, float(bx2 - bx1))
    bh = max(20.0, float(by2 - by1))
    if ref_center is not None:
        dy_top = max(4.0, min(bh - 4.0, float(ref_center[1]) - float(by1)))
        dy_bot = max(4.0, bh - dy_top)
    else:
        dy_top = 0.6 * bh
        dy_bot = 0.4 * bh
    return [px - 0.5 * bw, py - dy_top, px + 0.5 * bw, py + dy_bot]


def _select_target_candidate(detections, cur_primary, ref_points, max_jump_px: float):
    """
    Pick the nearest detection to the locked target's projected image position(s)
    to prevent jumping between multiple deer while tolerating sudden stops after
    350-700 ms scan gaps.
    """
    if not detections:
        return None
    if cur_primary is None:
        return detections[0]

    pts = [p for p in ref_points if p is not None]
    if not pts:
        pts = [cur_primary["center"]]

    best = None
    best_dist = float("inf")
    for cand in detections:
        cx, cy = cand["center"]
        dist = min(math.hypot(cx - rx, cy - ry) for rx, ry in pts)
        if dist <= max_jump_px and dist < best_dist:
            best_dist = dist
            best = cand
    return best


def _hud_encoder_worker(cfg: BombardeerConfig, state: SystemState, hud_queue: queue.Queue):
    set_os_thread_name("HudEncoder", cpu_cores=[1])
    cam, optics, ctrl, bal = cfg.camera, cfg.optics, cfg.control, cfg.ballistics
    sx, sy = cam.scale_x, cam.scale_y
    oc_x, oc_y = int(cam.optical_center[0] * sx), int(cam.optical_center[1] * sy)

    while not state.shutdown_event.is_set():
        try:
            pkt = hud_queue.get(timeout=0.2)
        except queue.Empty:
            continue

        (
            frame_lores,
            detections,
            debug_info,
            batch_capture_mono,
            local_target,
            sensor_fps,
            sensor_mode,
            det_name,
            mono_now,
        ) = pkt
        annotated = cv2.resize(frame_lores, (cam.display_width, cam.display_height), interpolation=cv2.INTER_LINEAR)
        cur_mode, is_primed = state.get_turret_mode()
        p_mrad, t_mrad, _, firing = state.get_instantaneous_telemetry()
        health = state.get_health_snapshot()
        npu_lat_sec = state.get_npu_latency_sec()
        lat_allowance_sec = max(0.160, npu_lat_sec + 0.080)
        effective_coast_horizon = max(ctrl.coast_max_horizon_sec, npu_lat_sec + 1.0)
        rng_m = local_target.get("range_m", optics.default_fallback_range_m) if local_target else optics.default_fallback_range_m

        metric_optics, yard_top_mrad, roll_mrad, _ = state.site_calibration.get_ground_plane_params(optics, p_mrad)
        _, far_deer_head_mrad, _ = compute_deer_head_ceiling_mrad(metric_optics, far_ground_mrad=yard_top_mrad)
        deer_ceil_optics = replace(metric_optics, ground_slope_mrad=far_deer_head_mrad, horizon_y_offset_720=0.0)
        for hx in range(0, cam.display_width, 16):
            hx_720 = hx / sx if sx > 0 else float(hx)
            hy_metric = int(round(compute_horizon_y_720(t_mrad, cam, metric_optics, x_px=hx_720, roll_mrad=roll_mrad) * sy))
            if 0 < hy_metric < cam.display_height:
                cv2.line(annotated, (hx, hy_metric), (min(cam.display_width, hx + 8), hy_metric), (120, 110, 40), 1)
            hy_deer = int(round(compute_horizon_y_720(t_mrad, cam, deer_ceil_optics, x_px=hx_720, roll_mrad=roll_mrad) * sy))
            if 0 < hy_deer < cam.display_height and abs(hy_deer - hy_metric) >= 4:
                cv2.line(annotated, (hx, hy_deer), (min(cam.display_width, hx + 6), hy_deer), (40, 140, 180), 1)

        cv2.drawMarker(annotated, (oc_x, oc_y), (0, 255, 255), cv2.MARKER_CROSS, 16, 1)

        _, drop_mrad, _, _ = compute_drag_ballistics(rng_m, t_mrad, 0.0, 0.0, bal, optics)
        total_tilt_adj = drop_mrad + optics.tilt_calibration_trim_mrad
        conv_x = cam.optical_center[0] + int((-optics.camera_offset_x_mm / (rng_m * 1000.0)) * optics.focal_length_px)
        conv_y = cam.optical_center[1] + int(
            ((optics.camera_offset_y_mm / (rng_m * 1000.0)) * 1000.0 - total_tilt_adj) * (optics.focal_length_px / 1000.0)
        )
        aim_x, aim_y = int(conv_x * sx), int(conv_y * sy)
        active_db = ctrl.calibrate_deadband_mrad if cur_mode == "CALIBRATE" else ctrl.fire_deadband_mrad
        db_radius = int((active_db / optics.mrad_per_pixel) * sx)

        ring_color = {
            "ARMED": (0, 0, 255),
            "CALIBRATE": (0, 165, 255) if is_primed else (100, 100, 100),
            "ON": (255, 200, 0),
        }.get(cur_mode, (0, 200, 50))
        if firing:
            ring_color = (0, 255, 255)

        cv2.circle(annotated, (aim_x, aim_y), db_radius, ring_color, 1)
        cv2.drawMarker(annotated, (aim_x, aim_y), ring_color, cv2.MARKER_CROSS, 24, 1)

        # Suppress raw detection/veto ghost overlays once older than one scan cycle + margin
        raw_overlay_fresh = batch_capture_mono > 0.0 and ((mono_now - batch_capture_mono) <= max(0.60, npu_lat_sec + 0.35))
        if raw_overlay_fresh:
            for v in debug_info.get("veto_boxes", []):
                vx1, vy1, vx2, vy2 = int(v["box"][0] * sx), int(v["box"][1] * sy), int(v["box"][2] * sx), int(v["box"][3] * sy)
                cv2.rectangle(annotated, (vx1, vy1), (vx2, vy2), (0, 0, 255), 1)
                cv2.putText(annotated, f"VETO:{v['class']} {v['score']*100:.0f}%", (vx1, max(vy1 - 4, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1, cv2.LINE_AA)

            for d in detections:
                cv2.rectangle(annotated, (int(d["box"][0] * sx), int(d["box"][1] * sy)),
                              (int(d["box"][2] * sx), int(d["box"][3] * sy)), (180, 180, 180), 1)

        tgt_cap_age = (
            max(0.0, mono_now - local_target.get("capture_mono", local_target["last_seen_mono"]))
            if local_target
            else 1e9
        )
        if local_target and (tgt_cap_age < effective_coast_horizon):
            max_arrival_gap = max(ctrl.fire_data_freshness_sec, npu_lat_sec + 0.120)
            is_fresh = (
                tgt_cap_age <= (ctrl.fire_data_freshness_sec + lat_allowance_sec)
                and (mono_now - local_target["last_seen_mono"]) <= max_arrival_gap
            )
            box_color = (0, 255, 0) if is_fresh else (0, 165, 255)

            # Project locked target into live turret pose so the HUD box stays aligned during slews
            disp_box = local_target["box"]
            disp_center = local_target["center"]
            pred_hud = state.target_filter.predict_state(mono_now)
            if pred_hud is not None:
                hx, hy = project_world_to_image_px(
                    pred_hud[0], pred_hud[1], p_mrad, t_mrad, rng_m, cam, optics
                )
                disp_center = (hx, hy)
                disp_box = _reconstruct_box_at_center(hx, hy, local_target["box"], local_target["center"])

            tx1, ty1 = int(disp_box[0] * sx), int(disp_box[1] * sy)
            tx2, ty2 = int(disp_box[2] * sx), int(disp_box[3] * sy)
            cv2.rectangle(annotated, (tx1, ty1), (tx2, ty2), box_color, 2)
            cv2.circle(annotated, (int(disp_center[0] * sx), int(disp_center[1] * sy)), 4, box_color, -1)
            cv2.putText(annotated, f"{local_target.get('label', 'TGT')} | {rng_m:.1f}m",
                        (tx1, max(ty1 - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_color, 1, cv2.LINE_AA)

            border = (0, 200, 0) if is_fresh else (0, 140, 255)
            cv2.rectangle(annotated, (cam.display_width - 215, 10), (cam.display_width - 10, 42), (18, 18, 18), -1)
            cv2.rectangle(annotated, (cam.display_width - 215, 10), (cam.display_width - 10, 42), border, 1)
            cv2.putText(annotated, f"TGT: {rng_m * 3.28084:4.1f}ft ({rng_m:3.1f}m)",
                        (cam.display_width - 208, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
        else:
            cv2.rectangle(annotated, (cam.display_width - 140, 10), (cam.display_width - 10, 36), (18, 18, 18), -1)
            cv2.rectangle(annotated, (cam.display_width - 140, 10), (cam.display_width - 10, 36), (60, 60, 60), 1)
            cv2.putText(annotated, "TGT: NONE", (cam.display_width - 130, 27),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 150), 1, cv2.LINE_AA)

        wm = {
            "ARMED": ("ARMED / LIVE", (0, 0, 180), 150),
            "CALIBRATE": ("TEST / PRIMED" if is_primed else "TEST / LOCKED", (0, 100, 200) if is_primed else (60, 60, 60), 165),
            "ON": ("TRACKING / SAFE", (180, 110, 0), 165),
        }.get(cur_mode, ("OFF / SAFE", (0, 140, 40), 130))
        cv2.rectangle(annotated, (10, 10), (wm[2], 34), wm[1], -1)
        cv2.circle(annotated, (22, 22), 4, (255, 255, 255), -1)
        cv2.putText(annotated, wm[0], (32, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        if firing:
            cv2.putText(annotated, "SOLENOID ENGAGED", (aim_x - 75, aim_y - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
        if health["recorder_active"]:
            cv2.circle(annotated, (cam.display_width - 25, 60), 8, (0, 0, 255), -1)
            cv2.putText(annotated, "REC", (cam.display_width - 70, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

        ser_ok = health["serial_connected"]
        cam_tag = "12MP" if sensor_mode == "DAY_12MP" else "3MP"
        det_tel = (
            f"{cam_tag} {health['npu_scan_mode']}:{health['npu_fps']:.1f}Hz({health['npu_latency_ms']:.0f}ms)"
            if det_name == "WILDLIFE"
            else f"{cam_tag} DET:ARUCO"
        )
        status = f"FPS:{sensor_fps:.1f} | {det_tel} | P:{p_mrad} T:{t_mrad} | {'SER:OK' if ser_ok else 'SER:DISC'}"
        st_font = cv2.FONT_HERSHEY_SIMPLEX
        st_scale = 0.56
        st_thick = 2
        pad_x, pad_y = 10, 7
        (tw, th), baseline = cv2.getTextSize(status, st_font, st_scale, st_thick)
        sx1 = 10
        sy2 = cam.display_height - 10
        sy1 = max(0, sy2 - th - baseline - pad_y * 2)
        sx2 = min(cam.display_width - 10, sx1 + tw + pad_x * 2)
        roi = annotated[sy1:sy2, sx1:sx2]
        bg = np.full_like(roi, (22, 17, 13))
        cv2.addWeighted(bg, 0.72, roi, 0.28, 0.0, dst=roi)
        cv2.rectangle(annotated, (sx1, sy1), (sx2, sy2), (61, 54, 48), 1)
        st_color = (160, 255, 160) if ser_ok else (110, 110, 255)
        cv2.putText(
            annotated,
            status,
            (sx1 + pad_x, sy2 - pad_y - baseline),
            st_font,
            st_scale,
            st_color,
            st_thick,
            cv2.LINE_AA,
        )

        alert_msg = state.site_calibration.get_hud_banner() or health.get("active_alert")
        if alert_msg:
            ay2 = max(26, sy1 - 6)
            ay1 = max(0, ay2 - 26)
            cv2.rectangle(annotated, (0, ay1), (cam.display_width, ay2), (0, 0, 180), -1)
            cv2.putText(
                annotated,
                alert_msg,
                (12, ay2 - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        ok, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if ok:
            state.set_latest_jpeg(buf.tobytes())


class VisionRuntime:
    """Lifecycle manager started/stopped by bombardeer.py."""

    def __init__(self, cfg: BombardeerConfig, state: SystemState):
        self.cfg = cfg
        self.state = state
        self.record_queue = queue.Queue(maxsize=90)
        self.hud_queue = queue.Queue(maxsize=1)
        self.wildlife_detector = None
        self._threads = []

    def start(self):
        ensure_storage_dirs(self.cfg.recordings_dir, self.cfg.debug_snapshot_dir)
        ensure_storage_headroom(self.cfg, self.state)

        # Spawn HailoNPUProc BEFORE starting any threads (eliminates multithreaded IPC hazards)
        self.wildlife_detector = HailoWildlifeDetector(self.cfg, self.state)
        self.state.wildlife_detector = self.wildlife_detector
        self.wildlife_detector.set_active(self.state.get_active_detector_type() == "WILDLIFE")

        self._threads = [
            threading.Thread(target=turret_serial_worker, args=(self.cfg, self.state), daemon=True, name="SerialWorker"),
            threading.Thread(target=video_recorder_worker, args=(self.cfg, self.state, self.record_queue), daemon=True, name="RecorderWorker"),
            threading.Thread(target=motion_servoing_worker, args=(self.cfg, self.state), daemon=True, name="MotionWorker"),
            threading.Thread(target=_hud_encoder_worker, args=(self.cfg, self.state, self.hud_queue), daemon=True, name="HudEncoder"),
            threading.Thread(target=self._vision_main_loop, daemon=True, name="VisionMain"),
        ]
        for t in self._threads:
            t.start()

    def stop(self):
        self.state.shutdown_event.set()
        for t in self._threads:
            t.join(timeout=2.0)
        if self.wildlife_detector is not None:
            self.wildlife_detector.shutdown()
            self.wildlife_detector = None

    def _vision_main_loop(self):
        cfg, state = self.cfg, self.state
        cam, optics, ctrl, st = cfg.camera, cfg.optics, cfg.control, cfg.storage
        kf = state.target_filter
        rate_log = RateLimitedLogger(interval_sec=5.0)

        set_os_thread_name("VisionMain", cpu_cores=[0, 1, 2])
        detectors = {
            "ARUCO": ArucoTargetDetector(cfg),
            "WILDLIFE": self.wildlife_detector,
        }

        sensor_mode = "DUSK_3MP"
        is_monochrome = False
        last_ev = 0.0
        picam2 = None

        try:
            active_tuning = next((f for f in NOIR_TUNING_CANDIDATES if os.path.exists(f)), None)
            picam2 = Picamera2()
            if active_tuning:
                try:
                    picam2.load_tuning_file(active_tuning)
                    log.info(f"[CAMERA] Loaded NoIR tuning profile: {active_tuning}")
                except Exception as te:
                    log.warning(f"[CAMERA] Tuning load warning ({active_tuning}): {te}")

            picam2.configure(_build_camera_config(picam2, cfg, sensor_mode, is_monochrome, last_ev))
            picam2.start()
            state.update_health(sensor_mode=sensor_mode)
            time.sleep(0.4)
        except Exception as exc:
            log.critical(f"[CAMERA FATAL] Init failed: {exc}\n{traceback.format_exc()}")
            state.set_alert("FATAL: Camera init failed", duration=10.0)
            return

        fps_mono = time.monotonic()
        frame_count = 0
        sensor_fps = 0.0
        last_valid_range_m = optics.default_fallback_range_m
        last_seen_connected = False
        last_detector_type = state.get_active_detector_type()

        tentative_target = None  # dict with center, box, range_m, world_pan, world_tilt, capture_mono
        tentative_streak = 0
        last_processed_capture_mono = 0.0
        min_valid_capture_mono = 0.0
        last_veto_audit_mono = 0.0
        last_hud_push_mono = 0.0
        last_watchdog_mono = 0.0

        is_recording = False
        lock_consecutive_frames = 0
        last_detection_mono = 0.0
        recording_start_mono = 0.0
        last_standby_log_mono = 0.0

        last_isp_eval_mono = time.monotonic()
        sensor_switch_candidate = None
        sensor_switch_since_mono = 0.0
        fast_eval_until_mono = time.monotonic() + cam.sensor_boot_eval_sec + 3.0

        pre_roll_cap = int(st.record_pre_roll_sec * cam.target_fps)
        pre_roll_pool = np.empty((pre_roll_cap, cam.frame_height, cam.frame_width, 3), dtype=np.uint8)
        pre_roll_idx = 0
        pre_roll_count = 0
        roi_scratch = None
        npu_lores_history = deque(maxlen=12)

        try:
            while not state.shutdown_event.is_set():
                req_det = state.get_active_detector_type()
                active_detector = detectors.get(req_det, detectors["ARUCO"])
                sensor_size = cam.sensor_day_size if sensor_mode == "DAY_12MP" else cam.sensor_dusk_size
                plan_bundle = None
                frame_roi = None
                need_native = False
                capture_mono = 0.0
                pan_cap = 0
                tilt_cap = 0
                active_track_box = None

                # --- Stage 1: MappedArray minimal-ROI capture with exact post-capture pose ---
                req_frame = None
                m = None
                roi_view = None
                try:
                    req_frame = picam2.capture_request()
                    mono_now = time.monotonic()
                    frame_lores = req_frame.make_array("lores")
                    metadata = req_frame.get_metadata()
                    capture_mono = _estimate_capture_mono(metadata, mono_now)
                    pan_cap, tilt_cap = state.get_turret_pos_at_time(capture_mono)

                    need_native = (
                        active_detector.name == "WILDLIFE"
                        and self.wildlife_detector.is_ready_for_frame()
                    )
                    if need_native:
                        npu_lat_sec = state.get_npu_latency_sec()
                        track_hold = max(ctrl.track_crop_hold_sec, npu_lat_sec + 0.35)
                        cur_primary = state.get_primary_target()
                        if cur_primary is not None:
                            prim_age = max(0.0, capture_mono - cur_primary.get("capture_mono", cur_primary["last_seen_mono"]))
                            if prim_age <= track_hold:
                                pred = kf.predict_state(capture_mono)
                                if pred is not None:
                                    px, py = project_world_to_image_px(
                                        pred[0], pred[1], pan_cap, tilt_cap,
                                        cur_primary.get("range_m", last_valid_range_m), cam, optics
                                    )
                                    active_track_box = _reconstruct_box_at_center(
                                        px, py, cur_primary["box"], cur_primary["center"]
                                    )
                                else:
                                    active_track_box = cur_primary["box"]
                        elif tentative_target is not None:
                            tx, ty = project_world_to_image_px(
                                tentative_target["world_pan"],
                                tentative_target["world_tilt"],
                                pan_cap,
                                tilt_cap,
                                tentative_target["range_m"],
                                cam,
                                optics,
                            )
                            active_track_box = _reconstruct_box_at_center(
                                tx, ty, tentative_target["box"], tentative_target["center"]
                            )

                        plan_bundle = self.wildlife_detector.prepare_plan(
                            sensor_size[0], sensor_size[1], tilt_cap, active_track_box, pan_mrad=pan_cap
                        )
                        rx0, ry0, rw, rh = plan_bundle[2]
                        with MappedArray(req_frame, "main") as m:
                            roi_view = m.array[ry0:ry0 + rh, rx0:rx0 + rw, :]
                            needed = roi_view.size
                            if roi_scratch is None or roi_scratch.size < needed:
                                roi_scratch = np.empty(needed, dtype=roi_view.dtype)
                            frame_roi = roi_scratch[:needed].reshape(roi_view.shape)
                            np.copyto(frame_roi, roi_view)
                            roi_view = None
                        m = None
                except Exception as exc:
                    rate_log.error("cam_drop", f"[CAMERA] Capture dropped: {exc}")
                    state.shutdown_event.wait(timeout=0.01)
                    continue
                finally:
                    roi_view = None
                    m = None
                    if req_frame is not None:
                        try:
                            req_frame.release()
                        except Exception:
                            pass
                        req_frame = None

                cur_pan_mrad, cur_tilt_mrad, cur_hw_moving, _ = state.get_instantaneous_telemetry()
                cur_mode_loop, _ = state.get_turret_mode()
                safe_for_sweep = (
                    cur_mode_loop in ("OFF", "ON")
                    and (state.get_primary_target() is None)
                    and (tentative_streak == 0)
                    and (not is_recording)
                )
                state.site_calibration.step_boot_sweep(
                    frame_lores=frame_lores,
                    lux=float(metadata.get("Lux", 50.0)),
                    is_monochrome=is_monochrome,
                    pan_cap=cur_pan_mrad,
                    tilt_cap=cur_tilt_mrad,
                    hw_moving=cur_hw_moving,
                    state=state,
                    mono_now=mono_now,
                    safe_for_sweep=safe_for_sweep,
                )
                if need_native and frame_roi is not None:
                    npu_lores_history.append((capture_mono, frame_lores.copy()))
                if mono_now - last_watchdog_mono >= 2.0:
                    last_watchdog_mono = mono_now
                    sd_notify("WATCHDOG=1")

                dynamic_ev = 1.0 if cur_tilt_mrad > 60 else (0.25 if is_monochrome else 0.0)
                if last_ev != dynamic_ev:
                    try:
                        picam2.set_controls({"ExposureValue": dynamic_ev})
                        last_ev = dynamic_ev
                    except Exception as exc:
                        rate_log.warning("isp_ev", f"[ISP] EV control warning: {exc}")

                frame_count += 1
                if mono_now - fps_mono >= 1.0:
                    sensor_fps = frame_count / (mono_now - fps_mono)
                    frame_count = 0
                    fps_mono = mono_now
                    state.update_health(sensor_fps=round(sensor_fps, 1))

                # --- Stage 1b: ISP Low-Light & Day/Dusk Sensor Switcher (~1 Hz) ---
                if mono_now - last_isp_eval_mono >= 1.0:
                    last_isp_eval_mono = mono_now
                    lux = float(metadata.get("Lux", 50.0))
                    gain = float(metadata.get("AnalogueGain", 1.0))
                    exp_us = float(metadata.get("ExposureTime", 33333))
                    has_tgt = state.get_primary_target() is not None
                    npu_idle = (active_detector.name != "WILDLIFE") or self.wildlife_detector.is_idle()
                    safe_to_switch = (
                        (not has_tgt)
                        and (tentative_streak == 0)
                        and (not is_recording)
                        and npu_idle
                        and (not state.site_calibration.is_motion_owned())
                    )

                    if (lux < 25.0 or gain > 6.0) and not is_monochrome and not has_tgt:
                        try:
                            picam2.set_controls({
                                "HdrMode": 0, "Saturation": 0.0, "Contrast": 1.15,
                                "Sharpness": 0.90, "ExposureValue": 0.25,
                            })
                            is_monochrome = True
                            last_ev = 0.25
                            log.info(f"[ISP] Low-Light B&W ACTIVE (Lux:{lux:.1f}, Gain:{gain:.1f}x, Exp:{exp_us/1000:.1f}ms)")
                        except Exception as exc:
                            rate_log.warning("isp_bw", f"[ISP] Low-light update warning: {exc}")
                    elif (lux >= 40.0 and gain <= 6.5) and is_monochrome and not has_tgt:
                        try:
                            picam2.set_controls({
                                "HdrMode": 2, "Saturation": 0.85, "Contrast": 1.05,
                                "Sharpness": 1.15, "ExposureValue": 0.0,
                            })
                            is_monochrome = False
                            last_ev = 0.0
                            log.info(f"[ISP] Daylight HDR RESTORED (Lux:{lux:.1f}, Gain:{gain:.1f}x)")
                        except Exception as exc:
                            rate_log.warning("isp_hdr", f"[ISP] Daylight update warning: {exc}")

                    desired_mode = sensor_mode
                    if active_detector.name != "WILDLIFE":
                        desired_mode = "DUSK_3MP"
                    elif sensor_mode == "DUSK_3MP":
                        if (not is_monochrome) and (lux >= cam.day_12mp_enter_lux) and (gain <= cam.day_12mp_enter_max_gain):
                            desired_mode = "DAY_12MP"
                    else:
                        if is_monochrome or (lux < cam.day_12mp_exit_lux) or (gain > cam.day_12mp_exit_min_gain):
                            desired_mode = "DUSK_3MP"

                    if desired_mode != sensor_mode:
                        if sensor_switch_candidate != desired_mode:
                            sensor_switch_candidate = desired_mode
                            sensor_switch_since_mono = mono_now
                        req_dwell = (
                            2.0 if active_detector.name != "WILDLIFE"
                            else (cam.sensor_boot_eval_sec if mono_now <= fast_eval_until_mono else cam.sensor_switch_dwell_sec)
                        )
                        if safe_to_switch and ((mono_now - sensor_switch_since_mono) >= req_dwell):
                            try:
                                log.info(f"[CAMERA] Switching sensor: {sensor_mode} -> {desired_mode} (Lux:{lux:.1f}, Gain:{gain:.1f}x)")
                                frame_roi = None
                                frame_lores = None
                                roi_scratch = None
                                npu_lores_history.clear()
                                min_valid_capture_mono = time.monotonic()
                                self.wildlife_detector.discard_older_than(min_valid_capture_mono)
                                picam2.stop()
                                gc.collect()
                                picam2.configure(_build_camera_config(picam2, cfg, desired_mode, is_monochrome, last_ev))
                                picam2.start()
                                sensor_mode = desired_mode
                                sensor_switch_candidate = None
                                pre_roll_idx = pre_roll_count = frame_count = 0
                                fps_mono = time.monotonic()
                                state.update_health(sensor_mode=sensor_mode)
                                state.set_alert(f"SENSOR: {sensor_mode}", duration=2.5)
                                time.sleep(0.25)
                                continue
                            except Exception as se:
                                log.error(f"[CAMERA] Sensor switch to {desired_mode} failed: {se}")
                                sensor_switch_candidate = None
                                try:
                                    gc.collect()
                                    picam2.configure(_build_camera_config(picam2, cfg, sensor_mode, is_monochrome, last_ev))
                                    picam2.start()
                                except Exception as re_exc:
                                    log.error(f"[CAMERA] Camera recovery failed: {re_exc}")
                    else:
                        sensor_switch_candidate = None

                serial_ok = state.is_serial_connected()
                if serial_ok != last_seen_connected:
                    last_seen_connected = serial_ok
                    kf.reset()
                    tentative_target = None
                    tentative_streak = 0
                    min_valid_capture_mono = mono_now
                    self.wildlife_detector.discard_older_than(min_valid_capture_mono)
                    if not serial_ok:
                        state.clear_primary_target()

                # --- Stage 2: Polymorphic Detection ---
                if active_detector.name != last_detector_type:
                    log.info(f"[PERCEPTION] Switching detector: {last_detector_type} -> {active_detector.name}")
                    min_valid_capture_mono = mono_now
                    self.wildlife_detector.discard_older_than(min_valid_capture_mono)
                    self.wildlife_detector.set_active(active_detector.name == "WILDLIFE")
                    last_detector_type = active_detector.name
                    tentative_target = None
                    tentative_streak = 0
                    kf.reset()
                    state.clear_primary_target()
                    npu_lores_history.clear()
                    sensor_switch_candidate = None
                    fast_eval_until_mono = mono_now + cam.sensor_boot_eval_sec + 3.0

                if active_detector.name == "WILDLIFE":
                    detections, eff_capture_mono = self.wildlife_detector.detect(
                        frame_roi,
                        capture_mono,
                        is_monochrome=is_monochrome,
                        tilt_mrad=tilt_cap,
                        track_box=active_track_box,
                        sensor_size=sensor_size,
                        plan_bundle=plan_bundle,
                    )
                    frame_roi = None
                else:
                    detections, eff_capture_mono = active_detector.detect(
                        frame_lores, capture_mono, is_monochrome=is_monochrome
                    )

                debug_info = active_detector.get_last_debug_info()
                npu_lat_sec = state.get_npu_latency_sec()
                effective_coast_horizon = max(ctrl.coast_max_horizon_sec, npu_lat_sec + 1.0)
                effective_decay_onset = max(ctrl.coast_decay_onset_sec, npu_lat_sec + 0.25)

                cur_primary = state.get_primary_target()
                if cur_primary is not None:
                    prim_cap_age = max(0.0, mono_now - cur_primary.get("capture_mono", cur_primary["last_seen_mono"]))
                    if prim_cap_age >= effective_coast_horizon:
                        state.clear_primary_target()
                        kf.reset()
                        cur_primary = None

                is_new_batch = (
                    eff_capture_mono > 0.0
                    and eff_capture_mono >= min_valid_capture_mono
                    and eff_capture_mono != last_processed_capture_mono
                )
                verified = None

                if is_new_batch:
                    last_processed_capture_mono = eff_capture_mono
                    pan_batch, tilt_batch = state.get_turret_pos_at_time(eff_capture_mono)

                    # Resolve the exact lores frame matching eff_capture_mono for audit snapshots
                    audit_frame = frame_lores
                    if active_detector.name == "WILDLIFE" and npu_lores_history:
                        best_ts, best_frm = min(
                            npu_lores_history,
                            key=lambda item: abs(item[0] - eff_capture_mono),
                        )
                        if abs(best_ts - eff_capture_mono) <= 0.08:
                            audit_frame = best_frm

                    # Project current locked target into eff_capture_mono pixel space (both extrapolated and stationary)
                    ref_xy = None
                    last_xy = None
                    active_box_at_batch = None
                    assoc_gate_px = float(ctrl.confirmation_max_jump_px)

                    if cur_primary is not None:
                        prim_rng = cur_primary.get("range_m", last_valid_range_m)
                        pred_cap = kf.predict_state(eff_capture_mono)
                        if pred_cap is not None:
                            ref_xy = project_world_to_image_px(
                                pred_cap[0], pred_cap[1], pan_batch, tilt_batch, prim_rng, cam, optics
                            )
                            active_box_at_batch = _reconstruct_box_at_center(
                                ref_xy[0], ref_xy[1], cur_primary["box"], cur_primary["center"]
                            )
                        else:
                            ref_xy = cur_primary["center"]
                            active_box_at_batch = cur_primary["box"]

                        last_state = kf.get_last_state()
                        if last_state is not None:
                            last_xy = project_world_to_image_px(
                                last_state[0], last_state[1], pan_batch, tilt_batch, prim_rng, cam, optics
                            )
                            dt_gap = max(0.0, eff_capture_mono - last_state[2])
                            kin_optics_b, _, _, _ = state.site_calibration.get_ground_plane_params(optics, pan_batch)
                            kin_gap = compute_target_kinematics(prim_rng, kin_optics_b)
                            kin_gate_px = ((kin_gap.max_v_pan_mrad_s * dt_gap) + kin_gap.max_step_margin_pan_mrad) / max(0.5, optics.mrad_per_pixel)
                            assoc_gate_px = max(float(ctrl.confirmation_max_jump_px), min(280.0, kin_gate_px))

                    best_cand = _select_target_candidate(
                        detections, cur_primary, (ref_xy, last_xy), assoc_gate_px
                    )

                    if active_detector.name == "WILDLIFE":
                        veto_boxes = debug_info.get("veto_boxes", [])
                        rejected = debug_info.get("rejected_animals", [])
                        if state.site_calibration.is_walk_test_active():
                            state.site_calibration.ingest_walk_test_detections(
                                veto_boxes, tilt_batch, eff_capture_mono, state=state
                            )
                            best_cand = None
                            tentative_target = None
                            tentative_streak = 0
                        elif state.site_calibration.is_motion_owned():
                            best_cand = None
                            tentative_target = None
                            tentative_streak = 0
                        if cur_primary is not None and veto_boxes:
                            hit, veto_obj = MDV6Decoder1280.check_veto_overlap(
                                active_box_at_batch, veto_boxes, pad=cfg.wildlife.veto_padding_px
                            )
                            if (not hit) and best_cand is not None:
                                hit, veto_obj = MDV6Decoder1280.check_veto_overlap(
                                    best_cand["box"], veto_boxes, pad=cfg.wildlife.veto_padding_px
                                )
                            if hit:
                                log.warning(f"[SAFETY VETO:ACTIVE] Aborting lock — {veto_obj['class']} ({veto_obj['score']:.2f})")
                                state.clear_primary_target()
                                kf.reset()
                                cur_primary = None
                                tentative_target = None
                                tentative_streak = 0
                                best_cand = None
                                state.command_bus.request_trigger(0)
                                if mono_now - last_veto_audit_mono >= st.audit_veto_cooldown_sec:
                                    last_veto_audit_mono = mono_now
                                    save_audit_snapshot(audit_frame, "VETO_ACTIVE", [], debug_info, cfg.debug_snapshot_dir, st.max_debug_snapshots)

                        if rejected and (mono_now - last_veto_audit_mono >= st.audit_veto_cooldown_sec):
                            last_veto_audit_mono = mono_now
                            save_audit_snapshot(audit_frame, "VETO", detections, debug_info, cfg.debug_snapshot_dir, st.max_debug_snapshots)

                    if best_cand is not None:
                        if cur_primary is not None:
                            verified = best_cand
                        else:
                            cand_rng = best_cand.get("range_m", last_valid_range_m)
                            cand_w_pan, cand_w_tilt = image_px_to_world_mrad(
                                best_cand["center"][0], best_cand["center"][1],
                                pan_batch, tilt_batch, cand_rng, cam, optics
                            )
                            if tentative_target is not None:
                                dt_tent = max(0.05, eff_capture_mono - tentative_target["capture_mono"])
                                kin_optics_t, _, _, _ = state.site_calibration.get_ground_plane_params(optics, pan_batch)
                                kin_tent = compute_target_kinematics(cand_rng, kin_optics_t)
                                max_pan_jump = max(
                                    ctrl.confirmation_max_jump_px * optics.mrad_per_pixel,
                                    (kin_tent.max_v_pan_mrad_s * dt_tent) + kin_tent.max_step_margin_pan_mrad,
                                )
                                max_tilt_jump = max(
                                    ctrl.confirmation_max_jump_px * optics.mrad_per_pixel,
                                    (kin_tent.max_v_tilt_mrad_s * dt_tent) + kin_tent.max_step_margin_tilt_mrad,
                                )
                                within_world_gate = (
                                    abs(cand_w_pan - tentative_target["world_pan"]) <= max_pan_jump
                                    and abs(cand_w_tilt - tentative_target["world_tilt"]) <= max_tilt_jump
                                )
                                tentative_streak = (tentative_streak + 1) if within_world_gate else 1
                            else:
                                tentative_streak = 1

                            tentative_target = {
                                "center": best_cand["center"],
                                "box": best_cand["box"],
                                "range_m": cand_rng,
                                "world_pan": cand_w_pan,
                                "world_tilt": cand_w_tilt,
                                "capture_mono": eff_capture_mono,
                            }
                            if tentative_streak >= ctrl.confirmation_frames:
                                verified = best_cand
                                tentative_target = None
                                tentative_streak = 0
                                if active_detector.name == "WILDLIFE":
                                    log.warning(
                                        f"[ACQUIRE:WILDLIFE] Locked ({sensor_mode}) | Box:{best_cand['box']} "
                                        f"| Range:{best_cand['range_m']:.1f}m"
                                    )
                                    save_audit_snapshot(audit_frame, "ACQUIRE", detections, debug_info, cfg.debug_snapshot_dir, st.max_debug_snapshots)
                    else:
                        tentative_target = None
                        tentative_streak = 0

                # --- Stage 3: Kalman Measurement Correction (True Capture Timestamp) ---
                if verified is not None:
                    pan_v, tilt_v = state.get_turret_pos_at_time(eff_capture_mono)
                    last_valid_range_m = verified.get("range_m", last_valid_range_m)
                    raw_w_pan, raw_w_tilt = image_px_to_world_mrad(
                        verified["center"][0], verified["center"][1],
                        pan_v, tilt_v, last_valid_range_m, cam, optics
                    )
                    if active_detector.name == "WILDLIFE":
                        kin_optics_v, _, _, _ = state.site_calibration.get_ground_plane_params(optics, pan_v)
                        kin = compute_target_kinematics(last_valid_range_m, kin_optics_v)
                    else:
                        kin = None
                    kf.update_measurement(raw_w_pan, raw_w_tilt, eff_capture_mono, kinematics=kin)
                    state.set_primary_target({
                        "center": verified["center"],
                        "box": verified["box"],
                        "score": verified["score"],
                        "area": verified["area"],
                        "range_m": last_valid_range_m,
                        "capture_mono": eff_capture_mono,
                        "last_seen_mono": mono_now,
                        "id": verified.get("id", 0),
                        "label": verified.get("label", "TGT"),
                    })
                elif state.get_primary_target() is None and (mono_now - last_standby_log_mono >= 2.0):
                    last_standby_log_mono = mono_now
                    isp_tag = "BW" if is_monochrome else "HDR"
                    tel = self.wildlife_detector.get_telemetry() if active_detector.name == "WILDLIFE" else {}
                    npu_str = (
                        f" | NPU:{tel.get('npu_fps', 0.0):4.1f}Hz({tel.get('npu_latency_ms', 0.0):3.0f}ms {tel.get('scan_mode', 'IDLE')})"
                        if active_detector.name == "WILDLIFE"
                        else ""
                    )
                    log.info(
                        f"[STANDBY:{active_detector.name}] Cam:{sensor_fps:4.1f}FPS({sensor_mode}/{isp_tag}) | "
                        f"Lux:{metadata.get('Lux', 0.0):5.1f} Gain:{metadata.get('AnalogueGain', 1.0):4.1f}x"
                        f"{npu_str} | Turret:({cur_pan_mrad:+5d},{cur_tilt_mrad:+4d})"
                    )

                # --- Stage 4: Event Archiving (1280x720 lores) ---
                local_target = state.get_primary_target()
                target_locked = (
                    local_target is not None
                    and (max(0.0, mono_now - local_target.get("capture_mono", local_target["last_seen_mono"])) <= effective_decay_onset)
                )
                if target_locked:
                    lock_consecutive_frames += 1
                    last_detection_mono = mono_now
                else:
                    lock_consecutive_frames = 0

                if not is_recording:
                    np.copyto(pre_roll_pool[pre_roll_idx], frame_lores)
                    pre_roll_idx = (pre_roll_idx + 1) % pre_roll_cap
                    pre_roll_count = min(pre_roll_cap, pre_roll_count + 1)

                    if target_locked and lock_consecutive_frames >= st.min_lock_frames_to_record:
                        is_recording = True
                        recording_start_mono = mono_now
                        live_fps = float(np.clip(sensor_fps if sensor_fps >= 5.0 else cam.target_fps, 10.0, 30.0))
                        wanted = min(pre_roll_count, int(round(st.record_pre_roll_sec * live_fps)))
                        if pre_roll_count < pre_roll_cap:
                            saved = [pre_roll_pool[i].copy() for i in range(max(0, pre_roll_count - wanted), pre_roll_count)]
                        else:
                            off = pre_roll_cap - wanted
                            saved = [pre_roll_pool[(pre_roll_idx + off + i) % pre_roll_cap].copy() for i in range(wanted)]
                        try:
                            self.record_queue.put_nowait({"cmd": "start", "fps": round(live_fps, 1), "pre_roll": saved})
                        except queue.Full:
                            pass
                        pre_roll_idx = pre_roll_count = 0
                else:
                    if (
                        (mono_now - last_detection_mono) > st.record_post_roll_sec
                        or (mono_now - recording_start_mono) > st.max_recording_duration_sec
                    ):
                        is_recording = False
                        pre_roll_idx = pre_roll_count = 0
                        try:
                            self.record_queue.put_nowait({"cmd": "stop"})
                        except queue.Full:
                            pass
                    else:
                        try:
                            self.record_queue.put_nowait({"cmd": "frame", "frame": frame_lores.copy()})
                        except queue.Full:
                            pass

                # --- Stage 5: Offloaded HUD Dispatch (<= hud_max_fps when viewers connected) ---
                if state.has_web_clients() and (mono_now - last_hud_push_mono >= (1.0 / cam.hud_max_fps)):
                    last_hud_push_mono = mono_now
                    try:
                        self.hud_queue.put_nowait((
                            frame_lores.copy(),
                            list(detections),
                            debug_info,
                            eff_capture_mono,
                            local_target,
                            sensor_fps,
                            sensor_mode,
                            active_detector.name,
                            mono_now,
                        ))
                    except queue.Full:
                        pass
        finally:
            if is_recording:
                try:
                    self.record_queue.put_nowait({"cmd": "stop"})
                except queue.Full:
                    pass
            if picam2 is not None:
                try:
                    picam2.stop()
                    picam2.close()
                except Exception:
                    pass
            log.info("[CAMERA] VisionMain shut down cleanly.")
