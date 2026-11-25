"""
VisionRuntime orchestrator + Core-0 VisionMain loop + Core-1 HudEncoder thread.
- Spawns HailoNPUProc BEFORE starting worker threads (deadlock-free spawn).
- MappedArray minimal-ROI capture into a reusable scratch buffer (zero per-frame allocs, DMA-safe).
- PiSP hardware BGR888 (RGB in memory) skips CPU BGR2RGB conversion.
- Projects Kalman world state back into current image coordinates for TRK-1:1 crops.
- Drives systemd watchdog (sd_notify) and monotonic Day/Dusk sensor switching.
"""

from collections import deque
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
    compute_horizon_y_720,
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


def _select_target_candidate(detections, cur_primary, ref_xy, max_jump_px: float):
    """Pick the nearest detection to the locked target to prevent jumping between multiple deer."""
    if not detections:
        return None
    if cur_primary is None:
        return detections[0]

    rx, ry = ref_xy if ref_xy is not None else cur_primary["center"]
    best = None
    best_dist = float("inf")
    for cand in detections:
        cx, cy = cand["center"]
        dist = math.hypot(cx - rx, cy - ry)
        if dist <= max_jump_px and dist < best_dist:
            best_dist = dist
            best = cand
    return best


def _clamp_kalman_velocity(kf, max_v_pan_mrad_s: float = 350.0, max_v_tilt_mrad_s: float = 200.0):
    """Clamp 4-state Kalman velocity [pan, tilt, v_pan, v_tilt] to prevent coast runaway."""
    x = getattr(kf, "x", None)
    if x is None:
        return
    try:
        flat = x.reshape(-1)
        if flat.size >= 4:
            flat[2] = float(np.clip(flat[2], -max_v_pan_mrad_s, max_v_pan_mrad_s))
            flat[3] = float(np.clip(flat[3], -max_v_tilt_mrad_s, max_v_tilt_mrad_s))
    except Exception:
        pass


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

        frame_lores, detections, debug_info, local_target, sensor_fps, sensor_mode, det_name, mono_now = pkt
        annotated = cv2.resize(frame_lores, (cam.display_width, cam.display_height), interpolation=cv2.INTER_LINEAR)
        cur_mode, is_primed = state.get_turret_mode()
        p_mrad, t_mrad, _, firing = state.get_instantaneous_telemetry()
        health = state.get_health_snapshot()
        rng_m = local_target.get("range_m", optics.default_fallback_range_m) if local_target else optics.default_fallback_range_m

        hor_y = int(round(compute_horizon_y_720(t_mrad, cam, optics) * sy))
        if 0 < hor_y < cam.display_height:
            for hx in range(0, cam.display_width, 16):
                cv2.line(annotated, (hx, hor_y), (min(cam.display_width, hx + 8), hor_y), (120, 110, 40), 1)

        cv2.drawMarker(annotated, (oc_x, oc_y), (0, 255, 255), cv2.MARKER_CROSS, 16, 1)

        _, drop_mrad, _, _ = compute_drag_ballistics(rng_m, 0.0, 0.0, 0.0, bal, optics)
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

        for v in debug_info.get("veto_boxes", []):
            vx1, vy1, vx2, vy2 = int(v["box"][0] * sx), int(v["box"][1] * sy), int(v["box"][2] * sx), int(v["box"][3] * sy)
            cv2.rectangle(annotated, (vx1, vy1), (vx2, vy2), (0, 0, 255), 1)
            cv2.putText(annotated, f"VETO:{v['class']} {v['score']*100:.0f}%", (vx1, max(vy1 - 4, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1, cv2.LINE_AA)

        for d in detections:
            cv2.rectangle(annotated, (int(d["box"][0] * sx), int(d["box"][1] * sy)),
                          (int(d["box"][2] * sx), int(d["box"][3] * sy)), (180, 180, 180), 1)

        if local_target and (mono_now - local_target["last_seen_mono"] < ctrl.coast_max_horizon_sec):
            is_fresh = (mono_now - local_target["last_seen_mono"]) <= ctrl.fire_data_freshness_sec
            box_color = (0, 255, 0) if is_fresh else (0, 165, 255)
            tx1, ty1 = int(local_target["box"][0] * sx), int(local_target["box"][1] * sy)
            tx2, ty2 = int(local_target["box"][2] * sx), int(local_target["box"][3] * sy)
            cv2.rectangle(annotated, (tx1, ty1), (tx2, ty2), box_color, 2)
            cv2.circle(annotated, (int(local_target["center"][0] * sx), int(local_target["center"][1] * sy)), 4, box_color, -1)
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

        alert_msg = health.get("active_alert")
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

        set_os_thread_name("VisionMain", cpu_cores=[0])
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

        tentative_target = None
        tentative_streak = 0
        last_processed_capture_mono = 0.0
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
        npu_lores_history = deque(maxlen=6)

        try:
            while not state.shutdown_event.is_set():
                mono_now = time.monotonic()
                req_det = state.get_active_detector_type()
                active_detector = detectors.get(req_det, detectors["ARUCO"])
                cur_pan_mrad, cur_tilt_mrad, _, _ = state.get_instantaneous_telemetry()

                # Project Kalman world target into CURRENT image coordinates so TRK-1:1 stays locked during slews
                cur_primary = state.get_primary_target()
                active_track_box = None
                if cur_primary is not None and (mono_now - cur_primary["last_seen_mono"] <= ctrl.track_crop_hold_sec):
                    pred = kf.predict_state(mono_now)
                    if pred is not None:
                        px, py = project_world_to_image_px(
                            pred[0], pred[1], cur_pan_mrad, cur_tilt_mrad,
                            cur_primary.get("range_m", last_valid_range_m), cam, optics
                        )
                        bw = max(20.0, cur_primary["box"][2] - cur_primary["box"][0])
                        bh = max(20.0, cur_primary["box"][3] - cur_primary["box"][1])
                        active_track_box = [px - 0.5 * bw, py - 0.6 * bh, px + 0.5 * bw, py + 0.4 * bh]
                    else:
                        active_track_box = cur_primary["box"]
                elif tentative_target is not None:
                    active_track_box = tentative_target["box"]

                need_native = active_detector.name == "WILDLIFE" and self.wildlife_detector.is_ready_for_frame()
                sensor_size = cam.sensor_day_size if sensor_mode == "DAY_12MP" else cam.sensor_dusk_size
                plan_bundle = None
                frame_roi = None

                # --- Stage 1: MappedArray minimal-ROI capture into reusable scratch buffer ---
                req_frame = None
                m = None
                roi_view = None
                try:
                    req_frame = picam2.capture_request()
                    frame_lores = req_frame.make_array("lores")
                    metadata = req_frame.get_metadata()
                    if need_native:
                        plan_bundle = self.wildlife_detector.prepare_plan(
                            sensor_size[0], sensor_size[1], cur_tilt_mrad, active_track_box
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

                capture_mono = _estimate_capture_mono(metadata, time.monotonic())
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
                    safe_to_switch = (not has_tgt) and (tentative_streak == 0) and (not is_recording)

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
                    if not serial_ok:
                        state.clear_primary_target()

                # --- Stage 2: Polymorphic Detection ---
                if active_detector.name != last_detector_type:
                    log.info(f"[PERCEPTION] Switching detector: {last_detector_type} -> {active_detector.name}")
                    self.wildlife_detector.set_active(active_detector.name == "WILDLIFE")
                    last_detector_type = active_detector.name
                    tentative_target = None
                    tentative_streak = 0
                    kf.reset()
                    state.clear_primary_target()
                    sensor_switch_candidate = None
                    fast_eval_until_mono = mono_now + cam.sensor_boot_eval_sec + 3.0

                if active_detector.name == "WILDLIFE":
                    detections, eff_capture_mono = self.wildlife_detector.detect(
                        frame_roi,
                        capture_mono,
                        is_monochrome=is_monochrome,
                        tilt_mrad=cur_tilt_mrad,
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
                is_new_batch = eff_capture_mono != last_processed_capture_mono
                cur_primary = state.get_primary_target()
                if cur_primary is not None and (mono_now - cur_primary["last_seen_mono"] >= ctrl.coast_max_horizon_sec):
                    state.clear_primary_target()
                    kf.reset()
                    cur_primary = None

                ref_xy = None
                if cur_primary is not None:
                    pred_cap = kf.predict_state(eff_capture_mono)
                    if pred_cap is not None:
                        pan_ref, tilt_ref = state.get_turret_pos_at_time(eff_capture_mono)
                        ref_xy = project_world_to_image_px(
                            pred_cap[0], pred_cap[1], pan_ref, tilt_ref,
                            cur_primary.get("range_m", last_valid_range_m), cam, optics
                        )
                    else:
                        ref_xy = cur_primary["center"]

                best_cand = _select_target_candidate(detections, cur_primary, ref_xy, ctrl.confirmation_max_jump_px)
                verified = None

                if is_new_batch:
                    last_processed_capture_mono = eff_capture_mono
                    audit_frame = frame_lores
                    if active_detector.name == "WILDLIFE" and npu_lores_history:
                        best_ts, best_frm = min(
                            npu_lores_history,
                            key=lambda item: abs(item[0] - eff_capture_mono),
                        )
                        if abs(best_ts - eff_capture_mono) <= 0.05:
                            audit_frame = best_frm

                    if active_detector.name == "WILDLIFE":
                        veto_boxes = debug_info.get("veto_boxes", [])
                        rejected = debug_info.get("rejected_animals", [])
                        cur_primary = state.get_primary_target()
                        if cur_primary is not None and veto_boxes:
                            hit, veto_obj = MDV6Decoder1280.check_veto_overlap(
                                cur_primary["box"], veto_boxes, pad=cfg.wildlife.veto_padding_px
                            )
                            if hit:
                                log.warning(f"[SAFETY VETO:ACTIVE] Aborting lock — {veto_obj['class']} ({veto_obj['score']:.2f})")
                                state.clear_primary_target()
                                kf.reset()
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
                        if state.get_primary_target() is not None:
                            verified = best_cand
                        elif tentative_target is not None:
                            px, py = tentative_target["center"]
                            cx, cy = best_cand["center"]
                            tentative_streak = (
                                tentative_streak + 1
                                if math.hypot(cx - px, cy - py) <= ctrl.confirmation_max_jump_px
                                else 1
                            )
                            tentative_target = best_cand
                            if tentative_streak >= ctrl.confirmation_frames:
                                verified = best_cand
                                if active_detector.name == "WILDLIFE":
                                    log.warning(
                                        f"[ACQUIRE:WILDLIFE] Locked ({sensor_mode}) | Box:{best_cand['box']} "
                                        f"| Range:{best_cand['range_m']:.1f}m"
                                    )
                                    save_audit_snapshot(audit_frame, "ACQUIRE", detections, debug_info, cfg.debug_snapshot_dir, st.max_debug_snapshots)
                        else:
                            tentative_target = best_cand
                            tentative_streak = 1
                    else:
                        tentative_target = None
                        tentative_streak = 0

                # --- Stage 3: Kalman Measurement Correction (True Capture Timestamp) ---
                if verified is not None:
                    pan_cap, tilt_cap = state.get_turret_pos_at_time(eff_capture_mono)
                    last_valid_range_m = verified.get("range_m", last_valid_range_m)
                    raw_w_pan, raw_w_tilt = image_px_to_world_mrad(
                        verified["center"][0], verified["center"][1],
                        pan_cap, tilt_cap, last_valid_range_m, cam, optics
                    )
                    kf.update_measurement(raw_w_pan, raw_w_tilt, eff_capture_mono)
                    _clamp_kalman_velocity(kf)
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
                    and (mono_now - local_target["last_seen_mono"] <= ctrl.coast_decay_onset_sec)
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
                            frame_lores.copy(), list(detections), debug_info,
                            local_target, sensor_fps, sensor_mode, active_detector.name, mono_now,
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