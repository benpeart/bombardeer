"""
Process-isolated Hailo-8 wildlife detector with spawn IPC, exact minimal-ROI
planning, supervisor auto-restart, and ground-plane/horizon safety vetoes.
"""

import atexit
import logging
import multiprocessing as mp
import os
import threading
import time
import traceback
from multiprocessing import shared_memory

import numpy as np

from bombardeer_pkg.config import BombardeerConfig
from bombardeer_pkg.control.ballistics import evaluate_ground_plane
from bombardeer_pkg.perception.base import BaseTargetDetector
from bombardeer_pkg.perception.hailo_worker import hailo_worker_main
from bombardeer_pkg.perception.tiling import build_pass_plan, compute_plan_roi_bounds
from bombardeer_pkg.state import SystemState
from bombardeer_pkg.utils import RateLimitedLogger

log = logging.getLogger("Bombardeer")

try:
    import hailo_platform  # noqa: F401
    HAILO_AVAILABLE = True
except ImportError:
    HAILO_AVAILABLE = False


class HailoWildlifeDetector(BaseTargetDetector):
    """Manages spawned HailoNPUProc, minimal ROI transfer, and bounded restart supervision."""

    RESTART_BACKOFF_BASE_SEC = 2.0

    def __init__(self, cfg: BombardeerConfig, state: SystemState):
        self.cfg = cfg
        self.state = state
        self._lock = threading.Lock()
        self._rate_log = RateLimitedLogger(interval_sec=5.0)

        self._latest_detections = []
        self._latest_debug_info = {"veto_boxes": [], "rejected_animals": []}
        self._latest_capture_mono = 0.0
        self._npu_latency_ms = 0.0
        self._npu_fps = 0.0
        self._npu_scan_mode = "IDLE"
        self._active = False
        self._track_miss_streak = 0

        self._proc = None
        self._shm = None
        self._req_tx = None
        self._ack_rx = None
        self._res_rx = None
        self._avail_flag = None
        self._stop_flag = None
        self._shm_busy = False
        self._shm_sent_mono = 0.0
        self._restart_count = 0
        self._next_restart_mono = 0.0

        self.model_path = self._find_model()
        if not HAILO_AVAILABLE or not self.model_path:
            log.warning("[HAILO] HailoRT or MDV6-yolov9-c-1280.hef unavailable.")
            return

        max_bytes = cfg.camera.max_sensor_width * cfg.camera.max_sensor_height * 3
        self._shm = shared_memory.SharedMemory(create=True, size=max_bytes)
        self._spawn_worker()
        atexit.register(self.shutdown)

    def _find_model(self):
        candidates = [
            os.path.join(self.cfg.base_dir, "MDV6-yolov9-c-1280.hef"),
            os.path.join(self.cfg.base_dir, "models", "MDV6-yolov9-c-1280.hef"),
            "/home/ben/bombardeer/MDV6-yolov9-c-1280.hef",
            "/home/ben/bombardeer/models/MDV6-yolov9-c-1280.hef",
            "/home/ben/Bombardeer/MDV6-yolov9-c-1280.hef",
            "/home/ben/Bombardeer/models/MDV6-yolov9-c-1280.hef",
        ]
        return next((p for p in candidates if os.path.exists(p)), None)

    def _spawn_worker(self):
        ctx = mp.get_context("spawn")
        req_rx, self._req_tx = ctx.Pipe(duplex=False)
        self._ack_rx, ack_tx = ctx.Pipe(duplex=False)
        self._res_rx, res_tx = ctx.Pipe(duplex=False)
        self._avail_flag = ctx.Value("i", 0, lock=False)
        self._stop_flag = ctx.Value("i", 0, lock=False)
        self._shm_busy = False

        self._proc = ctx.Process(
            target=hailo_worker_main,
            args=(
                self.model_path,
                self._shm.name,
                req_rx,
                ack_tx,
                res_tx,
                self._avail_flag,
                self._stop_flag,
                self.cfg.camera,
                self.cfg.optics,
                self.cfg.wildlife,
            ),
            daemon=True,
            name="HailoNPUProcess",
        )
        self._proc.start()

    def _cleanup_worker(self):
        try:
            if self._stop_flag is not None:
                self._stop_flag.value = 1
            if self._req_tx is not None:
                try:
                    self._req_tx.send(None)
                except Exception:
                    pass
            if self._proc is not None and self._proc.is_alive():
                self._proc.join(timeout=1.0)
                if self._proc.is_alive():
                    self._proc.terminate()
                    self._proc.join(timeout=0.5)
        except Exception as exc:
            log.warning(f"[HAILO] Worker cleanup warning: {exc}")
        self._proc = None
        self._shm_busy = False

    def shutdown(self):
        self._cleanup_worker()
        if self._shm is not None:
            try:
                self._shm.close()
                self._shm.unlink()
            except Exception:
                pass
            self._shm = None

    def check_supervisor(self):
        """Restarts HailoNPUProc on crash or >5s stall; disarms live fire on fault."""
        if self._shm is None or self._proc is None:
            return
        now = time.monotonic()
        stalled = self._shm_busy and ((now - self._shm_sent_mono) > 5.0)
        dead = not self._proc.is_alive()
        if not dead and not stalled:
            return
        if now < self._next_restart_mono:
            return

        reason = "stalled >5s" if stalled else f"exited rc={self._proc.exitcode}"
        log.error(f"[HAILO SUPERVISOR] Worker {reason}! Disarming and scheduling restart.")
        self.state.update_health(hailo_available=False, npu_fps=0.0, npu_scan_mode="FAULT")
        self.state.set_alert("HAILO NPU FAULT - RESTARTING", duration=4.0)

        mode, _ = self.state.get_turret_mode()
        if mode in ("ARMED", "CALIBRATE"):
            self.state.set_turret_mode("ON")
            self.state.command_bus.request_trigger(0)

        self._cleanup_worker()
        if self._restart_count < self.cfg.wildlife.max_npu_restarts:
            self._restart_count += 1
            self.state.inc_health("npu_restarts", 1)
            delay = self.RESTART_BACKOFF_BASE_SEC * (2 ** (self._restart_count - 1))
            self._next_restart_mono = now + min(30.0, delay)
            try:
                self._spawn_worker()
            except Exception as exc:
                log.error(f"[HAILO SUPERVISOR] Restart #{self._restart_count} failed: {exc}")
        else:
            self.state.set_alert("HAILO NPU PERMANENT FAULT", duration=10.0)
            self._next_restart_mono = now + 60.0

    @property
    def name(self) -> str:
        return "WILDLIFE"

    @property
    def is_available(self) -> bool:
        ok = (
            self._avail_flag is not None
            and self._proc is not None
            and self._proc.is_alive()
            and self._avail_flag.value == 1
        )
        self.state.update_health(hailo_available=ok)
        return ok

    def is_ready_for_frame(self) -> bool:
        if not self.is_available:
            return False
        while self._ack_rx is not None and self._ack_rx.poll():
            try:
                self._ack_rx.recv()
                self._shm_busy = False
            except (EOFError, OSError):
                break
        return not self._shm_busy

    def set_active(self, active: bool):
        self._active = active
        if not active:
            self._track_miss_streak = 0
            with self._lock:
                self._latest_detections = []
                self._latest_debug_info = {"veto_boxes": [], "rejected_animals": []}
                self._npu_fps = 0.0
                self._npu_scan_mode = "IDLE"
            self.state.update_health(npu_fps=0.0, npu_scan_mode="IDLE")

    def get_telemetry(self) -> dict:
        with self._lock:
            return {
                "npu_fps": self._npu_fps,
                "npu_latency_ms": self._npu_latency_ms,
                "scan_mode": self._npu_scan_mode,
                "available": self.is_available,
            }

    def get_last_debug_info(self) -> dict:
        with self._lock:
            return {
                "veto_boxes": list(self._latest_debug_info.get("veto_boxes", [])),
                "rejected_animals": list(self._latest_debug_info.get("rejected_animals", [])),
            }

    def prepare_plan(self, sensor_w: int, sensor_h: int, tilt_mrad: float, track_box):
        """
        Builds the exact multi-tile or tracking-crop plan and its minimal enclosing
        native ROI rectangle so VisionMain copies only required sensor pixels.
        """
        track_active = track_box is not None
        if not track_active:
            self._track_miss_streak = 0
        force_scan = track_active and (self._track_miss_streak >= 2)
        mode_tag, pass_plan = build_pass_plan(
            sensor_w=sensor_w,
            sensor_h=sensor_h,
            tilt_mrad=tilt_mrad,
            track_active=track_active,
            track_box=track_box,
            cam=self.cfg.camera,
            optics=self.cfg.optics,
            wl=self.cfg.wildlife,
            force_scan=force_scan,
        )
        roi_bounds = compute_plan_roi_bounds(pass_plan)
        return mode_tag, pass_plan, roi_bounds, track_active, force_scan

    def _drain_results(self, is_monochrome: bool):
        if self._res_rx is None:
            return
        while self._res_rx.poll():
            try:
                (
                    dets,
                    debug_info,
                    eff_mono,
                    dt_ms,
                    cur_fps,
                    scan_mode_tag,
                    was_tracking,
                    was_force_scan,
                ) = self._res_rx.recv()
            except (EOFError, OSError) as exc:
                self._rate_log.error("pipe_recv", f"[HAILO] Pipe recv error: {exc}")
                break
            except Exception as exc:
                self._rate_log.error("decode_err", f"[HAILO] Result error: {exc}\n{traceback.format_exc()}")
                break

            if dets:
                _, tilt_at_cap = self.state.get_turret_pos_at_time(eff_mono)
                rejected = debug_info.setdefault("rejected_animals", [])
                filtered = []
                for d in dets:
                    pa, pp, pv = d.get("probs", (d["score"], 0.0, 0.0))
                    if is_monochrome and d["score"] < self.cfg.wildlife.monochrome_conf_thresh:
                        rejected.append({"reason": f"MONO_CONF({d['score']:.2f})", "box": d["box"], "probs": (pa, pp, pv)})
                        continue

                    ok, rej_reason, fused_range = evaluate_ground_plane(
                        box_xyxy=d["box"],
                        tilt_at_cap=tilt_at_cap,
                        stadia_range_m=d.get("range_m", self.cfg.optics.default_fallback_range_m),
                        max_animal_height_m=self.cfg.wildlife.max_animal_height_m,
                        cam=self.cfg.camera,
                        optics=self.cfg.optics,
                    )
                    if not ok:
                        rejected.append({"reason": rej_reason, "box": d["box"], "probs": (pa, pp, pv)})
                        continue
                    d["range_m"] = fused_range
                    filtered.append(d)
                dets = filtered

            if was_tracking:
                if dets or was_force_scan:
                    self._track_miss_streak = 0
                else:
                    self._track_miss_streak += 1

            with self._lock:
                self._latest_detections = dets
                self._latest_debug_info = debug_info
                self._latest_capture_mono = eff_mono
                self._npu_latency_ms = dt_ms
                self._npu_fps = cur_fps
                self._npu_scan_mode = scan_mode_tag
            self.state.update_health(npu_fps=cur_fps, npu_latency_ms=dt_ms, npu_scan_mode=scan_mode_tag)
            self._restart_count = 0

    def detect(
        self,
        frame_roi: np.ndarray,
        capture_mono_time: float,
        is_monochrome: bool = False,
        tilt_mrad: float = 0.0,
        track_box=None,
        sensor_size=None,
        plan_bundle=None,
    ):
        self.check_supervisor()
        if not self.is_available:
            return [], capture_mono_time

        self._drain_results(is_monochrome)

        if frame_roi is not None and self.is_ready_for_frame():
            roi_h, roi_w = frame_roi.shape[:2]
            full_w, full_h = sensor_size if sensor_size is not None else (roi_w, roi_h)
            if plan_bundle is None:
                plan_bundle = self.prepare_plan(full_w, full_h, tilt_mrad, track_box)
            mode_tag, pass_plan, (roi_x0, roi_y0, exp_w, exp_h), was_tracking, was_force_scan = plan_bundle

            if roi_w == exp_w and roi_h == exp_h and (roi_w * roi_h * 3) <= self._shm.size:
                shm_view = np.ndarray((roi_h, roi_w, 3), dtype=np.uint8, buffer=self._shm.buf)
                np.copyto(shm_view, frame_roi)
                try:
                    self._req_tx.send((
                        float(capture_mono_time),
                        int(full_w),
                        int(full_h),
                        int(roi_x0),
                        int(roi_y0),
                        int(roi_w),
                        int(roi_h),
                        mode_tag,
                        pass_plan,
                        was_tracking,
                        was_force_scan,
                    ))
                    self._shm_busy = True
                    self._shm_sent_mono = time.monotonic()
                except (BrokenPipeError, OSError) as exc:
                    self._rate_log.error("pipe_send", f"[HAILO] Submit error: {exc}")

        with self._lock:
            dets = list(self._latest_detections)
            eff_mono = self._latest_capture_mono if self._latest_capture_mono > 0.0 else capture_mono_time
        return dets, eff_mono