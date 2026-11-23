"""Core-1 asynchronous 1280x720 lores event video recorder and storage headroom manager."""

import datetime
import glob
import logging
import os
import queue
import shutil

import cv2

from bombardeer_pkg.config import BombardeerConfig
from bombardeer_pkg.state import SystemState
from bombardeer_pkg.utils import set_os_thread_name

log = logging.getLogger("Bombardeer")


def ensure_storage_headroom(cfg: BombardeerConfig, state: SystemState):
    st = cfg.storage
    try:
        _, _, free = shutil.disk_usage(cfg.recordings_dir)
        free_gb = free / (1024 ** 3)
        state.update_health(free_disk_gb=free_gb)

        if free_gb < st.min_free_space_gb:
            state.set_alert(f"LOW STORAGE: {free_gb:.1f} GB Free", duration=4.0)

        files = glob.glob(os.path.join(cfg.recordings_dir, "wildlife_*.*"))
        if not files:
            return
        files.sort(key=os.path.getmtime)
        archive_gb = sum(os.path.getsize(f) for f in files) / (1024 ** 3)

        for fpath in files:
            if free_gb >= st.min_free_space_gb and archive_gb <= st.max_recordings_storage_gb:
                break
            sz = os.path.getsize(fpath)
            try:
                os.remove(fpath)
                log.info(f"[STORAGE] Purged old recording: {fpath}")
                archive_gb -= sz / (1024 ** 3)
                _, _, free = shutil.disk_usage(cfg.recordings_dir)
                free_gb = free / (1024 ** 3)
                state.update_health(free_disk_gb=free_gb)
            except OSError as exc:
                log.error(f"[STORAGE] Purge failed on {fpath}: {exc}")
    except Exception as exc:
        log.error(f"[STORAGE] Disk check failed: {exc}")


def video_recorder_worker(cfg: BombardeerConfig, state: SystemState, record_queue: queue.Queue):
    writer = None
    recording_active = False
    current_filepath = None
    frames_written = 0

    codecs = (
        ("mp4v", cv2.VideoWriter_fourcc(*"mp4v"), ".mp4"),
        ("avc1", cv2.VideoWriter_fourcc(*"avc1"), ".mp4"),
        ("MJPG", cv2.VideoWriter_fourcc(*"MJPG"), ".avi"),
        ("XVID", cv2.VideoWriter_fourcc(*"XVID"), ".avi"),
    )

    set_os_thread_name("VidRecorder", cpu_cores=[1])

    def _finalize():
        nonlocal writer, recording_active, current_filepath, frames_written
        if not recording_active:
            return
        recording_active = False
        state.update_health(recorder_active=False)
        if writer is not None:
            try:
                writer.release()
                writer = None
                if current_filepath and os.path.exists(current_filepath):
                    kb = os.path.getsize(current_filepath) / 1024.0
                    log.info(f"[RECORDER] Finalized {current_filepath} ({frames_written} frames, {kb:.1f} KB)")
                    state.set_alert(f"Saved: {os.path.basename(current_filepath)}", duration=3.0)
            except Exception as exc:
                log.error(f"[RECORDER] Release error: {exc}")
        ensure_storage_headroom(cfg, state)

    while not state.shutdown_event.is_set():
        try:
            item = record_queue.get(timeout=0.3)
        except queue.Empty:
            continue

        cmd = item.get("cmd")
        if cmd == "start" and not recording_active:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            writer = None
            frames_written = 0
            current_filepath = None
            clip_fps = float(item.get("fps", cfg.camera.target_fps))

            for tag, fourcc, ext in codecs:
                cand_path = os.path.join(cfg.recordings_dir, f"wildlife_{ts}{ext}")
                try:
                    cand = cv2.VideoWriter(cand_path, fourcc, clip_fps, (cfg.camera.frame_width, cfg.camera.frame_height))
                    if cand is not None and cand.isOpened():
                        writer = cand
                        current_filepath = cand_path
                        log.info(f"[RECORDER] Opened ({clip_fps:.1f} FPS, '{tag}') -> {current_filepath}")
                        break
                    if cand is not None:
                        cand.release()
                except Exception as exc:
                    log.debug(f"[RECORDER] Codec {tag} rejected: {exc}")

            if writer is not None and writer.isOpened():
                recording_active = True
                state.update_health(recorder_active=True, recorder_error=None)
                for f in item.get("pre_roll", []):
                    try:
                        writer.write(f)
                        frames_written += 1
                    except Exception as exc:
                        log.error(f"[RECORDER] Pre-roll write error: {exc}")
            else:
                state.update_health(recorder_active=False, recorder_error="All VideoWriter codecs failed")
                state.set_alert("RECORDER FAILED: Check Codecs", duration=5.0)

        elif cmd == "frame" and recording_active:
            frame = item.get("frame")
            if writer is not None and writer.isOpened() and frame is not None:
                try:
                    writer.write(frame)
                    frames_written += 1
                except Exception as exc:
                    log.error(f"[RECORDER] Frame write fault: {exc}")
                    state.update_health(recorder_error=str(exc))

        elif cmd == "stop":
            _finalize()

    _finalize()
    log.info("[RECORDER] VidRecorder shut down cleanly.")