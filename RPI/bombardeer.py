#!/usr/bin/env python3
"""
Bombardeer Turret Controller — Thin WSGI & Standalone Entry Point.
Deploy with:
    gunicorn --workers 1 --threads 4 --bind 0.0.0.0:5000 bombardeer:app
"""

import os
os.environ["LIBCAMERA_LOG_LEVELS"] = "IPARPI:FATAL"

import atexit
import multiprocessing as mp
import signal
import sys
import threading

import cv2

from bombardeer_pkg.config import load_config
from bombardeer_pkg.hardware.vision_pipeline import VisionRuntime
from bombardeer_pkg.state import SystemState
from bombardeer_pkg.utils import InstanceLock, configure_logging, log, sd_notify
from bombardeer_pkg.web.app import create_app

cv2.setNumThreads(2)
cv2.ocl.setUseOpenCL(False)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
configure_logging()

cfg = load_config(os.path.join(BASE_DIR, "config.yaml"), base_dir=BASE_DIR)
state = SystemState(cfg)
app = create_app(cfg, state)

_runtime = None
_instance_lock = None
_bootstrap_lock = threading.Lock()


def start_runtime():
    global _runtime, _instance_lock
    with _bootstrap_lock:
        if _runtime is not None:
            return _runtime

        _instance_lock = InstanceLock(os.path.join(BASE_DIR, ".bombardeer.lock"))
        if not _instance_lock.acquire():
            log.critical(
                "[STARTUP] Another Bombardeer instance already holds .bombardeer.lock "
                "(check that Gunicorn uses --workers 1). Aborting."
            )
            sys.exit(1)

        _runtime = VisionRuntime(cfg, state)
        _runtime.start()
        sd_notify("READY=1")
        return _runtime


def shutdown_runtime():
    global _runtime, _instance_lock
    if state.shutdown_event.is_set():
        return
    log.info("[SHUTDOWN] Initiating graceful Bombardeer shutdown...")
    state.shutdown_event.set()
    state.command_bus.request_halt()
    if _runtime is not None:
        _runtime.stop()
        _runtime = None
    if _instance_lock is not None:
        _instance_lock.release()
        _instance_lock = None


if mp.current_process().name == "MainProcess":
    _orig_sigterm = signal.getsignal(signal.SIGTERM)
    _orig_sigint = signal.getsignal(signal.SIGINT)

    def _signal_handler(signum, frame):
        shutdown_runtime()
        if callable(_orig_sigterm) and signum == signal.SIGTERM:
            _orig_sigterm(signum, frame)
        elif callable(_orig_sigint) and signum == signal.SIGINT:
            _orig_sigint(signum, frame)
        else:
            sys.exit(0)

    try:
        signal.signal(signal.SIGTERM, _signal_handler)
        signal.signal(signal.SIGINT, _signal_handler)
    except ValueError:
        pass

    atexit.register(shutdown_runtime)
    if os.environ.get("BOMBARDEER_SKIP_AUTOSTART", "0") != "1":
        start_runtime()


if __name__ == "__main__":
    log.info("Starting local Flask development server. For production, use Gunicorn --workers 1.")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True, use_reloader=False)