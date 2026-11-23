"""
Flask Web Console & Control API.
- Plaintext HTTP Basic Auth (runs behind user's local reverse proxy).
- Refuses ARMED mode (HTTP 403) when cfg.web.armed_allowed is False.
- Exposes /api/mode, /api/health, /video_feed, /favicon.ico, /static/*, and /.
"""

import hmac
import logging
import os

from flask import Flask, Response, jsonify, render_template, request, send_from_directory

from bombardeer_pkg.config import BombardeerConfig
from bombardeer_pkg.state import SystemState
from bombardeer_pkg.utils import read_soc_temp_c

log = logging.getLogger("Bombardeer")


def create_app(cfg: BombardeerConfig, state: SystemState) -> Flask:
    template_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "templates")
    app = Flask(
        __name__,
        template_folder=template_dir,
        static_folder=cfg.static_dir,
        static_url_path="/static",
    )

    @app.before_request
    def require_web_auth():
        if not cfg.web.auth_user and not cfg.web.auth_pass:
            return None
        auth = request.authorization
        if (
            auth is None
            or not hmac.compare_digest(auth.username or "", cfg.web.auth_user)
            or not hmac.compare_digest(auth.password or "", cfg.web.auth_pass)
        ):
            return Response(
                "Authentication required.",
                401,
                {"WWW-Authenticate": 'Basic realm="Bombardeer Console"'},
            )
        return None

    def _mode_payload():
        m, p = state.get_turret_mode()
        return {
            "status": "ok",
            "mode": m,
            "primed": p,
            "detector": state.get_active_detector_type(),
            "hailo_available": state.is_hailo_ready(),
            "armed_allowed": cfg.web.armed_allowed,
        }

    @app.route("/api/mode", methods=["GET", "POST"])
    def api_mode():
        if request.method == "POST":
            req = request.get_json(silent=True) or {}
            target_mode = str(req.get("mode", "")).upper()
            target_detector = str(req.get("detector", "")).upper()
            reprime_req = bool(req.get("reprime", False))
            cur_mode, _ = state.get_turret_mode()

            if target_detector:
                if target_detector not in ("ARUCO", "WILDLIFE"):
                    return jsonify({"status": "error", "message": "Invalid detector specified"}), 400
                if target_detector == "WILDLIFE" and not state.is_hailo_ready():
                    state.set_alert("HAILO NPU UNAVAILABLE", duration=3.0)
                    return jsonify({"status": "error", "message": "Hailo-8 NPU unavailable"}), 400
                state.set_active_detector_type(target_detector)
                state.clear_primary_target()
                state.target_filter.reset()
                state.set_alert(f"DETECTOR: {target_detector} ACTIVE", duration=2.5)
                log.info(f"[API] Runtime detector switched to {target_detector}")
                return jsonify(_mode_payload())

            if reprime_req:
                if cur_mode == "CALIBRATE":
                    state.set_turret_mode("CALIBRATE", primed=True)
                    state.set_alert("TEST FIRE: RE-PRIMED (READY)", duration=2.5)
                return jsonify(_mode_payload())

            if target_mode not in ("OFF", "ON", "ARMED", "CALIBRATE"):
                return jsonify({"status": "error", "message": "Invalid mode specified"}), 400

            if target_mode in ("ARMED", "CALIBRATE") and cur_mode != "ON":
                return jsonify({"status": "error", "message": "Turret must be ON before arming or testing"}), 400

            if target_mode == "ARMED" and not cfg.web.armed_allowed:
                state.set_alert("ARMED REFUSED: SET BOMBARDEER_PASS", duration=4.0)
                log.warning("[SECURITY] Refused ARMED mode: BOMBARDEER_PASS is unset or default.")
                return jsonify({
                    "status": "error",
                    "message": "ARMED mode refused: set a non-default BOMBARDEER_PASS environment variable.",
                }), 403

            state.set_turret_mode(target_mode)
            if target_mode == "OFF":
                state.command_bus.request_halt()
                state.set_alert("TURRET DISENGAGED (OFF)", duration=2.5)
            elif target_mode == "ON":
                state.command_bus.request_trigger(0)
                state.set_alert("TRACKING ACTIVE (SAFE)", duration=2.5)
            elif target_mode == "CALIBRATE":
                state.set_alert("CALIBRATION MODE: PRIMED (1-SHOT)", duration=3.0)
            elif target_mode == "ARMED":
                state.set_alert("WARNING: TURRET ARMED / LIVE FIRE", duration=4.0)

            log.warning(f"[MODE TRANSITION] Turret switched to {target_mode}")
            return jsonify(_mode_payload())

        return jsonify(_mode_payload())

    @app.route("/api/health", methods=["GET"])
    def api_health():
        health = state.get_health_snapshot()
        pan_mrad, tilt_mrad, is_moving, is_firing = state.get_instantaneous_telemetry()
        mode, primed = state.get_turret_mode()
        return jsonify({
            "status": "ok",
            "uptime_sec": health["uptime_sec"],
            "mode": mode,
            "primed": primed,
            "armed_allowed": cfg.web.armed_allowed,
            "detector": state.get_active_detector_type(),
            "sensor_mode": health["sensor_mode"],
            "sensor_fps": health["sensor_fps"],
            "hailo_available": state.is_hailo_ready(),
            "npu_fps": round(health["npu_fps"], 2),
            "npu_latency_ms": round(health["npu_latency_ms"], 1),
            "npu_scan_mode": health["npu_scan_mode"],
            "npu_restarts": health["npu_restarts"],
            "serial_connected": health["serial_connected"],
            "serial_tx_count": health["serial_tx_count"],
            "serial_rx_count": health["serial_rx_count"],
            "turret": {
                "pan_mrad": pan_mrad,
                "tilt_mrad": tilt_mrad,
                "is_moving": is_moving,
                "is_firing": is_firing,
            },
            "recorder_active": health["recorder_active"],
            "recorder_error": health["recorder_error"],
            "free_disk_gb": round(health["free_disk_gb"], 2),
            "soc_temp_c": read_soc_temp_c(),
            "active_alert": health["active_alert"],
        })

    @app.route("/")
    def index():
        return render_template("index.html", display_width=cfg.camera.display_width)

    def _generate_frames():
        state.adjust_web_clients(1)
        last_sent = None
        try:
            while not state.shutdown_event.is_set():
                frame = state.get_latest_jpeg()
                if frame is not None and frame is not last_sent:
                    last_sent = frame
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                    yield frame
                    yield b"\r\n"
                if state.shutdown_event.wait(timeout=0.04):
                    break
        finally:
            state.adjust_web_clients(-1)

    @app.route("/video_feed")
    def video_feed():
        return Response(_generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/favicon.ico")
    def favicon():
        ico = os.path.join(cfg.static_dir, "bombardeer.ico")
        if os.path.exists(ico):
            return send_from_directory(cfg.static_dir, "bombardeer.ico", mimetype="image/vnd.microsoft.icon")
        return ("", 204)

    return app
