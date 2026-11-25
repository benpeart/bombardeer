"""Tests for non-droppable TurretCommandBus, SystemState latency/telemetry planes, Basic Auth, ARMED guard, /api/health, and /api/site_calibration."""

import base64
import math

from bombardeer_pkg.config import load_config
from bombardeer_pkg.control.site_calibration import DEFAULT_OPERATOR_HEIGHT_M, SiteCalibrationManager
from bombardeer_pkg.state import SystemState
from bombardeer_pkg.web.app import create_app


def _auth_header(user: str, pw: str) -> dict:
    tok = base64.b64encode(f"{user}:{pw}".encode("utf-8")).decode("ascii")
    return {"Authorization": f"Basic {tok}"}


def test_command_bus_halt_clears_motion_and_blocks_trigger_on():
    cfg = load_config(env={})
    state = SystemState(cfg)
    bus = state.command_bus

    for i in range(50):
        bus.request_velocity(i, -i)
    bus.request_halt()
    # Pending HALT must block subsequent TRIGGER 1 and VELOCITY until drained
    bus.request_trigger(1)
    bus.request_velocity(500, 500)

    cmds = bus.drain()
    assert [c["cmd"] for c in cmds] == ["HALT", "TRIGGER"]
    assert cmds[1]["state"] == 0

    # Normal velocity coalescing resumes after drain
    bus.request_velocity(10, -10)
    bus.request_velocity(42, -42)
    cmds2 = bus.drain()
    assert cmds2 == [{"cmd": "VELOCITY", "pan_s": 42, "tilt_s": -42}]


def test_state_defaults_to_wildlife_and_retains_deep_telemetry_history():
    """Prevents boot-time CPU spike from ARUCO default and ensures 350ms-1.5s pose lookback."""
    cfg = load_config(env={})
    state = SystemState(cfg)

    assert state.get_active_detector_type() == "WILDLIFE"

    # Simulate 2.0 seconds of 100 Hz ESP32 telemetry (200 samples > old maxlen=60)
    t0 = 500.0
    for i in range(200):
        state.update_turret_telemetry(pan=i * 4, tilt=-i, moving=True, rx_mono=t0 + i * 0.01)

    # Interpolate a capture pose from 1.20 seconds ago (120 samples back)
    pan_past, tilt_past = state.get_turret_pos_at_time(t0 + 0.805)
    assert pan_past == 322
    assert tilt_past in (-80, -81)

    # NPU latency helper reflects active WILDLIFE latency and zeroes out in ARUCO mode
    state.update_health(npu_latency_ms=345.0)
    assert abs(state.get_npu_latency_sec() - 0.345) < 1e-6
    state.set_active_detector_type("ARUCO")
    assert state.get_npu_latency_sec() == 0.0


def test_web_auth_and_armed_refused_with_default_password():
    cfg = load_config(env={})
    state = SystemState(cfg)
    client = create_app(cfg, state).test_client()

    assert client.get("/api/mode").status_code == 401
    assert client.get("/api/site_calibration").status_code == 401

    hdrs = _auth_header(cfg.web.auth_user, cfg.web.auth_pass)
    r = client.get("/api/mode", headers=hdrs)
    assert r.status_code == 200
    body = r.get_json()
    assert body["armed_allowed"] is False
    assert body["detector"] == "WILDLIFE"

    assert client.post("/api/mode", json={"mode": "ON"}, headers=hdrs).status_code == 200
    r_arm = client.post("/api/mode", json={"mode": "ARMED"}, headers=hdrs)
    assert r_arm.status_code == 403
    assert state.get_turret_mode()[0] == "ON"


def test_web_armed_allowed_with_custom_password_and_health_endpoint():
    cfg = load_config(env={"BOMBARDEER_USER": "ben", "BOMBARDEER_PASS": "CustomSecurePass!2026"})
    state = SystemState(cfg)
    client = create_app(cfg, state).test_client()
    hdrs = _auth_header("ben", "CustomSecurePass!2026")

    assert client.post("/api/mode", json={"mode": "ON"}, headers=hdrs).status_code == 200
    r_arm = client.post("/api/mode", json={"mode": "ARMED"}, headers=hdrs)
    assert r_arm.status_code == 200 and r_arm.get_json()["mode"] == "ARMED"

    # Disarming to OFF issues non-droppable HALT + TRIGGER 0
    assert client.post("/api/mode", json={"mode": "OFF"}, headers=hdrs).status_code == 200
    cmds = state.command_bus.drain()
    assert [c["cmd"] for c in cmds] == ["HALT", "TRIGGER"]
    assert cmds[1]["state"] == 0

    r_health = client.get("/api/health", headers=hdrs)
    assert r_health.status_code == 200
    body = r_health.get_json()
    assert body["armed_allowed"] is True
    assert "turret" in body and "soc_temp_c" in body and "uptime_sec" in body
    assert isinstance(body["site_calibration"], dict)


def test_web_site_calibration_walk_trigger_safety_lockout_and_verification_payload(tmp_path):
    cfg = load_config(env={"BOMBARDEER_USER": "ben", "BOMBARDEER_PASS": "CustomSecurePass!2026"})
    cfg.base_dir = str(tmp_path)
    state = SystemState(cfg)
    state.site_calibration = SiteCalibrationManager(
        cfg, cal_path=str(tmp_path / "site_calibration.json"), auto_boot_sweep=False
    )
    client = create_app(cfg, state).test_client()
    hdrs = _auth_header("ben", "CustomSecurePass!2026")

    # Root console HTML includes walk-cal controls and visual verification overlay
    r_html = client.get("/", headers=hdrs)
    assert r_html.status_code == 200
    html = r_html.get_data(as_text=True)
    assert "siteVerifyOverlay" in html
    assert "treelineProfileSvg" in html
    assert "/api/site_calibration" in html

    # GET /api/site_calibration returns optics & thresholds needed for visual verification
    r_get = client.get("/api/site_calibration", headers=hdrs)
    assert r_get.status_code == 200
    payload = r_get.get_json()
    assert "calibration" in payload and "optics" in payload and "thresholds" in payload
    assert payload["optics"]["focal_length_px"] == cfg.optics.focal_length_px

    # Put turret in ON, then trigger 5'9" Walk Calibration -> must disarm to OFF & block re-arming (409)
    assert client.post("/api/mode", json={"mode": "ON"}, headers=hdrs).status_code == 200
    r_start = client.post("/api/site_calibration", json={"action": "walk_start"}, headers=hdrs)
    assert r_start.status_code == 200
    start_body = r_start.get_json()
    assert start_body["ok"] is True
    assert start_body["motion_owned"] is True
    assert start_body["calibration"]["walk_test_active"] is True
    assert abs(start_body["calibration"]["operator_height_m"] - DEFAULT_OPERATOR_HEIGHT_M) < 1e-4
    assert state.get_turret_mode()[0] == "OFF"

    # Attempting to enter ON while walk-test is active must be rejected (409)
    r_blocked = client.post("/api/mode", json={"mode": "ON"}, headers=hdrs)
    assert r_blocked.status_code == 409

    # Feed synthetic 5'9" operator detections across 6m..22m and finish via API
    true_h, true_slope = 2.05, 22.0
    s = true_slope / 1000.0
    for idx, z_w in enumerate([6.0, 8.5, 11.0, 14.0, 17.0, 20.0, 22.0, 15.0, 9.5]):
        yf = cfg.camera.optical_center[1] + cfg.optics.focal_length_px * ((true_h + z_w * math.tan(s)) / z_w)
        yh = cfg.camera.optical_center[1] + cfg.optics.focal_length_px * ((true_h + z_w * math.tan(s) - DEFAULT_OPERATOR_HEIGHT_M) / z_w)
        bw = (yf - yh) * 0.32
        box = [640.0 - 0.5 * bw, yh, 640.0 + 0.5 * bw, yf]
        state.site_calibration.ingest_walk_test_detections(
            [{"class": "PERSON", "score": 0.90, "box": box}],
            tilt_at_cap=0.0,
            eff_capture_mono=50.0 + idx * 0.5,
            state=state,
        )

    r_fin = client.post("/api/site_calibration", json={"action": "walk_finish"}, headers=hdrs)
    assert r_fin.status_code == 200
    fin_body = r_fin.get_json()
    assert fin_body["ok"] is True
    assert fin_body["calibration"]["metric_source"] == "WALK_TEST"
    assert abs(fin_body["calibration"]["camera_mount_height_m"] - true_h) < 0.05
    assert abs(fin_body["calibration"]["ground_slope_mrad"] - true_slope) < 3.0
    assert fin_body["use_ground_range"] is True

    # Normal ON mode resumes cleanly once calibration finishes
    assert client.post("/api/mode", json={"mode": "ON"}, headers=hdrs).status_code == 200

    # OFF during an active sweep immediately cancels calibration
    assert client.post("/api/site_calibration", json={"action": "sweep"}, headers=hdrs).status_code == 200
    assert state.site_calibration.is_motion_owned() is True
    assert client.post("/api/mode", json={"mode": "OFF"}, headers=hdrs).status_code == 200
    assert state.site_calibration.is_motion_owned() is False
