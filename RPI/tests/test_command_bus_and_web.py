"""Tests for non-droppable TurretCommandBus, SystemState latency/telemetry planes, Basic Auth, ARMED guard, and /api/health."""

import base64

from bombardeer_pkg.config import load_config
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
