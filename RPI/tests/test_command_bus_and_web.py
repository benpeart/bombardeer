"""Tests for non-droppable TurretCommandBus, Basic Auth, ARMED password guard, and /api/health."""

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


def test_web_auth_and_armed_refused_with_default_password():
    cfg = load_config(env={})
    state = SystemState(cfg)
    client = create_app(cfg, state).test_client()

    assert client.get("/api/mode").status_code == 401

    hdrs = _auth_header(cfg.web.auth_user, cfg.web.auth_pass)
    r = client.get("/api/mode", headers=hdrs)
    assert r.status_code == 200 and r.get_json()["armed_allowed"] is False

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