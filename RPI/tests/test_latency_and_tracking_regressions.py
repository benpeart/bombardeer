"""
Regression tests for 350ms asynchronous NPU latency, Hailo IPC freshness & fencing,
motion_servo stability/fire-gate timing, and vision_pipeline target association.
"""

from collections import deque
import threading
import time
import types

import numpy as np

from bombardeer_pkg.config import load_config
from bombardeer_pkg.control.ballistics import compute_drag_ballistics
from bombardeer_pkg.control.motion_servo import _compute_axis_velocity, motion_servoing_worker
from bombardeer_pkg.hardware import vision_pipeline as vp
from bombardeer_pkg.perception.hailo_detector import HailoWildlifeDetector
from bombardeer_pkg.state import SystemState
from bombardeer_pkg.utils import RateLimitedLogger


class _FakePipe:
    def __init__(self):
        self.items = deque()

    def poll(self):
        return len(self.items) > 0

    def send(self, obj):
        self.items.append(obj)

    def recv(self):
        return self.items.popleft()


class _FakeProc:
    def __init__(self, alive=True, exitcode=None):
        self._alive = alive
        self.exitcode = exitcode
        self.terminated = False

    def is_alive(self):
        return self._alive

    def join(self, timeout=None):
        pass

    def terminate(self):
        self.terminated = True
        self._alive = False


class _FakeValue:
    def __init__(self, v):
        self.value = v


def _make_stub_hailo_detector(cfg, state):
    """Creates a HailoWildlifeDetector wired to in-memory fake IPC pipes (no NPU hardware required)."""
    det = HailoWildlifeDetector.__new__(HailoWildlifeDetector)
    det.cfg = cfg
    det.state = state
    det._lock = threading.Lock()
    det._rate_log = RateLimitedLogger(interval_sec=5.0)
    det._latest_detections = []
    det._latest_debug_info = {"veto_boxes": [], "rejected_animals": []}
    det._latest_capture_mono = 0.0
    det._npu_latency_ms = 0.0
    det._npu_fps = 0.0
    det._npu_scan_mode = "IDLE"
    det._active = True
    det._track_miss_streak = 0
    det._proc = _FakeProc(alive=True)
    det._shm = types.SimpleNamespace(size=200_000, buf=bytearray(200_000))
    det._req_tx = _FakePipe()
    det._ack_rx = _FakePipe()
    det._res_rx = _FakePipe()
    det._avail_flag = _FakeValue(1)
    det._stop_flag = _FakeValue(0)
    det._shm_busy = False
    det._in_flight_jobs = 0
    det._pending_jobs = deque(maxlen=8)
    det._min_valid_capture_mono = 0.0
    det._shm_sent_mono = 0.0
    det._restart_count = 0
    det._next_restart_mono = 0.0
    return det


# =============================================================================
# 1. HailoWildlifeDetector Async IPC & Latency Regressions
# =============================================================================


def test_hailo_returns_zero_timestamp_before_first_real_batch():
    """Prevents phantom is_new_batch triggers on every camera frame before the first NPU result."""
    cfg = load_config(env={})
    state = SystemState(cfg)
    det = _make_stub_hailo_detector(cfg, state)

    dets, eff_mono = det.detect(None, capture_mono_time=123.45)
    assert dets == []
    assert eff_mono == 0.0


def test_hailo_enforces_single_in_flight_job_after_shm_ack():
    """
    Worker acks SHM copy immediately before 350ms NPU inference finishes.
    Detector must NOT accept a second frame until the inference result arrives,
    eliminating ~280ms hidden pipe queueing lag.
    """
    cfg = load_config(env={})
    state = SystemState(cfg)
    det = _make_stub_hailo_detector(cfg, state)

    roi = np.zeros((10, 10, 3), dtype=np.uint8)
    plan = ("SCAN-12MP", [], (0, 0, 10, 10), False, False)

    # Submit job 1
    det.detect(roi, capture_mono_time=50.0, is_monochrome=False, sensor_size=(10, 10), plan_bundle=plan)
    assert det._in_flight_jobs == 1
    assert not det.is_ready_for_frame()
    assert not det.is_idle()

    # Worker finishes SHM copy and sends ACK, but NPU is still running inference
    det._ack_rx.send(True)
    assert not det.is_ready_for_frame()
    assert not det.is_idle()

    # Attempting to submit job 2 while job 1 is still on the NPU must be ignored
    det.detect(roi, capture_mono_time=50.07, is_monochrome=False, sensor_size=(10, 10), plan_bundle=plan)
    assert len(det._req_tx.items) == 1
    assert det._in_flight_jobs == 1

    # Once job 1 result arrives, is_ready_for_frame() drains it immediately (zero wasted frame)
    det._res_rx.send(([], {"veto_boxes": [], "rejected_animals": []}, 50.0, 345.0, 2.9, "SCAN-12MP", False, False))
    assert det.is_ready_for_frame()
    assert det.is_idle()
    assert det._in_flight_jobs == 0


def test_hailo_uses_per_job_is_monochrome_recorded_at_submission():
    """ISP monochrome transitions during a 350ms NPU pass must not alter filtering of the in-flight frame."""
    cfg = load_config(env={})
    state = SystemState(cfg)
    state.update_turret_telemetry(0, 0, False, 100.0)
    det = _make_stub_hailo_detector(cfg, state)

    roi = np.zeros((10, 10, 3), dtype=np.uint8)
    plan = ("SCAN-3MP", [], (0, 0, 10, 10), False, False)
    low_score = max(0.05, cfg.wildlife.monochrome_conf_thresh - 0.10)

    # Submit frame while ISP is in monochrome B&W mode
    det.detect(roi, capture_mono_time=100.0, is_monochrome=True, sensor_size=(10, 10), plan_bundle=plan)
    det._ack_rx.send(True)

    # Result returns 350ms later AFTER ISP has already restored daylight color (is_monochrome=False)
    det._res_rx.send((
        [{"box": [550, 420, 620, 470], "center": (585, 445), "score": low_score, "area": 3500, "range_m": 15.0}],
        {"veto_boxes": [], "rejected_animals": []},
        100.0,
        345.0,
        2.9,
        "SCAN-3MP",
        False,
        False,
    ))
    dets, eff_mono = det.detect(None, capture_mono_time=100.35, is_monochrome=False)
    assert eff_mono == 100.0
    assert dets == []
    assert any("MONO_CONF" in r["reason"] for r in det.get_last_debug_info()["rejected_animals"])


def test_hailo_epoch_fencing_discards_late_pre_switch_results():
    """Late NPU results captured before a sensor/detector switch must be discarded cleanly."""
    cfg = load_config(env={})
    state = SystemState(cfg)
    det = _make_stub_hailo_detector(cfg, state)

    roi = np.zeros((10, 10, 3), dtype=np.uint8)
    plan = ("SCAN-12MP", [], (0, 0, 10, 10), False, False)

    det.detect(roi, capture_mono_time=200.0, is_monochrome=False, sensor_size=(10, 10), plan_bundle=plan)
    det.discard_older_than(200.2)

    det._ack_rx.send(True)
    det._res_rx.send((
        [{"box": [550, 420, 620, 470], "center": (585, 445), "score": 0.92, "area": 3500, "range_m": 15.0}],
        {"veto_boxes": [], "rejected_animals": []},
        200.0,
        345.0,
        2.9,
        "SCAN-12MP",
        False,
        False,
    ))
    dets, eff_mono = det.detect(None, capture_mono_time=200.4)
    assert dets == []
    assert eff_mono == 0.0
    assert det.is_idle()


def test_hailo_supervisor_catches_inference_stall_after_shm_ack():
    """If the NPU hangs AFTER SHM copy ACK (_shm_busy=False, _in_flight_jobs=1), supervisor must disarm & restart."""
    cfg = load_config(env={})
    state = SystemState(cfg)
    state.set_turret_mode("ARMED")
    det = _make_stub_hailo_detector(cfg, state)

    spawned = []
    det._spawn_worker = lambda: spawned.append(True)

    det._shm_busy = False
    det._in_flight_jobs = 1
    det._shm_sent_mono = time.monotonic() - 6.0

    det.check_supervisor()

    assert state.get_turret_mode()[0] == "ON"
    cmds = state.command_bus.drain()
    assert any(c["cmd"] == "TRIGGER" and c["state"] == 0 for c in cmds)
    assert len(spawned) == 1
    assert state.get_health_snapshot()["npu_restarts"] == 1


# =============================================================================
# 2. MotionServo Latency Scaling, Tilt Cap & Fire-Gate Regressions
# =============================================================================


def test_axis_velocity_respects_deadband_asymmetry_and_latency_scaling():
    assert _compute_axis_velocity(3.0, 20.0, deadband=4.0) == 0

    # Tilt gains are intentionally softer than pan gains
    v_pan = _compute_axis_velocity(40.0, 50.0, 4.0, kp_large=2.0, kv_large=0.85, latency_scale=1.0)
    v_tilt = _compute_axis_velocity(40.0, 50.0, 4.0, kp_large=1.45, kv_large=0.55, latency_scale=1.0)
    assert v_pan > v_tilt > 0

    # Higher NPU latency (350ms -> latency_scale ~ 0.715) scales command down to preserve phase margin
    v_pan_slow_npu = _compute_axis_velocity(40.0, 50.0, 4.0, kp_large=2.0, kv_large=0.85, latency_scale=0.715)
    assert v_pan_slow_npu < v_pan


def test_motion_servo_fire_gate_supports_350ms_scan_and_blocks_stale():
    """
    A healthy 345ms SCAN-12MP detection arrives with capture_age=0.35s and reaches ~0.55s
    before the next batch. CALIBRATE mode must discharge on a fresh 0.45s capture_age
    but refuse a stale 1.2s capture_age.
    """
    cfg = load_config(env={})
    cfg.control.motion_loop_interval_sec = 0.005
    _, drop_mrad, _, _ = compute_drag_ballistics(15.0, -50.0, 0.0, 0.0, cfg.ballistics, cfg.optics)

    # --- Case 1: Fresh 345ms SCAN-12MP detection (capture_age = 0.45s, arrival age = 0.10s) ---
    state = SystemState(cfg)
    trig_events = []
    orig_req_trig = state.command_bus.request_trigger

    def _spy_trig(val, pulse_len=None, timestamp_mono=None):
        trig_events.append((val, pulse_len))
        orig_req_trig(val, pulse_len=pulse_len, timestamp_mono=timestamp_mono)

    state.command_bus.request_trigger = _spy_trig

    now = time.monotonic()
    state.update_health(serial_connected=True, npu_latency_ms=345.0)
    state.update_turret_telemetry(100, -50, False, now)
    state.set_turret_mode("CALIBRATE", primed=True)
    state.target_filter.update_measurement(
        100.0, -50.0 - drop_mrad - cfg.optics.tilt_calibration_trim_mrad, now - 0.45
    )
    state.set_primary_target({
        "center": (640, 360),
        "box": [610, 340, 670, 380],
        "score": 0.90,
        "area": 2400,
        "range_m": 15.0,
        "capture_mono": now - 0.45,
        "last_seen_mono": now - 0.10,
    })

    t = threading.Thread(target=motion_servoing_worker, args=(cfg, state), daemon=True)
    t.start()
    time.sleep(0.06)
    state.shutdown_event.set()
    t.join(timeout=1.0)

    assert any(val == 1 for val, _ in trig_events)
    assert state.get_turret_mode()[1] is False  # Calibration prime was consumed

    # --- Case 2: Stale target (capture_age = 1.2s) must NOT fire ---
    state2 = SystemState(cfg)
    trig_events2 = []
    orig_req_trig2 = state2.command_bus.request_trigger

    def _spy_trig2(val, pulse_len=None, timestamp_mono=None):
        trig_events2.append((val, pulse_len))
        orig_req_trig2(val, pulse_len=pulse_len, timestamp_mono=timestamp_mono)

    state2.command_bus.request_trigger = _spy_trig2

    now2 = time.monotonic()
    state2.update_health(serial_connected=True, npu_latency_ms=345.0)
    state2.update_turret_telemetry(100, -50, False, now2)
    state2.set_turret_mode("CALIBRATE", primed=True)
    state2.target_filter.update_measurement(
        100.0, -50.0 - drop_mrad - cfg.optics.tilt_calibration_trim_mrad, now2 - 1.20
    )
    state2.set_primary_target({
        "center": (640, 360),
        "box": [610, 340, 670, 380],
        "score": 0.90,
        "area": 2400,
        "range_m": 15.0,
        "capture_mono": now2 - 1.20,
        "last_seen_mono": now2 - 0.85,
    })

    t2 = threading.Thread(target=motion_servoing_worker, args=(cfg, state2), daemon=True)
    t2.start()
    time.sleep(0.06)
    state2.shutdown_event.set()
    t2.join(timeout=1.0)

    assert not any(val == 1 for val, _ in trig_events2)
    assert state2.get_turret_mode()[1] is True  # Prime stayed intact


def test_motion_servo_caps_tilt_velocity_and_expires_by_capture_age():
    cfg = load_config(env={})
    cfg.control.motion_loop_interval_sec = 0.005
    cfg.control.setpoint_min_interval_sec = 0.005
    cfg.control.ballistic_acquire_mrad = 250.0  # Keep deterministic in velocity-servo mode

    state = SystemState(cfg)
    vel_events = []
    orig_req_vel = state.command_bus.request_velocity

    def _spy_vel(pan_s, tilt_s):
        vel_events.append((pan_s, tilt_s))
        orig_req_vel(pan_s, tilt_s)

    state.command_bus.request_velocity = _spy_vel

    now = time.monotonic()
    state.update_health(serial_connected=True, npu_latency_ms=110.0)
    state.update_turret_telemetry(0, 0, False, now)
    state.set_turret_mode("ON")
    state.target_filter.update_measurement(0.0, -150.0, now - 0.10)
    state.target_filter.update_measurement(0.0, -180.0, now - 0.05)
    state.set_primary_target({
        "center": (640, 460),
        "box": [610, 440, 670, 480],
        "score": 0.90,
        "area": 2400,
        "range_m": 15.0,
        "capture_mono": now - 0.05,
        "last_seen_mono": now - 0.01,
    })

    t = threading.Thread(target=motion_servoing_worker, args=(cfg, state), daemon=True)
    t.start()
    time.sleep(0.06)
    state.shutdown_event.set()
    t.join(timeout=1.0)

    assert vel_events
    assert any(vt < 0 for _, vt in vel_events)
    assert all(abs(vt) <= 300 for _, vt in vel_events)

    # Expired capture_age (> coast_max_horizon_sec) clears primary target cleanly
    state_exp = SystemState(cfg)
    now_e = time.monotonic()
    state_exp.update_health(serial_connected=True, npu_latency_ms=110.0)
    state_exp.update_turret_telemetry(0, 0, False, now_e)
    state_exp.set_turret_mode("ON")
    state_exp.target_filter.update_measurement(0.0, 0.0, now_e - 2.5)
    state_exp.set_primary_target({
        "center": (640, 360),
        "box": [610, 340, 670, 380],
        "score": 0.90,
        "area": 2400,
        "range_m": 15.0,
        "capture_mono": now_e - 2.5,
        "last_seen_mono": now_e - 0.2,  # Even if arrival looked recent, capture_age expired!
    })

    t_exp = threading.Thread(target=motion_servoing_worker, args=(cfg, state_exp), daemon=True)
    t_exp.start()
    time.sleep(0.06)
    state_exp.shutdown_event.set()
    t_exp.join(timeout=1.0)

    assert state_exp.get_primary_target() is None
    assert not state_exp.target_filter.is_initialized


# =============================================================================
# 3. VisionPipeline Target Locking & Box Reconstruction Regressions
# =============================================================================


def test_nearest_neighbor_locking_prevents_multi_deer_jumping():
    locked = {"center": (500.0, 400.0), "box": [470, 380, 530, 420]}
    deer_b_higher_conf = {"center": (920.0, 405.0), "box": [890, 385, 950, 425], "score": 0.94}
    deer_a_locked = {"center": (512.0, 398.0), "box": [482, 378, 542, 418], "score": 0.81}

    # Even though Deer B is detections[0] (highest confidence), lock must stay on Deer A
    chosen = vp._select_target_candidate(
        [deer_b_higher_conf, deer_a_locked],
        locked,
        ref_points=((505.0, 400.0), (500.0, 400.0)),
        max_jump_px=90.0,
    )
    assert chosen is deer_a_locked

    # If only distant Deer B is visible, return None rather than injecting a cross-yard jump
    chosen_none = vp._select_target_candidate(
        [deer_b_higher_conf],
        locked,
        ref_points=((505.0, 400.0), (500.0, 400.0)),
        max_jump_px=90.0,
    )
    assert chosen_none is None


def test_nearest_neighbor_dual_reference_tolerates_sudden_stop_after_350ms_gap():
    locked = {"center": (500.0, 400.0), "box": [470, 380, 530, 420]}
    stopped_deer = {"center": (506.0, 402.0), "box": [476, 382, 536, 422], "score": 0.85}
    distant_deer = {"center": (880.0, 400.0), "box": [850, 380, 910, 420], "score": 0.90}

    # Extrapolated ref_xy overshot by 160px after a missed scan, while stationary last_xy stayed at (500, 400)
    chosen = vp._select_target_candidate(
        [distant_deer, stopped_deer],
        locked,
        ref_points=((660.0, 400.0), (500.0, 400.0)),
        max_jump_px=90.0,
    )
    assert chosen is stopped_deer


def test_reconstruct_box_at_center_preserves_size_and_vertical_aim_offset():
    orig_box = [100, 200, 180, 260]  # w=80, h=60
    orig_center = (140.0, 236.0)  # dy_top=36, dy_bot=24
    shifted = vp._reconstruct_box_at_center(300.0, 400.0, orig_box, orig_center)

    assert shifted == [260.0, 364.0, 340.0, 424.0]
    assert (shifted[2] - shifted[0]) == 80.0
    assert (shifted[3] - shifted[1]) == 60.0
