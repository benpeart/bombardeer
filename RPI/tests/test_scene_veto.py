"""Regression tests for scene-wide person/vehicle safety veto, barrel-mask integration, and Walk-Test compatibility."""

from collections import deque
import threading

from bombardeer_pkg.config import load_config
from bombardeer_pkg.control.site_calibration import DEFAULT_OPERATOR_HEIGHT_M, SiteCalibrationManager
from bombardeer_pkg.hardware.vision_pipeline import _apply_scene_wide_veto
from bombardeer_pkg.perception.hailo_detector import HailoWildlifeDetector
from bombardeer_pkg.perception.mdv6_decoder import MDV6Decoder1280
from bombardeer_pkg.state import SystemState
from bombardeer_pkg.utils import RateLimitedLogger


class _FakePipe:
    def __init__(self, items=None):
        self._q = deque(items or [])

    def poll(self):
        return bool(self._q)

    def recv(self):
        return self._q.popleft()


def _make_stub_detector(cfg, state, res_items):
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
    det._shm_busy = False
    det._in_flight_jobs = len(res_items)
    det._pending_jobs = deque((item[2], False) for item in res_items)
    det._min_valid_capture_mono = 0.0
    det._restart_count = 0
    det._res_rx = _FakePipe(res_items)
    return det


def test_hailo_scene_veto_rejects_non_overlapping_animal_when_person_present(tmp_path):
    """Any PERSON anywhere in the frame rejects all animals even with zero bounding-box overlap."""
    cfg = load_config(env={})
    cfg.base_dir = str(tmp_path)
    state = SystemState(cfg)
    state.site_calibration = SiteCalibrationManager(
        cfg, cal_path=str(tmp_path / "site_calibration.json"), auto_boot_sweep=False
    )

    animal = {
        "box": [80, 420, 180, 500],
        "center": (130.0, 460.0),
        "score": 0.91,
        "area": 8000.0,
        "range_m": 15.0,
        "probs": (0.91, 0.0, 0.0),
        "label": "ANIMAL",
    }
    person_veto = {"class": "PERSON", "score": 0.78, "box": [1050, 360, 1120, 560]}
    batch = ([animal], {"veto_boxes": [person_veto], "rejected_animals": []}, 100.0, 115.0, 8.5, "SCAN-3MP", False, False)

    det = _make_stub_detector(cfg, state, [batch])
    det._drain_results()

    assert det._latest_detections == []
    dbg = det.get_last_debug_info()
    assert len(dbg["veto_boxes"]) == 1 and dbg["veto_boxes"][0]["class"] == "PERSON"
    assert len(dbg["rejected_animals"]) == 1
    assert "SCENE_VETO:PERSON(0.78)" in dbg["rejected_animals"][0]["reason"]
    assert state.site_calibration.get_status_dict()["passive_samples"] == 0


def test_hailo_scene_veto_rejects_animal_when_vehicle_present_and_accepts_when_clear(tmp_path):
    cfg = load_config(env={})
    cfg.base_dir = str(tmp_path)
    state = SystemState(cfg)
    state.site_calibration = SiteCalibrationManager(
        cfg, cal_path=str(tmp_path / "site_calibration.json"), auto_boot_sweep=False
    )

    animal = {
        "box": [120, 430, 220, 500],
        "center": (170.0, 465.0),
        "score": 0.89,
        "area": 7000.0,
        "range_m": 15.0,
        "probs": (0.89, 0.0, 0.0),
        "label": "ANIMAL",
    }
    vehicle_veto = {"class": "VEHICLE", "score": 0.64, "box": [900, 380, 1220, 540]}

    b1 = ([dict(animal)], {"veto_boxes": [vehicle_veto], "rejected_animals": []}, 101.0, 110.0, 8.5, "SCAN-3MP", False, False)
    det = _make_stub_detector(cfg, state, [b1])
    det._drain_results()
    assert det._latest_detections == []
    assert "SCENE_VETO:VEHICLE(0.64)" in det.get_last_debug_info()["rejected_animals"][0]["reason"]

    b2 = ([dict(animal)], {"veto_boxes": [], "rejected_animals": []}, 102.0, 110.0, 8.5, "SCAN-3MP", False, False)
    det._res_rx = _FakePipe([b2])
    det._in_flight_jobs = 1
    det._pending_jobs.append((102.0, False))
    det._drain_results()
    assert len(det._latest_detections) == 1


def test_scene_wide_veto_aborts_active_lock_and_disarms_trigger(tmp_path):
    """Any person or vehicle anywhere in the frame aborts an active lock and commands TRIGGER 0."""
    cfg = load_config(env={})
    cfg.base_dir = str(tmp_path)
    state = SystemState(cfg)
    kf = state.target_filter

    kf.update_measurement(150.0, -40.0, 10.0)
    cur_primary = {"box": [100, 430, 200, 500], "center": (150.0, 465.0), "range_m": 15.0}
    state.set_primary_target(cur_primary)

    veto_boxes = [
        {"class": "VEHICLE", "score": 0.52, "box": [950, 390, 1180, 520]},
        {"class": "PERSON", "score": 0.85, "box": [1020, 370, 1080, 540]},
    ]
    aborted, top_veto = _apply_scene_wide_veto(state, kf, cur_primary, veto_boxes)

    assert aborted is True
    assert top_veto["class"] == "PERSON"
    assert state.get_primary_target() is None
    assert kf.predict_state(10.2) is None
    cmds = state.command_bus.drain()
    assert any(c["cmd"] == "TRIGGER" and c["state"] == 0 for c in cmds)


def test_scene_wide_veto_noop_when_clear_or_no_active_lock(tmp_path):
    """Empty veto_boxes must leave locks untouched; unlocked standby must not emit redundant TRIGGER 0."""
    cfg = load_config(env={})
    cfg.base_dir = str(tmp_path)
    state = SystemState(cfg)
    kf = state.target_filter

    # Case 1: Active lock + empty veto_boxes -> no abort, no command bus traffic
    kf.update_measurement(120.0, -25.0, 20.0)
    cur_primary = {"box": [500, 400, 580, 460], "center": (540.0, 430.0), "range_m": 15.0}
    state.set_primary_target(cur_primary)
    aborted, top_veto = _apply_scene_wide_veto(state, kf, cur_primary, [])
    assert aborted is False and top_veto is None
    assert state.get_primary_target() is not None
    assert state.command_bus.drain() == []

    # Case 2: No active lock (cur_primary=None) + person present -> returns top_veto without TRIGGER 0 spam
    state.clear_primary_target()
    kf.reset()
    veto_boxes = [{"class": "PERSON", "score": 0.81, "box": [400, 320, 460, 500]}]
    aborted2, top_veto2 = _apply_scene_wide_veto(state, kf, None, veto_boxes)
    assert aborted2 is False
    assert top_veto2 is not None and top_veto2["class"] == "PERSON"
    assert state.command_bus.drain() == []


def test_end_to_end_barrel_mask_does_not_abort_active_deer_lock(tmp_path):
    """A barrel-tip false PERSON filtered by MDV6Decoder1280 must NOT trigger _apply_scene_wide_veto()."""
    cfg = load_config(env={})
    cfg.base_dir = str(tmp_path)
    state = SystemState(cfg)
    kf = state.target_filter
    dec = MDV6Decoder1280(cfg.camera, cfg.optics, cfg.wildlife, hef=None)

    kf.update_measurement(80.0, -30.0, 30.0)
    cur_primary = {"box": [480, 390, 560, 450], "center": (520.0, 420.0), "range_m": 18.0}
    state.set_primary_target(cur_primary)

    veto_acc = {"boxes": [[877, 239, 96, 48]], "scores": [0.65], "meta": [("PERSON", 0.65)]}
    anim_acc = {"boxes": [[480, 390, 80, 60]], "scores": [0.90], "probs": [(0.90, 0.001, 0.001)]}
    dets, dbg = dec.finalize_sweep(veto_acc, anim_acc)

    assert len(dets) == 1
    assert dbg["veto_boxes"] == []
    aborted, _ = _apply_scene_wide_veto(state, kf, cur_primary, dbg["veto_boxes"])
    assert aborted is False
    assert state.get_primary_target() is not None
    assert state.command_bus.drain() == []


def test_end_to_end_decoder_barrel_mask_feeds_walk_calibration_cleanly(tmp_path):
    """
    During Walk Calibration, a simultaneous barrel-tip false PERSON, standing operator,
    and yard animal through MDV6Decoder1280.finalize_sweep() must reject the animal,
    strip the barrel box, and increment walk_test_samples cleanly.
    """
    cfg = load_config(env={})
    cfg.base_dir = str(tmp_path)
    state = SystemState(cfg)
    cal = SiteCalibrationManager(cfg, cal_path=str(tmp_path / "site_calibration.json"), auto_boot_sweep=False)
    cal.start_walk_test(state)
    state.command_bus.drain()

    dec = MDV6Decoder1280(cfg.camera, cfg.optics, cfg.wildlife, hef=None)

    # Standing 5'9" operator at Z=10m on a 1.0m sawhorse mount
    z_w, h_cam = 10.0, 1.0
    yf = cfg.camera.optical_center[1] + cfg.optics.focal_length_px * (h_cam / z_w)
    yh = cfg.camera.optical_center[1] + cfg.optics.focal_length_px * ((h_cam - DEFAULT_OPERATOR_HEIGHT_M) / z_w)
    bh = int(round(yf - yh))
    bw = max(24, int(round(bh * 0.34)))
    op_xywh = [int(round(600 - 0.5 * bw)), int(round(yh)), bw, bh]

    veto_acc = {
        "boxes": [[877, 239, 96, 48], op_xywh],
        "scores": [0.57, 0.91],
        "meta": [("PERSON", 0.57), ("PERSON", 0.91)],
    }
    anim_acc = {
        "boxes": [[220, 410, 70, 50]],
        "scores": [0.89],
        "probs": [(0.89, 0.001, 0.001)],
    }
    dets, dbg = dec.finalize_sweep(veto_acc, anim_acc)

    assert dets == []
    assert len(dbg["veto_boxes"]) == 1
    assert any("SCENE_VETO:PERSON(0.91)" in r["reason"] for r in dbg["rejected_animals"])

    cal.ingest_walk_test_detections(dbg["veto_boxes"], tilt_at_cap=0.0, eff_capture_mono=50.0, state=state)
    assert cal.get_status_dict()["walk_test_samples"] == 1
