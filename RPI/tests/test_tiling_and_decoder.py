"""Tests for horizon tiling, minimal ROI slicing, tensor packing, barrel masking, and scene-wide MDV6 safety vetoes."""

import numpy as np

from bombardeer_pkg.config import load_config
from bombardeer_pkg.perception.mdv6_decoder import MDV6Decoder1280
from bombardeer_pkg.perception.tiling import (
    build_pass_plan,
    compute_plan_roi_bounds,
    fill_tensor_from_slots,
)


def test_day_12mp_plan_and_minimal_roi_bounds():
    cfg = load_config(env={})
    tag, passes = build_pass_plan(4608, 2592, 0.0, False, None, cfg.camera, cfg.optics, cfg.wildlife)
    assert tag == "SCAN-12MP"
    assert len(passes) == 3
    assert all(len(p) == 2 for p in passes)

    rx0, ry0, rw, rh = compute_plan_roi_bounds(passes)
    assert rx0 == 0 and rw == 4608
    assert 0 <= ry0 < (ry0 + rh) <= 2592
    assert rh < 2000  # Minimal ROI excludes sky/ledge rows
    for slots in passes:
        for s in slots:
            assert s["src_y"] >= ry0 and (s["src_y"] + s["src_h"]) <= (ry0 + rh)


def test_dusk_3mp_plan_and_tensor_slicing():
    cfg = load_config(env={})
    tag, passes = build_pass_plan(2304, 1296, 0.0, False, None, cfg.camera, cfg.optics, cfg.wildlife)
    assert tag == "SCAN-3MP"
    assert len(passes) == 1
    s0, s1 = passes[0]
    assert s0["src_x"] == 0 and s1["src_x"] == 1024 and s0["overlap_px"] == 256.0

    rx0, ry0, rw, rh = compute_plan_roi_bounds(passes)
    roi = np.full((rh, rw, 3), 200, dtype=np.uint8)
    tensor = np.zeros((1280, 1280, 3), dtype=np.uint8)
    fill_tensor_from_slots(roi, passes[0], tensor, roi_x0=rx0, roi_y0=ry0, src_is_rgb=True)

    assert np.all(tensor[0:620] == 200)
    assert np.all(tensor[620:660] == 114)  # Gray divider bar
    assert np.all(tensor[660:1280] == 200)


def test_tracking_crop_plan():
    cfg = load_config(env={})
    tag, passes = build_pass_plan(4608, 2592, 0.0, True, (600, 340, 680, 420), cfg.camera, cfg.optics, cfg.wildlife)
    assert tag in ("TRK-1:1", "TRK-ZOOM")
    assert len(passes) == 1 and len(passes[0]) == 1
    slot = passes[0][0]
    assert slot["dst_y1"] == 0 and slot["dst_y2"] == 1280
    assert slot["src_w"] == slot["src_h"] >= 1280


def test_decoder_scene_wide_vetoes_and_synthetic_decode_pass():
    cfg = load_config(env={})
    dec = MDV6Decoder1280(cfg.camera, cfg.optics, cfg.wildlife, hef=None)

    # Dead rectangle-overlap helper must be completely removed
    assert not hasattr(MDV6Decoder1280, "check_veto_overlap")

    # 1. Scene-wide person veto rejects animal even on opposite side of frame (zero overlap)
    veto_acc = {"boxes": [[100, 320, 60, 140]], "scores": [0.72], "meta": [("PERSON", 0.72)]}
    anim_acc = {"boxes": [[680, 380, 60, 45]], "scores": [0.92], "probs": [(0.92, 0.005, 0.001)]}
    dets, dbg = dec.finalize_sweep(veto_acc, anim_acc)
    assert dets == []
    assert len(dbg["veto_boxes"]) == 1 and dbg["veto_boxes"][0]["class"] == "PERSON"
    assert any("SCENE_VETO:PERSON(0.72)" in r["reason"] for r in dbg["rejected_animals"])

    # 2. Standing-human aspect ratio (>1.65) rejected when scene is clear
    veto_acc = {"boxes": [], "scores": [], "meta": []}
    anim_acc = {"boxes": [[500, 300, 30, 80]], "scores": [0.90], "probs": [(0.90, 0.001, 0.001)]}
    dets, dbg = dec.finalize_sweep(veto_acc, anim_acc)
    assert dets == []
    assert any("ASPECT_RATIO" in r["reason"] for r in dbg["rejected_animals"])

    # 3. Clean deer candidate accepted when no person/vehicle is present
    anim_acc = {"boxes": [[500, 380, 60, 45]], "scores": [0.88], "probs": [(0.88, 0.001, 0.001)]}
    dets, _ = dec.finalize_sweep(veto_acc, anim_acc)
    assert len(dets) == 1 and dets[0]["box"] == [500, 380, 560, 425]

    # 4. Synthetic stride-32 logit maps cleanly through vectorized decode_pass
    raw = {}
    for cls_l, box_l, stride in zip(dec.CLASS_LAYERS, dec.BOX_LAYERS, dec.strides):
        g = 1280 // stride
        raw[cls_l] = np.full((1, g, g, 3), -10.0, dtype=np.float32)
        raw[box_l] = np.zeros((1, g, g, 4 * 16), dtype=np.float32)

    raw[dec.CLASS_LAYERS[2]][0, 9, 20, 0] = 5.0
    box_vec = np.full((4, 16), -10.0, dtype=np.float32)
    box_vec[:, 2] = 10.0
    raw[dec.BOX_LAYERS[2]][0, 9, 20, :] = box_vec.reshape(-1)

    slots = [{
        "dst_y1": 0, "dst_y2": 620, "src_x": 0, "src_y": 300,
        "src_w": 1280, "src_h": 620, "clip_left": False, "clip_right": True, "overlap_px": 256.0,
    }]
    v_acc = {"boxes": [], "scores": [], "meta": []}
    a_acc = {"boxes": [], "scores": [], "probs": []}
    dec.decode_pass(raw, slots, sensor_w=2304, sensor_h=1296, veto_acc=v_acc, anim_acc=a_acc)
    assert len(a_acc["boxes"]) == 1


def test_barrel_mask_suppresses_barrel_false_person_without_blocking_animals():
    """
    When the turret tilts down and the scan strip includes the fixed upper-right
    paintball barrel, a false PERSON/VEHICLE detection on the barrel must be
    masked out so it does NOT trigger a scene-wide veto and reject yard deer.
    """
    cfg = load_config(env={})
    dec = MDV6Decoder1280(cfg.camera, cfg.optics, cfg.wildlife, hef=None)

    # Exact field-observed barrel-tip VETO:PERSON [877, 239, 973, 287] and receiver shroud box
    assert dec.is_in_barrel_mask([877, 239, 973, 287])
    assert dec.is_in_barrel_mask([980, 175, 1150, 255])
    assert dec.is_in_barrel_mask([1190, 115, 1279, 305])

    veto_acc = {
        "boxes": [[877, 239, 96, 48], [1190, 115, 89, 190]],
        "scores": [0.57, 0.62],
        "meta": [("PERSON", 0.57), ("VEHICLE", 0.62)],
    }
    anim_acc = {
        "boxes": [[500, 380, 60, 45], [880, 242, 90, 44]],  # real deer + false barrel animal
        "scores": [0.89, 0.85],
        "probs": [(0.89, 0.001, 0.001), (0.85, 0.001, 0.001)],
    }
    dets, dbg = dec.finalize_sweep(veto_acc, anim_acc)

    # Barrel artifacts suppressed -> real deer accepted, zero false vetoes emitted
    assert dbg["veto_boxes"] == []
    assert len(dets) == 1
    assert dets[0]["box"] == [500, 380, 560, 425]


def test_barrel_mask_still_vetoes_real_person_walking_below_barrel():
    """A real person standing in the yard beneath/behind the barrel tip must still trigger SCENE_VETO."""
    cfg = load_config(env={})
    dec = MDV6Decoder1280(cfg.camera, cfg.optics, cfg.wildlife, hef=None)

    # Person extends from y=260 (head touching barrel tip) down to y=480 (feet on yard)
    real_person_xywh = [890, 260, 70, 220]
    assert not dec.is_in_barrel_mask([890, 260, 960, 480])

    # Include a higher-scoring barrel false positive overlapping the person's head to verify
    # pre-NMS barrel filtering prevents the barrel artifact from suppressing the real person.
    veto_acc = {
        "boxes": [[877, 239, 96, 48], real_person_xywh],
        "scores": [0.95, 0.78],
        "meta": [("PERSON", 0.95), ("PERSON", 0.78)],
    }
    anim_acc = {
        "boxes": [[300, 410, 70, 50]],
        "scores": [0.91],
        "probs": [(0.91, 0.001, 0.001)],
    }
    dets, dbg = dec.finalize_sweep(veto_acc, anim_acc)

    assert dets == []
    assert len(dbg["veto_boxes"]) == 1
    assert dbg["veto_boxes"][0]["box"] == [890, 260, 960, 480]
    assert any("SCENE_VETO:PERSON(0.78)" in r["reason"] for r in dbg["rejected_animals"])
