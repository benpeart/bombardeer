"""Tests for horizon tiling, minimal ROI slicing, tensor packing, and MDV6 safety vetoes."""

import numpy as np

from bombardeer_pkg.config import load_config
from bombardeer_pkg.control.ballistics import image_px_to_world_mrad, project_world_to_image_px
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


def test_decoder_vetoes_and_synthetic_decode_pass():
    cfg = load_config(env={})
    dec = MDV6Decoder1280(cfg.camera, cfg.optics, cfg.wildlife, hef=None)

    # 1. Person overlap veto blocks overlapping animal
    veto_acc = {"boxes": [[200, 300, 60, 120]], "scores": [0.40], "meta": [("PERSON", 0.40)]}
    anim_acc = {"boxes": [[210, 340, 50, 40]], "scores": [0.92], "probs": [(0.92, 0.005, 0.001)]}
    dets, dbg = dec.finalize_sweep(veto_acc, anim_acc)
    assert dets == []
    assert any("OVERLAP_PERSON" in r["reason"] for r in dbg["rejected_animals"])

    # 2. Standing-human aspect ratio (>1.65) rejected
    veto_acc = {"boxes": [], "scores": [], "meta": []}
    anim_acc = {"boxes": [[500, 300, 30, 80]], "scores": [0.90], "probs": [(0.90, 0.001, 0.001)]}
    dets, dbg = dec.finalize_sweep(veto_acc, anim_acc)
    assert dets == []
    assert any("ASPECT_RATIO" in r["reason"] for r in dbg["rejected_animals"])

    # 3. Clean deer candidate accepted
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


def test_projected_active_box_catches_veto_overlap_after_turret_slew():
    """
    During a 350ms NPU cycle, turret pan slews shift pixel coordinates.
    Projecting the locked world target into the new batch's pose catches an
    overlapping person that the stale pre-slew pixel box would miss.
    """
    cfg = load_config(env={})
    rng = 20.0
    # At t0 (pan=0, tilt=0), deer is centered at (640, 400)
    stale_box = [615, 380, 665, 420]
    w_pan, w_tilt = image_px_to_world_mrad(640.0, 400.0, 0.0, 0.0, rng, cfg.camera, cfg.optics)

    # By t1 (350ms later), turret panned +180 mrad left -> deer shifts right in image (~+98px)
    px_t1, py_t1 = project_world_to_image_px(w_pan, w_tilt, 180.0, 0.0, rng, cfg.camera, cfg.optics)
    projected_box = [px_t1 - 25.0, py_t1 - 20.0, px_t1 + 25.0, py_t1 + 20.0]

    # Person stands right next to the deer in t1's image coordinates
    person_veto = [{"class": "PERSON", "score": 0.85, "box": [px_t1 - 15.0, py_t1 - 50.0, px_t1 + 20.0, py_t1 + 25.0]}]

    stale_hit, _ = MDV6Decoder1280.check_veto_overlap(stale_box, person_veto, pad=cfg.wildlife.veto_padding_px)
    proj_hit, veto_obj = MDV6Decoder1280.check_veto_overlap(projected_box, person_veto, pad=cfg.wildlife.veto_padding_px)

    assert not stale_hit
    assert proj_hit and veto_obj["class"] == "PERSON"
