"""Vectorized MegaDetector v6 (YOLOv9-c 1280x1280) Multi-Tile & Crop Post-Processor."""

import logging
import math
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from bombardeer_pkg.config import CameraConfig, OpticsConfig, WildlifeConfig

log = logging.getLogger("Bombardeer")


class MDV6Decoder1280:
    """
    Classes in MDV6:
        0: animal
        1: person
        2: vehicle
    Vectorized across slots and anchors; maps all tile/crop detections into canonical
    1280x720 coordinates before barrel-mask suppression, unified full-frame NMS,
    and scene-wide person/vehicle safety vetoes.
    """

    CLASS_LAYERS = ("MDV6-yolov9-c/conv100", "MDV6-yolov9-c/conv122", "MDV6-yolov9-c/conv144")
    BOX_LAYERS = ("MDV6-yolov9-c/conv98", "MDV6-yolov9-c/conv121", "MDV6-yolov9-c/conv143")

    # Fixed upper-right barrel & receiver shroud polygon in canonical 1280x720 space
    # (~16 px safety margin around the rigidly mounted paintball barrel and shroud).
    DEFAULT_BARREL_POLYGON_720: Tuple[Tuple[int, int], ...] = (
        (860, 232),   # upper-left of muzzle tip
        (1280, 86),   # upper-right above receiver
        (1280, 322),  # lower-right below receiver shroud
        (1215, 290),  # lower corner of receiver shroud
        (1190, 238),  # underside where barrel meets shroud
        (860, 304),   # lower-left of muzzle tip
    )
    DEFAULT_BARREL_MASK_OVERLAP_THRESH: float = 0.55

    def __init__(
        self,
        cam: CameraConfig,
        optics: OpticsConfig,
        wl: WildlifeConfig,
        hef=None,
        quant_params: Optional[Dict[str, Tuple[float, float]]] = None,
    ):
        self.cam = cam
        self.optics = optics
        self.wl = wl
        self.input_size = wl.infer_size
        self.strides = (8, 16, 32)
        self.reg_max = 16
        self.project = np.arange(self.reg_max, dtype=np.float32)

        self.logit_thresh = -math.log((1.0 / wl.conf_thresh) - 1.0)
        self.person_logit_thresh = -math.log((1.0 / wl.person_veto_thresh) - 1.0)
        self.vehicle_logit_thresh = -math.log((1.0 / wl.vehicle_veto_thresh) - 1.0)

        self.barrel_overlap_thresh = float(
            getattr(wl, "barrel_mask_overlap_thresh", self.DEFAULT_BARREL_MASK_OVERLAP_THRESH)
        )
        raw_poly = getattr(wl, "barrel_mask_polygon_720", None)
        self._barrel_integral = self._build_barrel_integral(
            self.DEFAULT_BARREL_POLYGON_720 if raw_poly is None else raw_poly
        )

        self.head_anchors = []
        for stride in self.strides:
            grid_size = self.input_size // stride
            grid_y, grid_x = np.meshgrid(
                np.arange(grid_size, dtype=np.float32),
                np.arange(grid_size, dtype=np.float32),
                indexing="ij",
            )
            anchor = np.stack((grid_x + 0.5, grid_y + 0.5), axis=-1) * stride
            self.head_anchors.append(anchor.reshape(-1, 2))

        self.quant_params: Dict[str, Tuple[float, float]] = {}
        self.u8_animal_thresholds: Dict[str, int] = {}
        self.u8_person_thresholds: Dict[str, int] = {}
        self.u8_vehicle_thresholds: Dict[str, int] = {}

        if quant_params is not None:
            self._init_quant_tables(quant_params)
        elif hef is not None:
            try:
                qp_map = {}
                for vinfo in hef.get_output_vstream_infos():
                    qp = vinfo.quant_info
                    qp_map[vinfo.name] = (float(qp.qp_scale), float(qp.qp_zp))
                self._init_quant_tables(qp_map)
            except Exception as exc:
                log.warning(f"[HAILO] Quantization metadata unavailable ({exc}); using float32 fallback.")
                self.use_uint8_fastpath = False
        else:
            self.use_uint8_fastpath = False

    def _build_barrel_integral(self, polygon_720: Sequence[Sequence[float]]) -> Optional[np.ndarray]:
        """Precomputes an integral mask of the fixed barrel region for O(1) box-overlap checks."""
        if not polygon_720 or len(polygon_720) < 3:
            return None
        fw = int(self.cam.frame_width)
        fh = int(self.cam.frame_height)
        sx = float(fw) / 1280.0
        sy = float(fh) / 720.0
        pts = np.array(
            [[int(round(float(x) * sx)), int(round(float(y) * sy))] for x, y in polygon_720],
            dtype=np.int32,
        )
        mask = np.zeros((fh, fw), dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 1)
        return cv2.integral(mask)

    def barrel_overlap_fraction(self, box_xyxy: Sequence[float]) -> float:
        """Returns fraction [0.0, 1.0] of box_xyxy's area covered by the fixed barrel mask."""
        if self._barrel_integral is None:
            return 0.0
        fw = int(self.cam.frame_width)
        fh = int(self.cam.frame_height)
        x1 = max(0, min(fw, int(round(float(box_xyxy[0])))))
        y1 = max(0, min(fh, int(round(float(box_xyxy[1])))))
        x2 = max(0, min(fw, int(round(float(box_xyxy[2])))))
        y2 = max(0, min(fh, int(round(float(box_xyxy[3])))))
        area = (x2 - x1) * (y2 - y1)
        if area <= 0:
            return 0.0
        integ = self._barrel_integral
        covered = int(integ[y2, x2] - integ[y1, x2] - integ[y2, x1] + integ[y1, x1])
        return float(covered) / float(area)

    def is_in_barrel_mask(self, box_xyxy: Sequence[float]) -> bool:
        """True if the candidate box predominantly sits on the rigidly mounted upper-right barrel."""
        return self.barrel_overlap_fraction(box_xyxy) >= self.barrel_overlap_thresh

    def _init_quant_tables(self, qp_map: Dict[str, Tuple[float, float]]):
        self.quant_params = dict(qp_map)
        for name, (scale, zp) in qp_map.items():
            if name in self.CLASS_LAYERS:
                self.u8_animal_thresholds[name] = int(max(0, min(255, math.ceil((self.logit_thresh / scale) + zp))))
                self.u8_person_thresholds[name] = int(max(0, min(255, math.ceil((self.person_logit_thresh / scale) + zp))))
                self.u8_vehicle_thresholds[name] = int(max(0, min(255, math.ceil((self.vehicle_logit_thresh / scale) + zp))))
        self.use_uint8_fastpath = (len(self.u8_animal_thresholds) == 3)

    @staticmethod
    def _softmax(x: np.ndarray, axis: int = -1) -> np.ndarray:
        e_x = np.exp(x - np.max(x, axis=axis, keepdims=True))
        return e_x / np.sum(e_x, axis=axis, keepdims=True)

    def decode_pass(self, raw_outputs: dict, slots: List[dict], sensor_w: int, sensor_h: int, veto_acc: dict, anim_acc: dict):
        cand_probs_list = []
        cand_boxes_list = []
        cand_anchors_list = []
        cand_scales_list = []

        for idx, stride in enumerate(self.strides):
            cls_name = self.CLASS_LAYERS[idx]
            box_name = self.BOX_LAYERS[idx]

            cls_tensor = raw_outputs[cls_name].reshape(-1, 3)
            animal_col = cls_tensor[:, 0]
            person_col = cls_tensor[:, 1]
            vehicle_col = cls_tensor[:, 2]

            if self.use_uint8_fastpath and cls_tensor.dtype == np.uint8:
                anim_mask = (animal_col >= self.u8_animal_thresholds[cls_name]) & (animal_col > person_col) & (animal_col > vehicle_col)
                pers_mask = person_col >= self.u8_person_thresholds[cls_name]
                veh_mask = vehicle_col >= self.u8_vehicle_thresholds[cls_name]
                combined_mask = anim_mask | pers_mask | veh_mask
                if not np.any(combined_mask):
                    continue
                valid_idx = np.flatnonzero(combined_mask)
                c_scale, c_zp = self.quant_params[cls_name]
                sub_cls_f32 = (cls_tensor[valid_idx].astype(np.float32) - c_zp) * c_scale
                b_scale, b_zp = self.quant_params[box_name]
                raw_boxes = (raw_outputs[box_name].reshape(-1, 4, self.reg_max)[valid_idx].astype(np.float32) - b_zp) * b_scale
            else:
                anim_mask = (animal_col > self.logit_thresh) & (animal_col > person_col) & (animal_col > vehicle_col)
                pers_mask = person_col > self.person_logit_thresh
                veh_mask = vehicle_col > self.vehicle_logit_thresh
                combined_mask = anim_mask | pers_mask | veh_mask
                if not np.any(combined_mask):
                    continue
                valid_idx = np.flatnonzero(combined_mask)
                sub_cls_f32 = cls_tensor[valid_idx].astype(np.float32)
                raw_boxes = raw_outputs[box_name].reshape(-1, 4, self.reg_max)[valid_idx].astype(np.float32)

            probs = 1.0 / (1.0 + np.exp(-sub_cls_f32))
            cand_probs_list.append(probs)
            cand_boxes_list.append(raw_boxes)
            cand_anchors_list.append(self.head_anchors[idx][valid_idx])
            cand_scales_list.append(np.full(len(valid_idx), stride, dtype=np.float32))

        if not cand_probs_list:
            return

        cand_probs = np.vstack(cand_probs_list)
        cand_boxes = np.vstack(cand_boxes_list)
        cand_anchors = np.vstack(cand_anchors_list)
        cand_scales = np.concatenate(cand_scales_list)

        dist = self._softmax(cand_boxes, axis=-1)
        ltrb = np.dot(dist, self.project) * cand_scales[:, None]

        tx1 = np.clip(cand_anchors[:, 0] - ltrb[:, 0], 0.0, float(self.input_size))
        ty1 = np.clip(cand_anchors[:, 1] - ltrb[:, 1], 0.0, float(self.input_size))
        tx2 = np.clip(cand_anchors[:, 0] + ltrb[:, 2], 0.0, float(self.input_size))
        ty2 = np.clip(cand_anchors[:, 1] + ltrb[:, 3], 0.0, float(self.input_size))
        tcy = 0.5 * (ty1 + ty2)

        to_canon_x = float(self.cam.frame_width) / float(sensor_w)
        to_canon_y = float(self.cam.frame_height) / float(sensor_h)
        seam_margin = self.wl.seam_clip_margin_px

        # Vectorized per-slot projection (at most 2 slots per pass)
        for s in slots:
            dst_y1 = float(s["dst_y1"])
            dst_y2 = float(s["dst_y2"])
            in_slot = (tcy >= dst_y1) & (tcy < dst_y2)
            if not np.any(in_slot):
                continue

            bx1 = tx1[in_slot]
            bx2 = tx2[in_slot]
            by1 = np.clip(ty1[in_slot], dst_y1, dst_y2)
            by2 = np.clip(ty2[in_slot], dst_y1, dst_y2)
            probs = cand_probs[in_slot]

            size_ok = ((bx2 - bx1) >= 8.0) & ((by2 - by1) >= 8.0)
            if not np.any(size_ok):
                continue

            bx1, bx2, by1, by2, probs = bx1[size_ok], bx2[size_ok], by1[size_ok], by2[size_ok], probs[size_ok]

            sx_tile = float(s["src_w"]) / float(self.input_size)
            sy_tile = float(s["src_h"]) / (dst_y2 - dst_y1)
            nx1 = s["src_x"] + (bx1 * sx_tile)
            ny1 = s["src_y"] + ((by1 - dst_y1) * sy_tile)
            nx2 = s["src_x"] + (bx2 * sx_tile)
            ny2 = s["src_y"] + ((by2 - dst_y1) * sy_tile)

            cx1 = np.rint(np.clip(nx1 * to_canon_x, 0.0, float(self.cam.frame_width))).astype(np.int32)
            cy1 = np.rint(np.clip(ny1 * to_canon_y, 0.0, float(self.cam.frame_height))).astype(np.int32)
            cx2 = np.rint(np.clip(nx2 * to_canon_x, 0.0, float(self.cam.frame_width))).astype(np.int32)
            cy2 = np.rint(np.clip(ny2 * to_canon_y, 0.0, float(self.cam.frame_height))).astype(np.int32)
            cbw = cx2 - cx1
            cbh = cy2 - cy1

            canon_ok = (cbw >= 4) & (cbh >= 4)
            if not np.any(canon_ok):
                continue

            bx1, bx2 = bx1[canon_ok], bx2[canon_ok]
            nx1, nx2 = nx1[canon_ok], nx2[canon_ok]
            cx1, cy1, cbw, cbh = cx1[canon_ok], cy1[canon_ok], cbw[canon_ok], cbh[canon_ok]
            p_anim = probs[canon_ok, 0]
            p_pers = probs[canon_ok, 1]
            p_veh = probs[canon_ok, 2]

            # Person & Vehicle vetoes (never seam-dropped; barrel-mask filtered before NMS)
            for idx_p in np.flatnonzero(p_pers >= self.wl.person_veto_thresh):
                veto_acc["boxes"].append([int(cx1[idx_p]), int(cy1[idx_p]), int(cbw[idx_p]), int(cbh[idx_p])])
                veto_acc["scores"].append(float(p_pers[idx_p]))
                veto_acc["meta"].append(("PERSON", float(p_pers[idx_p])))

            for idx_v in np.flatnonzero(p_veh >= self.wl.vehicle_veto_thresh):
                veto_acc["boxes"].append([int(cx1[idx_v]), int(cy1[idx_v]), int(cbw[idx_v]), int(cbh[idx_v])])
                veto_acc["scores"].append(float(p_veh[idx_v]))
                veto_acc["meta"].append(("VEHICLE", float(p_veh[idx_v])))

            # Animal candidates with internal-seam fragment filter
            anim_ok = (
                (p_anim >= self.wl.conf_thresh)
                & (p_anim >= (p_pers + self.wl.class_margin))
                & (p_anim >= (p_veh + self.wl.class_margin))
            )
            overlap_px = float(s.get("overlap_px", 0.0))
            if overlap_px > 0.0:
                fits_overlap = (nx2 - nx1) < (overlap_px - 16.0)
                if s.get("clip_left", False):
                    anim_ok &= ~(fits_overlap & (bx1 <= seam_margin))
                if s.get("clip_right", False):
                    anim_ok &= ~(fits_overlap & (bx2 >= (self.input_size - seam_margin)))

            for idx_a in np.flatnonzero(anim_ok):
                anim_acc["boxes"].append([int(cx1[idx_a]), int(cy1[idx_a]), int(cbw[idx_a]), int(cbh[idx_a])])
                anim_acc["scores"].append(float(p_anim[idx_a]))
                anim_acc["probs"].append((float(p_anim[idx_a]), float(p_pers[idx_a]), float(p_veh[idx_a])))

    def finalize_sweep(self, veto_acc: dict, anim_acc: dict) -> Tuple[List[dict], dict]:
        debug_info = {"veto_boxes": [], "rejected_animals": []}

        # Filter out fixed upper-right barrel false detections BEFORE NMS so a barrel
        # artifact never triggers a scene-wide veto or suppresses a real nearby person.
        veto_boxes = []
        if veto_acc["boxes"]:
            kept_v_boxes = []
            kept_v_scores = []
            kept_v_meta = []
            for box_xywh, score, meta in zip(veto_acc["boxes"], veto_acc["scores"], veto_acc["meta"]):
                vx, vy, vw, vh = box_xywh
                if self.is_in_barrel_mask((vx, vy, vx + vw, vy + vh)):
                    continue
                kept_v_boxes.append(box_xywh)
                kept_v_scores.append(score)
                kept_v_meta.append(meta)

            if kept_v_boxes:
                v_idx = cv2.dnn.NMSBoxes(
                    kept_v_boxes,
                    kept_v_scores,
                    min(self.wl.person_veto_thresh, self.wl.vehicle_veto_thresh),
                    0.45,
                )
                if len(v_idx) > 0:
                    for vi in v_idx.flatten():
                        vx, vy, vw, vh = kept_v_boxes[vi]
                        v_cls, v_score = kept_v_meta[vi]
                        veto_boxes.append({"class": v_cls, "score": v_score, "box": [vx, vy, vx + vw, vy + vh]})

        debug_info["veto_boxes"] = veto_boxes
        if not anim_acc["boxes"]:
            return [], debug_info

        kept_a_boxes = []
        kept_a_scores = []
        kept_a_probs = []
        for box_xywh, score, probs in zip(anim_acc["boxes"], anim_acc["scores"], anim_acc["probs"]):
            bx, by, bw, bh = box_xywh
            if self.is_in_barrel_mask((bx, by, bx + bw, by + bh)):
                continue
            kept_a_boxes.append(box_xywh)
            kept_a_scores.append(score)
            kept_a_probs.append(probs)

        if not kept_a_boxes:
            return [], debug_info

        indices = cv2.dnn.NMSBoxes(kept_a_boxes, kept_a_scores, self.wl.conf_thresh, self.wl.iou_thresh)
        if len(indices) == 0:
            return [], debug_info

        # Scene-wide safety veto: if ANY non-barrel person or vehicle is detected anywhere
        # in the frame, reject all animal detections in this sweep.
        if veto_boxes:
            top_veto = max(veto_boxes, key=lambda v: float(v.get("score", 0.0)))
            v_cls = top_veto["class"]
            v_score = float(top_veto["score"])
            for idx in indices.flatten():
                bx, by, bw, bh = kept_a_boxes[idx]
                debug_info["rejected_animals"].append({
                    "reason": f"SCENE_VETO:{v_cls}({v_score:.2f})",
                    "box": [bx, by, bx + bw, by + bh],
                    "probs": kept_a_probs[idx],
                })
            return [], debug_info

        detections = []
        frame_area = float(self.cam.frame_width * self.cam.frame_height)

        for idx in indices.flatten():
            bx, by, bw, bh = kept_a_boxes[idx]
            box_xyxy = [bx, by, bx + bw, by + bh]
            p_anim, p_pers, p_veh = kept_a_probs[idx]
            area = float(bw * bh)

            if (bw / self.cam.frame_width) > self.wl.max_animal_width_frac or (area / frame_area) > self.wl.max_animal_area_frac:
                debug_info["rejected_animals"].append({"reason": "SIZE_LIMIT", "box": box_xyxy, "probs": (p_anim, p_pers, p_veh)})
                continue

            if p_pers >= self.wl.max_non_animal_prob or p_veh >= self.wl.max_non_animal_prob:
                debug_info["rejected_animals"].append({
                    "reason": f"NON_ANIMAL_PROB(P:{p_pers:.2f},V:{p_veh:.2f})",
                    "box": box_xyxy,
                    "probs": (p_anim, p_pers, p_veh),
                })
                continue

            aspect_ratio = float(bh) / max(1.0, float(bw))
            if aspect_ratio > self.wl.max_animal_aspect_ratio:
                debug_info["rejected_animals"].append({
                    "reason": f"ASPECT_RATIO({aspect_ratio:.2f})",
                    "box": box_xyxy,
                    "probs": (p_anim, p_pers, p_veh),
                })
                continue

            cx = bx + (bw / 2.0)
            cy = by + (bh * 0.60)
            est_range_m = (self.wl.assumed_target_height_mm * self.optics.focal_length_px) / (max(5.0, float(bh)) * 1000.0)
            clamped_range_m = float(np.clip(est_range_m, self.optics.min_valid_range_m, self.optics.max_valid_range_m))

            detections.append({
                "box": box_xyxy,
                "center": (cx, cy),
                "score": p_anim,
                "probs": (p_anim, p_pers, p_veh),
                "area": area,
                "range_m": clamped_range_m,
                "id": 1,
                "label": f"ANIMAL {p_anim * 100:.0f}%",
            })

        detections.sort(key=lambda d: (d["score"], d["area"]), reverse=True)
        return detections, debug_info
