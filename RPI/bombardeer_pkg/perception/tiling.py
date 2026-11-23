"""Horizon-band multi-tile planning, tracking crop geometry, minimal ROI bounds, and tensor slicing."""

from typing import List, Tuple

import cv2
import numpy as np

from bombardeer_pkg.config import CameraConfig, OpticsConfig, WildlifeConfig
from bombardeer_pkg.control.ballistics import compute_horizon_y_720


def build_pass_plan(
    sensor_w: int,
    sensor_h: int,
    tilt_mrad: float,
    track_active: bool,
    track_box,
    cam: CameraConfig,
    optics: OpticsConfig,
    wl: WildlifeConfig,
    force_scan: bool = False,
) -> Tuple[str, List[List[dict]]]:
    """
    Constructs the list of 1280x1280 inference passes for the current frame.
    Returns (mode_tag, pass_plan) where each pass is a list of 1 or 2 slot dicts.
    """
    scale_x = float(sensor_w) / float(cam.frame_width)
    scale_y = float(sensor_h) / float(cam.frame_height)
    tile_w = wl.stack_tile_w
    tile_h = wl.stack_tile_h
    slot1_y = wl.stack_slot1_y

    # 1. Target-Centered Square Tracking Crop (works moving or still, Day & Dusk)
    if track_active and track_box is not None and not force_scan:
        bx1, by1, bx2, by2 = track_box
        nx1, ny1 = bx1 * scale_x, by1 * scale_y
        nx2, ny2 = bx2 * scale_x, by2 * scale_y
        ncx = 0.5 * (nx1 + nx2)
        ncy = 0.5 * (ny1 + ny2)
        nbw = max(16.0, nx2 - nx1)
        nbh = max(16.0, ny2 - ny1)

        max_side = min(sensor_w, sensor_h)
        desired_side = int(round(max(float(tile_w), max(nbw, nbh) * 2.8)))
        crop_side = max(tile_w, min(max_side, desired_side))

        sx = max(0, min(sensor_w - crop_side, int(round(ncx - crop_side * 0.5))))
        sy = max(0, min(sensor_h - crop_side, int(round(ncy - crop_side * 0.5))))

        slot = {
            "dst_y1": 0,
            "dst_y2": tile_w,
            "src_x": sx,
            "src_y": sy,
            "src_w": crop_side,
            "src_h": crop_side,
            "clip_left": False,
            "clip_right": False,
            "overlap_px": 0.0,
        }
        return ("TRK-1:1" if crop_side == tile_w else "TRK-ZOOM"), [[slot]]

    y_hor_native = compute_horizon_y_720(tilt_mrad, cam, optics) * scale_y

    # 2. DAY 12MP Mode (4608x2592): 3-pass two-band horizon tiling
    if sensor_w >= 4000:
        far_y = max(0, min(sensor_h - tile_h, int(round(y_hor_native - optics.horizon_far_headroom_720 * scale_y))))
        far_xs = (0, 1110, 2220, sensor_w - tile_w)
        far_overlap = 170.0
        far_tiles = [
            {
                "src_x": fx,
                "src_y": far_y,
                "src_w": tile_w,
                "src_h": tile_h,
                "clip_left": (idx > 0),
                "clip_right": (idx < len(far_xs) - 1),
                "overlap_px": far_overlap,
            }
            for idx, fx in enumerate(far_xs)
        ]

        pass0 = [
            dict(far_tiles[0], dst_y1=0, dst_y2=tile_h),
            dict(far_tiles[1], dst_y1=slot1_y, dst_y2=tile_w),
        ]
        pass1 = [
            dict(far_tiles[2], dst_y1=0, dst_y2=tile_h),
            dict(far_tiles[3], dst_y1=slot1_y, dst_y2=tile_w),
        ]

        wide_w = tile_w * 2
        wide_h = tile_h * 2
        near_y = max(0, min(sensor_h - wide_h, int(round(y_hor_native - optics.horizon_headroom_720 * scale_y))))
        near_overlap = float(wide_w - (sensor_w - wide_w))

        pass2 = [
            {
                "dst_y1": 0,
                "dst_y2": tile_h,
                "src_x": 0,
                "src_y": near_y,
                "src_w": wide_w,
                "src_h": wide_h,
                "clip_left": False,
                "clip_right": True,
                "overlap_px": near_overlap,
            },
            {
                "dst_y1": slot1_y,
                "dst_y2": tile_w,
                "src_x": sensor_w - wide_w,
                "src_y": near_y,
                "src_w": wide_w,
                "src_h": wide_h,
                "clip_left": True,
                "clip_right": False,
                "overlap_px": near_overlap,
            },
        ]
        return "SCAN-12MP", [pass0, pass1, pass2]

    # 3. DUSK/NIGHT 3MP Mode (2304x1296): 1-pass stacked 2-tile native scan
    dusk_y = max(0, min(sensor_h - tile_h, int(round(y_hor_native - optics.horizon_headroom_720 * scale_y))))
    right_x = max(0, sensor_w - tile_w)
    overlap = float(tile_w - right_x)

    pass0 = [
        {
            "dst_y1": 0,
            "dst_y2": tile_h,
            "src_x": 0,
            "src_y": dusk_y,
            "src_w": tile_w,
            "src_h": tile_h,
            "clip_left": False,
            "clip_right": True,
            "overlap_px": overlap,
        },
        {
            "dst_y1": slot1_y,
            "dst_y2": tile_w,
            "src_x": right_x,
            "src_y": dusk_y,
            "src_w": tile_w,
            "src_h": tile_h,
            "clip_left": True,
            "clip_right": False,
            "overlap_px": overlap,
        },
    ]
    return "SCAN-3MP", [pass0]


def compute_plan_roi_bounds(pass_plan: List[List[dict]]) -> Tuple[int, int, int, int]:
    """
    Computes the minimal enclosing native-sensor rectangle (roi_x0, roi_y0, roi_w, roi_h)
    needed by all passes in pass_plan so VisionMain copies only required rows/columns.
    """
    min_x = min(s["src_x"] for p in pass_plan for s in p)
    min_y = min(s["src_y"] for p in pass_plan for s in p)
    max_x = max(s["src_x"] + s["src_w"] for p in pass_plan for s in p)
    max_y = max(s["src_y"] + s["src_h"] for p in pass_plan for s in p)
    return min_x, min_y, max_x - min_x, max_y - min_y


def fill_tensor_from_slots(
    roi_frame: np.ndarray,
    slots: List[dict],
    tensor_hwc: np.ndarray,
    roi_x0: int = 0,
    roi_y0: int = 0,
    src_is_rgb: bool = True,
    divider_gray: int = 114,
    tile_w: int = 1280,
    tile_h: int = 620,
    slot1_y: int = 660,
):
    """
    Slices tiles from roi_frame (anchored at roi_x0, roi_y0) into tensor_hwc (1280x1280x3 RGB).
    Uses cv2.INTER_AREA for anti-aliased downscaling and skips BGR2RGB when PiSP provides RGB directly.
    """
    if len(slots) > 1:
        tensor_hwc[tile_h:slot1_y, :, :] = divider_gray

    for s in slots:
        sx = s["src_x"] - roi_x0
        sy = s["src_y"] - roi_y0
        sw, sh = s["src_w"], s["src_h"]
        dy1, dy2 = s["dst_y1"], s["dst_y2"]
        dst_h = dy2 - dy1

        src_roi = roi_frame[sy:sy + sh, sx:sx + sw]
        dst_roi = tensor_hwc[dy1:dy2, :, :]

        if sw == tile_w and sh == dst_h:
            if src_is_rgb:
                np.copyto(dst_roi, src_roi)
            else:
                cv2.cvtColor(src_roi, cv2.COLOR_BGR2RGB, dst=dst_roi)
        else:
            interp = cv2.INTER_AREA if (sw > tile_w or sh > dst_h) else cv2.INTER_LINEAR
            resized = cv2.resize(src_roi, (tile_w, dst_h), interpolation=interp)
            if src_is_rgb:
                np.copyto(dst_roi, resized)
            else:
                cv2.cvtColor(resized, cv2.COLOR_BGR2RGB, dst=dst_roi)