"""ArUco Fiducial Detector for precision calibration and range testing."""

import logging
import math

import cv2
import numpy as np

from bombardeer_pkg.config import BombardeerConfig
from bombardeer_pkg.perception.base import BaseTargetDetector

log = logging.getLogger("Bombardeer")


class ArucoTargetDetector(BaseTargetDetector):
    ARUCO_REAL_WIDTH_MM = 365.0
    TARGET_MARKER_ID = 0
    MIN_EDGE_PX = 8.0

    def __init__(self, cfg: BombardeerConfig, target_id=None, marker_width_mm=None):
        self.cfg = cfg
        self.target_id = self.TARGET_MARKER_ID if target_id is None else target_id
        self.marker_width_mm = self.ARUCO_REAL_WIDTH_MM if marker_width_mm is None else marker_width_mm

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.errorCorrectionRate = 0.55
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 19
        self.parameters.adaptiveThreshWinSizeStep = 4

        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        inv_gamma = 1.0 / 0.6
        self.gamma_lut = np.array(
            [((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)],
            dtype=np.uint8,
        )

    @property
    def name(self) -> str:
        return "ARUCO"

    @property
    def is_available(self) -> bool:
        return True

    def detect(
        self,
        frame_bgr: np.ndarray,
        capture_mono_time: float,
        is_monochrome: bool = False,
        tilt_mrad: float = 0.0,
        track_box=None,
    ):
        if frame_bgr is None:
            return [], capture_mono_time
        try:
            lifted = cv2.LUT(frame_bgr, self.gamma_lut)
            gray = cv2.cvtColor(lifted, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)
            detections = []

            if ids is not None and len(ids) > 0:
                optics = self.cfg.optics
                for marker_corners, marker_id in zip(corners, ids.flatten()):
                    if self.target_id is not None and marker_id != self.target_id:
                        continue

                    pts = marker_corners.reshape((4, 2))
                    d01 = math.hypot(pts[0][0] - pts[1][0], pts[0][1] - pts[1][1])
                    d12 = math.hypot(pts[1][0] - pts[2][0], pts[1][1] - pts[2][1])
                    if d01 < self.MIN_EDGE_PX or d12 < self.MIN_EDGE_PX:
                        continue

                    ratio = d01 / max(1.0, d12)
                    if not (0.35 <= ratio <= 2.85):
                        continue

                    cx = float(np.mean(pts[:, 0]))
                    cy = float(np.mean(pts[:, 1]))
                    x1, y1 = int(np.min(pts[:, 0])), int(np.min(pts[:, 1]))
                    x2, y2 = int(np.max(pts[:, 0])), int(np.max(pts[:, 1]))
                    area = float(cv2.contourArea(pts.astype(np.float32)))

                    marker_px = max(1.0, (d01 + d12) / 2.0)
                    est_range_m = (self.marker_width_mm * optics.focal_length_px) / (marker_px * 1000.0)
                    clamped_range_m = float(np.clip(est_range_m, optics.min_valid_range_m, optics.max_valid_range_m))

                    detections.append({
                        "box": [x1, y1, x2, y2],
                        "center": (cx, cy),
                        "score": 0.99,
                        "area": area,
                        "range_m": clamped_range_m,
                        "id": int(marker_id),
                        "label": f"ID:{int(marker_id)}",
                    })

                detections.sort(key=lambda d: d["area"], reverse=True)

            return detections, capture_mono_time
        except Exception as exc:
            log.error(f"[ARUCO DETECTOR ERROR] Vision parsing failed: {exc}")
            return [], capture_mono_time