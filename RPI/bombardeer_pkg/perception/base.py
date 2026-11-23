"""Abstract base interface for Bombardeer target perception engines."""

import abc
import numpy as np


class BaseTargetDetector(abc.ABC):
    @property
    @abc.abstractmethod
    def name(self) -> str:
        pass

    @property
    @abc.abstractmethod
    def is_available(self) -> bool:
        pass

    @abc.abstractmethod
    def detect(
        self,
        frame_bgr: np.ndarray,
        capture_mono_time: float,
        is_monochrome: bool = False,
        tilt_mrad: float = 0.0,
        track_box=None,
    ):
        pass

    def get_telemetry(self) -> dict:
        return {}

    def get_last_debug_info(self) -> dict:
        return {"veto_boxes": [], "rejected_animals": []}