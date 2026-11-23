"""Thread-safe world-space 4-state Kalman Filter with non-mutating high-rate prediction."""

import threading
from typing import Optional, Tuple

import cv2
import numpy as np


class TargetKalmanFilter:
    """
    State vector: [pan_mrad, tilt_mrad, v_pan_mrad_s, v_tilt_mrad_s].

    Design invariants:
      - update_measurement() advances the filter strictly using the frame's true
        monotonic capture timestamp and a kinematic Q(dt) acceleration noise model.
      - predict_state() is a pure, read-only kinematic extrapolation at 50 Hz so
        high-frequency servo polling never mutates statePost or inflates errorCovPost.
    """

    def __init__(
        self,
        accel_noise_mrad_s2: float = 150.0,
        measurement_noise: float = 10.0,
        process_noise: Optional[float] = None,
    ):
        self._lock = threading.Lock()
        self._accel_var = float(accel_noise_mrad_s2) ** 2
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.transitionMatrix = np.eye(4, dtype=np.float32)

        self.kf.measurementMatrix = np.zeros((2, 4), dtype=np.float32)
        self.kf.measurementMatrix[0, 0] = 1.0
        self.kf.measurementMatrix[1, 1] = 1.0

        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * float(measurement_noise)
        self._init_cov = np.diag([50.0, 50.0, 5000.0, 5000.0]).astype(np.float32)
        self.kf.errorCovPost = self._init_cov.copy()

        self.last_meas_mono: Optional[float] = None
        self.is_initialized = False

    def _build_process_noise(self, dt: float) -> np.ndarray:
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        q_pp = 0.25 * dt4 * self._accel_var
        q_pv = 0.50 * dt3 * self._accel_var
        q_vv = dt2 * self._accel_var

        q = np.zeros((4, 4), dtype=np.float32)
        q[0, 0] = q[1, 1] = q_pp
        q[0, 2] = q[2, 0] = q_pv
        q[1, 3] = q[3, 1] = q_pv
        q[2, 2] = q[3, 3] = q_vv
        return q

    def reset(self):
        with self._lock:
            self.is_initialized = False
            self.last_meas_mono = None

    def update_measurement(
        self, measured_pan: float, measured_tilt: float, capture_mono: float
    ) -> Tuple[int, int, float, float]:
        with self._lock:
            if not self.is_initialized or self.last_meas_mono is None:
                init_state = np.array(
                    [[float(measured_pan)], [float(measured_tilt)], [0.0], [0.0]],
                    dtype=np.float32,
                )
                self.kf.statePre = init_state.copy()
                self.kf.statePost = init_state.copy()
                self.kf.errorCovPre = self._init_cov.copy()
                self.kf.errorCovPost = self._init_cov.copy()
                self.last_meas_mono = capture_mono
                self.is_initialized = True
                return int(round(measured_pan)), int(round(measured_tilt)), 0.0, 0.0

            raw_dt = capture_mono - self.last_meas_mono
            if raw_dt < -1e-4:
                # Ignore out-of-order stale frame without corrupting filter velocity
                sp = self.kf.statePost
                return int(round(float(sp[0, 0]))), int(round(float(sp[1, 0]))), float(sp[2, 0]), float(sp[3, 0])

            dt = max(0.001, min(1.5, raw_dt))
            self.last_meas_mono = capture_mono

            self.kf.transitionMatrix[0, 2] = dt
            self.kf.transitionMatrix[1, 3] = dt
            self.kf.processNoiseCov = self._build_process_noise(dt)

            self.kf.predict()
            measurement = np.array([[float(measured_pan)], [float(measured_tilt)]], dtype=np.float32)
            estimated = self.kf.correct(measurement)

            return (
                int(round(float(estimated[0, 0]))),
                int(round(float(estimated[1, 0]))),
                float(estimated[2, 0]),
                float(estimated[3, 0]),
            )

    def predict_state(self, query_mono: float) -> Optional[Tuple[int, int, float, float]]:
        """Read-only extrapolation to query_mono without mutating filter covariance."""
        with self._lock:
            if not self.is_initialized or self.last_meas_mono is None:
                return None

            dt = max(0.0, min(2.0, query_mono - self.last_meas_mono))
            pan = float(self.kf.statePost[0, 0])
            tilt = float(self.kf.statePost[1, 0])
            v_pan = float(self.kf.statePost[2, 0])
            v_tilt = float(self.kf.statePost[3, 0])

            pred_pan = pan + (v_pan * dt)
            pred_tilt = tilt + (v_tilt * dt)
            return int(round(pred_pan)), int(round(pred_tilt)), v_pan, v_tilt

    def clamp_position(self, pan_clamped: bool, world_pan: float, tilt_clamped: bool, world_tilt: float):
        """Zeroes boundary velocity in-place when predicted world target hits software limits."""
        with self._lock:
            if not self.is_initialized:
                return
            if pan_clamped:
                self.kf.statePost[0, 0] = float(world_pan)
                self.kf.statePost[2, 0] = 0.0
            if tilt_clamped:
                self.kf.statePost[1, 0] = float(world_tilt)
                self.kf.statePost[3, 0] = 0.0