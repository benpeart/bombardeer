"""Thread-safe world-space 4-state Kalman Filter with encapsulated kinematic constraints."""

import threading
from typing import Optional, Tuple

import cv2
import numpy as np

from bombardeer_pkg.control.ballistics import TargetKinematics


class TargetKalmanFilter:
    """
    State vector: [pan_mrad, tilt_mrad, v_pan_mrad_s, v_tilt_mrad_s].

    Design invariants:
      - All state access, innovation gating, and velocity/position clamping occur
        atomically inside this class under self._lock.
      - update_measurement() advances the filter strictly using the frame's true
        monotonic capture timestamp and anisotropic deer-kinematic Q(dt)/R.
      - predict_state() is a pure, read-only kinematic extrapolation at 50 Hz so
        high-frequency servo polling never mutates statePost or inflates errorCovPost.
    """

    def __init__(
        self,
        accel_noise_mrad_s2: float = 150.0,
        measurement_noise: float = 35.0,
        process_noise: Optional[float] = None,
        default_max_v_pan_mrad_s: float = 850.0,
        default_max_v_tilt_mrad_s: float = 450.0,
    ):
        self._lock = threading.Lock()
        self._default_accel_pan_var = float(accel_noise_mrad_s2) ** 2
        self._default_accel_tilt_var = float(accel_noise_mrad_s2) ** 2
        self._default_meas_var = float(measurement_noise)
        self._default_max_v_pan = float(default_max_v_pan_mrad_s)
        self._default_max_v_tilt = float(default_max_v_tilt_mrad_s)

        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.transitionMatrix = np.eye(4, dtype=np.float32)

        self.kf.measurementMatrix = np.zeros((2, 4), dtype=np.float32)
        self.kf.measurementMatrix[0, 0] = 1.0
        self.kf.measurementMatrix[1, 1] = 1.0

        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * self._default_meas_var
        self._init_cov = np.diag([50.0, 50.0, 1500.0, 400.0]).astype(np.float32)
        self.kf.errorCovPost = self._init_cov.copy()

        self.last_meas_mono: Optional[float] = None
        self.is_initialized = False

    @staticmethod
    def _build_process_noise(
        dt: float, accel_pan_var: float, accel_tilt_var: float
    ) -> np.ndarray:
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2

        q = np.zeros((4, 4), dtype=np.float32)
        q[0, 0] = 0.25 * dt4 * accel_pan_var
        q[0, 2] = q[2, 0] = 0.50 * dt3 * accel_pan_var
        q[2, 2] = dt2 * accel_pan_var

        q[1, 1] = 0.25 * dt4 * accel_tilt_var
        q[1, 3] = q[3, 1] = 0.50 * dt3 * accel_tilt_var
        q[3, 3] = dt2 * accel_tilt_var
        return q

    def reset(self):
        with self._lock:
            self.is_initialized = False
            self.last_meas_mono = None

    def update_measurement(
        self,
        measured_pan: float,
        measured_tilt: float,
        capture_mono: float,
        kinematics: Optional[TargetKinematics] = None,
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
                return (
                    int(round(float(sp[0, 0]))),
                    int(round(float(sp[1, 0]))),
                    float(sp[2, 0]),
                    float(sp[3, 0]),
                )

            dt = max(0.001, min(1.8, raw_dt))
            self.last_meas_mono = capture_mono

            if kinematics is not None:
                accel_pan_var = float(kinematics.accel_pan_mrad_s2) ** 2
                accel_tilt_var = float(kinematics.accel_tilt_mrad_s2) ** 2
                meas_var_pan = float(kinematics.meas_var_pan)
                meas_var_tilt = float(kinematics.meas_var_tilt)
                max_v_pan = float(kinematics.max_v_pan_mrad_s)
                max_v_tilt = float(kinematics.max_v_tilt_mrad_s)
            else:
                accel_pan_var = self._default_accel_pan_var
                accel_tilt_var = self._default_accel_tilt_var
                meas_var_pan = self._default_meas_var
                meas_var_tilt = self._default_meas_var
                max_v_pan = self._default_max_v_pan
                max_v_tilt = self._default_max_v_tilt

            self.kf.transitionMatrix[0, 2] = dt
            self.kf.transitionMatrix[1, 3] = dt
            self.kf.processNoiseCov = self._build_process_noise(dt, accel_pan_var, accel_tilt_var)
            self.kf.measurementNoiseCov[0, 0] = meas_var_pan
            self.kf.measurementNoiseCov[1, 1] = meas_var_tilt

            predicted = self.kf.predict()

            meas_p = float(measured_pan)
            meas_t = float(measured_tilt)
            if kinematics is not None:
                # Physical-step innovation gate: clamp jumps exceeding a running deer + optical jitter
                max_step_pan = (max_v_pan * dt) + float(kinematics.max_step_margin_pan_mrad)
                max_step_tilt = (max_v_tilt * dt) + float(kinematics.max_step_margin_tilt_mrad)
                pred_pan = float(predicted[0, 0])
                pred_tilt = float(predicted[1, 0])
                meas_p = pred_pan + float(np.clip(meas_p - pred_pan, -max_step_pan, max_step_pan))
                meas_t = pred_tilt + float(np.clip(meas_t - pred_tilt, -max_step_tilt, max_step_tilt))

            measurement = np.array([[meas_p], [meas_t]], dtype=np.float32)
            estimated = self.kf.correct(measurement)

            # Enforce physical angular velocity ceiling atomically on statePost
            v_pan = float(np.clip(float(estimated[2, 0]), -max_v_pan, max_v_pan))
            v_tilt = float(np.clip(float(estimated[3, 0]), -max_v_tilt, max_v_tilt))
            self.kf.statePost[2, 0] = v_pan
            self.kf.statePost[3, 0] = v_tilt

            return (
                int(round(float(self.kf.statePost[0, 0]))),
                int(round(float(self.kf.statePost[1, 0]))),
                v_pan,
                v_tilt,
            )

    def predict_state(self, query_mono: float) -> Optional[Tuple[int, int, float, float]]:
        """Read-only extrapolation to query_mono without mutating filter covariance."""
        with self._lock:
            if not self.is_initialized or self.last_meas_mono is None:
                return None

            dt = max(0.0, min(2.5, query_mono - self.last_meas_mono))
            pan = float(self.kf.statePost[0, 0])
            tilt = float(self.kf.statePost[1, 0])
            v_pan = float(self.kf.statePost[2, 0])
            v_tilt = float(self.kf.statePost[3, 0])

            pred_pan = pan + (v_pan * dt)
            pred_tilt = tilt + (v_tilt * dt)
            return int(round(pred_pan)), int(round(pred_tilt)), v_pan, v_tilt

    def get_last_state(self) -> Optional[Tuple[int, int, float]]:
        """Read-only un-extrapolated last world position and capture timestamp."""
        with self._lock:
            if not self.is_initialized or self.last_meas_mono is None:
                return None
            return (
                int(round(float(self.kf.statePost[0, 0]))),
                int(round(float(self.kf.statePost[1, 0]))),
                float(self.last_meas_mono),
            )

    def clamp_position(
        self, pan_clamped: bool, world_pan: float, tilt_clamped: bool, world_tilt: float
    ):
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
