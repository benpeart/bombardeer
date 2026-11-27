"""Thread-safe world-space 4-state Kalman Filter with damped coasting and kinematic constraints."""

import math
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
      - Both update_measurement() and predict_state() share the exact same
        hold-then-exponential-decay velocity transition model so missed frames
        smoothly decelerate to a bounded wait position rather than linearly
        coasting into mechanical limits or slingshotting on re-acquisition.
      - predict_state() is pure and read-only at 50 Hz (never mutates statePost
        or inflates errorCovPost).
    """

    DEFAULT_PAN_COAST_HOLD_SEC = 0.35
    DEFAULT_PAN_COAST_DECAY_RATE = 3.5
    DEFAULT_TILT_COAST_HOLD_SEC = 0.12
    DEFAULT_TILT_COAST_DECAY_RATE = 6.5
    MAX_INNOVATION_GATE_DT_SEC = 0.65

    def __init__(
        self,
        accel_noise_mrad_s2: float = 150.0,
        measurement_noise: float = 35.0,
        default_max_v_pan_mrad_s: float = 850.0,
        default_max_v_tilt_mrad_s: float = 180.0,
        pan_coast_hold_sec: float = DEFAULT_PAN_COAST_HOLD_SEC,
        pan_coast_decay_rate: float = DEFAULT_PAN_COAST_DECAY_RATE,
        tilt_coast_hold_sec: float = DEFAULT_TILT_COAST_HOLD_SEC,
        tilt_coast_decay_rate: float = DEFAULT_TILT_COAST_DECAY_RATE,
    ):
        self._lock = threading.Lock()
        self._default_accel_pan_var = float(accel_noise_mrad_s2) ** 2
        self._default_accel_tilt_var = float(accel_noise_mrad_s2) ** 2
        self._default_meas_var = float(measurement_noise)
        self._default_max_v_pan = float(default_max_v_pan_mrad_s)
        self._default_max_v_tilt = float(default_max_v_tilt_mrad_s)
        self._pan_hold = max(0.0, float(pan_coast_hold_sec))
        self._pan_decay = max(0.0, float(pan_coast_decay_rate))
        self._tilt_hold = max(0.0, float(tilt_coast_hold_sec))
        self._tilt_decay = max(0.0, float(tilt_coast_decay_rate))

        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.transitionMatrix = np.eye(4, dtype=np.float32)

        self.kf.measurementMatrix = np.zeros((2, 4), dtype=np.float32)
        self.kf.measurementMatrix[0, 0] = 1.0
        self.kf.measurementMatrix[1, 1] = 1.0

        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * self._default_meas_var
        self._init_cov = np.diag([50.0, 50.0, 1500.0, 250.0]).astype(np.float32)
        self.kf.errorCovPost = self._init_cov.copy()

        self.last_meas_mono: Optional[float] = None
        self.is_initialized = False

    @staticmethod
    def _damped_coeffs(dt: float, hold_sec: float, decay_rate: float) -> Tuple[float, float]:
        """
        Analytical hold-then-exponential-decay velocity integration:
          - For dt <= hold_sec: constant velocity (pos_coeff = dt, vel_coeff = 1.0)
          - For dt >  hold_sec: v(t) = v0 * exp(-decay_rate * (t - hold_sec))
                                pos_coeff = hold_sec + (1 - exp(-decay_rate * u)) / decay_rate
        """
        dt = max(0.0, float(dt))
        if dt <= hold_sec or decay_rate <= 1e-6:
            return dt, 1.0
        u = dt - hold_sec
        vel_coeff = math.exp(-decay_rate * u)
        pos_coeff = hold_sec + ((1.0 - vel_coeff) / decay_rate)
        return pos_coeff, vel_coeff

    @staticmethod
    def _build_process_noise(
        dt: float,
        accel_pan_var: float,
        accel_tilt_var: float,
        pan_vel_c: float = 1.0,
        tilt_vel_c: float = 1.0,
    ) -> np.ndarray:
        """
        Builds positive semi-definite Q(dt) scaled by the velocity decay factor so
        long-dropout re-acquisitions update position cleanly without injecting a
        spurious position-to-velocity slingshot through Q[0, 2] / Q[1, 3].
        """
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2

        q = np.zeros((4, 4), dtype=np.float32)
        q[0, 0] = 0.25 * dt4 * accel_pan_var
        q[0, 2] = q[2, 0] = 0.50 * dt3 * accel_pan_var * pan_vel_c
        q[2, 2] = dt2 * accel_pan_var * max(0.05, pan_vel_c)

        q[1, 1] = 0.25 * dt4 * accel_tilt_var
        q[1, 3] = q[3, 1] = 0.50 * dt3 * accel_tilt_var * tilt_vel_c
        q[3, 3] = dt2 * accel_tilt_var * max(0.05, tilt_vel_c)
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
        pan_hold_sec: Optional[float] = None,
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

            eff_pan_hold = self._pan_hold if pan_hold_sec is None else max(0.0, float(pan_hold_sec))
            pan_pos_c, pan_vel_c = self._damped_coeffs(dt, eff_pan_hold, self._pan_decay)
            tilt_pos_c, tilt_vel_c = self._damped_coeffs(dt, self._tilt_hold, self._tilt_decay)

            self.kf.transitionMatrix[0, 2] = pan_pos_c
            self.kf.transitionMatrix[2, 2] = pan_vel_c
            self.kf.transitionMatrix[1, 3] = tilt_pos_c
            self.kf.transitionMatrix[3, 3] = tilt_vel_c
            self.kf.processNoiseCov = self._build_process_noise(dt, accel_pan_var, accel_tilt_var, pan_vel_c, tilt_vel_c)
            self.kf.measurementNoiseCov[0, 0] = meas_var_pan
            self.kf.measurementNoiseCov[1, 1] = meas_var_tilt

            predicted = self.kf.predict()

            meas_p = float(measured_pan)
            meas_t = float(measured_tilt)
            pan_outlier = False
            tilt_outlier = False
            if kinematics is not None:
                # Cap effective gate dt so multi-frame dropouts cannot admit cross-yard deer swaps
                dt_gate = min(dt, self.MAX_INNOVATION_GATE_DT_SEC)
                max_step_pan = (max_v_pan * dt_gate) + float(kinematics.max_step_margin_pan_mrad)
                max_step_tilt = (max_v_tilt * dt_gate) + float(kinematics.max_step_margin_tilt_mrad)
                pred_pan = float(predicted[0, 0])
                pred_tilt = float(predicted[1, 0])
                inn_p = meas_p - pred_pan
                inn_t = meas_t - pred_tilt
                if abs(inn_p) > max_step_pan:
                    pan_outlier = True
                    meas_p = pred_pan + math.copysign(max_step_pan, inn_p)
                if abs(inn_t) > max_step_tilt:
                    tilt_outlier = True
                    meas_t = pred_tilt + math.copysign(max_step_tilt, inn_t)

            measurement = np.array([[meas_p], [meas_t]], dtype=np.float32)
            estimated = self.kf.correct(measurement)

            # Enforce physical angular velocity ceilings and suppress outlier velocity spikes
            v_pan = float(np.clip(float(estimated[2, 0]), -max_v_pan, max_v_pan))
            v_tilt = float(np.clip(float(estimated[3, 0]), -max_v_tilt, max_v_tilt))
            if pan_outlier:
                v_pan *= 0.25
            if tilt_outlier:
                v_tilt = 0.0
            self.kf.statePost[2, 0] = v_pan
            self.kf.statePost[3, 0] = v_tilt

            return (
                int(round(float(self.kf.statePost[0, 0]))),
                int(round(float(self.kf.statePost[1, 0]))),
                v_pan,
                v_tilt,
            )

    def predict_state(
        self, query_mono: float, pan_hold_sec: Optional[float] = None
    ) -> Optional[Tuple[int, int, float, float]]:
        """
        Read-only damped kinematic extrapolation to query_mono without mutating filter
        covariance. Returns (pan_mrad, tilt_mrad, decayed_v_pan, decayed_v_tilt).
        """
        with self._lock:
            if not self.is_initialized or self.last_meas_mono is None:
                return None

            dt = max(0.0, min(2.5, query_mono - self.last_meas_mono))
            pan = float(self.kf.statePost[0, 0])
            tilt = float(self.kf.statePost[1, 0])
            v_pan0 = float(self.kf.statePost[2, 0])
            v_tilt0 = float(self.kf.statePost[3, 0])

            eff_pan_hold = self._pan_hold if pan_hold_sec is None else max(0.0, float(pan_hold_sec))
            pan_pos_c, pan_vel_c = self._damped_coeffs(dt, eff_pan_hold, self._pan_decay)
            tilt_pos_c, tilt_vel_c = self._damped_coeffs(dt, self._tilt_hold, self._tilt_decay)

            pred_pan = pan + (v_pan0 * pan_pos_c)
            pred_tilt = tilt + (v_tilt0 * tilt_pos_c)
            return (
                int(round(pred_pan)),
                int(round(pred_tilt)),
                v_pan0 * pan_vel_c,
                v_tilt0 * tilt_vel_c,
            )

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
