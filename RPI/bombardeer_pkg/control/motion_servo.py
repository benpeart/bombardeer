"""Deterministic 50 Hz Motion Servoing Loop (Core 3, strictly monotonic timing)."""

import logging
import math
import time

from bombardeer_pkg.config import BombardeerConfig
from bombardeer_pkg.control.ballistics import compute_drag_ballistics
from bombardeer_pkg.state import SystemState
from bombardeer_pkg.utils import set_os_thread_name

log = logging.getLogger("Bombardeer")


def _compute_axis_velocity(
    err_mrad: float,
    tgt_v: float,
    deadband: float,
    kp_small: float = 1.3,
    kp_large: float = 2.0,
    kv_small: float = 0.65,
    kv_large: float = 0.85,
    latency_scale: float = 1.0,
) -> int:
    mag = abs(err_mrad)
    if mag < deadband:
        return 0
    if mag <= 25.0:
        return int(round(err_mrad * (kp_small * latency_scale) + (kv_small * latency_scale) * tgt_v))
    return int(round(err_mrad * (kp_large * latency_scale) + (kv_large * latency_scale) * tgt_v))


def motion_servoing_worker(cfg: BombardeerConfig, state: SystemState):
    ctrl = cfg.control
    optics = cfg.optics
    bal = cfg.ballistics
    bus = state.command_bus
    kf = state.target_filter

    last_setpoint_mono = 0.0
    last_debug_log_mono = 0.0
    is_actively_tracking_velocity = False
    last_v_pan = 0
    last_v_tilt = 0
    last_sent_p_pan = 99999
    last_sent_p_tilt = 99999
    last_mode_seen = "OFF"
    last_fire_gate_ok_mono = -1e9
    last_sent_trigger_state = 0

    last_target_mono = time.monotonic()
    is_homing = False

    log.info("[MOTION] 50 Hz Real-Time Motion Controller (Monotonic Drag Ballistics & Auto-Home) started")
    set_os_thread_name("MotionServo", cpu_cores=[3])

    while not state.shutdown_event.is_set():
        loop_start = time.monotonic()
        now = loop_start

        mode, is_primed = state.get_turret_mode()
        target_data = state.get_primary_target()
        current_pan, current_tilt, hw_moving, _ = state.get_instantaneous_telemetry()
        serial_ok = state.is_serial_connected()

        if state.site_calibration.is_motion_owned():
            if last_sent_trigger_state != 0:
                bus.request_trigger(0, timestamp_mono=now)
                last_sent_trigger_state = 0
            if is_actively_tracking_velocity:
                bus.request_velocity(0, 0)
                is_actively_tracking_velocity = False
            last_v_pan = 0
            last_v_tilt = 0
            last_fire_gate_ok_mono = -1e9
            is_homing = False
            last_target_mono = now
            kf.reset()
            sleep_time = ctrl.motion_loop_interval_sec - (time.monotonic() - loop_start)
            if sleep_time > 0.001:
                state.shutdown_event.wait(timeout=sleep_time)
            continue

        if mode == "OFF":
            if last_mode_seen != "OFF":
                bus.request_halt()
                is_actively_tracking_velocity = False
                last_v_pan = 0
                last_v_tilt = 0
                last_sent_p_pan = 99999
                last_sent_p_tilt = 99999
                last_fire_gate_ok_mono = -1e9
                last_sent_trigger_state = 0
                kf.reset()
                log.info("[MODE] Turret disengaged and placed in OFF / STANDBY.")

            last_target_mono = now
            is_homing = False
            last_mode_seen = mode

            sleep_time = ctrl.motion_loop_interval_sec - (time.monotonic() - loop_start)
            if sleep_time > 0.001:
                state.shutdown_event.wait(timeout=sleep_time)
            continue

        last_mode_seen = mode

        if serial_ok:
            npu_lat_sec = state.get_npu_latency_sec()
            lat_allowance_sec = max(0.160, npu_lat_sec + 0.080)
            effective_coast_horizon = max(ctrl.coast_max_horizon_sec, npu_lat_sec + 1.0)
            effective_decay_onset = max(ctrl.coast_decay_onset_sec, npu_lat_sec + 0.25)

            target_active = False
            capture_age = 1e9
            time_since_seen = 1e9
            if target_data is not None:
                capture_age = max(0.0, now - target_data.get("capture_mono", target_data["last_seen_mono"]))
                time_since_seen = max(0.0, now - target_data["last_seen_mono"])
                if capture_age <= effective_coast_horizon:
                    target_active = True

            if target_active:
                last_target_mono = now
                is_homing = False

                predicted_state = kf.predict_state(now)
                if predicted_state is not None:
                    base_pan, base_tilt, tgt_v_pan, tgt_v_tilt = predicted_state
                    current_range_m = target_data.get("range_m", optics.default_fallback_range_m)

                    tof_sec, drop_mrad, lead_pan_mrad, lead_tilt_mrad = compute_drag_ballistics(
                        current_range_m, current_tilt, tgt_v_pan, tgt_v_tilt, bal, optics
                    )

                    world_pan = int(round(base_pan + lead_pan_mrad))
                    world_tilt = int(round(base_tilt + lead_tilt_mrad + drop_mrad + optics.tilt_calibration_trim_mrad))

                    pan_clamped = (world_pan < ctrl.pan_min_mrad or world_pan > ctrl.pan_max_mrad)
                    tilt_clamped = (world_tilt < ctrl.tilt_min_mrad or world_tilt > ctrl.tilt_max_mrad)
                    world_pan = max(ctrl.pan_min_mrad, min(ctrl.pan_max_mrad, world_pan))
                    world_tilt = max(ctrl.tilt_min_mrad, min(ctrl.tilt_max_mrad, world_tilt))

                    if pan_clamped or tilt_clamped:
                        kf.clamp_position(pan_clamped, world_pan, tilt_clamped, world_tilt)

                    realtime_err_pan = world_pan - current_pan
                    realtime_err_tilt = world_tilt - current_tilt
                    radial_error_mrad = math.hypot(realtime_err_pan, realtime_err_tilt)
                    turret_speed_mag = max(
                        math.hypot(last_v_pan, last_v_tilt),
                        state.get_measured_turret_speed_mrad_s(),
                    )

                    # Latency-aware capture freshness gate (supports both ~110ms TRK-1:1 and ~350ms SCAN-12MP)
                    in_engage_range = (optics.min_valid_range_m <= current_range_m <= optics.max_engage_range_m)
                    fire_deadband = ctrl.calibrate_deadband_mrad if mode == "CALIBRATE" else ctrl.fire_deadband_mrad
                    max_arrival_gap = max(ctrl.fire_data_freshness_sec, npu_lat_sec + 0.120)

                    production_fire_gate = (
                        in_engage_range
                        and capture_age <= (ctrl.fire_data_freshness_sec + lat_allowance_sec)
                        and time_since_seen <= max_arrival_gap
                        and radial_error_mrad <= fire_deadband
                        and turret_speed_mag < ctrl.max_fire_slew_speed_mrad_s
                    )

                    if production_fire_gate:
                        last_fire_gate_ok_mono = now

                    can_fire = False
                    shot_pulse_duration = None

                    if mode == "ARMED" and cfg.web.armed_allowed:
                        can_fire = (now - last_fire_gate_ok_mono <= 0.120)
                    elif mode == "CALIBRATE":
                        if production_fire_gate and is_primed and state.consume_calibration_prime():
                            can_fire = True
                            shot_pulse_duration = 0.080
                            log.info(
                                f"[CALIBRATE] Shot discharged! | Range:{current_range_m:.2f}m | "
                                f"ToF:{tof_sec:.3f}s | Lead:({lead_pan_mrad:.1f},{lead_tilt_mrad:.1f})mrad | "
                                f"Drop:{drop_mrad:.1f}mrad"
                            )

                    desired_trig = 1 if can_fire else 0
                    if desired_trig == 1 or desired_trig != last_sent_trigger_state:
                        bus.request_trigger(desired_trig, pulse_len=shot_pulse_duration, timestamp_mono=now)
                        last_sent_trigger_state = desired_trig

                    if not is_actively_tracking_velocity:
                        if radial_error_mrad > ctrl.ballistic_acquire_mrad:
                            use_ballistic = True
                        else:
                            use_ballistic = False
                            is_actively_tracking_velocity = True
                    else:
                        if radial_error_mrad > ctrl.ballistic_abort_mrad:
                            use_ballistic = True
                            is_actively_tracking_velocity = False
                        else:
                            use_ballistic = False

                    time_since_tx = now - last_setpoint_mono

                    if use_ballistic:
                        goal_delta = math.hypot(world_pan - last_sent_p_pan, world_tilt - last_sent_p_tilt)
                        if (time_since_tx >= 0.400 and goal_delta > 60.0) or last_v_pan != 0 or last_v_tilt != 0:
                            last_setpoint_mono = now
                            last_sent_p_pan = world_pan
                            last_sent_p_tilt = world_tilt
                            bus.request_setpoint(world_pan, world_tilt)
                            last_v_pan = 0
                            last_v_tilt = 0
                        cmd_type = f"P({world_pan:+4d},{world_tilt:+4d})"
                    else:
                        if time_since_tx >= ctrl.setpoint_min_interval_sec:
                            last_setpoint_mono = now

                            # Scale loop gain down smoothly as NPU latency exceeds 160 ms to preserve phase margin
                            latency_scale = max(0.65, min(1.0, 1.0 - 1.5 * max(0.0, npu_lat_sec - 0.160)))

                            raw_v_pan = _compute_axis_velocity(
                                realtime_err_pan,
                                tgt_v_pan,
                                ctrl.velocity_deadband_mrad,
                                kp_small=1.30,
                                kp_large=2.00,
                                kv_small=0.65,
                                kv_large=0.85,
                                latency_scale=latency_scale,
                            )
                            raw_v_tilt = _compute_axis_velocity(
                                realtime_err_tilt,
                                tgt_v_tilt,
                                ctrl.velocity_deadband_mrad,
                                kp_small=1.05,
                                kp_large=1.45,
                                kv_small=0.40,
                                kv_large=0.55,
                                latency_scale=latency_scale,
                            )

                            if capture_age > effective_decay_onset:
                                decay_factor = math.exp(
                                    -ctrl.coast_exp_decay_rate * (capture_age - effective_decay_onset)
                                )
                                raw_v_pan = int(round(raw_v_pan * decay_factor))
                                raw_v_tilt = int(round(raw_v_tilt * decay_factor))

                            v_pan_cmd = max(-850, min(850, raw_v_pan))
                            v_tilt_cmd = max(-300, min(300, raw_v_tilt))

                            if current_tilt >= ctrl.tilt_max_mrad and v_tilt_cmd > 0:
                                v_tilt_cmd = 0
                            elif current_tilt <= ctrl.tilt_min_mrad and v_tilt_cmd < 0:
                                v_tilt_cmd = 0

                            last_v_pan = v_pan_cmd
                            last_v_tilt = v_tilt_cmd
                            bus.request_velocity(v_pan_cmd, v_tilt_cmd)

                        cmd_type = f"V({last_v_pan:+4d},{last_v_tilt:+4d})"

                    if now - last_debug_log_mono >= 0.150:
                        last_debug_log_mono = now
                        log.info(
                            f"[{mode}] Cur:({current_pan:+5d},{current_tilt:+4d}) | "
                            f"Err:({realtime_err_pan:+5.1f},{realtime_err_tilt:+4.1f}) | "
                            f"Cmd:{cmd_type} | Rng:{current_range_m:4.1f}m | Mv:{int(hw_moving)}"
                        )
            else:
                if target_data is not None:
                    state.clear_primary_target()
                    last_fire_gate_ok_mono = -1e9
                    if last_sent_trigger_state != 0:
                        bus.request_trigger(0, timestamp_mono=now)
                        last_sent_trigger_state = 0
                    if is_actively_tracking_velocity:
                        bus.request_velocity(0, 0)
                        is_actively_tracking_velocity = False
                    last_v_pan = 0
                    last_v_tilt = 0
                    last_sent_p_pan = 99999
                    last_sent_p_tilt = 99999
                    kf.reset()
                    log.info(f"[{mode}] Target horizon expired (>{effective_coast_horizon:.1f}s). Standing by.")

                idle_duration = now - last_target_mono
                if idle_duration >= ctrl.home_timeout_sec:
                    off_home = (abs(current_pan) > 2 or abs(current_tilt) > 2)
                    if not is_homing and off_home:
                        log.info(
                            f"[{mode}] Idle for {idle_duration:.1f}s at "
                            f"({current_pan:+d},{current_tilt:+d}). Returning turret to scan home (0,0)."
                        )
                        is_homing = True
                        bus.request_setpoint(0, 0)
                        last_setpoint_mono = now
                    elif is_homing:
                        if not off_home:
                            is_homing = False
                            log.info(f"[{mode}] Returned to scan home (0,0).")
                        elif (not hw_moving) and (now - last_setpoint_mono >= 2.0):
                            log.info(
                                f"[{mode}] Re-issuing home setpoint (0,0) from "
                                f"({current_pan:+d},{current_tilt:+d})."
                            )
                            bus.request_setpoint(0, 0)
                            last_setpoint_mono = now

        sleep_time = ctrl.motion_loop_interval_sec - (time.monotonic() - loop_start)
        if sleep_time > 0.001:
            state.shutdown_event.wait(timeout=sleep_time)
