# Raspberry Pi 5 Software Architecture & Control Design

This document specifies the internal architecture of the Raspberry Pi 5 runtime (`bombardeer.py` and `bombardeer_pkg/`), covering process/thread isolation, concurrency locks, sensor timestamp alignment, deer-kinematics Kalman filtering, zero-config site self-calibration, parallax and drag ballistics, the 50 Hz latency-aware motion controller, and the web console API.

For the neural perception pipeline (PiSP `12 MP`/`3 MP` switching, strict single-in-flight Hailo IPC, horizon-anchored tile packing, and MegaDetector v6 safety vetoes), see [`WILDLIFE_PERCEPTION.md`](WILDLIFE_PERCEPTION.md). For the wire contract with the ESP32, see [`SERIAL_PROTOCOL.md`](SERIAL_PROTOCOL.md).

---

## 1. Package Structure (`bombardeer_pkg/`)

```text
bombardeer/
├── bombardeer.py                      # Gunicorn WSGI entrypoint & standalone CLI runner
├── config.yaml                        # Runtime defaults (camera, optics, ballistics, control, wildlife, storage)
├── pytest.ini                         # Pytest configuration (pythonpath = ., testpaths = tests)
├── site_calibration.json              # Persisted site calibration (far ground edge, roll, height, slope)
└── bombardeer_pkg/
    ├── config.py                      # Dataclass schema & YAML/environment loader
    ├── state.py                       # SystemState (4 lock planes, WILDLIFE default) & TurretCommandBus
    ├── utils.py                       # Linux sched_setaffinity, sd_notify, audit snapshot writer
    ├── control/
    │   ├── ballistics.py              # TargetKinematics, parallax, roll-aware horizon, 6'0" deer ceiling & drag ToF
    │   ├── kalman.py                  # Encapsulated 4-state TargetKalmanFilter (anisotropic Q/R & velocity clamp)
    │   ├── site_calibration.py        # SiteCalibrationManager (boot sweep, 5'9" walk-test, passive stadia fit)
    │   └── motion_servo.py            # 50 Hz dual-mode servo, latency-aware coasting/fire gate & calibration yield
    ├── hardware/
    │   ├── recorder.py                # Async 1280x720 event MP4 recorder & disk quota manager
    │   ├── serial_link.py             # Non-reset 921.6k baud ESP32 UART worker
    │   └── vision_pipeline.py         # VisionRuntime, VisionMain (Cores 0–2) & HudEncoder (Core 1)
    ├── perception/
    │   ├── base.py                    # BaseTargetDetector abstract base class
    │   ├── aruco.py                   # 1280x720 ArUco calibration detector (CPU)
    │   ├── tiling.py                  # Horizon-anchored multi-tile & TRK-1:1 crop planner
    │   ├── mdv6_decoder.py            # Vectorized YOLOv9 decoder & safety veto filter
    │   ├── hailo_worker.py            # Spawned HailoNPUProcess worker loop
    │   └── hailo_detector.py          # HailoWildlifeDetector IPC, single-in-flight gating & supervisor
    └── web/
        ├── app.py                     # Flask Basic-Auth REST API (/api/mode, /api/health, /api/site_calibration, /video_feed)
        └── templates/
            └── index.html             # Compact single-screen HUD console, Walk Cal UI & Verify Grid SVG overlay
```

---

## 2. Multi-Core & Multi-Process Topology

Gunicorn runs with **`--workers 1 --threads 16`** so exactly one OS process owns the camera, serial port, and Hailo-8 device while 16 worker threads serve concurrent HTTP API requests and MJPEG viewers without blocking.

Inside that worker process, `VisionRuntime.start()` spawns the `HailoNPUProcess` subprocess **before** starting any Python threads (preventing multithreaded `spawn`/`fork` deadlocks) and pins threads across the Raspberry Pi 5's four Cortex-A76 cores via `os.sched_setaffinity`. Core 3 is kept strictly isolated for deterministic `50 Hz` motion control and serial I/O, while `VisionMain` spans Cores `0–2` so OpenCV's internal thread pool can parallelize `ARUCO` mode without starving `MotionServo`:

```text
+-----------------------------------------------------------------------------------+
|                        Gunicorn Worker Process (PID 1)                            |
|                                                                                   |
|  [Cores 0–2] VisionMain Thread (14.4–30 Hz)                                       |
|              • Picamera2 capture_request() + post-capture (pan_cap, tilt_cap)     |
|              • MappedArray minimal-ROI copy -> reusable roi_scratch               |
|              • npu_lores_history (maxlen=12) for timestamp-matched audit frames   |
|              • Daylight boot sweep & 5'9" walk-test orchestration                 |
|              • 1 Hz fenced ISP Day/Dusk & HDR/Monochrome hysteresis evaluator     |
|              • World-space tentative confirmation, nearest-neighbor locking,      |
|                projected VETO_ACTIVE safety check & calibrated Kalman updates     |
|              • Pre-roll ring buffer & systemd WATCHDOG=1 heartbeat (every 2.0 s)  |
|                                                                                   |
|  [Core 1]    HudEncoder Thread (up to 15 Hz, active only when web clients open)   |
|              • Projects locked target box into live (pan, tilt) pose              |
|              • Draws roll-compensated metric horizon & 6'0" far deer head line,   |
|                reticle, boxes, high-contrast status pill & calibration banners    |
|              • Encodes 1280x720 JPEG (quality=75) -> SystemState.set_latest_jpeg()|
|                                                                                   |
|  [Core 2]    HailoNPUProcess (Separate OS Process via multiprocessing.spawn)      |
|              • Owns Hailo-8 PCIe VDevice & MDV6-yolov9-c-1280.hef                 |
|              • Reads 12 MP / 3 MP ROI from POSIX SharedMemory via Pipe IPC        |
|                                                                                   |
|  [Core 2]    VidRecorder Thread                                                   |
|              • Writes 1280x720 event clips (2.0 s pre-roll + 3.0 s post-roll)     |
|              • Enforces min_free_space_gb (5.0 GB) & max_recordings_storage_gb    |
|                                                                                   |
|  [Core 3]    MotionServo Thread (Deterministic 50 Hz / 20 ms tick)                |
|              • Yields & disarms trigger whenever SiteCalibrationManager owns bus  |
|              • Forward-projects Kalman state to actuation timestamp               |
|              • Computes drag ballistics, parallax & latency-scaled P/V commands   |
|              • Enforces tilt/pan clamps, range ceiling & latency-aware fire gate  |
|                                                                                   |
|  [Core 3]    TurretSerial Thread (921,600 Baud UART)                              |
|              • Drains TurretCommandBus (HALT / TRIGGER priority over motion)      |
|              • Parses ESP32 'S' telemetry into 300-sample circular history buffer |
+-----------------------------------------------------------------------------------+
```

---

## 3. Segregated Lock Architecture (`SystemState`)

To eliminate priority inversion between 50 Hz motor control, 30 Hz camera capture, and HTTP streaming, shared state in `bombardeer_pkg/state.py` is partitioned across **four independent locks** plus owned sub-controllers (`command_bus`, `target_filter`, `site_calibration`):

1. **`_mode_lock`:** Guards `turret_mode` (`OFF`, `ON`, `CALIBRATE`, `ARMED`), `calibrate_primed`, and `_active_detector_type` (defaults to **`"WILDLIFE"`** on startup to avoid CPU-bound ArUco overhead).
2. **`_telemetry_lock`:** Guards instantaneous turret state (`pan_mrad`, `tilt_mrad`, `is_moving`, `is_firing`) and the **300-sample** `_telemetry_history` deque (`3–6 s` at `50–100 Hz`, `15 s` at `20 Hz`) used for retroactive capture-time pose interpolation across `110–350 ms` NPU cycles.
3. **`_target_lock`:** Guards `_primary_target` (bounding box, center, fused range, `capture_mono`, `last_seen_mono`), health metrics (`serial_connected`, `sensor_mode`, `sensor_fps`, `npu_fps`, `npu_latency_ms`, `npu_scan_mode`, `recorder_active`, `free_disk_gb`), `get_npu_latency_sec()`, `site_calibration` status snapshot, and active HUD alerts.
4. **`_frame_lock`:** Guards `_latest_jpeg` bytes and `_active_web_clients` so `/video_feed` streaming threads never contend with `MotionServo` or `VisionMain`.

---

## 4. DMA-Safe Camera Capture, Epoch Fencing & Timestamp Alignment

### 4.1 Post-Capture Pose Lookup & Reusable Scratch Buffer
In `VisionMain` (`bombardeer_pkg/hardware/vision_pipeline.py`), `capture_request()` can block up to `~69 ms` in `DAY_12MP`. To prevent pose lag on `prepare_plan()` and `TRK-1:1` crops:
1. `mono_now = time.monotonic()` is stamped immediately after `capture_request()` returns.
2. `_estimate_capture_mono()` computes the true mid-exposure instant using `time.CLOCK_BOOTTIME` and `metadata["SensorTimestamp"]`:
   $$t_{\text{capture}} = t_{\text{arrival}} - \left(t_{\text{boot\_now}} - \frac{\text{SensorTimestamp}}{10^9}\right) - 0.5 \cdot t_{\text{exposure}}$$
3. `(pan_cap, tilt_cap) = state.get_turret_pos_at_time(capture_mono)` is queried *before* projecting `active_track_box` and calling `wildlife_detector.prepare_plan(..., tilt_cap, active_track_box, pan_mrad=pan_cap)`.
4. Inside `MappedArray`, the minimal native ROI is copied via `np.copyto()` into a reusable `roi_scratch` NumPy buffer, `roi_view` and `m` are cleared, and `req_frame.release()` executes immediately in `finally`—preventing `BufferError: cannot close exported pointers exist`.
5. Each frame submitted to the NPU appends `(capture_mono, frame_lores.copy())` to `npu_lores_history` (`deque(maxlen=12)`) so `ACQUIRE`, `VETO`, and `VETO_ACTIVE` audit snapshots draw boxes onto the exact frame that produced the detections (`<= 80 ms` match tolerance).

### 4.2 Transition Fencing (`DAY_12MP <-> DUSK_3MP`, Detector Switches & Serial Reconnect)
Before reconfiguring Picamera2 or switching detectors, `VisionMain` ensures `wildlife_detector.is_idle()` and `not site_calibration.is_motion_owned()`, clears `roi_scratch` and `npu_lores_history`, calls `wildlife_detector.discard_older_than(min_valid_capture_mono)`, and runs `gc.collect()` after `picam2.stop()`. Any late NPU batch with `eff_capture_mono < min_valid_capture_mono` is dropped cleanly.

---

## 5. Kinematics, Encapsulated Kalman Filter & Drag Ballistics

### 5.1 Camera-to-Bore Parallax, Inverse Projection & Deer Head Envelope (`ballistics.py`)
All detections map into canonical `1280×720` coordinates ($c_x = 640$, $c_y = 360$, $f = 540.4\text{ px}$, scale $= 1.850\text{ mrad/px}$). Because the camera sits `170 mm` left (`camera_offset_x_mm = -170.0`) and `60 mm` below (`camera_offset_y_mm = -60.0`) the bore centerline, `image_px_to_world_mrad()` applies range-dependent parallax convergence at fused distance $D$ (mm):

$$\Delta\theta_{\text{pan,parallax}} = \frac{-\text{OffsetX}}{D} \times 1000 \quad [\text{mrad}], \qquad \Delta\theta_{\text{tilt,parallax}} = \frac{\text{OffsetY}}{D} \times 1000 \quad [\text{mrad}]$$

* `project_world_to_image_px()` provides the exact inverse projection—used to center `TRK-1:1` crops, align `VETO_ACTIVE` overlap boxes at `eff_capture_mono`, project `tentative_target` across `350 ms` scans, and keep the live HUD target box locked on the deer during turret slews.
* `compute_deer_head_ceiling_mrad()` computes the perspective-accurate elevation of a `6'0"` (`1.8288 m`) standing deer's head across both near (`min_valid_range_m`) and far (`far_ground_mrad`, fallback `max_engage_range_m`) yard edges via $\theta_{\text{head}}(Z) = s + \arctan((H_{\text{cam}} - H_{\text{deer}}) / Z)$, returning `(ceiling_mrad, far_head_mrad, near_head_mrad)`.

### 5.2 Deer-Kinematics Bounds (`TargetKinematics`)
`compute_target_kinematics(range_m, optics)` (called with site-calibrated `metric_optics` from `SiteCalibrationManager`) derives physically grounded angular bounds from whitetail deer speeds ($v_{\max} = 10\text{ m/s}$, $a_{\max} = 3.0\text{ m/s}^2$, vertical bounding allowance $v_z = 1.0\text{ m/s}$, $a_z = 0.6\text{ m/s}^2$), calibrated mount height $H$, range $R$, and lens `mrad_per_pixel`:

$$\omega_{\text{pan,max}}(R) = \min\!\left(850,\ \frac{1000 \cdot v_{\max}}{R}\right), \qquad \omega_{\text{tilt,max}}(R) = \min\!\left(450,\ \frac{1000 \cdot H \cdot v_{\max}}{R^2} + \frac{1000 \cdot v_z}{R}\right)$$

Measurement variances reflect real night-IR bounding-box jitter ($\sigma_{\text{pan}} = 3.0\text{ px} \cdot \text{mrad/px}$, $\sigma_{\text{tilt}} = 4.0\text{ px} \cdot \text{mrad/px}$, $R_{\text{meas}} = \sigma^2$), cutting vertical velocity gain `K_vel` by 4×–9× on close-range stationary deer compared to an isotropic `R = 10` filter.

### 5.3 Encapsulated 4-State Kalman Filter (`kalman.py`)
`TargetKalmanFilter` tracks `[pan_mrad, tilt_mrad, v_pan_mrad_s, v_tilt_mrad_s]` in world coordinates with all state access and clamping atomic under `self._lock`:
* **Anisotropic `Q(dt)` & `R`:** `update_measurement(..., kinematics=kin)` applies separate pan/tilt acceleration process noise and measurement variances strictly using `capture_mono` (`dt` clamped to `[0.001, 1.8] s`, ignoring out-of-order frames).
* **Physical-Step Innovation Gate:** Measurement jumps exceeding $\omega_{\max}(R)\cdot dt + 3\sigma_\theta$ are clamped before `kf.correct()`, rejecting non-physical box glitches.
* **Atomic Velocity & Position Clamping:** `statePost[2, 0]` and `statePost[3, 0]` are clamped to `±max_v_pan` and `±max_v_tilt` inside `update_measurement()`, while `clamp_position()` zeroes boundary velocity when software travel limits are hit.
* **Non-Mutating Read-Only Queries:** `predict_state(query_mono)` extrapolates up to `2.5 s` without mutating `statePost` or `errorCovPost`, and `get_last_state()` returns the un-extrapolated position and capture timestamp for dual-reference target association.

### 5.4 Nearest-Neighbor Target Locking & World-Space Confirmation
* **Locked Target Association:** Inside `if is_new_batch:`, `VisionMain` projects both the extrapolated Kalman state (`ref_xy`) and the un-extrapolated last position (`last_xy`) into `eff_capture_mono` pixel space and picks the closest detection within a kinematic pixel gate (`min(280 px, kin_gate_px)`). This prevents target bouncing between multiple deer while still re-acquiring a deer that stops suddenly after a `350–700 ms` scan gap.
* **World-Space Confirmation:** `tentative_target` stores world `(world_pan, world_tilt)` and `capture_mono` so confirmation streaks are unaffected by homing or panning slews across a `350 ms` scan gap.

### 5.5 Aerodynamic Drag Ballistics
`compute_drag_ballistics()` computes projectile Time of Flight ($\text{ToF}$) under exponential air drag ($v_0 = 85.0\text{ m/s}$, $k = 0.0128$), gravity drop, velocity lead, and static mechanical trim (`tilt_calibration_trim_mrad = -8.3`):

$$\text{ToF} = \frac{e^{k \cdot R} - 1}{k \cdot v_0}, \qquad \Delta\theta_{\text{drop}} = \frac{0.5 \cdot g \cdot \cos(\theta_{\text{tilt}}) \cdot \text{ToF}^2}{R} \times 1000 \quad [\text{mrad}]$$

$$\theta_{\text{lead,pan}} = v_{\text{pan}} \cdot \text{ToF}, \qquad \theta_{\text{lead,tilt}} = v_{\text{tilt}} \cdot \text{ToF}$$

---

## 6. Zero-Config Site Self-Calibration & Web Verification (`site_calibration.py`, `web/`)

`SiteCalibrationManager` eliminates per-location edits to `config.yaml` using camera-only software calibration persisted in `site_calibration.json`:

1. **Daylight Boot Sweep & Relocation Detection:** Compares a `16×9` normalized luma fingerprint of the home view on first daylight (`corr >= 0.65` skips sweep). On a new site, runs a 7-stop pan sweep (`-750` to `+750 mrad` at `tilt = 0`) using Lab color/luma ground-appearance segmentation combined with sky/texture cues to lock onto the pan-indexed far ground edge (`yard_top_profile`) and estimates camera roll (`roll_mrad`) from smooth vertical edges. If triggered in the dark, defers cleanly to `PENDING_DAYLIGHT` without holding motion ownership.
2. **Exact Projective `5'9"` (`1.7526 m`) Walk-Test Fit:** Disarms the turret (`OFF`, `request_trigger(0)`, `clear_motion()`, `request_setpoint(cur_pan, 0)` so `_halt_pending` never drops the tilt-leveling command), pre-filters `veto_boxes` to standing `PERSON` detections (`bh/bw >= 1.30`, unclipped, `bh >= 30`) so horizontal barrel-tip artifacts are ignored, and solves `(camera_mount_height_m, ground_slope_mrad)` via 8-iteration Huber IRLS with zero small-angle approximation:
   $$\frac{1}{Z_i} = \frac{\tan(\theta_{\text{feet}, i}) - \tan(\theta_{\text{head}, i})}{H_{\text{op}}}, \qquad \tan(\theta_{\text{feet}, i}) = \tan(s) + H_{\text{cam}} \cdot \frac{1}{Z_i}$$
3. **Passive Stadia-to-Ground Convergence:** On an uncalibrated new site where no Walk-Test was run, `get_ground_plane_params()` returns `use_ground_range = False` (stadia-only ranging with far-ground-edge `ABOVE_HORIZON` active) while `observe_passive_wildlife()` accumulates wildlife `(1 / stadia_range_m, tan(dep_feet))` observations and promotes to `"PASSIVE_STADIA"` once `>= 15` points span `>= 5.0 m` (`rms <= 35 mrad`).
4. **Web Console API & Visual Verification (`web/app.py`, `templates/index.html`):**
   * Authenticated `/api/site_calibration` (`GET`/`POST`) exposes full calibration metrics, `yard_top_profile`, active optics, and `deer_far_head_mrad`, and handles `walk_start` (auto-switching to `WILDLIFE`), `walk_finish`, `sweep` (rejected if `ARMED`/`CALIBRATE`), `cancel`, and `reset`. `/api/mode` blocks `ON`/`CALIBRATE`/`ARMED` (`HTTP 409`) while calibration owns motion, and `OFF` cancels active calibration immediately.
   * The compact single-screen console integrates `Perception` toggles into the header, provides one-click Walk Cal controls with live sample/depth progress bars, displays height/slope/roll/fit cards plus an unstretched Pan Treeline Profile SVG, and renders a live roll-compensated **`Verify Grid`** SVG overlay (`FAR DEER HEAD 6'0"`, `METRIC HORIZON`, `FAR GROUND EDGE`, `5m–40m` range lines, and `5'9" @ 10m` / `1.0m deer @ 20m` scale brackets).

---

## 7. 50 Hz Latency-Aware Motion Servo & Firing Gates (`motion_servo.py`)

Every `20 ms` (`motion_loop_interval_sec = 0.020`), `motion_servoing_worker()` executes:

1. **Calibration Motion Yield:** If `state.site_calibration.is_motion_owned()` is `True` (during a boot sweep or Walk-Test), immediately disarms trigger, stops velocity tracking, resets the Kalman filter, and yields the command bus.
2. **Latency-Aware Coasting & Horizon Expiry (Driven by `capture_age`):**
   * `npu_lat_sec = state.get_npu_latency_sec()`.
   * `effective_decay_onset = max(coast_decay_onset_sec, npu_lat_sec + 0.25)` and `effective_coast_horizon = max(coast_max_horizon_sec, npu_lat_sec + 1.0)`.
   * Beyond `effective_decay_onset`, commanded velocities decay exponentially ($e^{-\lambda \cdot (\text{capture\_age} - t_{\text{onset}})}$); beyond `effective_coast_horizon`, the primary target clears and the Kalman filter resets.
3. **Dual-Mode Hysteresis (`P` vs. `V`) & Asymmetric Latency-Scaled Gains:**
   * **Mode 1 — Ballistic Setpoint (`P <pan> <tilt>`):** Engages when radial error exceeds `ballistic_acquire_mrad` (`120 mrad`), or aborts continuous pursuit if error exceeds `ballistic_abort_mrad` (`260 mrad`).
   * **Mode 2 — Continuous Velocity Pursuit (`V <pan_s> <tilt_s>`):** Throttled to `<= 25 Hz` (`setpoint_min_interval_sec = 0.040`) with `latency_scale = max(0.65, min(1.0, 1.0 - 1.5 * max(0.0, npu_lat_sec - 0.160)))` to preserve phase margin during `350 ms` `SCAN-12MP` cycles:
     * **Pan:** `kp = 1.30` (`<= 25 mrad`) / `2.00` (`> 25 mrad`), `kv = 0.65` / `0.85`, clamped to **`±850 mrad/s`**.
     * **Tilt:** `kp = 1.05` (`<= 25 mrad`) / `1.45` (`> 25 mrad`), `kv = 0.40` / `0.55`, clamped to **`±300 mrad/s`**.
4. **Latency-Aware Slew-Gated Firing Interlock (`T 1`):** Trigger engagement requires **all** of:
   * Mode is `ARMED` (`cfg.web.armed_allowed == True`) or `CALIBRATE` (`primed == True`).
   * Serial link connected and target range within `[min_valid_range_m, max_engage_range_m]`.
   * `capture_age <= fire_data_freshness_sec + max(0.160, npu_lat_sec + 0.080)` **and** arrival gap `time_since_seen <= max(fire_data_freshness_sec, npu_lat_sec + 0.120)`.
   * Radial aim error `<= fire_deadband_mrad` (or `calibrate_deadband_mrad` in `CALIBRATE`).
   * Total barrel slew speed `< max_fire_slew_speed_mrad_s`.
5. **Idle Auto-Home:** After `home_timeout_sec` with no active target, commands `P 0 0` (re-issuing every `2.0 s` if stopped off-home).
