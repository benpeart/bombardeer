# Raspberry Pi 5 Software Architecture & Control Design

This document specifies the internal architecture of the Raspberry Pi 5 runtime (`bombardeer.py` and `bombardeer_pkg/`), covering process/thread isolation, concurrency locks, sensor timestamp alignment, parallax and drag ballistics, Kalman state estimation, and the 50 Hz dual-mode motion controller.

For the neural perception pipeline (PiSP `12 MP`/`3 MP` switching, horizon-anchored tile packing, and MegaDetector v6 safety vetoes), see [`WILDLIFE_PERCEPTION.md`](WILDLIFE_PERCEPTION.md). For the wire contract with the ESP32, see [`SERIAL_PROTOCOL.md`](SERIAL_PROTOCOL.md).

---

## 1. Package Structure (`bombardeer_pkg/`)

```text
bombardeer/
├── bombardeer.py                  # Gunicorn WSGI entrypoint & standalone CLI runner
├── config.yaml                    # Runtime configuration (camera, optics, ballistics, control, wildlife, storage)
└── bombardeer_pkg/
    ├── config.py                  # Dataclass schema & YAML/environment loader
    ├── state.py                   # SystemState (4 lock planes) & CommandBus
    ├── utils.py                   # Linux sched_setaffinity, sd_notify, audit snapshot writer
    ├── control/
    │   ├── ballistics.py          # Parallax, horizon projection, drag ToF & 4-state Kalman filter
    │   └── motion_servo.py        # 50 Hz dual-mode servo, coasting & firing interlock loop
    ├── hardware/
    │   ├── recorder.py            # Async 1280x720 event MP4 recorder & disk quota manager
    │   ├── serial_link.py         # Non-reset 921.6k baud ESP32 UART worker
    │   └── vision_pipeline.py     # VisionRuntime, VisionMain (Core 0) & HudEncoder (Core 1)
    ├── perception/
    │   ├── aruco.py               # 1280x720 ArUco calibration detector
    │   ├── hailo_detector.py      # Forked HailoNPUProc manager & POSIX SharedMemory IPC
    │   └── mdv6_decoder.py        # Horizon-tile planner, YOLOv9 decoder & safety veto filter
    └── web/
        ├── app.py                 # Flask Basic-Auth REST API & deduplicated MJPEG stream
        └── templates/
            └── index.html         # Responsive 1280x720 HUD console & safety arming cover
```

---

## 2. Multi-Core & Multi-Process Topology

Gunicorn runs with **`--workers 1 --threads 16`** so exactly one OS process owns the camera, serial port, and Hailo-8 device while 16 worker threads serve concurrent HTTP API requests and MJPEG viewers without blocking.

Inside that worker process, `VisionRuntime.start()` spawns the `HailoNPUProc` subprocess **before** starting any Python threads (preventing `fork()`/`spawn` deadlocks with active locks) and pins threads across the Raspberry Pi 5's four Cortex-A76 cores via `os.sched_setaffinity`:

```text
+-----------------------------------------------------------------------------------+
|                        Gunicorn Worker Process (PID 1)                            |
|                                                                                   |
|  [Core 0] VisionMain Thread (14.4–30 Hz)                                          |
|           • Picamera2 capture_request() + MappedArray -> reusable roi_scratch     |
|           • 1 Hz ISP Day/Dusk & HDR/Monochrome hysteresis evaluator               |
|           • Detector dispatch (ArucoTargetDetector or HailoWildlifeDetector)      |
|           • 4-frame spatial target confirmation & Kalman measurement update       |
|           • Pre-roll ring buffer & systemd WATCHDOG=1 heartbeat (every 2.0 s)     |
|                                                                                   |
|  [Core 1] HudEncoder Thread (up to 15 Hz, active only when web clients connected) |
|           • Draws horizon, parallax reticle, veto/target boxes & status bar       |
|           • Encodes 1280x720 JPEG (quality=75) -> SystemState.set_latest_jpeg()   |
|                                                                                   |
|  [Core 2] HailoNPUProc (Separate OS Process via multiprocessing.spawn)            |
|           • Owns Hailo-8 PCIe VDevice & MDV6-yolov9-c-1280.hef                    |
|           • Reads 12 MP / 3 MP ROI from 34.2 MiB POSIX SharedMemory (lock-free)   |
|                                                                                   |
|  [Core 2] VidRecorder Thread                                                      |
|           • Writes 1280x720 event clips (2.0 s pre-roll + 3.0 s post-roll)        |
|           • Enforces min_free_space_gb (5.0 GB) & max_recordings_storage_gb (10 GB) |
|                                                                                   |
|  [Core 3] MotionServo Thread (Deterministic 50 Hz / 20 ms tick)                   |
|           • Forward-projects Kalman state to actuation timestamp                  |
|           • Computes drag ballistics, parallax & dual-mode P/V commands           |
|           • Enforces tilt/pan clamps, range ceiling & slew-gated firing           |
|                                                                                   |
|  [Core 3] TurretSerial Thread (921,600 Baud UART)                                 |
|           • Drains CommandBus (halt/trigger priority over motion commands)        |
|           • Parses ESP32 'S' telemetry into 60-sample circular history buffer     |
+-----------------------------------------------------------------------------------+
```

---

## 3. Segregated Lock Architecture (`SystemState`)

To eliminate priority inversion between 50 Hz motor control, 30 Hz camera capture, and HTTP streaming, shared state in `bombardeer_pkg/state.py` is partitioned across **four independent locks**:

1. **`mode_lock`:** Guards `turret_mode` (`OFF`, `ON`, `CALIBRATE`, `ARMED`), `calibrate_primed`, and `active_detector_type` (`ARUCO`, `WILDLIFE`).
2. **`telemetry_lock`:** Guards instantaneous turret state (`pan_mrad`, `tilt_mrad`, `is_moving`, `is_firing`, `serial_connected`) and the 60-sample `telemetry_history` deque used for retroactive capture-time pose interpolation.
3. **`target_lock`:** Guards `primary_target` (world-projected coordinates, bounding box, fused range, capture timestamp), health metrics (`sensor_mode`, `sensor_fps`, `npu_fps`, `npu_latency_ms`, `recorder_active`, `free_disk_gb`), and active HUD alert banners.
4. **`frame_lock`:** Guards `latest_jpeg` bytes and `web_client_count` so `/video_feed` streaming threads never contend with `MotionServo` or `VisionMain`.

---

## 4. DMA-Safe Camera Capture & Zero-Churn Memory Management

In `VisionMain` (`bombardeer_pkg/hardware/vision_pipeline.py`), `Picamera2` exports Direct Memory Access (DMA) buffers for the native `main` stream (`4608×2592` or `2304×1296`) and `1280×720` `lores` stream:

* **Reusable ROI Scratch Buffer (`roi_scratch`):** Full-width slices of a C-contiguous `MappedArray` return a view into Picamera2's underlying DMA `mmap`. Holding a live view across `picam2.stop()` / `picam2.configure()` triggers `BufferError: cannot close exported pointers exist`. Instead, `VisionMain` copies the minimal horizon or tracking ROI into a pre-allocated reusable `roi_scratch` NumPy buffer via `np.copyto()` inside the `MappedArray` context, clears `roi_view` and `m`, and calls `req_frame.release()` immediately in a `finally` block.
* **Clean Sensor Reconfiguration:** Before any `DUSK_3MP <-> DAY_12MP` switch, `VisionMain` clears `frame_roi`, `frame_lores`, and `roi_scratch` and runs `gc.collect()` after `picam2.stop()`, ensuring deterministic `~160 ms` sensor transitions with zero memory leaks or DMA pointer errors.

---

## 5. Timestamp Alignment, Parallax & Drag Ballistics (`ballistics.py`)

Asynchronous exposure, ISP transfer, and Hailo-8 inference introduce ~40 ms (`ARUCO`) to ~110–345 ms (`WILDLIFE`) of latency between photon capture and target update.

### 5.1 Mid-Exposure Timestamp & Pose Interpolation
`_estimate_capture_mono()` computes the true mid-exposure capture instant using `time.CLOCK_BOOTTIME` and `metadata["SensorTimestamp"]`:

$$t_{\text{capture}} = t_{\text{arrival}} - \left(t_{\text{boot\_now}} - \frac{\text{SensorTimestamp}}{10^9}\right) - 0.5 \cdot t_{\text{exposure}}$$

When a target batch is confirmed, `state.get_turret_pos_at_time(eff_capture_mono)` linearly interpolates `(pan_mrad, tilt_mrad)` from the 60-sample `telemetry_history` ring buffer at $t_{\text{capture}}$, preventing slewing motion from corrupting world-space target coordinates.

### 5.2 Camera-to-Bore Parallax Convergence
All detections are mapped into canonical `1280×720` pixel coordinates ($c_x = 640$, $c_y = 360$, $f = 540.4\text{ px}$, scale $= 1.850\text{ mrad/px}$). Because the camera sits `170 mm` left (`camera_offset_x_mm = -170.0`) and `60 mm` below (`camera_offset_y_mm = -60.0`) the bore centerline, `image_px_to_world_mrad()` applies range-dependent parallax convergence at fused distance $D$ (in mm):

$$\Delta\theta_{\text{pan,parallax}} = \frac{-\text{OffsetX}}{D} \times 1000 = \frac{170}{D} \times 1000 \quad [\text{mrad}]$$

$$\Delta\theta_{\text{tilt,parallax}} = \frac{\text{OffsetY}}{D} \times 1000 = \frac{-60}{D} \times 1000 \quad [\text{mrad}]$$

### 5.3 4-State World-Space Kalman Filter
`TargetKalmanFilter` tracks `[pan_mrad, tilt_mrad, v_pan_mrad_s, v_tilt_mrad_s]` in world coordinates:
* **Measurement Update:** Corrects state using `(raw_w_pan, raw_w_tilt)` and `dt` since the previous confirmed capture.
* **Prediction (`predict_state(mono_now)`):** Projects target position forward across the full inference/control pipeline delay on every 20 ms `MotionServo` tick, and also projects world coordinates back into current image space (`project_world_to_image_px()`) so `TRK-1:1` NPU crops stay centered on the target while the turret slews.

### 5.4 Aerodynamic Drag Ballistics
`compute_drag_ballistics()` computes projectile Time of Flight ($\text{ToF}$) under exponential air drag ($v_0 = 85.0\text{ m/s}$, $k = 0.0128$), gravity drop, velocity lead, and static mechanical trim (`tilt_calibration_trim_mrad = -8.3`):

$$\text{ToF} = \frac{e^{k \cdot R} - 1}{k \cdot v_0}, \qquad \Delta\theta_{\text{drop}} = \arctan\!\left(\frac{0.5 \cdot g \cdot \cos(\theta_{\text{tilt}}) \cdot \text{ToF}^2}{R}\right) \times 1000 \quad [\text{mrad}]$$

$$\theta_{\text{lead,pan}} = v_{\text{pan}} \cdot \text{ToF}, \qquad \theta_{\text{lead,tilt}} = v_{\text{tilt}} \cdot \text{ToF}$$

---

## 6. 50 Hz Dual-Mode Motion Servo & Firing Gates (`motion_servo.py`)

Every `20 ms` (`motion_loop_interval_sec = 0.020`), `motion_servoing_worker()` executes the following control sequence:

1. **Coasting & Dropout Horizon (`1.8 s`):**
   * For `age <= 0.650 s` (`coast_decay_onset_sec`), the Kalman filter projects target position at full estimated velocity—bridging two missed `TRK-1:1` passes plus a full 3-pass `SCAN-12MP` recovery sweep (`~342 ms`).
   * Between `0.650 s` and `1.800 s` (`coast_max_horizon_sec`), feed-forward velocities decay exponentially ($e^{-2.5 \cdot (age - 0.650)}$) to decelerate smoothly without losing bearing.
   * Beyond `1.800 s`, the target lock clears and the filter resets.
2. **Dual-Mode Hysteresis (`P` vs. `V`):**
   * **Mode 1 — Ballistic Setpoint (`P <pan> <tilt>`):** Engages when radial error exceeds `120 mrad` (`ballistic_acquire_mrad`), or aborts continuous pursuit if error exceeds `260 mrad` (`ballistic_abort_mrad`).
   * **Mode 2 — Continuous Velocity Pursuit (`V <pan_s> <tilt_s>`):** Inside the tracking envelope, commands piecewise proportional + `1.0×` velocity feed-forward rates (`kp = 1.4` for error `<= 25 mrad`, `kp = 2.8` above; `4.0 mrad` deadband; clamped to `±850 mrad/s` pan and `±450 mrad/s` tilt), throttled to `<= 25 Hz` (`setpoint_min_interval_sec = 0.040`).
3. **Software Travel Clamps:** Zeroes pan velocity if approaching `±3,141 mrad` and zeroes tilt velocity outside `[-420, +400] mrad`.
4. **Slew-Gated Firing Interlock (`T 1`):** Trigger engagement requires **all** of:
   * Mode is `ARMED` (with `cfg.web.armed_allowed == True`) or `CALIBRATE` (with `primed == True`).
   * Serial link connected and optical measurement age `<= 0.220 s` (`fire_data_freshness_sec`).
   * Target range `<= 40.0 m` (`max_engage_range_m`).
   * Radial aim error `<= 40.0 mrad` (`fire_deadband_mrad`, or `15.0 mrad` in `CALIBRATE`).
   * Total barrel slew speed `< 180.0 mrad/s` (`max_fire_slew_speed_mrad_s`).
5. **Idle Auto-Home:** After `60.0 s` (`home_timeout_sec`) with no target, commands `P 0 0` (re-issuing every `2.0 s` if stopped off-home).