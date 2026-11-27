# Wildlife Perception Engine Design Specification

This document details the computer vision and neural inference architecture used by **Bombardeer** to detect deer and other target wildlife across a wide outdoor yard at ranges up to `75 m` while eliminating asynchronous latency races, adapting automatically to each site's far ground edge, and enforcing hard human/vehicle safety vetoes.

---

## 1. Design Problem: Why Full-Frame Letterboxing Fails

MegaDetector v6 (`MDV6-yolov9-c-1280.hef`) takes a square **`1280×1280×3` UINT8** input tensor on the Hailo-8 NPU. Downscaling a full `16:9` (`4608×2592` or `1280×720`) camera frame directly into `1280×1280` creates two severe bottlenecks:

1. **43.75% Wasted NPU Compute:** Letterboxing a `16:9` image into `1:1` fills the top and bottom `280 px` bars with gray padding (`114`), burning nearly half of every inference pass on empty pixels.
2. **Sub-Threshold Target Resolution:** A deer standing `40–60 m` away at the treeline occupies only `~40–60 px` of height in a `1280×720` frame—falling near or below YOLOv9's stride-8 feature floor, especially against complex branch textures.

**Solution:** Capture native sensor resolution (`12 MP` in daylight, `3 MP` at dusk/night), anchor crop windows to the **pan-indexed, tilt-compensated far ground edge / horizon**, and pack overlapping native-resolution tiles directly into the `1280×1280` NPU tensor.

---

## 2. Dual-Resolution PiSP Camera Pipeline

`VisionMain` (`bombardeer_pkg/hardware/vision_pipeline.py`) configures the Raspberry Pi 5 Image Signal Processor (PiSP) with two simultaneous hardware streams and the `imx708_wide_noir.json` tuning profile:

* **`main` Stream (Native Sensor Resolution, `BGR888` -> RGB in memory):**
  * **`DAY_12MP` (`4608×2592`, `14.4 FPS`, `buffer_count=4`):** Unbinned full-sensor readout for maximum distant treeline detail.
  * **`DUSK_3MP` (`2304×1296`, `30.0 FPS`, `buffer_count=6`):** `2×2` binned readout for higher pixel SNR and faster frame cadence in low light or ArUco calibration mode.
  * Using `BGR888` on PiSP outputs RGB bytes directly in memory (`main_stream_is_rgb: true`), eliminating a full-frame CPU `BGR2RGB` conversion on every inference pass.
* **`lores` Stream (`1280×720` `RGB888`):** Hardware-downscaled by the ISP for ArUco fiducial tracking, `VideoWriter` event archiving, timestamp-matched audit snapshots (`npu_lores_history`, `maxlen=12`), daylight boot-sweep analysis, and the `1280×720` Web HUD. All detection coordinates from `12 MP` or `3 MP` tiles normalize into this single canonical `1280×720` coordinate space.

### 2.1 Dual Hysteresis Lighting Controllers
Evaluated at `~1 Hz` (blocked whenever a target is locked, confirming, recording, NPU inference is in flight, or site calibration owns turret motion):

1. **ISP Daylight HDR ↔ Low-Light Monochrome:**
   * **Enter B&W (`is_monochrome = True`):** `Lux < 25.0` or `AnalogueGain > 6.0` -> sets `HdrMode=0`, `Saturation=0.0`, `Contrast=1.15`, `Sharpness=0.90`, `ExposureValue=+0.25`.
   * **Restore Color (`is_monochrome = False`):** `Lux >= 40.0` and `AnalogueGain <= 6.5` -> restores `HdrMode=2`, `Saturation=0.85`, `Contrast=1.05`, `Sharpness=1.15`, `ExposureValue=0.0`.
2. **Sensor `DUSK_3MP` ↔ `DAY_12MP`:**
   * Boots in `DUSK_3MP`, runs a `5.0 s` initial daylight check (`sensor_boot_eval_sec`), and thereafter requires **`60.0 s`** (`sensor_switch_dwell_sec`) of sustained light change:
     * **Promote `DUSK_3MP -> DAY_12MP`:** `WILDLIFE` detector active, color HDR mode, `Lux >= 60.0`, and `Gain <= 3.5`.
     * **Revert `DAY_12MP -> DUSK_3MP`:** Monochrome active, `Lux < 35.0`, `Gain > 5.0`, or `ARUCO` detector selected.

---

## 3. Isolated Hailo-8 Subprocess, Single-in-Flight IPC & Fencing

`HailoWildlifeDetector` (`bombardeer_pkg/perception/hailo_detector.py`) isolates all HailoRT PCIe calls in a **spawned** child process (`HailoNPUProcess` in `hailo_worker.py`, pinned to **Core 2**) communicating with `VisionMain` (**Cores 0–2**) over a pre-allocated **`34.2 MiB` POSIX `SharedMemory`** block (`4608 × 2592 × 3` bytes max) and three unidirectional `multiprocessing.Pipe` channels (`_req_tx`, `_ack_rx`, `_res_rx`):

1. **Strict Single-in-Flight Discipline (Zero Queue Staleness):**
   * `is_ready_for_frame()` drains `_ack_rx` and `_res_rx` immediately and returns `True` **only when `(not _shm_busy) and (_in_flight_jobs == 0)`**.
   * Even though the worker acknowledges the SHM copy immediately before running `350 ms` of NPU inference, `VisionMain` will not enqueue a second frame into the pipe behind an active pass—eliminating `~280 ms` of hidden IPC queueing delay while wasting zero camera frames on completion.
2. **Per-Job Capture Metadata:**
   * Each submitted job records `(capture_mono, is_monochrome)` in `_pending_jobs` (`deque(maxlen=8)`), so mid-inference ISP color/B&W transitions never apply the wrong confidence threshold when the batch returns.
   * Before the first real NPU batch completes, `detect()` returns `eff_mono = 0.0` so `VisionMain` never fires phantom `is_new_batch` events.
3. **Epoch Fencing & Fault Supervisor:**
   * `is_idle()` and `discard_older_than(cutoff_mono)` fence sensor switches, detector switches, serial reconnects, and supervisor restarts so pre-switch NPU results can never leak into a new mode.
   * `check_supervisor()` monitors both process exit and `> 5.0 s` stalls across `_shm_busy or _in_flight_jobs > 0` (catching hangs *after* SHM copy ACK), immediately disarming `ARMED`/`CALIBRATE` -> `ON` + `TRIGGER 0` and restarting the worker with exponential backoff (`2 s` base, capped at `30 s`).

---

## 4. Horizon- & Far-Ground-Edge-Anchored Multi-Tile Scanning (`tiling.py`, `mdv6_decoder.py`)

`HailoWildlifeDetector.prepare_plan()` passes `state.site_calibration.get_tiling_optics(cfg.optics, pan_cap)` into `build_pass_plan()` (`bombardeer_pkg/perception/tiling.py`). When a daylight boot sweep has completed, the tile band automatically tracks the pan-indexed far ground edge (`yard_top_profile`) across the entire azimuth sweep; otherwise it falls back to `compute_horizon_y_720(tilt_cap)` + `horizon_y_offset_720`:

| Mode | Native Frame | NPU Passes & `1280×1280` Tile Layout | Effective Zoom | Typical Cadence |
| :--- | :--- | :--- | :--- | :--- |
| **`SCAN-3MP`** | `2304×1296` | **1 pass:** Two `1280×620` native crops (`x=0` and `x=1024`, `256 px` horizontal overlap) stacked vertically with a `40 px` gray (`114`) divider | **1.8×** across full width | **~7.4–9.0 Hz** (`~110 ms`) |
| **`SCAN-12MP`** | `4608×2592` | **3 passes on 1 frozen frame:**<br>• **Pass 0 (Far Left/Center-Left):** `x=0` and `x=1110` (`1280×620` 1:1 native tiles)<br>• **Pass 1 (Far Center-Right/Right):** `x=2220` and `x=3328` (`1280×620` 1:1 native tiles, `~170 px` overlap)<br>• **Pass 2 (Mid/Near Foreground):** Two `2560×1240` tiles (`x=0` and `x=2048`, `512 px` overlap) downscaled 2:1 to `1280×620` | **3.6×** far band / **1.8×** mid-near band | **~2.4–4.0 Hz** (`~342 ms`) |
| **`TRK-1:1` / `TRK-ZOOM`** | `4608×2592` or `2304×1296` | **1 pass:** Single square crop (`1280×1280` up to `min(W, H)`, `~2.8×` target box padding) centered on Kalman-projected target coordinates at exact `capture_mono` pose | **Up to 3.6×** (100% tensor fill) | **~7.4–9.0 Hz** (`~110 ms`) |

### 4.1 Seam-Clipping Suppression & Unified NMS
* **Seam Guard:** When an animal stands across the boundary between two adjacent tiles, `MDV6Decoder1280` drops small animal boxes touching an internal tile seam whenever an overlapping neighbor contains a larger animal detection. **Person and vehicle veto boxes are never seam-dropped.**
* **Unified Merging:** All tiles from a 1-pass or 3-pass sweep are remapped into canonical `1280×720` coordinates and merged via class-aware Non-Maximum Suppression (`iou_thresh: 0.45`).
* **Tracking Crop Fallback:** `TRK-1:1` holds for `max(track_crop_hold_sec, npu_lat_sec + 0.35)` after the last detection; if `2` consecutive `TRK-1:1` passes miss, `force_scan` immediately triggers a full `SCAN-12MP` or `SCAN-3MP` sweep.

---

## 5. Multi-Layer Validation & Safety Veto Funnel

MegaDetector v6 outputs three classes: **`0: animal`**, **`1: person`**, and **`2: vehicle`**. Every candidate box passes through five sequential gates before `VisionMain` can confirm a target:

```text
Raw MDv6 Boxes (Canonical 1280x720, tagged with eff_capture_mono)
        │
        ├──► Fixed Upper-Right Barrel Exclusion Mask (MDV6Decoder1280, pre-NMS)
        │         └──► Any candidate box with >= 55% of its area inside the rigidly mounted
        │              canonical 1280x720 barrel/shroud polygon is suppressed before NMS so
        │              tilting down never causes the barrel to trigger false scene-wide vetoes
        │              (real people walking in the yard below the barrel remain well under 55%
        │              overlap and still veto immediately).
        │
        ├──► Class 1 (Person >= 0.48) or Class 2 (Vehicle >= 0.35)
        │         └──► Added to veto_boxes[] (scene-wide safety veto — no overlap required)
        │              • During active Walk-Test: pre-filtered standing PERSON boxes (bh/bw >= 1.30,
        │                unclipped) feed SiteCalibrationManager
        │              • During scanning/tracking: ANY non-barrel person or vehicle anywhere in the
        │                frame rejects all animal detections in that sweep (SCENE_VETO), clears
        │                tentative confirmation, and immediately aborts any active lock + commands T 0
        │
        └──► Class 0 (Animal)
                  │
                  ├─ 1. Confidence & Dominance Gate
                  │     • score >= conf_thresh (HDR) or >= monochrome_conf_thresh (per-job B&W flag)
                  │     • beats person & vehicle scores by >= class_margin (0.25)
                  │     • person_prob and vehicle_prob < max_non_animal_prob (0.02)
                  │
                  ├─ 2. Geometric & Aspect Ratio Gate
                  │     • width <= 55% frame (max_animal_width_frac)
                  │     • area  <= 40% frame (max_animal_area_frac)
                  │     • height / width <= 1.65 (rejects narrow upright human/post silhouettes)
                  │
                  ├─ 3. Roll-Compensated Far Ground Edge & Ground-Plane Physical Gate
                  │     • Feet (y2) must not sit > horizon_feet_margin_720 above scanned far ground edge
                  │     • When metric height/slope calibrated (WALK_TEST or PASSIVE_STADIA):
                  │         - If net depression >= 25 mrad: height must be <= max_animal_height_m (default 1.85 m, Western VA whitetail buck)
                  │         - Fused range = 70% ground-plane + 30% stadia (clamped 3.0–75.0 m)
                  │     • When uncalibrated on a new site (use_ground_range = False):
                  │         - Uses stadia_range_m directly & feeds passive ground-plane convergence
                  │
                  ├─ 4. Scene-Wide Safety Veto Gate
                  │     • Rejected immediately (SCENE_VETO) if ANY person or vehicle is detected
                  │       anywhere in the frame (also aborts active locks & passive calibration)
                  │
                  └─ 5. World-Space Confirmation & Anisotropic Range-Gated Lock (VisionMain)
                        • Confirmation streak gates in world mrad scaled by dt & calibrated TargetKinematics
                        • Locked tracking picks nearest candidate inside an elliptical (dx/gx, dy/gy) gate
                          (gx <= 220 px, gy in [45, 110] px, dt capped at 0.65 s) + radial range consistency
                          (|ΔR| <= 6.0 * dt_eff + max(1.6, 0.15 * R)) to block multi-deer herd swaps
                        • Saves timestamp-matched ACQUIRE/VETO audit JPEG from npu_lores_history
```
