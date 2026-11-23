# Wildlife Perception Engine Design Specification

This document details the computer vision and neural inference architecture used by **Bombardeer** to detect deer and other target wildlife across a wide outdoor yard at ranges up to `75 m` while rejecting false positives and enforcing hard human/vehicle safety vetoes.

---

## 1. Design Problem: Why Full-Frame Letterboxing Fails

MegaDetector v6 (`MDV6-yolov9-c-1280.hef`) takes a square **`1280×1280×3` UINT8** input tensor on the Hailo-8 NPU. Downscaling a full `16:9` (`4608×2592` or `1280×720`) camera frame directly into `1280×1280` creates two severe bottlenecks:

1. **43.75% Wasted NPU Compute:** Letterboxing a `16:9` image into `1:1` fills the top and bottom `280 px` bars with gray padding (`114`), burning nearly half of every inference pass on empty pixels.
2. **Sub-Threshold Target Resolution:** A deer standing `40–60 m` away at the treeline occupies only `~40–60 px` of height in a `1280×720` frame—falling near or below YOLOv9's stride-8 feature floor, especially against complex branch textures.

**Solution:** Capture native sensor resolution (`12 MP` in daylight, `3 MP` at dusk/night), anchor crop windows to the **tilt-compensated ground horizon**, and pack overlapping native-resolution tiles directly into the `1280×1280` NPU tensor.

---

## 2. Dual-Resolution PiSP Camera Pipeline

`VisionMain` (`bombardeer_pkg/hardware/vision_pipeline.py`) configures the Raspberry Pi 5 Image Signal Processor (PiSP) with two simultaneous hardware streams and the `imx708_wide_noir.json` tuning profile:

* **`main` Stream (Native Sensor Resolution, `BGR888` -> RGB in memory):**
  * **`DAY_12MP` (`4608×2592`, `14.4 FPS`, `buffer_count=4`):** Unbinned full-sensor readout for maximum distant treeline detail.
  * **`DUSK_3MP` (`2304×1296`, `30.0 FPS`, `buffer_count=6`):** `2×2` binned readout for higher pixel SNR and faster frame cadence in low light or ArUco calibration mode.
  * Using `BGR888` on PiSP outputs RGB bytes directly in memory (`main_stream_is_rgb: true`), eliminating a full-frame CPU `BGR2RGB` conversion on every inference pass.
* **`lores` Stream (`1280×720` `RGB888`):** Hardware-downscaled by the ISP for ArUco fiducial tracking, `VideoWriter` event archiving, audit snapshots, and the `1280×720` Web HUD. All detection coordinates from `12 MP` or `3 MP` tiles are normalized back into this single canonical `1280×720` coordinate space.

### 2.1 Dual Hysteresis Lighting Controllers
Evaluated at `~1 Hz` (blocked whenever a target is locked, confirming, or recording):

1. **ISP Daylight HDR ↔ Low-Light Monochrome:**
   * **Enter B&W (`is_monochrome = True`):** `Lux < 25.0` or `AnalogueGain > 6.0` -> sets `HdrMode=0`, `Saturation=0.0`, `Contrast=1.15`, `Sharpness=0.90`, `ExposureValue=+0.25`.
   * **Restore Color (`is_monochrome = False`):** `Lux >= 40.0` and `AnalogueGain <= 6.5` -> restores `HdrMode=2`, `Saturation=0.85`, `Contrast=1.05`, `Sharpness=1.15`, `ExposureValue=0.0`.
2. **Sensor `DUSK_3MP` ↔ `DAY_12MP`:**
   * Boots in `DUSK_3MP`, runs a `5.0 s` initial daylight check (`sensor_boot_eval_sec`), and thereafter requires **`60.0 s`** (`sensor_switch_dwell_sec`) of sustained light change:
     * **Promote `DUSK_3MP -> DAY_12MP`:** `WILDLIFE` detector active, color HDR mode, `Lux >= 60.0`, and `Gain <= 3.5`.
     * **Revert `DAY_12MP -> DUSK_3MP`:** Monochrome active, `Lux < 35.0`, `Gain > 5.0`, or `ARUCO` detector selected.

---

## 3. Isolated Hailo-8 Subprocess & SharedMemory IPC

`HailoWildlifeDetector` (`bombardeer_pkg/perception/hailo_detector.py`) isolates all HailoRT PCIe calls in a forked child process (`HailoNPUProc`, pinned to **Core 2**) communicating with `VisionMain` (**Core 0**) over a pre-allocated **`34.2 MiB` POSIX `SharedMemory`** block (`4608 × 2592 × 3` bytes max) and lock-free `multiprocessing.Value` state flags:

1. `VisionMain` checks `is_ready_for_frame()` (`shm_state == 0`), queries `MDV6Decoder1280.prepare_plan()` for the minimal bounding ROI needed for the next sweep, copies only that ROI out of `MappedArray` into `roi_scratch`, and writes it into `SharedMemory` (`shm_state = 1`).
2. `HailoNPUProc` wakes on `frame_event`, slices/packs the tiles on Core 2, runs 1 or 3 synchronous Hailo-8 `InferVStreams` passes, decodes and merges all detections, stores the result in `SharedMemory`, and sets `shm_state = 2`.
3. `VisionMain` harvests the completed batch (`shm_state == 2 -> 0`) tagged with its original mid-exposure `capture_mono` timestamp.

Because `SharedMemory` is sized for the worst-case `12 MP` frame, switching between `3 MP` and `12 MP` requires zero IPC reallocation or NPU restarts.

---

## 4. Horizon-Anchored Multi-Tile Scanning (`MDV6Decoder1280`)

`MDV6Decoder1280` (`bombardeer_pkg/perception/mdv6_decoder.py`) anchors every scan sweep to the dynamic ground horizon (`compute_horizon_y_720(tilt_mrad)` + `horizon_y_offset_720`), bounded upward by `horizon_headroom_720` (`105 px` in 720p space) and downward by `horizon_far_headroom_720` (`50 px`):

| Mode | Native Frame | NPU Passes & `1280×1280` Tile Layout | Effective Zoom | Typical Cadence |
| :--- | :--- | :--- | :--- | :--- |
| **`SCAN-3MP`** | `2304×1296` | **1 pass:** Two `1280×620` native crops (`x=0` and `x=1024`, `256 px` horizontal overlap) stacked vertically with a `40 px` gray (`114`) divider | **1.8×** across full width | **~9.0 Hz** (`~110 ms`) |
| **`SCAN-12MP`** | `4608×2592` | **3 passes on 1 frozen frame:**<br>• **Pass 0 (Far Left/Center-Left):** `x=0` and `x=1110` (`1280×620` 1:1 native tiles)<br>• **Pass 1 (Far Center-Right/Right):** `x=2220` and `x=3328` (`1280×620` 1:1 native tiles, `~170 px` overlap)<br>• **Pass 2 (Mid/Near Foreground):** Two `2560×1240` tiles (`x=0` and `x=2048`, `512 px` overlap) downscaled 2:1 to `1280×620` | **3.6×** far band / **1.8×** mid-near band | **~2.9–4.0 Hz** (`~342 ms`) |
| **`TRK-1:1` / `TRK-ZOOM`** | `4608×2592` or `2304×1296` | **1 pass:** Single square crop (`1280×1280` up to `min(W, H)`, `~2.8×` target box padding) centered on Kalman-projected target coordinates | **Up to 3.6×** (100% tensor fill) | **~9.0 Hz** (`~110 ms`) |

### 4.1 Seam-Clipping Suppression & Unified NMS
* **Seam Guard:** When an animal stands across the boundary between two adjacent tiles, one tile sees a clipped partial fragment while its overlapping neighbor sees the full body. `MDV6Decoder1280` drops small animal boxes touching an internal tile seam whenever an overlapping neighbor contains a larger animal detection. **Person and vehicle veto boxes are never seam-dropped.**
* **Unified Merging:** All tiles from a 1-pass or 3-pass sweep are remapped into canonical `1280×720` coordinates and merged via class-aware Non-Maximum Suppression (`iou_thresh: 0.45`).
* **Tracking Crop Fallback:** `TRK-1:1` holds for `700 ms` (`track_crop_hold_sec`) after the last detection; if `2` consecutive `TRK-1:1` passes miss, `force_scan` immediately triggers a full `SCAN-12MP` or `SCAN-3MP` horizon sweep.

---

## 5. Multi-Layer Validation & Safety Veto Funnel

MegaDetector v6 outputs three classes: **`0: animal`**, **`1: person`**, and **`2: vehicle`**. Every candidate box passes through five sequential gates before `VisionMain` can confirm a target:

```text
Raw MDv6 Boxes (Canonical 1280x720)
        │
        ├──► Class 1 (Person >= 0.48) or Class 2 (Vehicle >= 0.35)
        │         └──► Added to veto_boxes[] (expanded by veto_padding_px = 24 px)
        │              • Immediately aborts active target lock + commands T 0 if overlapping
        │
        └──► Class 0 (Animal)
                  │
                  ├─ 1. Confidence & Dominance Gate
                  │     • score >= 0.65 (HDR) or >= 0.62 (Monochrome)
                  │     • beats person & vehicle scores by >= 0.25 (class_margin)
                  │     • person_prob < 0.02 and vehicle_prob < 0.02 (max_non_animal_prob)
                  │
                  ├─ 2. Geometric & Aspect Ratio Gate
                  │     • width <= 55% frame (max_animal_width_frac)
                  │     • area  <= 40% frame (max_animal_area_frac)
                  │     • height / width <= 1.65 (rejects narrow upright human/post silhouettes)
                  │
                  ├─ 3. Horizon & Ground-Plane Physical Gate
                  │     • Feet (y2) must not sit > 45 px above horizon (horizon_feet_margin_720)
                  │     • If depression >= 25 mrad: trigonometric height must be <= 1.50 m
                  │     • Fused range = 70% ground-plane + 30% stadia (clamped 3.0–75.0 m)
                  │
                  ├─ 4. Zero-Overlap Safety Veto Gate
                  │     • Rejected if box overlaps any padded person/vehicle veto box
                  │
                  └─ 5. Temporal Confirmation Gate (VisionMain)
                        • Requires 4 consecutive coherent batches (center jump <= 120 px)
                        • Saves ACQUIRE audit JPEG & hands lock to TargetKalmanFilter
```