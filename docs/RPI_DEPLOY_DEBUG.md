# Raspberry Pi 5 Setup, Deployment & Debugging Guide

This guide covers provisioning a **Raspberry Pi 5** running **Raspberry Pi OS (64-bit)** for **Bombardeer**, deploying code and static web assets from a development PC, running the unit test suite, performing zero-config site self-calibration (including **Walk to Calibrate** and visual verification via **Verify Grid**), configuring the off-grid Wi-Fi Access Point fallback, and troubleshooting each module in `bombardeer_pkg/`.

---

## Repository Layout

```text
bombardeer/
├── bombardeer.py                         # Gunicorn / CLI entrypoint
├── config.yaml                           # Camera, optics, ballistics, control, wildlife & storage defaults
├── pytest.ini                            # Pytest configuration (pythonpath = ., testpaths = tests)
├── site_calibration.json                 # Auto-generated site calibration (far ground edge, roll, height, slope)
├── MDV6-yolov9-c-1280.hef                # Compiled Hailo-8 MegaDetector v6 model (gitignored)
├── systemd/
│   └── bombardeer.service                # systemd unit (Type=notify, WatchdogSec=15, dual-stack [::]:5000)
├── static/                               # Web console favicons & PWA manifest (served at /static/* & /favicon.ico)
│   ├── bombardeer.ico                    # Multi-size favicon (16x16, 32x32, 48x48)
│   ├── bombardeer.svg                    # Vector favicon for modern desktop browsers
│   ├── favicon-16x16.png                 # 16x16 PNG favicon
│   ├── favicon-32x32.png                 # 32x32 PNG favicon
│   ├── apple-touch-icon.png              # 180x180 iOS/iPadOS home-screen icon
│   ├── android-chrome-192x192.png        # 192x192 Android/Chrome icon
│   ├── android-chrome-512x512.png        # 512x512 PWA splash icon
│   └── site.webmanifest                  # Web app manifest JSON
├── tests/                                # Offline & on-device pytest regression suite
│   ├── conftest.py                       # Stubs Pi-only libcamera/picamera2 on Windows/dev hosts
│   ├── test_config.py
│   ├── test_command_bus_and_web.py
│   ├── test_kalman_ballistics.py
│   ├── test_tiling_and_decoder.py
│   ├── test_latency_and_tracking_regressions.py
│   ├── test_scene_veto.py
│   └── test_site_calibration.py
└── bombardeer_pkg/
    ├── config.py                         # Typed YAML + environment loader
    ├── state.py                          # SystemState (4 lock planes, WILDLIFE default) & TurretCommandBus
    ├── utils.py                          # Thread affinity, sd_notify, audit snapshot writer
    ├── control/
    │   ├── ballistics.py                 # TargetKinematics, parallax, roll-aware horizon, 6'0" deer ceiling & drag ToF
    │   ├── kalman.py                     # Thread-safe 4-state TargetKalmanFilter (anisotropic Q/R & velocity clamp)
    │   ├── site_calibration.py           # Daylight boot sweep, 5'9" Walk-Test Huber fit & passive stadia convergence
    │   └── motion_servo.py               # 50 Hz dual-mode servo, latency-aware fire gate & calibration yield
    ├── hardware/
    │   ├── recorder.py                   # Async 1280x720 event video recorder & quota manager
    │   ├── serial_link.py                # Non-reset 921.6k baud ESP32 UART worker
    │   └── vision_pipeline.py            # VisionRuntime, VisionMain (Cores 0–2) & HudEncoder (Core 1)
    ├── perception/
    │   ├── base.py                       # BaseTargetDetector interface
    │   ├── aruco.py                      # 1280x720 ArUco calibration detector (CPU)
    │   ├── tiling.py                     # Horizon-anchored multi-tile & TRK-1:1 crop planner
    │   ├── mdv6_decoder.py               # Vectorized YOLOv9 decoder & person/vehicle safety veto filter
    │   ├── hailo_worker.py               # Spawned HailoNPUProcess worker loop (Core 2)
    │   └── hailo_detector.py             # HailoWildlifeDetector IPC, single-in-flight gating & supervisor
    └── web/
        ├── app.py                        # Flask Basic-Auth API (/api/mode, /api/health, /api/site_calibration, /video_feed)
        └── templates/
            └── index.html                # Compact single-screen HUD console, Walk Cal UI & Verify Grid SVG overlay
```

---

## 1. Base OS, User Account & Hardware Permissions

1. **Flash Raspberry Pi OS (64-bit)** using Raspberry Pi Imager with **OS Customisation**:
   * **Hostname:** `bombardeer`
   * **Username:** `ben`
   * **SSH & Primary Wi-Fi:** Enabled and configured for your local network.
2. **Update packages and add `ben` to hardware groups** so the service accesses the USB UART and CSI camera without running as root:
   ```bash
   sudo apt update && sudo apt full-upgrade -y
   sudo usermod -aG dialout,video,plugdev ben
   ```
3. **Log out and back in, then verify groups and camera hardware:**
   ```bash
   groups
   rpicam-hello --list-cameras
   ```
   Confirm `dialout` and `video` appear in `groups`, and `rpicam-hello` lists `imx708` / `imx708_wide_noir`.

---

## 2. Enable PCIe Gen 3 for Hailo-8

Raspberry Pi OS defaults the external PCIe slot to Gen 2 (5 GT/s). Enabling **PCIe Gen 3** (8 GT/s) doubles DMA throughput to the Hailo-8 M.2 HAT+:

```bash
sudo tee -a /boot/firmware/config.txt << 'EOF'

# Enable PCIe Gen 3 for Hailo-8 NPU
dtparam=pciex1
dtparam=pciex1_gen=3
EOF

sudo reboot
```

After rebooting, verify the link speed negotiated at `8GT/s`:

```bash
sudo lspci -vv -d 1e60: | grep -i "LnkSta:"
```

---

## 3. Install HailoRT, Camera & Python System Dependencies

Install all runtime and test packages via `apt` so `picamera2`, `libcamera`, `opencv`, `pytest`, and `hailo_platform` share the system Python 3 interpreter:

```bash
sudo apt install -y hailofw hailo-all
sudo apt install -y \
  python3-pip \
  python3-pytest \
  python3-picamera2 \
  python3-opencv \
  python3-numpy \
  python3-flask \
  python3-serial \
  python3-yaml \
  gunicorn \
  dnsmasq-base \
  network-manager
```

Verify the NPU firmware, Python bindings, and IMX708 NoIR ISP tuning file:

```bash
hailortcli fw-control identify
python3 -c "import hailo_platform; print('HailoRT OK')"
ls -l /usr/share/libcamera/ipa/rpi/pisp/imx708_wide_noir.json
```

---

## 4. Deploy Code, Verify Tests & Benchmark the HEF Model

### 4.1 Initial Clone on the Pi
```bash
cd ~
git clone <YOUR_GIT_REPO_URL> bombardeer
cd ~/bombardeer
mkdir -p static recordings/debug_snapshots
chmod +x bombardeer.py
python3 -m compileall -q bombardeer.py bombardeer_pkg
python3 -m pytest -q
```

### 4.2 Safe Deployment from a Windows Development PC
From `C:\src\bombardeer\RPI`, run local syntax checks and unit tests first (`pytest.ini` adds `.` to `pythonpath` so bare `pytest` works directly):

```bat
python -m compileall -q bombardeer_pkg
pytest -v
```

Copy all application code, web assets, and tests to the Pi **without** overwriting Pi-side `recordings/`, `debug_snapshots/`, `models/`, `site_calibration.json`, or `config.yaml` (using built-in Windows `tar` over SSH avoids copying Windows `__pycache__` folders):

```bat
tar --exclude=__pycache__ --exclude=*.pyc -cf - bombardeer.py pytest.ini bombardeer_pkg static tests | ssh ben@bombardeer "tar -xf - -C ~/bombardeer"
```

*(Alternative `scp` command if you do not mind clearing `__pycache__` afterward: `scp -r bombardeer.py pytest.ini bombardeer_pkg static tests ben@bombardeer:~/bombardeer/`. Always target `~/bombardeer/`—never `~/bombardeer/bombardeer_pkg`—and only include `config.yaml` when you intentionally want to overwrite the Pi's copy.)*

Compile, run the full on-device test suite, and restart the service:

```bat
ssh ben@bombardeer "cd ~/bombardeer && find bombardeer_pkg tests -name __pycache__ -exec rm -rf {} + && python3 -m compileall -q bombardeer_pkg && python3 -m pytest -q && sudo systemctl restart bombardeer.service"
```

### 4.3 Verify Static Assets & Benchmark the Hailo Model
* Confirm `~/bombardeer/static/` contains all 8 icon/manifest files (`bombardeer.ico`, `bombardeer.svg`, `favicon-16x16.png`, `favicon-32x32.png`, `apple-touch-icon.png`, `android-chrome-192x192.png`, `android-chrome-512x512.png`, `site.webmanifest`).
* Place `MDV6-yolov9-c-1280.hef` in `~/bombardeer/` (or `~/bombardeer/models/`) and verify NPU throughput:
  ```bash
  hailortcli benchmark ~/bombardeer/MDV6-yolov9-c-1280.hef
  ```

---

## 5. Configure & Enable `bombardeer.service`

Gunicorn **must** run with `--workers 1` so only one process owns the camera, serial port, and NPU, while `--threads 16` and `--bind [::]:5000` handle concurrent MJPEG viewers and API calls over both IPv4 and IPv6. Gunicorn 20.1+ forwards `NOTIFY_SOCKET` directly to the worker, so no `gunicorn.conf.py` file is needed.

1. **Install the service unit (`systemd/bombardeer.service`):**
   ```bash
   sudo cp ~/bombardeer/systemd/bombardeer.service /etc/systemd/system/bombardeer.service
   ```
2. **Set Web UI credentials via a `systemd` drop-in override** (keeps secrets out of Git and unlocks `ARMED` mode when `BOMBARDEER_PASS` is non-default):
   ```bash
   sudo systemctl edit bombardeer.service
   ```
   Insert:
   ```ini
   [Service]
   Environment="BOMBARDEER_USER=ben"
   Environment="BOMBARDEER_PASS=your-strong-password"
   ```
3. **Enable, start, and verify startup mode:**
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable --now bombardeer.service
   sudo systemctl status bombardeer.service --no-pager
   curl -s -u ben:your-strong-password http://127.0.0.1:5000/api/mode
   ```
   Confirm `"detector": "WILDLIFE"` and `"hailo_available": true`. `SystemState` defaults to `WILDLIFE` (Hailo-8 NPU) on startup so the service boots at low CPU instead of running CPU-bound OpenCV ArUco detection.

---

## 6. Zero-Config Site Self-Calibration & "Walk to Calibrate"

Bombardeer is designed to be set down at any new physical location (for example on sawhorses at ~1.0 m height or an elevated mount) and booted **without editing `config.yaml`**. Hardware-fixed offsets (`camera_offset_x_mm`, `camera_offset_y_mm`, `focal_length_px`, `tilt_calibration_trim_mrad`) stay constant across deployments, while site-specific geometry (`yard_top_profile`, `roll_mrad`, `camera_mount_height_m`, `ground_slope_mrad`) is self-calibrated and persisted in `~/bombardeer/site_calibration.json`.

### 6.1 Automatic Daylight Boot Sweep (Far Ground Edge, Roll & Relocation Check)
* **On every boot at home `(0, 0)`:** Once daylight is present (`Lux >= 60`, color HDR mode), `SiteCalibrationManager` compares a lighting-invariant `16×9` scene fingerprint of the home view against `site_calibration.json`.
  * **Same location (`corr >= 0.65`):** Skips the sweep completely—restarts do not move the turret.
  * **New location (`corr < 0.65`) or first boot:** Automatically clears the old site's metric height/slope (`metric_source: "UNSET"`) and runs a **7-stop pan sweep** across `[-750, +750] mrad` at `tilt = 0` (~10 seconds total) when in safe standby (`OFF`/`ON`, serial connected, no target/recording).
  * **Booted at night (or `Boot Sweep` clicked in the dark):** Defers cleanly to `PENDING_DAYLIGHT` without locking out turret motion, and runs automatically at first daylight.
* **What the sweep calibrates:**
  * **Pan-indexed far ground edge (`yard_top_profile`):** Uses Lab color/luma ground-appearance segmentation (plus sky/texture cues) to lock onto the actual lawn-to-tree-base boundary at every pan angle, anchoring `tiling.py` horizon crops and the `ABOVE_HORIZON` bird veto.
  * **Camera roll (`roll_mrad`):** Measures horizon tilt from vertical edges (trunks/posts), compensates both horizon lines and ground-plane checks, and raises a HUD alert (`LEVEL MOUNT: Tilted X.X deg left/right`) if roll exceeds `2.5°` (`44 mrad`).

### 6.2 How to "Walk to Calibrate" (`5'9"` Operator Walk-Test)
A static pan-tilt camera cannot observe metric mount height (`camera_mount_height_m`) from rotation alone. Running a 15–20 second **Walk-Test** immediately solves both **`camera_mount_height_m`** and **`ground_slope_mrad`** using MegaDetector's standing `PERSON` detections and an exact projective Huber-IRLS fit (default operator height is **`5'9"` / `1.7526 m`**).

#### Prerequisites
* Turret is positioned for deployment, powered on, serial connected (`SER:OK`), and in daylight or a well-lit yard so MegaDetector detects a full standing `PERSON` box.
* Only **one standing person** (you) will be inside the camera's field of view during the walk. *(Horizontal non-person false detections—such as the paintball barrel tip in the upper-right—are automatically filtered out before counting standing persons.)*

#### Step-by-Step Procedure (Web Console)
1. **Open the Bombardeer Web Console** (`http://bombardeer:5000`) on your phone or laptop.
2. *(Optional)* If someone of a different height is walking, click **`Details ▾`** in the **Site Calibration** bar and adjust the **Walk-Test Operator Height** (`ft` / `in`) inputs (defaults to `5 ft 9 in` / `1.753 m`).
3. **Click `Walk Cal (5'9")`** in the **Site Calibration** bar below the mode buttons:
   * If currently in `ARUCO` mode, `/api/site_calibration` automatically switches the detector to **`WILDLIFE`** (Hailo-8 NPU).
   * The turret immediately disarms to **`OFF / SAFE`**, forces `TRIGGER 0`, clears pending motion, levels tilt to **`0 mrad`** at the current pan, and locks out `TRACK` / `1-SHOT` / `ARM LIVE` (`SITE CALIBRATING` badge, HTTP `409` on mode changes until finished or canceled).
   * The Web Console automatically opens **`Details ▾`**, enables **`Verify Grid`**, and shows live **Person Samples (`0 / 16`, min `8`)** and **Depth Span (`0.0 m / 5.5 m`, min `4.0 m`)** progress bars alongside the HUD banner.
4. **Walk Out & Back Through the Yard:**
   * Step into view roughly **5–6 m** (~18 ft) in front of the turret, standing upright so both your head and feet are fully visible (`>= 10 px` inside the top and bottom frame edges).
   * Walk smoothly away out to **20–25 m** (~65–80 ft) and walk back toward the turret, pausing ~0.5 s every few meters when in `SCAN-12MP` (`~2.4–2.9 Hz`) so sharp frames span a wide depth range.
   * Avoid crouching or having a second person enter the frame (`bh / bw < 1.30` or clipped boxes are ignored automatically).
5. **Completion (Automatic or Manual):**
   * **Auto-finish:** Once **`>= 16` valid samples** span **`>= 5.5 m`** of depth, the walk-test locks automatically, saves `site_calibration.json`, and flashes `SITE CAL LOCKED: H=X.XXm | Slope=+X.Xmrad` on the HUD.
   * **Early manual finish:** Once the progress bars pass the orange minimum markers (`>= 8` samples and `>= 4.0 m` span), **`Finish Walk`** turns green and enabled—click it to lock immediately.
   * **Timeout:** Auto-evaluates and exits after **`45 seconds`**. Click **`Cancel`** (or **`OFF`**) at any time to abort without saving.
6. **Re-enable Tracking:** Once clear of the yard and visually verified (Section 6.3), switch back to **`TRACK`** (safe) or lift the **`LIFT TO ARM`** cover (`4 s` countdown) to enter **`ARMED`**.

*(CLI equivalent via `curl`: `POST /api/site_calibration` with `{"action":"walk_start"}` (optional `"operator_height_m"`), `{"action":"walk_finish"}`, `{"action":"sweep"}`, `{"action":"cancel"}`, or `{"action":"reset"}`.)*

### 6.3 Visually Verifying Site Configuration (`Verify Grid` & `Details ▾`)
Click **`Verify Grid`** (in the Site Calibration bar or fullscreen overlay) and expand **`Details ▾`** to confirm the site geometry against your physical yard:

* **Top Overlay Status Pill:** Shows green **`VERIFY GRID (WALK_TEST)`** (or `PASSIVE_STADIA`) with locked `H`, `Slope`, and `Roll` once calibrated; shows amber **`UNCALIBRATED FALLBACK`** beforehand.
* **Three Horizontal Reference Lines (roll-compensated):**
  * **`FAR DEER HEAD 6'0"` (orange dashed):** Perspective-computed height of a `6'0"` (`1.83 m`) standing buck at the far ground edge (`far_head_mrad`).
  * **`METRIC HORIZON` (gold dashed):** Solved ground-plane vanishing line (`ground_slope_mrad`).
  * **`FAR GROUND EDGE` (cyan dashed):** Scanned lawn-to-treeline boundary (`yard_top_mrad`) at the current pan angle.
* **Projected Range Grid (`5m`–`40m`) & Scale Brackets:**
  * Check where the green `5m (16ft)`, `10m (33ft)`, `15m (49ft)`, and `20m (66ft)` lines cross known yard landmarks (walkway, driveway, trees).
  * Confirm the orange **`5'9" @ 10m`** bracket and green **`1.0m deer @ 20m`** bracket look proportional—or stand briefly on the `10m` line to confirm your body matches the orange bracket.
* **Diagnostic Cards & Pan Treeline Profile (`Details ▾`):**
  * **Mount Height & Ground Slope:** Confirm height matches your physical camera lens height (within ~0.1–0.2 m) and slope sign (`+` downhill, `-` uphill).
  * **Mount Roll:** Spirit-level bubble stays green/centered (`< 2.5°`).
  * **Fit & Sweep:** Walk-Test `RMS` residual should typically be `< 10–15 mrad` with `7 treeline stops (DONE)`.
  * **Pan Treeline Profile:** Plots `yard_top_mrad` across `Left (+750)` to `Right (-750)` (matching the right-handed camera view) with a live green cursor tracking current pan—the blue curve should vary smoothly without sharp single-stop spikes.

### 6.4 Hands-Off Fallback (Skipping the Walk-Test)
If you set the turret down in a new spot and do **not** run a Walk-Test:
* Bombardeer automatically sets `use_ground_range = False` until metric height is learned—keeping the scanned far-ground-edge `ABOVE_HORIZON` veto active while using `stadia_range_m` directly so an uncalibrated mount height never triggers false `HEIGHT` rejections.
* As wildlife detections occur across the yard, `SiteCalibrationManager.observe_passive_wildlife()` passively collects `(1 / stadia_range_m, tan(dep_feet))` pairs. Once **`>= 15` observations** span **`>= 5.0 m`** of depth with low residual (`<= 35 mrad`), it automatically locks `"metric_source": "PASSIVE_STADIA"`, re-enables `70% / 30%` ground-plane range fusion and `HEIGHT` vetoes, and saves `site_calibration.json`. Running a Walk-Test at any time overrides passive calibration.

---

## 7. Off-Grid Wi-Fi Access Point Fallback (`wifi-fallback.service`)

If both `eth0` and `wlan0` remain disconnected for 45 seconds, `/usr/local/bin/wifi-fallback.sh` creates an open AP named `bombardeer` at `10.42.0.1/24` and tears it down automatically if Ethernet is connected.

```bash
sudo tee /usr/local/bin/wifi-fallback.sh << 'EOF'
#!/bin/bash
WIFI_IF="wlan0"
ETH_IF="eth0"
AP_SSID="bombardeer"
CHECK_INTERVAL=15
CONNECTION_TIMEOUT=45

log() { echo "$(date '+%Y-%m-%d %H:%M:%S') [WIFI-WATCHDOG] $1"; }
is_connected() {
    local state
    state=$(nmcli -t -f GENERAL.STATE device show "$1" 2>/dev/null | cut -d: -f2)
    [[ "$state" == 100* ]]
}
start_ap() {
    nmcli connection delete Hotspot 2>/dev/null || true
    nmcli connection add type wifi ifname "$WIFI_IF" con-name Hotspot autoconnect no ssid "$AP_SSID" \
        802-11-wireless.mode ap 802-11-wireless.band bg \
        ipv4.addresses 10.42.0.1/24 ipv4.method shared
    nmcli connection up Hotspot
}
teardown_ap() {
    if nmcli -t -f NAME connection show --active | grep -qx "Hotspot"; then
        log "Tearing down AP mode ($AP_SSID) and restoring normal Wi-Fi..."
        nmcli connection down Hotspot 2>/dev/null || true
        nmcli connection delete Hotspot 2>/dev/null || true
        nmcli device connect "$WIFI_IF" 2>/dev/null || true
    fi
}

sleep 10
while true; do
    if is_connected "$ETH_IF"; then teardown_ap; sleep "$CHECK_INTERVAL"; continue; fi
    if is_connected "$WIFI_IF" && ! nmcli -t -f NAME connection show --active | grep -qx "Hotspot"; then
        sleep "$CHECK_INTERVAL"; continue
    fi
    log "No network connection on $ETH_IF or $WIFI_IF. Waiting ${CONNECTION_TIMEOUT}s..."
    ELAPSED=0
    while [ "$ELAPSED" -lt "$CONNECTION_TIMEOUT" ]; do
        sleep 5; ELAPSED=$((ELAPSED + 5))
        if is_connected "$ETH_IF" || is_connected "$WIFI_IF"; then break; fi
    done
    if ! is_connected "$ETH_IF" && ! is_connected "$WIFI_IF"; then
        log "Starting open Access Point mode ($AP_SSID)..."
        start_ap
        while true; do
            sleep 5
            if is_connected "$ETH_IF"; then teardown_ap; break; fi
        done
    fi
done
EOF
sudo chmod +x /usr/local/bin/wifi-fallback.sh

sudo tee /etc/systemd/system/wifi-fallback.service << 'EOF'
[Unit]
Description=Wi-Fi Fallback to AP Mode Watchdog
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=simple
ExecStart=/usr/local/bin/wifi-fallback.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now wifi-fallback.service
```

---

## 8. Package-by-Package Verification & Troubleshooting

Use `/api/health`, `/api/site_calibration`, static asset probes, live per-thread `top -H`, and `journalctl` to inspect telemetry across all four subpackages:

```bash
curl -s -u ben:your-strong-password http://127.0.0.1:5000/api/health | python3 -m json.tool
curl -s -u ben:your-strong-password http://127.0.0.1:5000/api/site_calibration | python3 -m json.tool
curl -s -o /dev/null -w "favicon.ico: %{http_code}\n" -u ben:your-strong-password http://127.0.0.1:5000/favicon.ico
curl -s -o /dev/null -w "bombardeer.svg: %{http_code}\n" -u ben:your-strong-password http://127.0.0.1:5000/static/bombardeer.svg
curl -s -o /dev/null -w "site.webmanifest: %{http_code}\n" -u ben:your-strong-password http://127.0.0.1:5000/static/site.webmanifest
top -H -b -n 2 -d 0.5 -p $(pgrep -n -f "gunicorn.*bombardeer") | tail -n 20
journalctl -u bombardeer -f
```

### 8.1 CPU Usage & `top` Interpretation
* **Per-Core vs. Total CPU:** `top` defaults to **Irix mode** (`100%` = 1 core, `400%` = all 4 Pi 5 cores). Compare `%Cpu(s)` in the header (`100% - id%`) against the `gunicorn` row (`row / 4 = total CPU`). Use `top -H` (above) rather than `ps -L --sort=-pcpu` to see live per-thread CPU for `VisionMain`, `HudEncoder`, `MotionServo`, `TurretSerial`, and `VidRecorder`.
* **Expected CPU by Detector:**
  * **`WILDLIFE` (Hailo-8 NPU):** Low host CPU (`~15–35%` Irix / `< 10%` total) because neural inference runs on the Hailo-8 PCIe module.
  * **`ARUCO` (CPU):** Runs OpenCV `detectMarkers()` synchronously on `1280×720` frames across `VisionMain`'s `[0, 1, 2]` core pool (`~19 FPS`, `~190%` Irix / `~48%` total CPU on high-texture outdoor scenes while leaving Core 3 dedicated to `MotionServo` and `TurretSerial`). Switch back to `WILDLIFE` when finished with ArUco bench calibration.

### 8.2 `bombardeer_pkg/hardware/` (`vision_pipeline.py`, `serial_link.py`, `recorder.py`)
* **Camera & DMA Buffers:** If `Picamera2` fails to switch between `DUSK_3MP` and `DAY_12MP`, check `journalctl` for `[CAMERA]`. `VisionMain` waits until `wildlife_detector.is_idle()` and `not site_calibration.is_motion_owned()`, fences stale NPU results with `discard_older_than()`, copies native ROIs into `roi_scratch` inside `MappedArray`, and calls `gc.collect()` before `picam2.configure()`.
* **NoIR Pink/Magenta Foliage Tint:** The IMX708 Wide NoIR lacks an IR-cut filter; sunlit grass and pine needles reflect near-infrared light strongly and appear pink/magenta in daylight HDR mode. This is normal and expected.
* **ESP32 Serial Link:** Confirm `"serial_connected": true` and incrementing `"serial_rx_count"` (`~20 Hz`) in `/api/health`. Verify the CP2102 adapter appears under `/dev/serial/by-id/`.
* **Storage & Thermal Headroom:** Confirm `"soc_temp_c"` stays below `75 °C` (typical `55–62 °C` under continuous `SCAN-12MP`) and `"free_disk_gb"` remains above `5.0 GB`.

### 8.3 `bombardeer_pkg/perception/` (`aruco.py`, `hailo_detector.py`, `hailo_worker.py`, `tiling.py`, `mdv6_decoder.py`)
* **Hailo Subprocess Health:** Confirm `"hailo_available": true` and `"npu_restarts": 0` in `/api/health`. In `DAY_12MP`, `SCAN-12MP` runs at `~2.4–4.0 Hz` (`~342 ms` per 3-pass sweep) with strict single-in-flight job gating; in `DUSK_3MP`, `SCAN-3MP` runs at `~7.4–9.0 Hz` (`~110 ms`).
* **Timestamp-Matched Audit Snapshots:** `ACQUIRE_*.jpg`, `VETO_*.jpg`, and `VETO_ACTIVE_*.jpg` in `recordings/debug_snapshots/` pull the exact `1280×720` `lores` frame matching `eff_capture_mono` from `npu_lores_history` (`maxlen=12`), so bounding boxes align squarely on moving vehicles and deer without NPU latency lag.
* **False Positives vs. Missed Detections:**
  * Static tree-trunk or background false locks: adjust `wildlife.conf_thresh` (default `0.65`) or `control.confirmation_frames` in `config.yaml`.
  * Tall bucks or slight slope variance rejected by `REJ:HEIGHT`: run **`Walk Cal (5'9")`** to lock exact site height/slope, and confirm `wildlife.max_animal_height_m` (default `1.85 m` / `6'1"` for mature Western Virginia white-tailed bucks head-up with antlers) in `config.yaml`.
  * **Scene-Wide Safety Veto & Fixed Barrel Mask:** Any non-barrel `PERSON` (`>= 0.48`) or `VEHICLE` (`>= 0.35`) anywhere in the frame rejects all animal detections in that sweep (`SCENE_VETO`) and aborts active locks (`TRIGGER 0`). False detections on the rigidly mounted upper-right barrel and receiver shroud are suppressed pre-NMS by `MDV6Decoder1280`'s `1280×720` barrel exclusion mask (`>= 55%` overlap), while real people walking beneath the barrel still veto immediately. If flickering `VETO:PERSON` boxes ever appear on distant tree trunks, adjust `wildlife.person_veto_thresh` conservatively.

### 8.4 `bombardeer_pkg/control/` (`ballistics.py`, `kalman.py`, `site_calibration.py`, `motion_servo.py`)
* **Horizon & Far-Edge Alignment:** On the live OpenCV HUD, the teal dashed line shows the metric ground-plane horizon and the gold dashed line shows the perspective-computed `6'0"` far-edge deer head line (both roll-compensated). Enable **`Verify Grid`** in the Web UI to inspect all three lines (`FAR DEER HEAD 6'0"`, `METRIC HORIZON`, `FAR GROUND EDGE`) plus projected `5m–40m` range lines and height brackets. If misaligned after relocating, click **`Reset`** or **`Boot Sweep`** + **`Walk Cal (5'9")`**.
* **Tracking Oscillation, Grazing Bounce & Multi-Deer Slingshot Protection:** Damped-velocity Kalman coasting (`0.35 s+` pan hold / `0.12 s` tilt hold with exponential decay in both `update_measurement()` and `predict_state()`), a `180 mrad/s` sustained tilt velocity ceiling, range-consistent elliptical target association, a `10.0 mrad` enter / `4.0 mrad` exit tilt deadband hysteresis (`control.tilt_velocity_deadband_mrad`), and range-specific site-calibrated elevation clamping prevent grazing tilt chatter, cross-yard multi-deer swaps, and blind coasting into eaves or hard stops. If wobble ever reappears, inspect `journalctl -u bombardeer.service` for `Err:` vs `Cmd:V(...)` and verify ESP32 serial telemetry arrives smoothly without bursts.
* **Elevation Point-of-Impact Trim:** If 1-SHOT `CALIBRATE` impacts consistently high or low at measured range, adjust `optics.tilt_calibration_trim_mrad` (default `-8.3 mrad`) in `config.yaml`.
* **ARMED Refused (`HTTP 403`):** Verify `/etc/systemd/system/bombardeer.service.d/override.conf` sets a non-default `BOMBARDEER_PASS` and run `sudo systemctl daemon-reload && sudo systemctl restart bombardeer`.

### 8.5 `bombardeer_pkg/web/` (`app.py`, `templates/index.html`)
* **Reverse Proxy `502 Bad Gateway` or Stalled Buttons:** Point Nginx/Caddy directly to the Pi's static LAN IP (`http://<pi-ip>:5000`, avoiding `.local` mDNS delays) and disable proxy response buffering (`proxy_buffering off;` in Nginx or `flush_interval -1` in Caddy) so `/video_feed` does not block `POST /api/mode` or `/api/site_calibration`.
* **Favicon / Manifest `401` or `404`:** Ensure `index.html` includes `crossorigin="use-credentials"` on `<link rel="manifest" href="/static/site.webmanifest" ...>` so browsers send HTTP Basic Auth credentials for the manifest, verify all 8 files exist in `~/bombardeer/static/`, and hard-refresh (`Ctrl+F5`).
* **HUD Resolution & Bandwidth:** `camera.display_width: 1280` and `display_height: 720` stream native 720p frames at up to `15 FPS` (`~15–18 Mbps`). For slower remote uplinks, lower `camera.hud_max_fps` to `5.0–8.0` or set `display_width: 640`, `display_height: 360`.
