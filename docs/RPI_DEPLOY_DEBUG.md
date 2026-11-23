# Raspberry Pi 5 Setup, Deployment & Debugging Guide

This guide covers provisioning a **Raspberry Pi 5** running **Raspberry Pi OS (64-bit)** for **Bombardeer**, deploying the Gunicorn + `systemd` service and static web assets, configuring the off-grid Wi-Fi Access Point fallback, and troubleshooting each module in `bombardeer_pkg/`.

---

## Repository Layout

```text
bombardeer/
├── bombardeer.py                     # Gunicorn / CLI entrypoint
├── config.yaml                       # Camera, optics, ballistics, control, wildlife & storage settings
├── MDV6-yolov9-c-1280.hef            # Compiled Hailo-8 MegaDetector v6 model (gitignored)
├── systemd/
│   └── bombardeer.service            # systemd unit (Type=notify, WatchdogSec=15, dual-stack [::]:5000)
├── static/                           # Web console favicons & PWA manifest (served at /static/* & /favicon.ico)
│   ├── bombardeer.ico                # Multi-size favicon (16x16, 32x32, 48x48)
│   ├── bombardeer.svg                # Vector favicon for modern desktop browsers
│   ├── favicon-16x16.png             # 16x16 PNG favicon
│   ├── favicon-32x32.png             # 32x32 PNG favicon
│   ├── apple-touch-icon.png          # 180x180 iOS/iPadOS home-screen icon
│   ├── android-chrome-192x192.png    # 192x192 Android/Chrome icon
│   ├── android-chrome-512x512.png    # 512x512 PWA splash icon
│   └── site.webmanifest              # Web app manifest JSON
└── bombardeer_pkg/
    ├── config.py                     # Typed YAML + environment loader
    ├── state.py                      # Thread-safe SystemState & CommandBus
    ├── utils.py                      # Thread affinity, sd_notify, audit snapshot writer
    ├── control/
    │   ├── ballistics.py             # Parallax, horizon projection, drag ToF & Kalman filter
    │   └── motion_servo.py           # 50 Hz dual-mode servo & firing interlock loop
    ├── hardware/
    │   ├── recorder.py               # Async 1280x720 event video recorder & quota manager
    │   ├── serial_link.py            # Non-reset 921.6k baud ESP32 UART worker
    │   └── vision_pipeline.py        # VisionRuntime, VisionMain (Core 0) & HudEncoder (Core 1)
    ├── perception/
    │   ├── aruco.py                  # 1280x720 ArUco calibration detector
    │   ├── hailo_detector.py         # Forked HailoNPUProc manager & SharedMemory IPC
    │   └── mdv6_decoder.py           # Multi-tile horizon planner, decoder & safety veto filter
    └── web/
        ├── app.py                    # Flask Basic-Auth API (/api/mode, /api/health, /video_feed, /static/*)
        └── templates/
            └── index.html            # Responsive 1280x720 HUD console & safety arming cover
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

Install all runtime packages via `apt` so `picamera2`, `libcamera`, `opencv`, and `hailo_platform` share the system Python 3 interpreter:

```bash
sudo apt install -y hailofw hailo-all
sudo apt install -y \
  python3-pip \
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

## 4. Clone Repository & Benchmark the HEF Model

1. **Clone into `/home/ben/bombardeer` and verify syntax:**
   ```bash
   cd ~
   git clone <YOUR_GIT_REPO_URL> bombardeer
   cd ~/bombardeer
   mkdir -p static recordings/debug_snapshots
   chmod +x bombardeer.py
   python3 -m compileall -q bombardeer.py bombardeer_pkg
   ```
2. **Verify `static/` web assets and model weights are present:**
   * Confirm `static/` contains `bombardeer.ico`, `bombardeer.svg`, `favicon-16x16.png`, `favicon-32x32.png`, `apple-touch-icon.png`, `android-chrome-192x192.png`, `android-chrome-512x512.png`, and `site.webmanifest`.
   * Place `MDV6-yolov9-c-1280.hef` in `~/bombardeer/` and run a hardware benchmark:
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
3. **Enable and start the service:**
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable --now bombardeer.service
   sudo systemctl status bombardeer.service --no-pager
   ```

---

## 6. Off-Grid Wi-Fi Access Point Fallback (`wifi-fallback.service`)

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

## 7. Package-by-Package Verification & Troubleshooting

Use `/api/health`, static asset probes, and `journalctl` to inspect live telemetry across all four subpackages:

```bash
curl -s -u ben:your-strong-password http://127.0.0.1:5000/api/health | python3 -m json.tool
curl -s -o /dev/null -w "favicon.ico: %{http_code}\n" -u ben:your-strong-password http://127.0.0.1:5000/favicon.ico
curl -s -o /dev/null -w "bombardeer.svg: %{http_code}\n" -u ben:your-strong-password http://127.0.0.1:5000/static/bombardeer.svg
curl -s -o /dev/null -w "site.webmanifest: %{http_code}\n" -u ben:your-strong-password http://127.0.0.1:5000/static/site.webmanifest
journalctl -u bombardeer -f
```

### 7.1 `bombardeer_pkg/hardware/` (`vision_pipeline.py`, `serial_link.py`, `recorder.py`)
* **Camera & DMA Buffers:** If `Picamera2` fails to switch between `DUSK_3MP` and `DAY_12MP`, check `journalctl` for `[CAMERA]`. `VisionMain` copies native ROIs into `roi_scratch` inside `MappedArray` and calls `gc.collect()` before `picam2.configure()` so no exported DMA pointers remain open.
* **NoIR Pink/Magenta Foliage Tint:** The IMX708 Wide NoIR lacks an IR-cut filter; sunlit grass and pine needles reflect near-infrared light strongly and appear pink/magenta in daylight HDR mode. This is normal and expected.
* **ESP32 Serial Link:** Confirm `"serial_connected": true` and incrementing `"serial_rx_count"` (`~20 Hz`) in `/api/health`. Verify the CP2102 adapter appears under `/dev/serial/by-id/`.
* **Storage & Thermal Headroom:** Confirm `"soc_temp_c"` stays below `75 °C` (typical `55–62 °C` under continuous `SCAN-12MP`) and `"free_disk_gb"` remains above `5.0 GB`.

### 7.2 `bombardeer_pkg/perception/` (`aruco.py`, `hailo_detector.py`, `mdv6_decoder.py`)
* **Hailo Subprocess Health:** Confirm `"hailo_available": true` and `"npu_restarts": 0` in `/api/health`. In `DAY_12MP`, `SCAN-12MP` runs at `~2.9–4.0 Hz` (`~342 ms` per 3-pass sweep); in `DUSK_3MP`, `SCAN-3MP` runs at `~9.0 Hz` (`~110 ms`).
* **False Positives vs. Missed Detections:**
  * Static tree-trunk or background false locks: adjust `wildlife.conf_thresh` (default `0.65`) or `control.confirmation_frames` (default `4`) in `config.yaml`.
  * Flickering red `VETO:PERSON` boxes on high-contrast NoIR tree trunks: adjust `wildlife.person_veto_thresh` (default `0.48`)—keep it conservative enough that real people walking across the yard are always boxed in red.
  * Inspect `recordings/debug_snapshots/ACQUIRE_*.jpg` and `VETO_*.jpg` to review exact confidence scores and bounding boxes after field tests.

### 7.3 `bombardeer_pkg/control/` (`ballistics.py`, `motion_servo.py`)
* **Horizon Line Alignment:** In the Web HUD, the dashed line should sit along the distant ground/treeline transition. Adjust `optics.horizon_y_offset_720` in `config.yaml` (positive moves the line down in pixels; default `15.0`).
* **Elevation Point-of-Impact Trim:** If 1-SHOT calibration impacts consistently high or low at measured range, adjust `optics.tilt_calibration_trim_mrad` (default `-8.3 mrad`).
* **ARMED Refused (`HTTP 403`):** Verify `/etc/systemd/system/bombardeer.service.d/override.conf` sets a non-default `BOMBARDEER_PASS` and run `sudo systemctl daemon-reload && sudo systemctl restart bombardeer`.

### 7.4 `bombardeer_pkg/web/` (`app.py`, `templates/index.html`)
* **Reverse Proxy `502 Bad Gateway` or Stalled Buttons:** If accessing Bombardeer through Nginx or Caddy, point the upstream directly to the Pi's static LAN IP (`http://<pi-ip>:5000`, avoiding `.local` mDNS delays) and disable proxy response buffering (`proxy_buffering off;` in Nginx or `flush_interval -1` in Caddy) so the continuous `/video_feed` MJPEG stream does not block `POST /api/mode` requests.
* **Favicon / Manifest `404` or `204`:** Ensure all 8 files from `static/` are copied to `~/bombardeer/static/` on the Pi (`scp -r static\* ben@bombardeer:~/bombardeer/static/`), verify `static_folder=cfg.static_dir, static_url_path="/static"` is set in `app.py`, and hard-refresh your browser (`Ctrl+F5`) to clear cached `204` responses.
* **HUD Resolution & Bandwidth:** `camera.display_width: 1280` and `display_height: 720` stream native 720p frames at up to `15 FPS` (`~15–18 Mbps`). For slower remote uplinks, lower `camera.hud_max_fps` to `5.0–8.0` or set `display_width: 640`, `display_height: 360`.