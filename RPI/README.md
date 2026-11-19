# Bombardeer — Raspberry Pi 5 Setup & Deployment Guide

This guide walks through converting a fresh **Raspberry Pi OS (64-bit)** installation on a Raspberry Pi 5 into a production **Bombardeer** autonomous turret controller with:

* **Hailo-8 (26 TOPS) NPU** acceleration over PCIe Gen 3 running MegaDetector v6 (`MDV6-yolov9-c-1280.hef`)
* **IMX708 Wide NoIR** camera with PiSP hardware tuning and automatic day/dusk ISP switching
* **Gunicorn + Flask Web UI** with HTTP Basic Authentication and live 16:9 MJPEG HUD (`1280×720` capture / `640×360` stream)
* **ESP32 closed-loop serial control** at 921.6k baud
* **Automatic off-grid Wi-Fi Access Point fallback** (`bombardeer` on `10.42.0.1/24`)

---

## Hardware Requirements

* **SBC:** Raspberry Pi 5 (Active Cooler recommended)
* **AI Accelerator:** Hailo-8 M.2 Module mounted via M.2 HAT+
* **Camera:** Raspberry Pi Camera Module 3 (IMX708 Wide NoIR) connected to `CAM/DISP 0` or `1`
* **Motion Controller:** ESP32 turret controller connected via USB serial (`/dev/ttyUSB0` or `/dev/ttyACM0`)
* **OS:** Raspberry Pi OS 64-bit (Debian 12 Bookworm / Linux Kernel 6.12+)
* **Default User:** `ben`

---

## Step 1: Base OS, User Account & Hardware Permissions

1. **Flash Raspberry Pi OS (64-bit)** using Raspberry Pi Imager. Under **OS Customisation**:
   * Set **Hostname** to `bombardeer`
   * Create user **`ben`** with your password
   * Enable **SSH** and pre-configure your primary Wi-Fi network (e.g., `IOT`)

2. **Update the OS and add `ben` to required hardware groups** so the service can access the ESP32 USB serial port and camera without running as root:
   ```bash
   sudo apt update && sudo apt full-upgrade -y
   sudo usermod -aG dialout,video,plugdev ben
   ```

3. **Verify group membership and camera detection:** Log out and back in, then run:
   ```bash
   groups
   rpicam-hello --list-cameras
   ```
   Confirm `dialout` and `video` are listed, and that the camera is detected as `imx708` or `imx708_wide_noir`.

---

## Step 2: Enable PCIe Gen 3 Hardware Link

By default, Raspberry Pi OS limits the single PCIe lane to **Gen 2** (5 GT/s). Enabling **PCIe Gen 3** (8 GT/s) doubles bus bandwidth to the Hailo-8 AI HAT+ so 1280×1280 MegaDetector v6 inference runs at full frame rate without bottlenecking DMA transfers.

1. **Append PCIe Gen 3 parameters to `/boot/firmware/config.txt` and reboot:**
   ```bash
   sudo tee -a /boot/firmware/config.txt << 'EOF'

   # Enable PCIe Gen 3 for Hailo-8 NPU
   dtparam=pciex1
   dtparam=pciex1_gen=3
   EOF

   sudo reboot
   ```

2. **Verify the hardware link negotiated at 8 GT/s:**
   ```bash
   sudo lspci -vv -d 1e60: | grep -i "LnkSta:"
   ```
   *Expected output:* `LnkSta: Speed 8GT/s, Width x1` (may include `(downgraded)` if the M.2 device supports x4).

---

## Step 3: Install HailoRT, Camera & Python Dependencies

Install the Hailo firmware/runtime, computer vision libraries, Gunicorn, and NetworkManager packages via `apt` so `picamera2`, `libcamera`, `opencv`, and `hailo_platform` share the system Python interpreter cleanly:

```bash
# 1. Install Hailo firmware, PCIe kernel driver, CLI, and Python bindings
sudo apt install -y hailofw hailo-all

# 2. Install camera, computer vision, serial, web server, and AP fallback dependencies
sudo apt install -y \
  python3-pip \
  python3-picamera2 \
  python3-opencv \
  python3-numpy \
  python3-flask \
  python3-serial \
  gunicorn \
  dnsmasq-base \
  network-manager
```

**Verify the Hailo-8 NPU and IMX708 NoIR ISP tuning profile:**
```bash
hailortcli fw-control identify
python3 -c "import hailo_platform; print('HailoRT OK')"
ls -l /usr/share/libcamera/ipa/rpi/pisp/imx708_wide_noir.json
```
Confirm `hailortcli` outputs the Hailo-8 board info, Python prints `HailoRT OK`, and the `imx708_wide_noir.json` tuning file exists.

---

## Step 4: Clone Repository & Verify Model Benchmark

1. **Clone the repository and ensure storage directories exist:**
   ```bash
   cd ~
   git clone <YOUR_GIT_REPO_URL> Bombardeer
   cd ~/Bombardeer
   mkdir -p models static recordings/debug_snapshots
   chmod +x bombardeer.py
   python3 -m py_compile bombardeer.py
   ```
   Ensure the compiled MegaDetector v6 model is located at `~/Bombardeer/models/MDV6-yolov9-c-1280.hef`.

2. **Benchmark the compiled HEF model on the Hailo-8 NPU** to verify hardware allocation and baseline throughput before starting the service:
   ```bash
   hailortcli benchmark ~/Bombardeer/models/MDV6-yolov9-c-1280.hef
   ```
   Confirm the benchmark completes cleanly with sustained FPS and low hardware latency.

---

## Step 5: Configure & Enable `bombardeer.service` (Gunicorn + Basic Auth)

Gunicorn **must** run with `--workers 1` so only a single process owns the camera, ESP32 serial port, and Hailo-8 NPU, while `--threads` serves concurrent MJPEG streams and API calls.

1. **Install the base service unit from the repository (`RPI/systemd/bombardeer.service`):**
   ```bash
   sudo cp ~/Bombardeer/RPI/systemd/bombardeer.service /etc/systemd/system/
   ```

2. **Set Web UI credentials using a systemd drop-in override** (this keeps passwords out of the Git-tracked service file and preserves them across future service updates):
   ```bash
   sudo systemctl edit bombardeer.service
   ```
   Add **only** the following lines between the top comment and the `### Lines below this comment will be discarded` marker (do **not** uncomment or duplicate `ExecStart=` from the base file):
   ```ini
   [Service]
   Environment="BOMBARDEER_USER=ben"
   Environment="BOMBARDEER_PASS=change-this-password"
   ```
   Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`).

3. **Reload systemd, enable on boot, and start immediately:**
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable --now bombardeer.service
   ```

4. **Verify startup logs:**
   ```bash
   sudo systemctl status bombardeer.service --no-pager
   sudo journalctl -u bombardeer.service -n 30 --no-pager
   ```
   Confirm `Active: active (running)` with `override.conf` listed under `Drop-In:`, and verify log lines for `[HAILO NPU PROCESS]`, `[CAMERA]`, and `[SERIAL]`.

---

## Step 6: Configure Off-Grid Wi-Fi Fallback (`wifi-fallback.service`)

This watchdog monitors `eth0` and `wlan0`. If both remain disconnected for 45 seconds, it launches an open access point (`bombardeer`) with static IP `10.42.0.1/24` and DHCP via `dnsmasq-base`. When Ethernet is plugged back in, it tears down the AP and restores normal Wi-Fi.

1. **Configure saved Wi-Fi autoconnect priorities** (adjust connection names to match your environment; SSIDs are strictly case-sensitive):
   ```bash
   sudo nmcli connection modify "IOT" connection.autoconnect yes connection.autoconnect-priority 10
   sudo nmcli connection modify "Guest" connection.autoconnect yes connection.autoconnect-priority 5
   ```

2. **Install `/usr/local/bin/wifi-fallback.sh`:**
   ```bash
   sudo tee /usr/local/bin/wifi-fallback.sh << 'EOF'
   #!/bin/bash
   WIFI_IF="wlan0"
   ETH_IF="eth0"
   AP_SSID="bombardeer"
   CHECK_INTERVAL=15
   CONNECTION_TIMEOUT=45

   log() {
       echo "$(date '+%Y-%m-%d %H:%M:%S') [WIFI-WATCHDOG] $1"
   }

   is_connected() {
       local state
       state=$(nmcli -t -f GENERAL.STATE device show "$1" 2>/dev/null | cut -d: -f2)
       [[ "$state" == 100* ]]
   }

   start_ap() {
       nmcli connection delete Hotspot 2>/dev/null || true
       nmcli connection add type wifi ifname "$WIFI_IF" con-name Hotspot autoconnect no ssid "$AP_SSID" \
           802-11-wireless.mode ap \
           802-11-wireless.band bg \
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
       if is_connected "$ETH_IF"; then
           teardown_ap
           sleep "$CHECK_INTERVAL"
           continue
       fi

       if is_connected "$WIFI_IF" && ! nmcli -t -f NAME connection show --active | grep -qx "Hotspot"; then
           sleep "$CHECK_INTERVAL"
           continue
       fi

       log "No network connection on $ETH_IF or $WIFI_IF. Waiting ${CONNECTION_TIMEOUT}s for auto-reconnect..."
       ELAPSED=0
       while [ "$ELAPSED" -lt "$CONNECTION_TIMEOUT" ]; do
           sleep 5
           ELAPSED=$((ELAPSED + 5))
           if is_connected "$ETH_IF" || is_connected "$WIFI_IF"; then
               log "Network connection established."
               break
           fi
       done

       if ! is_connected "$ETH_IF" && ! is_connected "$WIFI_IF"; then
           log "Connection attempts failed. Starting open Access Point mode ($AP_SSID)..."
           start_ap
           while true; do
               sleep 5
               if is_connected "$ETH_IF"; then
                   log "Ethernet ($ETH_IF) plugged in! Exiting AP mode..."
                   teardown_ap
                   break
               fi
           done
       fi
   done
   EOF
   sudo chmod +x /usr/local/bin/wifi-fallback.sh
   ```

3. **Create and enable `/etc/systemd/system/wifi-fallback.service`:**
   ```bash
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
   sudo systemctl status wifi-fallback.service --no-pager
   ```

---

## Step 7: Final Verification & Live HUD Access

Reboot the Pi (`sudo reboot`) and run the following checks:

1. **API & Authentication Check:**
   ```bash
   curl -I http://127.0.0.1:5000/api/mode
   # Expected: HTTP/1.1 401 UNAUTHORIZED

   curl -u ben:change-this-password http://127.0.0.1:5000/api/mode
   # Expected: JSON response containing "hailo_available": true
   ```

2. **Web HUD Access:**
   * **On LAN / Home Wi-Fi:** Navigate to `http://bombardeer:5000` (or `http://<pi-ip>:5000`).
   * **On Off-Grid Fallback AP:** Connect to the `bombardeer` Wi-Fi network and navigate explicitly to `http://10.42.0.1:5000`.
   * *Browser Note:* If Microsoft Edge aborts with `ERR_CONNECTION_ABORTED`, ensure no mobile VPN is active and disable Automatic HTTPS (`edge://flags/#edge-automatic-https`), or use Google Chrome.

3. **Live Telemetry & Post-Mortem Audit Snapshots:**
   * Follow real-time detection telemetry, safety vetoes, and tracking offsets in your terminal:
     ```bash
     journalctl -u bombardeer.service -f
     ```
   * Annotated JPEGs for wildlife acquisitions (`ACQUIRE_*.jpg`) and person/vehicle safety vetoes (`VETO_*.jpg`) are saved automatically under `~/Bombardeer/recordings/debug_snapshots/`.