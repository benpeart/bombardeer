# Raspberry Pi 5 Setup Guide: Bombardeer Vision System

This document provides the verified, clean deployment guide for setting up the Raspberry Pi 5 hardware, PCIe Gen 3 acceleration, Hailo-8 runtime, camera ISP pipelines, and web HUD streaming interface for the Bombardeer project using the assets in this repository.

---

### Hardware Requirements

* **SBC:** Raspberry Pi 5 (Active Cooler recommended)
* **AI Accelerator:** Hailo-8 M.2 Module mounted via M.2 HAT+
* **Camera:** Raspberry Pi Camera Module 3 (IMX708 NoIR / Wide) connected to `CAM/DISP 0` or `1`
* **OS:** Raspberry Pi OS 64-bit (Debian 12 Bookworm / Linux Kernel 6.12+)
* **Default User:** `ben`

---

### Step 1: Base OS Update & Camera Verification

Update the system packages:

```bash
sudo apt update && sudo apt full-upgrade -y

```

Verify that the kernel and the Raspberry Pi Image Signal Processor (PiSP) recognize the camera sensor:

```bash
rpicam-hello --list-cameras

```

*Expected detection:* Camera detected as `imx708` or `imx708_wide_noir` on `/base/axi/pcie@...`.

---

### Step 2: Enable PCIe Gen 3 Hardware Link

Raspberry Pi OS limits the single PCIe lane to Gen 2 (5 GT/s) by default. Enable PCIe Gen 3 (8 GT/s) to ensure uninterrupted data transfer:

```bash
# Append Gen 3 parameter to firmware boot config
echo "dtparam=pciex1_gen=3" | sudo tee -a /boot/firmware/config.txt

# Reboot to renegotiate the hardware link
sudo reboot

```

After the Pi boots back up, confirm the link negotiated at 8 GT/s:

```bash
sudo lspci -vv -d 1e60: | grep -i "LnkSta:"

```

*Required output:* `LnkSta: Speed 8GT/s, Width x1 (downgraded)`

---

### Step 3: Install HailoRT & Python Runtime Dependencies

Install the official Hailo-8 kernel drivers, runtime CLI, Python platform bindings, and system web packages using the Debian package manager:

```bash
# 1. Install Hailo runtime and PCIe kernel driver
sudo apt install -y hailofw hailo-all

# 2. Install computer vision and web server runtime
sudo apt install -y python3-pip python3-flask python3-opencv python3-numpy python3-picamera2

# 3. Ensure picamera2 is present and up to date
sudo apt install -y python3-picamera2

```

Verify that the Hailo PCIe driver is loaded and the NPU responds:

```bash
hailortcli fw-control identify

```

---

### Step 4: Clone the Repository

Clone your repository into the user's home folder:

```bash
cd ~
git clone <YOUR_GIT_REPO_URL> Bombardeer
cd ~/Bombardeer

```

---

### Step 5: Verify Model Benchmark on Hardware

Test the pre-compiled `MDV6-yolov9-c.hef` on the Hailo-8 NPU to ensure proper hardware allocation:

```bash
hailortcli benchmark ~/Bombardeer/models/MDV6-yolov9-c.hef

```

*Expected baseline performance:*

* **Throughput:** ~`24.66 FPS`
* **Hardware Latency:** ~`37.7 ms`

---

### Step 6: Install & Enable the Systemd Auto-Start Service

Install the service file included in the repository so the vision system starts automatically on boot:

```bash
# 1. Copy service file into systemd
sudo cp ~/Bombardeer/systemd/bombardeer.service /etc/systemd/system/

# 2. Reload daemon, enable on boot, and start immediately
sudo systemctl daemon-reload
sudo systemctl enable bombardeer.service
sudo systemctl start bombardeer.service

```

Check service status to verify the camera and NPU streams initialized cleanly:

```bash
sudo systemctl status bombardeer.service

```

---

### Step 7: Access the Live HUD

From any computer or smartphone on the local network, open a web browser and visit:

```text
http://turret.local:5000

```

*(or via IP: `[http://192.168.2.199:5000](http://192.168.2.199:5000)`)*

The HUD interface will display:

* Live 640×640 camera stream processed directly by the Raspberry Pi hardware ISP.
* Centered crosshairs at optical zero `(320, 320)`.
* Green bounding boxes around detected animals with tracking displacement vectors `(dx, dy)`.
* **Bombardeer** custom favicon in your browser tab.
* Sustained ~24.6 FPS execution across all 4 Hailo-8 hardware contexts.

To follow real-time detection telemetry and tracking offsets in your terminal:

```bash
journalctl -u bombardeer.service -f

```