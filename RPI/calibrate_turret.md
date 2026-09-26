# 🎯 Bombardeer Automated Optical-to-Mechanical Calibration Suite

`calibrate_turret.py` is an automated calibration script that computes the precise conversion ratios between **camera pixel displacements** and **stepper motor steps** for both the Pan and Tilt axes.

By sweeping the turret across a known step grid while continuously tracking a stationary target, the script uses linear regression to calculate:

* `STEPS_PER_PIXEL_PAN`

* `STEPS_PER_PIXEL_TILT`

* Coordinate inversion flags (`PAN_INVERT`, `TILT_INVERT`)



The results are automatically saved as JSON to `calibration_data.json` for ingestion by downstream targeting software.

---

## ⚙️ How It Works

1. **Hardware Link & Zeroing:** Connects to the ESP32 via serial (`115200` baud) and zeroes the mechanical coordinates using the `H` command.


2. **Camera & Perception:** Captures $1280 \times 720$ video using native `Picamera2` (with horizontal and vertical sensor flip applied). It enhances grayscale frames with CLAHE (Contrast Limited Adaptive Histogram Equalization) and locates a standard ArUco marker (`DICT_4X4_50`).


3. **Motion Grid Sweep:**
* **Pan Axis:** Sweeps through `[-1200, -600, 0, 600, 1200]` steps.


* **Tilt Axis:** Sweeps through `[-1000, -500, 0, 500, 1000]` steps.




4. **Vibration Dampening & Motion Queries:** Uses `Q` commands to poll the ESP32 until the motors report completion (`R ... 0`), adding a $150\text{ ms}$ mechanical settling delay before reading pixel positions.


5. **Multi-Frame Median Centroiding:** Captures 5 frames at each waypoint and calculates the median $X$ and $Y$ coordinates to eliminate optical noise and micro-jitter.


6. **Regression Analysis:** Computes a first-degree polynomial fit (`numpy.polyfit`) correlating motor step counts to pixel coordinates.



---

## 🧰 Prerequisites

* **Hardware:**
* Bombardeer Turret connected to the Raspberry Pi via USB serial (`/dev/ttyUSB0` or `/dev/ttyACM0`).


* Raspberry Pi Camera Module aligned with the turret.


* One printed **ArUco marker** from the `DICT_4X4_50` dictionary.




* **System Packages & Python Dependencies:**
```bash
sudo apt install -y python3-opencv python3-serial python3-numpy python3-picamera2

```



---

## 📋 Step-by-Step Instructions

### Step 1: Physical Setup

1. Mount or place the turret on a stable, vibration-free surface with complete clearance to rotate.


2. Place the printed ArUco marker directly in front of the camera, approximately **$1.5\text{ to }3\text{ meters}$** away.


3. Ensure the marker is roughly centered in the camera's line of sight and well-illuminated without glare.



### Step 2: Stop Conflicting Processes

Ensure no other services or scripts (such as `bombardeer.py`) are using the camera or serial port:

```bash
pkill -f bombardeer.py

```

### Step 3: Run the Calibration Script

Execute the script with Python 3:

```bash
python3 calibrate_turret.py

```

1. The script will initialize the camera, locate the serial port, and print the confirmation prompt:


```text
=======================================================
    BOMBARDEER AUTOMATED CALIBRATION SYSTEM
=======================================================
1. Place an ArUco marker directly in view of the turret.
2. Ensure the turret has mechanical clearance to move.

Press ENTER when the marker is placed and visible...

```


2. Confirm the marker is visible and unobstructed, then press **Enter**.



### Step 4: Automated Execution

* **Target Check:** Verifies marker detection across 8 sample frames and records the initial optical center position.


* **Origin Setting:** Sends `H` to reset the starting point to $(0, 0)$.


* **Pan Sweeps:** Moves the Pan axis through 5 positions while keeping Tilt at $0$, recording the pixel $X$ position at each stop.


* **Tilt Sweeps:** Returns Pan to $0$ and sweeps the Tilt axis through 5 vertical positions, recording the pixel $Y$ position.


* **Home Return:** Returns both axes to $(0, 0)$.



### Step 5: Verify the Output

Upon completion, the final steps-per-pixel ratios are displayed:

```text
=======================================================
              CALIBRATION COMPLETE                     
=======================================================
 Pan Axis Ratio  : 3.421 steps / pixel
 Tilt Axis Ratio : 25.814 steps / pixel
=======================================================
[Success] Configuration saved to /home/ben/Bombardeer/calibration_data.json

```

The output file (`/home/ben/Bombardeer/calibration_data.json`) is populated with the calibration results:

```json
{
    "timestamp": 1727375519.123,
    "STEPS_PER_PIXEL_PAN": 3.421,
    "STEPS_PER_PIXEL_TILT": 25.814,
    "PAN_INVERT": false,
    "TILT_INVERT": false
}

```

---

## 🔍 Troubleshooting

* **`[Error] Could not locate ESP32 serial port. Exiting.`**
* Verify the USB cable is plugged in and recognized using `ls /dev/ttyUSB* /dev/ttyACM*`.


* Ensure your user has dialout privileges: `sudo usermod -a -G dialout $USER`.


* **`[ERROR] No ArUco marker detected! Check alignment and lighting.`**
* Confirm the marker belongs to `DICT_4X4_50`.


* Improve ambient lighting or move the marker closer to the camera.




* **`[Warning] Marker lost at pan/tilt step X. Skipping point.`**
* The calibration step offsets (`PAN_OFFSETS` or `TILT_OFFSETS`) moved the camera so far that the marker went out of the frame.


* Move the marker farther away from the turret to widen the effective field of view, or reduce the offset values in `PAN_OFFSETS` / `TILT_OFFSETS` at the top of `calibrate_turret.py`.




* **`[ERROR] Insufficient data points collected to perform calibration.`**
* The script requires at least 3 valid marker readings per axis to perform regression. Ensure the marker stays inside the frame during the sweeps.