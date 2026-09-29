# Target Measurement & Range Calibration Tool

`measure_target_range.py` is a standalone single-shot diagnostic utility that initializes the Raspberry Pi Camera Module, captures a single frame, detects a standard ArUco marker, measures its pixel dimensions, and validates distance estimation geometry.

---

## 🎯 What It Does

1. **Native Sensor Capture**: Configures `Picamera2` to capture a $1280 \times 720$ frame using on-sensor binning (`output_size: (2304, 1296)`)[cite: 4].
2. **Subpixel Geometry Extraction**: Locates the `DICT_4X4_50` marker (ID: 0) and computes the average pixel edge length across all four edges to minimize perspective distortion error[cite: 4].
3. **Pinhole Distance Estimation**: Evaluates the real-time distance estimate in both meters and feet:
   $$D = \frac{W_{\text{real}} \times f_{\text{px}}}{W_{\text{px}} \times 1000}$$
4. **Intrinsics Solving (Optional)**: If you provide a ground-truth physical tape-measured distance (`--distance`), the script solves for the exact optical focal length ($f_{\text{px}}$) and the milliradians-per-pixel ratio ($1000 / f_{\text{px}}$).

---

## 📋 Prerequisites

Ensure your system has the standard Bombardeer computer vision dependencies installed:

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-picamera2

```

Ensure no conflicting processes (like `bombardeer.py`) are actively using the camera:

```bash
pkill -f bombardeer.py

```

---

## 🚀 How to Use

### 1. Simple Distance Verification

Hold the $100\text{ mm}$ ($10\text{ cm}$) ArUco target at any test distance (e.g. 6 feet) facing the camera:

```bash
python3 measure_target_range.py

```

**Example Output:**

```text
[1/3] Initializing Camera Module via Picamera2...
[2/3] Capturing diagnostic frame...
[3/3] Processing ArUco geometry (DICT_4X4_50)...

============================================================
           TARGET MEASUREMENT RESULTS
============================================================
 Target Marker ID       : 0
 Measured Target Width  : 29.02 pixels
 Individual Edges       : T=29.1px, R=29.0px, B=29.0px, L=28.9px
------------------------------------------------------------
 Using Focal Length     : 540.4 px
 Target Physical Width  : 100.0 mm
 Estimated Distance     : 1.862 meters (6.11 feet)
============================================================

```

---

### 2. Calibrate & Solve New Lens Intrinsics

To calculate the exact focal length for a new lens or crop mode, place the $100\text{ mm}$ marker at an exact measured tape distance (e.g., $2.000\text{ meters}$) and supply the `--distance` argument:

```bash
python3 measure_target_range.py --distance 2.0

```

**Example Output:**

```text
============================================================
           TARGET MEASUREMENT RESULTS
============================================================
 Target Marker ID       : 0
 Measured Target Width  : 27.02 pixels
 Individual Edges       : T=27.0px, R=27.1px, B=27.0px, L=27.0px
------------------------------------------------------------
 Using Focal Length     : 540.4 px
 Target Physical Width  : 100.0 mm
 Estimated Distance     : 2.000 meters (6.56 feet)
------------------------------------------------------------
          CALIBRATED INTRINSICS (SOLVED)
------------------------------------------------------------
 Ground Truth Distance  : 2.000 meters (6.56 feet)
 Solved FOCAL_LENGTH_PX : 540.40
 Solved MRAD_PER_PIXEL  : 1.8505 mrad/px
============================================================

```

---

## 🔧 Updating `bombardeer.py` with Solved Values

Once `measure_target_range.py --distance <METERS>` outputs the **`CALIBRATED INTRINSICS (SOLVED)`** block, transfer those values into `bombardeer.py` to keep targeting calculations, parallax convergence, and HUD telemetry aligned.

1. Open `bombardeer.py` in your text editor:
```bash
nano /home/ben/Bombardeer/bombardeer.py

```


2. Locate the **`Configuration & Hardware Calibration`** section near line 65:


```python
# Camera Optical Intrinsics (RPi Cam Module 3 Wide NoIR @ 1280x720 native crop)
FOCAL_LENGTH_PX = 540.4
MRAD_PER_PIXEL_X = 1000.0 / FOCAL_LENGTH_PX  # ~1.8505 mrad/px
MRAD_PER_PIXEL_Y = 1000.0 / FOCAL_LENGTH_PX  # ~1.8505 mrad/px
```[cite: 4]


```


3. Update `FOCAL_LENGTH_PX` with the `Solved FOCAL_LENGTH_PX` value printed by the calibration tool:
```python
# Replace with the exact solved focal length from measure_target_range.py
FOCAL_LENGTH_PX = 540.40
MRAD_PER_PIXEL_X = 1000.0 / FOCAL_LENGTH_PX  # Automatically recalculates mrad/px
MRAD_PER_PIXEL_Y = 1000.0 / FOCAL_LENGTH_PX
```[cite: 4]


```


4. Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X` in nano).
5. Restart the Bombardeer service or script:
```bash
python3 /home/ben/Bombardeer/bombardeer.py

```



---

## ⚙️ Command-Line Arguments

| Flag | Type | Default | Description |
| --- | --- | --- | --- |
| `--distance` | `float` | `None` | Ground-truth distance from camera lens to target in **meters**. Triggers focal length solving. |
| `--width` | `float` | `100.0` | Physical outer black edge width of the marker in **millimeters**.

 |
| `--focal-length` | `float` | `540.4` | Assumed optical focal length in pixels (540.4 for IMX708 Wide NoIR).

 |

---

## 🔍 Troubleshooting

* **`[FAIL] No ArUco marker detected!`**:
Ensure the marker belongs to the `DICT_4X4_50` dictionary and has sufficient lighting without reflections or glare on glossy paper.


* **Camera Resource Busy**:
If the script reports that the camera device is busy, run `pkill -f python3` to terminate any hanging background instances of `bombardeer.py`.
