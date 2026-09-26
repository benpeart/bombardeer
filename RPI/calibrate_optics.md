# Bombardeer Optical Focal Length Calibration Suite

`calibrate_optics.py` is an automated calibration utility designed to calculate the exact optical focal lengths (`fx` and `fy`) of the camera in pixels. It bridges the physical kinematics of the Bombardeer pan-tilt turret with camera pixel coordinates, ensuring that vision-based targeting and parallax compensation remain accurate.

---

## What It Does

The script executes controlled, bidirectional angular sweeps on the turret while tracking a stationary ArUco marker:

* **Large Angular Baselines:** Commands $320\text{ mrad}$ ($\sim 18.3^\circ$) on the pan axis and $140\text{ mrad}$ ($\sim 8.0^\circ$) on the tilt axis to generate large pixel shifts and minimize measurement error.


* **Mechanical Settling:** Monitors ESP32 telemetry status packets (`S ...`) for motion completion, then enforces a $2.5\text{ s}$ pause to let vibrations and gear resonance die down.


* **Statistical Filtering:** Captures $30$ frames at every stopping point, applying subpixel corner refinement and a $2\sigma$ outlier filter to reject jitter.


* **Backlash Compensation:** Sweeps both forward and backward across both axes, averaging the forward/reverse results and calculating gear backlash in pixels.


* **Pinhole Projection Solver:** Uses binary search to solve the non-linear angular displacement equation:

$$\left\vert{}\arctan\left(\frac{p_1 - c}{f}\right) - \arctan\left(\frac{p_2 - c}{f}\right)\right\vert{} = \theta_{\text{target}}$$

---

## Hardware & Environment Prerequisites

* **Turret Hardware:** Connected to the host via USB serial (`/dev/ttyUSB0`, `/dev/ttyACM0`, or `/dev/ttyUSB1`) running the ESP32 controller firmware at `115200` baud.


* **Camera:** Raspberry Pi Camera Module configured via `picamera2` (native $1280 \times 720$ mode).


* **Target Marker:** A printed **ArUco marker** from the `DICT_4X4_50` dictionary with **ID: 0**.



---

## Step-by-Step Instructions

### Step 1: Position the Turret and Target

1. Secure the turret on a rigid, stable surface to prevent base slipping during rapid motion.
2. Affix the ArUco marker (ID: `0`) to a stationary, vertical surface approximately $2\text{ to }5\text{ meters}$ away.


3. Ensure the marker is well lit and completely unobstructed.



### Step 2: Set Initial Marker Alignment

Because the script commands a positive pan sweep of $+320\text{ mrad}$, the camera rotates to the right, causing the marker in the video frame to move toward the right side of the screen.

* Manually align the turret so the marker starts on the **left side of the camera's field of view** ($X \approx 350\text{–}450\text{ px}$).


* Vertically, keep the marker slightly **above center** ($Y \approx 250\text{–}320\text{ px}$) to prevent it from sweeping off the bottom edge during the vertical displacement.



### Step 3: Run the Script

Execute the script from the Raspberry Pi terminal:

```bash
python3 calibrate_optics.py

```

The script will detect the serial port, initialize the camera, and wait for confirmation:

```text
[ALIGNMENT INSTRUCTIONS]
Place the ArUco marker toward the LEFT side of the screen (X ~= 350-450 px).
This allows the turret to execute a large 320 mrad sweep without losing it.

Press ENTER once the target is placed and camera has an unobstructed view...

```

Press **Enter** once the marker is confirmed in position.

### Step 4: Automated Calibration Sequence

The script runs the calibration cycle automatically without manual intervention:

1. **Position A (Baseline):** Samples 30 frames to establish the starting marker centroid.


2. **Pan Sweep (+320 mrad):** Moves to Position B, lets mechanical vibration settle, and computes forward `fx`.


3. **Pan Return (-320 mrad):** Moves back to Position A', calculates reverse `fx`, and outputs pan backlash.


4. **Tilt Sweep (+140 mrad):** Moves downward to Position C, settles, and computes forward `fy`.


5. **Tilt Return (-140 mrad):** Returns to baseline, computes reverse `fy`, and outputs tilt backlash.



### Step 5: Apply Calibrated Values

When the sweep completes, the script displays the averaged results:

```text
======================================================================
 OPTICAL CALIBRATION CONVERGED SUCCESSFULLY
======================================================================
Update bombardeer.py with these values:

FOCAL_LENGTH_X_PX = 985.50
FOCAL_LENGTH_Y_PX = 982.10
======================================================================

```

Copy the computed `FOCAL_LENGTH_X_PX` and `FOCAL_LENGTH_Y_PX` values into `bombardeer.py` and `turret_test.py`.

---

## Troubleshooting

* **`[ERROR] Could not connect to ESP32.`**
Ensure the ESP32 is connected via USB, permissions are granted (`sudo usermod -a -G dialout $USER`), and no background process (such as `bombardeer.py`) is occupying the serial port.


* **`[ERROR] Could not acquire target at start. Aborting.`**
Verify the marker is printed from `DICT_4X4_50` with `ID: 0` and that ambient lighting is sufficient.


* **`[ERROR] Lost marker! It may have swept off screen.`**
The marker was placed too close to the center or right edge at the start. Move the marker further left ($X \approx 350\text{ px}$) or reduce `TEST_PAN_MRAD` if working with a narrow field of view.


* **`[WARN] Jitter (...) px is high.`**
Ensure both the turret mount and the marker backing are rigidly stationary and not vibrating or flexing in the wind.