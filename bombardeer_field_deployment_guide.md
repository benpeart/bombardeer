# Bombardeer — Field Deployment & Calibration Guide

This guide covers everything required to deploy, calibrate, and verify the **Bombardeer** turret at a new field location. Always complete all measurements, range checks, and safety veto tests in **`TRACK` (`ON`)** or **`1-SHOT` (`CALIBRATE`)** mode before ever arming the system.

---

## 1. Physical Site Setup & Leveling

1. **Position & Anchor the Mount:**
   * Place the turret on a rigid, level tripod or fixed post with a clear view of the wildlife approach corridor and a safe downrange backstop.
   * Confirm the pan/tilt head has unobstructed physical clearance across its software travel limits (`PAN`: $-3141$ to $+3141\text{ mrad}$; `TILT`: $-420$ to $+400\text{ mrad}$).
2. **Level the Base:**
   * Use a bubble level on the turret baseplate to level it in both pitch (front-to-back) and roll (side-to-side).
   * *Why this matters:* The ESP32 home angle `(Pan: 0, Tilt: 0)` is the reference horizon for ground-plane ranging (`CAMERA_MOUNT_HEIGHT_M` and `GROUND_SLOPE_MRAD`). If the base is tilted, every ground-plane range calculation will be skewed.
3. **Orient Scan Home `(0, 0)`:**
   * Point the turret's physical `(0, 0)` home direction toward the center of the expected target zone. After `HOME_TIMEOUT_SEC` ($60\text{ s}$) of inactivity, the turret automatically returns to `(0, 0)`.
4. **Power Up & Connect to the Web HUD:**
   * Power on the ESP32 controller and Raspberry Pi 5.
   * **On LAN / Wi-Fi:** Open `http://bombardeer:5000` (or `http://<pi-ip>:5000`).
   * **Off-Grid (No Wi-Fi/Ethernet):** Wait ~45 seconds for `wifi-fallback.service` to bring up the `bombardeer` Wi-Fi hotspot, connect your phone/laptop to `bombardeer`, and open `http://10.42.0.1:5000` (disable mobile VPNs and Edge Automatic HTTPS if prompted, or use Chrome).
   * Sign in with your configured Basic Auth credentials (`BOMBARDEER_USER` / `BOMBARDEER_PASS`) and confirm the HUD shows **`OFF / SAFE`** and **`SERIAL: OK`** in green at the bottom-left.

---

## 2. Measuring & Updating Site Geometry

In `WILDLIFE` mode, Bombardeer fuses **70% ground-plane trigonometry** with **30% target-height stadia** whenever the target's hooves (bottom of the bounding box) are visible above the bottom edge of the frame and the net depression angle is $\ge 25\text{ mrad}$ ($\approx 1.4^\circ$):

$$\text{net\_dep\_mrad} = \text{opt\_dep\_mrad} - \text{tilt\_at\_capture} - \text{GROUND\_SLOPE\_MRAD}$$

$$r_{\text{ground}} = \frac{\text{CAMERA\_MOUNT\_HEIGHT\_M}}{\sin(\text{net\_dep\_mrad} / 1000)}$$

Accurate values for `CAMERA_MOUNT_HEIGHT_M` and `GROUND_SLOPE_MRAD` ensure accurate range estimates, which directly drive ballistic drop and parallax compensation.

### A. Measure `CAMERA_MOUNT_HEIGHT_M`

Measure vertically from the ground directly beneath the turret to the **center of the IMX708 camera lens**.

* Convert inches to meters:
  $$\text{height}_{\text{m}} = \text{height}_{\text{in}} \times 0.0254$$
* *Example:* A lens mounted $45.3\text{ in}$ above the ground is $45.3 \times 0.0254 = 1.15\text{ m}$.

### B. Measure `GROUND_SLOPE_MRAD`

Measure the average slope of the ground **along the primary target corridor** moving away from the turret.

* **Sign Convention (Critical):**
  * **Positive (`+`)** if the terrain slopes **uphill** away from the turret.
  * **Negative (`-`)** if the terrain slopes **downhill** away from the turret.
  * **`0.0`** for flat ground.
* **Method 1 — Digital Inclinometer / Phone Level App:**
  Sight along the ground slope away from the turret in degrees, then convert to milliradians:
  $$\text{GROUND\_SLOPE\_MRAD} = \text{slope}_{\text{deg}} \times 17.453$$
  *(Example: a $2.5^\circ$ downhill slope is $-2.5 \times 17.453 \approx -43.6\text{ mrad}$.)*
* **Method 2 — Rise Over Run:**
  Measure the vertical elevation change ($\Delta h$) over a known horizontal distance ($D$, in the same units):
  $$\text{GROUND\_SLOPE\_MRAD} = \frac{\Delta h}{D} \times 1000$$
  *(Example: ground dropping $1.5\text{ ft}$ over $50\text{ ft}$ is $\frac{-1.5}{50} \times 1000 = -30.0\text{ mrad}$.)*

### C. Update `bombardeer.py` & Restart

SSH into the Pi, edit the configuration constants near the top of `/home/ben/bombardeer/bombardeer.py`:

```bash
nano /home/ben/bombardeer/bombardeer.py
```

Update these lines:

```python
CAMERA_MOUNT_HEIGHT_M = 1.15   # Measured lens height above ground (meters)
GROUND_SLOPE_MRAD = 0.0        # + uphill away from turret, - downhill away from turret
```

Verify syntax and restart the service:

```bash
python3 -m py_compile /home/ben/bombardeer/bombardeer.py
sudo systemctl restart bombardeer.service
```

### D. Verify Range Accuracy in the Field

1. Measure a known reference distance downrange (e.g., $40\text{ ft}$ / $12.2\text{ m}$) with a tape measure or laser rangefinder.
2. In the Web HUD, verify the **ArUco** range first (using a $365\text{ mm}$ `DICT_4X4_50` ID `0` marker at that exact spot) and check the top-right `TGT: XX.Xft (XX.Xm)` card.
3. For `WILDLIFE` ground-plane verification, place a wildlife decoy (or observe a real animal) at a known landmark distance in `WILDLIFE` + `TRACK` mode and confirm the HUD `TGT` distance matches within $\approx 10\%$.
   * If the reported range is consistently **too short** on sloped terrain, double-check the sign of `GROUND_SLOPE_MRAD`.
   * If the reported range is off across all distances on flat ground, re-measure `CAMERA_MOUNT_HEIGHT_M` and confirm the turret base is level.

---

## 3. Ballistic Zeroing (`TILT_CALIBRATION_TRIM_MRAD`)

`TILT_CALIBRATION_TRIM_MRAD` compensates for mechanical barrel-to-camera bore offset and real-world ballistic velocity differences. It is added directly to the commanded tilt angle:

$$\text{world\_tilt} = \text{base\_tilt} + \text{lead\_tilt} + \text{drop\_mrad} + \text{TILT\_CALIBRATION\_TRIM\_MRAD}$$

* **More negative** (`-`) $\rightarrow$ **lowers** the point of impact (POI).
* **More positive** (`+`) $\rightarrow$ **raises** the point of impact (POI).

### Step-by-Step 1-Shot Zeroing Procedure

1. **Set Up the Calibration Target:**
   * Mount a flat, printed **ArUco `DICT_4X4_50` ID `0`** marker measuring **$365\text{ mm}$** wide onto a cardboard/foam backstop at your primary engagement distance (recommended: **$40\text{ ft}$ / $12.2\text{ m}$**).
   * Mark the exact center of the ArUco marker as your Point of Aim (POA).
2. **Acquire in Safe Tracking Mode:**
   * In the Web HUD, select **`ArUco Target (Test)`** as the Perception Engine.
   * Click **`TRACK` (`ON`)**. Confirm the green box locks onto `ID:0`, the turret smoothly centers the target inside the reticle, and the top-right `TGT` range matches your measured distance.
3. **Fire a Single Calibration Shot:**
   * Ensure downrange is completely clear.
   * Click **`1-SHOT` (`CALIBRATE`)**. The badge changes to `CAL: PRIMED` and the turret will automatically discharge **one** $80\text{ ms}$ pulse once target error is within the fire deadband and slew speed is stabilized.
   * Click **`RE-PRIME`** to fire 2–3 shots for a reliable group average, then click **`OFF`** before walking downrange.
4. **Calculate the `TILT_CALIBRATION_TRIM_MRAD` Adjustment:**
   Measure the vertical distance between the center of your impact group (POI) and the center of the ArUco marker (POA):

   $$\Delta\text{trim}_{\text{mrad}} = \frac{\text{vertical error}}{\text{target distance}} \times 1000$$

   * Use the **same units** for vertical error and target distance (both inches, or both meters).
   * If impacts are **HIGH**, **subtract** $\Delta\text{trim}_{\text{mrad}}$ from `TILT_CALIBRATION_TRIM_MRAD`.
   * If impacts are **LOW**, **add** $\Delta\text{trim}_{\text{mrad}}$ to `TILT_CALIBRATION_TRIM_MRAD`.

   **Quick Reference Table (mrad per $1\text{ inch}$ of vertical POI shift):**

   | Target Range | Distance in Inches | $\text{mrad}$ per $1\text{ in}$ Shift | $\text{mrad}$ per $4\text{ in}$ Shift |
   | :--- | :--- | :--- | :--- |
   | $30\text{ ft}$ ($9.1\text{ m}$) | $360\text{ in}$ | $2.78\text{ mrad}$ | $11.1\text{ mrad}$ |
   | $40\text{ ft}$ ($12.2\text{ m}$) | $480\text{ in}$ | $2.08\text{ mrad}$ | $8.3\text{ mrad}$ |
   | $50\text{ ft}$ ($15.2\text{ m}$) | $600\text{ in}$ | $1.67\text{ mrad}$ | $6.7\text{ mrad}$ |
   | $75\text{ ft}$ ($22.9\text{ m}$) | $900\text{ in}$ | $1.11\text{ mrad}$ | $4.4\text{ mrad}$ |

   *Example:* Currently `TILT_CALIBRATION_TRIM_MRAD = -8.3`. At $40\text{ ft}$, your group hits $3\text{ in}$ **high**. Adjustment $= 3 \times 2.08 = 6.24\text{ mrad}$ downward, so the new value is $-8.3 - 6.2 = -14.5$.

5. **Apply & Confirm:**
   * Update `TILT_CALIBRATION_TRIM_MRAD` in `/home/ben/bombardeer/bombardeer.py`, check syntax, and restart:
     ```bash
     python3 -m py_compile /home/ben/bombardeer/bombardeer.py
     sudo systemctl restart bombardeer.service
     ```
   * Fire one confirmation `1-SHOT` group to verify POI matches POA.
   * *Note on Horizontal (Windage) Error:* If impacts are consistently left or right across all ranges, verify that the camera housing is physically parallel to the barrel and that `CAMERA_OFFSET_X_MM` (currently `-170.0`, camera $170\text{ mm}$ left of bore) and `CAMERA_OFFSET_Y_MM` (`-60.0`, camera $60\text{ mm}$ below bore) still match your hardware mount.

---

## 4. Pre-Arming Safety & Functional Verification Checklist

Before leaving the turret unattended or switching to `ARMED`, run through every item below in **`TRACK` (`ON`)** mode.

### 1. Verify Wildlife NPU Is Active
* Click **`Wildlife NPU (Hailo-8)`** in the Web HUD.
* Confirm the bottom-left green telemetry bar changes from `DET:ARUCO` to **`NPU:XX.XHz(XXms)`** and the `FPS` readout is actively updating.

### 2. Person & Vehicle Exclusion Veto Test (Mandatory)
* Keep the turret in **`TRACK` (`ON`)** mode (**never** `ARMED`).
* Walk across the field of view and, if roads/driveways are in view, drive a vehicle across the field of view.
* **Pass Criteria:**
  * Red bounding boxes labeled **`VETO:PERSON`** / **`VETO:VEHICLE`** appear on the HUD.
  * The top-right range card stays **`TGT: NONE`**.
  * The turret does **not** slew toward or lock onto the person or vehicle.
  * Check `journalctl` and `recordings/debug_snapshots/` to confirm any stray candidate anchors were blocked with `[AUDIT:REJECTED]` or `[SAFETY VETO:ACTIVE]`:
    ```bash
    sudo journalctl -u bombardeer.service -n 40 --no-pager
    ls -lh /home/ben/bombardeer/recordings/debug_snapshots/
    ```

### 3. Auto-Home `(0, 0)` Verification
* While in **`TRACK` (`ON`)** mode, briefly present an ArUco target off-center so the turret slews away from `(0, 0)`, then remove the target (or switch back to `WILDLIFE` with an empty scene).
* Wait **$60\text{ seconds}$** (`HOME_TIMEOUT_SEC`).
* **Pass Criteria:**
  * The turret smoothly slews back to `Pan: 0, Tilt: 0`.
  * `journalctl` logs:
    ```text
    [ON] Idle for 60.0s at (...). Returning turret to scan home (0,0).
    [ON] Returned to scan home (0,0).
    ```

### 4. Storage & Low-Light ISP Check
* Confirm disk headroom is healthy (warnings trigger below $5.0\text{ GB}$ free; old `wildlife_*` recordings auto-purge above $10.0\text{ GB}$ and `debug_snapshots` auto-cap at $200$ JPEGs).
* If deploying for dawn/dusk operation, confirm in `journalctl` that `[ISP] Dawn/Dusk Low-Light Mode ACTIVE` transitions cleanly when ambient `Lux < 25.0` or `Gain > 6.0`.

---

## 5. Arming & Emergency Disarming

1. **Final Downrange Sweep:** Confirm no people, vehicles, or pets are in the field of view and that all calibration tests passed.
2. **Arming Sequence:**
   * Ensure the Perception Engine is set to **`Wildlife NPU (Hailo-8)`** and mode is **`TRACK` (`ON`)**.
   * Tap the striped **`LIFT TO ARM`** safety cover, then tap **`ARM LIVE`** within $4\text{ seconds}$.
   * Confirm the HUD watermark turns red (**`ARMED / LIVE`**).
3. **Disarming / Safe Shutdown:**
   * Tap **`DISARM`** (returns to `TRACK` safe mode) or **`OFF`** (halts motors immediately and cuts trigger authorization).
   * In fullscreen view, use the green **`SAFE / DISARM`** button in the top-left corner for immediate disengagement.