# Bombardeer Wildlife Deterrent Turret: System Architecture Specification

**Bombardeer** is an open-source, vision-guided autonomous deterrent turret designed to protect outdoor spaces, gardens, orchards, and agricultural property from intrusive wildlife (such as deer).

<img src="images/bombardeer.jpg" alt="Bombardeer: Autonomous Vision-Guided Deterrent Turret" width="480"  />

Powered by a **Raspberry Pi 5** for real-time visual perception, range estimation, and dynamic parallax compensation, paired with a dedicated **ESP32** microcontroller executing hardware-timed pan/tilt motion profiles across dual TMC2209 silent stepper drivers.

---

## 🚀 Features

* **Real-Time Predictive Edge Tracking:** Raspberry Pi 5 runs low-latency visual tracking with a constant-velocity Kalman filter and dynamic slew-rate limiting to eliminate overshoot and stalls.
* **Dynamic Range Estimation & Parallax Convergence:** Computes target distance via pinhole projection (ArUco in test mode, bounding-box height in production) and dynamically converges the bore sightline to compensate for the physical camera offset ($170\text{ mm}$ left, $60\text{ mm}$ below the bore).
* **Native Milliradian (mrad) Motion Protocol:** Host communicates velocity trims (`mrad/s`) and positions (`mrad`), allowing complete decoupling from driver microstepping and hardware gear reductions.


* **Precision Kinematic Reductions:**
* **Pan Axis:** Custom $6.65:1$ timing ring drive ($246\text{T} / 37\text{T}$) achieving up to $11{,}600\text{ steps/s}$ ($\approx 3{,}425\text{ mrad/s}$).


* **Tilt Axis:** Anti-backdrive $50:1$ single-start worm drive ($50\text{T} / 1\text{T}$) achieving up to $14{,}800\text{ steps/s}$ ($\approx 580\text{ mrad/s}$) with zero backdrive.




* **Dual Control Modes with Clean Handoff:**
* **Autonomous Mode:** High-speed streaming velocity trims (`V <pan_s> <tilt_s>`) and waypoint displacements (`M <d_p> <d_t>`) over UART.


* **Manual Override:** Wireless Xbox Series X/S controller integration via Bluetooth with quadratic stick response curves, fail-safe disconnect clamping, and single-shot coordinate re-zeroing ('A' button).




* **TMC2209 UART Integration:** Dynamically manages current limits ($1200\text{ mA}$ Pan, $1100\text{ mA}$ Tilt), microstepping ($1/16$ interpolated to $1/256$), standstill `ihold` power saving, and automatic StealthChop-to-SpreadCycle transitions.


* **Non-Blocking Solenoid Actuator:** Dedicated hardware state machine driving a Heschen HS-1564B 24V solenoid ($60\text{ ms}$ pulse / $190\text{ ms}$ cooldown; $4.0\text{ BPS}$) with automated safety shutoffs.



---

## 🛠️ System Architecture

```
       +---------------------------------------------+
       |             Raspberry Pi 5                  |
       |  Perception, Kinematic Projection & Safety  |
       +---------------------------------------------+
                              |
                              |  Bidirectional UART (921600 Baud)
                              |  Packets in Milliradians (mrad / mrad/s)
                              v
       +---------------------------------------------+    +-------------------------------+
       |             ESP32 Microcontroller           |<---|   Xbox Series X Controller    |
       |  Motion Planning & Low-Level Actuation      |    |       (Bluetooth Manual)      |
       +---------------------------------------------+    +---------------+---------------+
               /              |              \
              v               v               v
     +----------------+ +----------------+ +--------------------+
     | TMC2209 / Pan  | | TMC2209 / Tilt | | 24V Firing Circuit |
     | Stepper Motor  | | Stepper Motor  | | Heschen Solenoid   |
     +----------------+ +----------------+ +--------------------+

```

---

## 🧰 Hardware Requirements

Here is the updated **Hardware Requirements** table including the Hailo-8 AI accelerator:

| Component | Specification | Description |
| --- | --- | --- |
| **Compute Board** | Raspberry Pi 5 (8GB) | Target tracking, HUD streaming, and telemetry logging |
| **AI Accelerator** | Hailo-8 M.2 Module (26 TOPS) | Low-latency neural network inference for real-time target detection |
| **Vision Sensor** | RPi Camera Module 3 (Wide/Standard) | $1280 \times 720$ native crop @ $30\text{ FPS}$ low-light capture |
| **Motion MCU** | ESP32-WROOM-32 | Multi-axis hardware timer pulse generation & safety watchdogs |
| **Stepper Drivers** | $2\times$ TMC2209 (v1.2+) | Single-wire UART addressing (`0b00` Pan, `0b01` Tilt) |
| **Stepper Motors** | $2\times$ STEPPERONLINE 17HS19-2004S1 | NEMA 17 ($2.0\text{A}$ Peak, $59\text{ N}\cdot\text{cm}$ Holding Torque) |
| **Pan Drive** | $6.65:1$ GT2 Timing Belt Reduction ($246\text{T} / 37\text{T}$) | High-slew azimuth tracking ($3.386\text{ steps/mrad}$) |
| **Tilt Drive** | $50:1$ Anti-Backdrive Worm Gear ($50\text{T} / 1\text{T}$) | Self-locking elevation drive ($25.465\text{ steps/mrad}$) |
| **Actuator** | Heschen HS-1564B / HS-4564B Solenoid | 24V push-pull trigger actuation ($4.0\text{ BPS}$) |
| **Mounting Offset** | $170\text{ mm}$ Left, $60\text{ mm}$ Below Bore Centerline | Baseline camera-to-barrel offset compensated in software |
| **Power Supply** | 12V–24V LiFePO4 / Battery Bank | Powers steppers, solenoid circuit, and logic step-downs |

---

## Subsystem Roles## Subsystem Roles

### 1. Vision & Trajectory Planning (Raspberry Pi 5)

* **Sensor Capture:** RPi Camera Module 3 operating via `Picamera2` in native $1280 \times 720$ resolution[cite: 1].

* **Perception Engine:** CLAHE-enhanced subpixel ArUco marker detector (`DICT_4X4_50`) for diagnostic calibration and YOLO-based neural inference for wildlife tracking[cite: 1, 5].

* **Dynamic Range Estimation:** Computes metric range $D$ using pinhole projection geometry:

$$D = \frac{W_{\text{real}} \times f_x}{W_{\text{px}}}$$

* **Dynamic Parallax Compensation:** Adjusts angular setpoints based on estimated distance $D$ (in millimeters) to align the physical barrel with the camera sightline:

$$\Delta \theta_{\text{pan}} = \frac{-\text{OffsetX}}{D} \times 1000 = \frac{170\text{ mm}}{D} \times 1000 \quad [\text{mrad}]$$

$$\Delta \theta_{\text{tilt}} = \frac{\text{OffsetY}}{D} \times 1000 = \frac{-60\text{ mm}}{D} \times 1000 \quad [\text{mrad}]$$

* **Milliradian Projection:** Converts pixel tracking errors into angular tracking errors:

$$\text{Error}_{\text{pan}} = -(\text{target}_x - c_x) \times K_{\text{pan}} + \Delta \theta_{\text{pan}}$$

$$\text{Error}_{\text{tilt}} = -(\text{target}_y - c_y) \times K_{\text{tilt}} + \Delta \theta_{\text{tilt}}$$

* **Velocity Servoing & Slew Limiter:** Shapes proportional tracking demands into acceleration-safe velocity commands (`V <pan_s> <tilt_s>`), enforcing maximum speed ceilings and per-frame acceleration clamps to prevent motor stalls.


* **Diagnostic HUD & Stream:** Flask-based real-time HUD streaming ($640 \times 360$) displaying optical center crosshairs, dynamic bore-convergence impact markers, live telemetry, and automated event video recording.

### 2. Motion & Actuation Engine (ESP32)

* **Hardware Motion Profiler:** `FastAccelStepper` dynamically executes trapezoidal acceleration profiles on ESP32 hardware timer interrupts ($13{,}500\text{ steps/s}^2$ Pan, $50{,}000\text{ steps/s}^2$ Tilt).


* **Milliradian-to-Step Translation:** Converts incoming integer milliradians directly to stepper motor step coordinates using compile-time geometric ratios:



$$\text{Pan Steps} = \text{Angle [mrad]} \times \left(\frac{3200 \times (246 / 37)}{2000\pi}\right) \approx \text{Angle [mrad]} \times 3.3863$$

$$\text{Tilt Steps} = \text{Angle [mrad]} \times \left(\frac{3200 \times (50 / 1)}{2000\pi}\right) \approx \text{Angle [mrad]} \times 25.4648$$

* **Driver Control:** Dual TMC2209 silent stepper drivers managed over a shared single-wire UART bus (`Serial2`) with read-back validation, register reset auto-recovery, and dynamic hybrid StealthChop/SpreadCycle transitions.


* **Actuation State Machine:** Hardware-timed driver managing the Heschen firing solenoid ($60\text{ ms}$ pulse / $190\text{ ms}$ cooldown) with a hard autonomous firing cutoff of $2000\text{ ms}$.


* **Safety & Manual Override:** Dedicated `processXboxOverride()` routine handling Bluetooth input from an Xbox Series X controller with fail-safe zeroing on disconnect, quadratic stick shaping, and a single-shot 'A' button to reset coordinate origin to $(0, 0)\text{ mrad}$.



---

## Serial Communication Interface

Communication operates over UART at **115,200 baud, 8N1**, utilizing plain ASCII newline-terminated strings (`\n`).

### Commands: Raspberry Pi -> ESP32

| Command | Arguments | Unit | Description | Example |
| --- | --- | --- | --- | --- |
| `V` | `<pan_s> <tilt_s>` | `mrad/s` | **Velocity Trim:** Dynamic continuous slew velocity trim. | `V 350 -120` |
| `M` | `<d_pan> <d_tilt>` | `mrad` | **Move Relative:** Displaces axes by the commanded angular delta from current position. | `M 35 -12` |
| `X` | *none* | — | **Halt:** Immediately initiates controlled deceleration to stop and hold current position. | `X` |
| `T` | `<state: 0/1>` | flag | **Trigger:** Commands the solenoid firing state machine (`1` = fire pulse sequence, `0` = disengage). | `T 1` |
| `H` | *none* | — | **Home:** Stops motion and re-zeros current motor coordinates to $(0, 0\text{ mrad})$. | `H` |
| `Q` | *none* | — | **Status Query:** Requests an immediate diagnostic telemetry packet. | `Q` |

### Telemetry: ESP32 -> Raspberry Pi

| Packet | Arguments | Unit | Description | Example |
| --- | --- | --- | --- | --- |
| `S` | `<pan> <tilt> <firing> <moving>` | `mrad`, `mrad`, `flag`, `flag` | **Periodic State (20 Hz):** Broadcasts current pan angle, tilt angle, firing state, and stepper activity. | `S 32 -10 0 1` |
| `R` | `<is_moving> <pan> <tilt>` | `flag`, `mrad`, `mrad` | **Query Response:** Transmitted immediately in response to `Q`. | `R 0 120 -45` |

---

## Hardware Specification & Kinematics

### Mechanical & Stepper Parameters

* **Pan Stepper:** 1.8° step angle ($200\text{ steps/rev}$), $16\times$ microstepping $\rightarrow 3{,}200\text{ native steps/rev}$.


* **Tilt Stepper:** 1.8° step angle ($200\text{ steps/rev}$), $16\times$ microstepping $\rightarrow 3{,}200\text{ native steps/rev}$.


* **Gear Ratios:**
* Pan: $246\text{T} / 37\text{T} \approx 6.6486:1$

* Tilt: $50\text{T} / 1\text{T} = 50:1$ (worm drive)




* **Speed Ceilings:**
* Pan: $11{,}600\text{ steps/s}$ ($\approx 3{,}425\text{ mrad/s}$)


* Tilt: $14{,}800\text{ steps/s}$ ($\approx 580\text{ mrad/s}$)




* **Acceleration Ceilings:**
* Pan: $13{,}500\text{ steps/s}^2$

* Tilt: $50{,}000\text{ steps/s}^2$



* **Physical Travel Envelopes (Hard Stops):**
* Pan: $\pm 180.0^\circ$ ($\pm 10{,}638\text{ steps}$ / $\approx \pm 3{,}141\text{ mrad}$)


* Tilt: $\pm 22.5^\circ$ ($\pm 10{,}000\text{ steps}$ / $\approx \pm 392\text{ mrad}$)





### Optical Calibration (RPi Camera Module 3 @ 1280x720 Native Crop)

* **Active Sensor Resolution:** $1280 \times 720$ pixels


* **Calibrated Focal Length ($f_x, f_y$):** $\approx 985.5\text{ pixels}$
* **Optical Scale Factor:** $\approx 1.015\text{ mrad/pixel}$
* **Optical Center:** $(c_x, c_y) = (640, 360)\text{ px}$


---

## 🕹️ Xbox Controller Manual Override

Manual input automatically overrides autonomous targeting, locking out serial velocity commands until $400\text{ ms}$ after stick release:

* **Left Stick (Horizontal):** Pan axis continuous velocity jog (quadratic curve shaping).


* **Left Stick (Vertical):** Tilt axis continuous velocity jog (quadratic curve shaping).


* **Right Trigger (RT):** Manually trips the solenoid actuator state machine.


* **'A' Button:** Single-shot command that immediately brings both axes to a controlled halt and re-zeroes coordinate tracking to $(0, 0)\text{ mrad}$.



---

## ⚠️ Safety Disclaimer

This project is intended strictly for agricultural property management and wildlife deterrence. Ensure all hardware deployment complies with local ordinances regarding non-lethal wildlife management. Always verify hardware safety stops and clear line-of-sight before arming the turret.