# Bombardeer Wildlife Deterrent Turret: System Architecture Specification

**Bombardeer** is an open-source, vision-guided autonomous deterrent turret designed to protect outdoor spaces, gardens, orchards, and agricultural property from intrusive wildlife (such as deer).

<img src="images/bombardeer.jpg" alt="Bombardeer: Autonomous Vision-Guided Deterrent Turret" width="480"  />

Powered by a **Raspberry Pi 5** for real-time visual perception, range estimation, and dynamic parallax compensation, paired with a dedicated **ESP32** microcontroller executing hardware-timed pan/tilt motion profiles across dual TMC2209 silent stepper drivers.

---

## 🚀 Features

* **Asynchronous Perception & Motion Decoupling:** Vision inference runs at sensor rate (15–30 Hz) while a dedicated 50 Hz (20 ms tick) real-time motion servo calculates trajectory vectors, preventing camera frame drops from stalling motor actuation.


* **Segregated Lock Architecture:** Isolates concurrency across three independent lock domains (`telemetry_lock`, `target_lock`, and `frame_lock`) to eliminate lock contention, priority inversion, and serial UART stalls at 921,600 baud.


* **Forward-Projected Kalman State Estimation:** 4-state constant-velocity Kalman filter estimates 3D world-space coordinates, projecting target state forward across a fixed ~40 ms optical pipeline exposure delay directly to the actuation instant.


* **1.5-Second Dead-Reckoning Coasting:** Maintains smooth pursuit through optical occlusions, foliage, and motion blur with exponential velocity decay, avoiding target loss or abrupt stops.


* **Dual-Mode Control Hysteresis:** Seamlessly transitions between open-loop ballistic waypoint snaps (`P <pan> <tilt>`) for large angular displacements (>120 mrad) and closed-loop proportional velocity pursuit (`V <pan_s> <tilt_s>`) with 1.0× feed-forward velocity matching.


* **Dynamic Slew-Gated Solenoid Engagement:** Heschen 24V firing solenoid actuation requires optical freshness (<150 ms), verified crosshair deadband alignment (<40 mrad), and a barrel slew speed ceiling (<180 mrad/s) to prevent firing during rapid transits.


* **Software Axis Travel Clamping:** Real-time telemetry monitoring actively zeroes commanded velocity before the tilt axis can impact mechanical endstops (-420 to +400 mrad).


* **Dynamic Range Estimation & Parallax Convergence:** Computes target distance via pinhole projection (ArUco calibration or bounding-box height) and dynamically converges the bore sightline to compensate for the physical camera offset ($170\text{ mm}$ left, $60\text{ mm}$ below the bore).


* **Native Milliradian (mrad) Motion Protocol:** High-speed binary/ASCII UART link at **921,600 baud** commands velocity trims (`mrad/s`) and absolute waypoints (`mrad`), completely decoupled from hardware microstepping and gearing.


* **Precision Kinematic Reductions:**
* **Pan Axis:** Custom $6.65:1$ timing ring drive ($246\text{T} / 37\text{T}$) achieving up to $11{,}600\text{ steps/s}$ ($\approx 3{,}425\text{ mrad/s}$).
* **Tilt Axis:** Anti-backdrive $50:1$ single-start worm drive ($50\text{T} / 1\text{T}$) achieving up to $14{,}800\text{ steps/s}$ ($\approx 580\text{ mrad/s}$) with zero backdrive.


* **Dual Control Modes with Clean Handoff:** Autonomous visual pursuit over high-speed UART with seamless manual override via Bluetooth Xbox Series X/S controller integration.
* **TMC2209 UART Integration:** Dynamically manages current limits ($1200\text{ mA}$ Pan, $1100\text{ mA}$ Tilt), microstepping ($1/16$ interpolated to $1/256$), standstill `ihold` power saving, and automatic StealthChop-to-SpreadCycle transitions.

---

## 🛠️ System Architecture

```
       +-----------------------------------------------------+
       |                  Raspberry Pi 5                     |
       |  - Vision Worker (15-30 Hz PiSP / OpenCV / CLAHE)   |
       |  - Kalman Predictive State Estimator (World Space)  |
       |  - 50 Hz Motion Servoing Loop (20ms Cadence)        |
       |  - Flask Diagnostic HUD & Auto-Video Recorder       |
       +-----------------------------------------------------+
                                  |
                                  |  Bidirectional UART (921,600 Baud)
                                  |  Packets in Milliradians (mrad / mrad/s)
                                  v
       +---------------------------------------------+    +-------------------------------+
       |             ESP32 Microcontroller           |<---|   Xbox Series X Controller    |
       |  - FastAccelStepper Hardware Pulse Timer    |    |       (Bluetooth Manual)      |
       |  - Solenoid Timing State Machine            |    +---------------+---------------+
       +---------------------------------------------+
               /              |              \
              v               v               v
     +----------------+ +----------------+ +--------------------+
     | TMC2209 / Pan  | | TMC2209 / Tilt | | 24V Firing Circuit |
     | Stepper Motor  | | Stepper Motor  | | Heschen Solenoid   |
     +----------------+ +----------------+ +--------------------+

```

---

## 🧰 Hardware Requirements

| Component | Specification | Description |
| --- | --- | --- |
| **Compute Board** | Raspberry Pi 5 (8GB) | Vision pipeline, 50 Hz motion servo, Kalman prediction, HUD streaming |
| **AI Accelerator** | Hailo-8 M.2 Module (26 TOPS) | Low-latency neural network inference for real-time target detection |
| **Vision Sensor** | RPi Camera Module 3 (Wide/Standard) | $1280 \times 720$ @ $30\text{ FPS}$ low-light capture via `Picamera2`<br> |
| **Motion MCU** | ESP32-WROOM-32 | Multi-axis hardware timer pulse generation & safety watchdogs |
| **Stepper Drivers** | $2\times$ TMC2209 (v1.2+) | Single-wire UART addressing (`0b00` Pan, `0b01` Tilt) |
| **Stepper Motors** | $2\times$ STEPPERONLINE 17HS19-2004S1 | NEMA 17 ($2.0\text{A}$ Peak, $59\text{ N}\cdot\text{cm}$ Holding Torque) |
| **Pan Drive** | $6.65:1$ GT2 Timing Belt Reduction ($246\text{T} / 37\text{T}$) | High-slew azimuth tracking ($3.386\text{ steps/mrad}$) |
| **Tilt Drive** | $50:1$ Anti-Backdrive Worm Gear ($50\text{T} / 1\text{T}$) | Self-locking elevation drive ($25.465\text{ steps/mrad}$) |
| **Actuator** | Heschen HS-1564B / HS-4564B Solenoid | 24V push-pull trigger actuation ($4.0\text{ BPS}$) |
| **Mounting Offset** | $170\text{ mm}$ Left, $60\text{ mm}$ Below Bore Centerline | Baseline camera-to-barrel offset compensated in software |
| **Power Supply** | 12V–24V LiFePO4 / Battery Bank | Powers steppers, solenoid circuit, and logic step-downs |

---

## Subsystem Roles

### 1. Vision & Trajectory Planning (Raspberry Pi 5)

* **Multi-Threaded Architecture:** Clamps OpenCV to 2 dedicated physical cores, preserving remaining CPU cores for 50 Hz motion servo timing, PiSP DMA frame transfers, and serial I/O.


* **Lock Segregation:**
* `telemetry_lock`: Protects high-speed motor angles, movement flags, and historical telemetry deques.


* `target_lock`: Protects vision coordinates, target bounding boxes, disk metrics, and supervisory alerts.


* `frame_lock`: Protects JPEG buffers for the Flask diagnostic web stream.




* **Optical Intrinsics & Parallax Compensation:** Native $1280 \times 720$ capture with $f = 540.4\text{ px}$ ($1.850\text{ mrad/px}$). Calibrated trigonometric parallax adjustment aligns the barrel at range $D$ (in millimeters):



$$\Delta \theta_{\text{pan}} = \frac{-\text{OffsetX}}{D} \times 1000 = \frac{170\text{ mm}}{D} \times 1000 \quad [\text{mrad}]$$

$$\Delta \theta_{\text{tilt}} = \frac{\text{OffsetY}}{D} \times 1000 = \frac{-60\text{ mm}}{D} \times 1000 \quad [\text{mrad}]$$

* **Dynamic Firing Gate:** Actuation signals (`T 1`) are only transmitted if the target is fresh (<150 ms), radial aim error is $\le 40\text{ mrad}$, and total barrel slew speed is $<180\text{ mrad/s}$.


* **Autonomous Archival Recording:** Automatic rolling video buffer records 2.0 seconds of pre-lock context and 3.0 seconds of post-roll context upon confirmed target engagement, with automatic disk-space quota recycling.



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

Communication operates over UART at **921,600 baud, 8N1**, utilizing plain ASCII newline-terminated strings (`\n`).

### Commands: Raspberry Pi -> ESP32

| Command | Arguments | Unit | Description | Example |
| --- | --- | --- | --- | --- |
| `P` | `<pan> <tilt>` | `mrad` | **Ballistic Setpoint:** Absolute waypoint jump with 400 ms rate limit and 60 mrad delta lockout. | `P 280 -65` |
| `V` | `<pan_s> <tilt_s>` | `mrad/s` | **Velocity Pursuit:** Continuous speed tracking vector (up to $\pm 850\text{ pan}$, $\pm 450\text{ tilt}$). | `V 320 -85` |
| `X` | *none* | — | **Halt:** Commands immediate controlled deceleration to stop and hold position. | `X` |
| `T` | `<state: 0/1>` | flag | **Trigger:** Commands the solenoid firing state machine (`1` = fire, `0` = disengage). | `T 1` |
| `H` | *none* | — | **Home:** Stops motion and re-zeros current motor coordinates to $(0, 0\text{ mrad})$. | `H` |
| `Q` | *none* | — | **Status Query:** Requests an immediate diagnostic telemetry packet. | `Q` |

### Telemetry: ESP32 -> Raspberry Pi

| Packet | Arguments | Unit | Description | Example |
| --- | --- | --- | --- | --- |
| `S` | `<time> <pan> <tilt> <firing> <moving>` | `us`, `mrad`, `mrad`, `flag`, `flag` | **Periodic State (20–50 Hz):** Broadcasts timestamp, axis angles, firing state, and movement flag. | `S 1840291 195 45 0 1` |
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


* **Travel Envelopes & Software Safety Clamps:**
* Pan Envelope: Physical hard stops at $\pm 180.0^\circ$ ($\approx \pm 3{,}141\text{ mrad}$)
* Tilt Envelope: Physical travel $\pm 22.5^\circ$ ($\approx \pm 392\text{ mrad}$); software velocity clamped at **`-420 mrad` (Min)** and **`+400 mrad` (Max)**.





### Optical Calibration (RPi Camera Module 3 @ 1280x720 Native Capture)

* **Active Sensor Resolution:** $1280 \times 720$ pixels


* **Focal Length ($f_x, f_y$):** $540.4\text{ pixels}$

* **Optical Scale Factor:** $1.850\text{ mrad/pixel}$ ($1000 / 540.4$)


* **Optical Center:** $(c_x, c_y) = (640, 360)\text{ px}$

* **Latency Compensation Offset:** Fixed $40\text{ ms}$ history interpolation window for sensor exposure and ISP DMA transfer.



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