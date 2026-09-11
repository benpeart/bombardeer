# Bombardeer Wildlife Deterrent Turret: System Architecture Specification

**Bombardeer** is an open-source, vision-guided autonomous deterrent turret designed to protect outdoor spaces, gardens, orchards, and agricultural property from intrusive wildlife (such as deer). 

<img src="images/bombardeer.jpg" alt="Bombardeer: Autonomous Vision-Guided Deterrent Turret" width="480"  />

Powered by a **Raspberry Pi 5** with a **26 TOPS Hailo-8 AI accelerator** for real-time visual tracking and an **ESP32** dedicated microcontroller for precision pan/tilt kinematic execution.

---

## 🚀 Features

* **Real-Time Edge AI Tracking:** Raspberry Pi 5 + Hailo-8 AI accelerator runs high-FPS YOLO-based inference pipelines for target detection.
* **Precision Pan/Tilt Motion:** ESP32 hardware-timer motor controller utilizing microstepped TMC2209 silent stepper drivers.
* **Heavy Payload Mechanics:** Built to handle a full paintball assembly (~4.5–5.0 kg total mass including marker, CO2 tank, and hopper) via a self-locking 30:1 worm gear on tilt and gear reduction on pan.
* **Dual Control Modes:**
  * **Autonomous Mode:** High-speed UART targeting stream (`P:<pan>,T:<tilt>`) driven by Pi 5 AI inference.
  * **Manual Override:** Wireless Xbox Series X/S controller integration via Bluetooth with velocity scaling, and exponential response curves.
* **TMC2209 UART Integration:** Dynamically manages current limits, microstepping (1/16 interpolated to 1/256), and native `ihold` power saving over UART.

---

## 🛠️ System Architecture

```
       +---------------------------------------------+
       |             Raspberry Pi 5                  |
       |  Perception, Kinematic Projection & Safety  |
       +---------------------------------------------+
                              |
                              |  Bidirectional UART (115200 Baud)
                              |  Packets in Milliradians (mrad)
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

| Component | Specification | Description |
| :--- | :--- | :--- |
| **Compute Board** | Raspberry Pi 5 | Target tracking & system coordination |
| **AI Accelerator** | Hailo-8 M.2 Module (26 TOPS) | Low-latency YOLO target detection |
| **Motion MCU** | ESP32-WROOM-32 | Multi-axis hardware timer pulse generation |
| **Stepper Drivers** | $2\times$ TMC2209 (v1.2+) | SilentStepStick with UART address jumpers |
| **Motors** | $2\times$ STEPPERONLINE 17HS19-2004S1 | NEMA 17 ($2.0\text{A}$ Peak, $59\text{ N}\cdot\text{cm}$ Holding Torque) |
| **Tilt Reduction** | 30:1 Worm Gear | Self-locking anti-backdrive mechanism |
| **Payload** | Paintball Marker + Hopper + CO2 | Active non-lethal deterrent |
| **Power Supply** | 12V–24V LiFePO4 / Battery Bank|

---

## Subsystem Roles

### 1. Vision & Trajectory Planning (Raspberry Pi 5)

* **Sensor Capture:** Sony IMX708 NoIR module operating via `Picamera2` and hardware PiSP color management.
* **Perception Engine:** OpenCV ArUco detector (diagnostic testing) and Hailo-8 AI neural inference pipeline (production target detection).
* **Optical Ray Projection:** Converts raw pixel displacement $(\Delta x, \Delta y)$ from optical center into metric angles:

$$\Delta \theta_{pan} = \arctan\left(\frac{x - c_x}{f_x}\right) \times 1000 \quad [\text{mrad}]$$


$$\Delta \theta_{tilt} = \arctan\left(\frac{y - c_y}{f_y}\right) \times 1000 \quad [\text{mrad}]$$


* **Visual Servoing:** Formulates target waypoints in relative or absolute milliradians and streams them to the motion controller.
* **Diagnostic HUD & Stream:** Flask-based real-time telemetry streaming accompanied by automated rolling buffer video recording.

### 2. Motion & Actuation Engine (ESP32)

* **Hardware Motion Profiler:** `FastAccelStepper` dynamically solves trapezoidal acceleration ramps natively on hardware timers at $16{,}000\text{ steps/s}^2$.
* **Angular-to-Step Translation:** Maps incoming integer milliradians directly to stepper motor step coordinates:

$$\text{Steps} = \text{Target Angle [mrad]} \times \left(\frac{\text{Steps per Revolution} \times \text{Microsteps} \times \text{Gear Ratio}}{2000 \pi}\right)$$


* **Driver Control:** Dual TMC2209 silent stepper drivers driven over a shared single-wire UART bus with runtime current configuration and StealthChop/SpreadCycle dynamic switching.
* **Actuation State Machine:** Hardware timing driver managing the high-speed pulse-and-cooldown cycles for the Heschen 24-volt firing solenoid.
* **Safety & Manual Override:** Low-latency Bluetooth listener for an Xbox Series X controller with stick deflection interrupts that instantly take precedence over autonomous tracking.

---

## Serial Communication Interface

Communication operates over UART at **115,200 baud, 8N1**, utilizing plain ASCII newline-terminated strings.

### Commands: Raspberry Pi -> ESP32

| Command | Arguments | Unit | Description | Example |
| --- | --- | --- | --- | --- |
| `M` | `<d_pan> <d_tilt>` | `mrad` | **Move Relative:** Displaces axes by the commanded angular delta from current position. | `M 35 -12` |
| `A` | `<pan> <tilt>` | `mrad` | **Move Absolute:** Drives axes to the specified angular coordinates referenced to mechanical zero. | `A 120 -45` |
| `X` | *none* | — | **Halt:** Immediately initiates controlled deceleration to stop and hold current position. | `X` |
| `T` | `<state: 0|1>` | boolean | **Trigger:** Commands the solenoid firing state machine (`1` = fire cycle, `0` = release). | `T 1` |
| `H` | *none* | — | **Home:** Clears current motor coordinates to angular zero $(0, 0\text{ mrad})$. | `H` |
| `Q` | *none* | — | **Status Query:** Requests an immediate diagnostic telemetry packet. | `Q` |

### Telemetry: ESP32 -> Raspberry Pi

| Packet | Arguments | Unit | Description | Example |
| --- | --- | --- | --- | --- |
| `S` | `<pan> <tilt> <firing>` | `mrad`, `mrad`, `flag` | **Periodic State (20 Hz):** Broadcasts current pan angle, tilt angle, and solenoid firing state. | `S 32 -10 0` |
| `R` | `<is_moving> <pan> <tilt>` | `flag`, `mrad`, `mrad` | **Query Response:** Transmitted immediately in response to `Q`. | `R 0 120 -45` |

---

## Hardware Specification & Kinematics

### Mechanical & Stepper Parameters

* **Pan Stepper:** 1.8° step angle ($200\text{ steps/rev}$), $16\times$ microstepping $\rightarrow 3{,}200\text{ native steps/rev}$.
* **Tilt Stepper:** 1.8° step angle ($200\text{ steps/rev}$), $16\times$ microstepping $\rightarrow 3{,}200\text{ native steps/rev}$.
* **Pan Speed Ceiling:** $16{,}000\text{ steps/s}$ ($5.0\text{ rev/s}$).
* **Pan Acceleration Ceiling:** $16{,}000\text{ steps/s}^2$.
* **Travel Envelopes:**
* Pan: $\pm 10{,}000\text{ steps}$ ($\approx \pm 3{,}125\text{ mrad}$ / $\pm 179.0^\circ$).
* Tilt: $\pm 15{,}000\text{ steps}$ ($\approx \pm 4{,}687\text{ mrad}$ / $\pm 268.5^\circ$).



### Optical Calibration (Sony IMX708 Wide NoIR)

* **Active Sensor Width ($W_{px}$):** 1280 pixels
* **Active Sensor Height ($H_{px}$):** 720 pixels
* **Horizontal Field of View ($\text{HFOV}$):** $\approx 102^\circ$ ($1.780\text{ rad}$)
* **Calibrated Horizontal Focal Length ($f_x$):**

$$f_x = \frac{W_{px} / 2}{\tan(\text{HFOV} / 2)} = \frac{640}{\tan(51^\circ)} \approx 518.2\text{ pixels}$$


* **Milliradians Per Pixel (Center Optical Axis):**

$$\text{mrad/px} = \frac{1000}{f_x} \approx \frac{1000}{518.2} \approx 1.93\text{ mrad/px}$$



---

Note: If an Xbox controller is actively connected and generating manual joystick/button input, manual override automatically gates serial commands to prevent trajectory conflict.

## ⚠️ Safety Disclaimer
This project is intended strictly for agricultural property management and wildlife deterrence. Ensure all hardware deployment complies with local ordinances regarding non-lethal wildlife management. Always verify hardware safety stops and clear line-of-sight before arming the turret.
