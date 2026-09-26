# 🎯 Bombardeer: Autonomous Vision-Guided Deterrent Turret

**Bombardeer** is an open-source, vision-guided autonomous deterrent turret designed to protect outdoor spaces, gardens, orchards, and agricultural property from intrusive wildlife (such as deer).

Powered by a **Raspberry Pi 5** running low-latency predictive visual servoing with libcamera/Picamera2 and an **ESP32** dedicated microcontroller executing hardware-timed, trapezoidal motion profiles across dual TMC2209 stepper drivers.

---

## 🚀 Key Features

* **Native Milliradian (mrad) Kinematic Abstraction:** Complete decoupling between vision processing and hardware execution. The host sends commands in angular space (`mrad` and `mrad/s`), while the ESP32 internally manages motor microstepping, gear ratios, and acceleration envelopes.


* **Dynamic Range Estimation & Parallax Convergence:** Automatically calculates target distance using pinhole geometry (ArUco markers in test mode, bounding box height for white-tailed deer in production) and converges the physical barrel sightline (offset by 170 mm left and 60 mm below the bore).


* **Precision Kinematic Motion:**
* **Pan Axis:** Custom $6.65:1$ timing ring drive ($246\text{T} / 37\text{T}$), benchmarked at $11,600\text{ steps/s}$ ($~3,425\text{ mrad/s}$) and $13,500\text{ steps/s}^2$.


* **Tilt Axis:** Low-inertia, anti-backdrive $50:1$ single-start worm drive ($50\text{T} / 1\text{T}$), benchmarked at $14,800\text{ steps/s}$ ($~580\text{ mrad/s}$) and $50,000\text{ steps/s}^2$.




* **Hardware-Timed Step Generation:** ESP32 hardware timer engine (`FastAccelStepper`) driving TMC2209 drivers with StealthChop standstill quieting and dynamic high-speed torque transitions.


* **Dual Control Architecture with Seamless Handoff:**
* **Autonomous Visual Servoing:** Closed-loop PD velocity tracking with slew-rate limiting to eliminate overshoot and motor stalls.


* **Manual Bluetooth Override:** Xbox Series X/S controller integration with quadratic stick response curves, auto-timeout failsafes, and single-shot coordinate re-zeroing.




* **Solenoid Trigger Pulse Sequencer:** Non-blocking state machine governing a Heschen HS-1564B actuator ($60\text{ ms}$ pulse / $190\text{ ms}$ cooldown; $4.0\text{ BPS}$ cycle) with hardware runtime watchdogs.


* **Integrated Web Diagnostic HUD:** Real-time Flask streaming interface at $640 \times 360$ showing optical center crosshairs, dynamic bore-convergence impact markers, live telemetry, and automated event video recording.



---

## 🛠️ System Architecture

```
                                                 +-------------------------------+
                                                 |   Xbox Series X Controller    |
                                                 |       (Bluetooth Manual)      |
                                                 +---------------+---------------+
                                                                 |
                                                                 v
+-----------------------+     UART (115200 Baud)     +-----------------------+
|    Raspberry Pi 5     |--------------------------->|    ESP32 Controller   |
|   (Picamera2 / HUD)   |   mrad / mrad/s Commands   |   (FastAccelStepper)  |
+-----------------------+<---------------------------+-----------+-----------+
                          20 Hz Telemetry (S line)               |
                                                         UART / Step / Dir
                                                                 |
                                                                 v
                                                     +-----------------------+
                                                     |  Dual TMC2209 Drivers |
                                                     +-----------+-----------+
                                                                 |
                                                            4-Wire Stepper
                                                                 |
                                                                 v
                                                     +-----------------------+
                                                     | STEPPERONLINE NEMA 17 |
                                                     |   Pan & Tilt Motors   |
                                                     +-----------------------+

```

---

## 🧰 Hardware Specifications

| Component | Specification | Function |
| --- | --- | --- |
| **Host Computer** | Raspberry Pi 5 (8GB) | Target tracking, HUD streaming, telemetry logging |
| **Vision Sensor** | RPi Camera Module 3 (Wide/Standard) | $1280 \times 720$ @ $30\text{ FPS}$ low-light capture crop |
| **Motion MCU** | ESP32-WROOM-32 | Hardware-timed pulse generation & safety watchdogs |
| **Stepper Drivers** | $2\times$ TMC2209 (v1.2+) | Single-wire UART addressing (`0b00` Pan, `0b01` Tilt) |
| **Stepper Motors** | $2\times$ STEPPERONLINE 17HS19-2004S1 | NEMA 17 ($2.0\text{A}$, $59\text{ N}\cdot\text{cm}$ torque) |
| **Pan Drive** | $6.65:1$ GT2 Belt Reduction ($246\text{T} / 37\text{T}$) | High-slew azimuth tracking ($3.386\text{ steps/mrad}$) |
| **Tilt Drive** | $50:1$ Anti-Backdrive Worm Gear ($50\text{T} / 1\text{T}$) | Self-locking elevation drive ($25.465\text{ steps/mrad}$) |
| **Actuator** | Heschen HS-1564B / HS-4564B Solenoid | Push-pull trigger actuation |
| **Camera Offset** | $170\text{ mm}$ Left, $60\text{ mm}$ Below Bore Centerline | Baseline geometry compensated via dynamic parallax |

---

## 📡 Serial Communication Protocol (Pi 5 $\leftrightarrow$ ESP32)

Communication operates over UART at **`115200` baud (8N1)** using non-blocking, zero-allocation newline-terminated frames (`\n`).

### Host Commands (Pi 5 $\rightarrow$ ESP32)

| Command | Format | Description |
| --- | --- | --- |
| **Velocity Trim** | `V <pan_mrad_s> <tilt_mrad_s>` | Dynamic continuous slew velocity trim |
| **Relative Move** | `M <d_pan_mrad> <d_tilt_mrad>` | Discrete positional waypoint displacement |
| **Emergency Halt** | `X` | Controlled dynamic braking to a full stop |
| **Set Coordinate Zero** | `H` | Resets current physical positions to mechanical $(0, 0)$<br> |
| **State Query** | `Q` | Returns immediate motion state and coordinates |
| **Fire Trigger** | `T <0`\|`1>` | Arms/pulses or disengages autonomous firing |

### Telemetry Broadcast (ESP32 $\rightarrow$ Pi 5)

The ESP32 broadcasts status every $50\text{ ms}$ ($20\text{ Hz}$):

`S <pan_mrad> <tilt_mrad> <firing_active> <moving>`

* **`pan_mrad`**: Current pan position in milliradians relative to home zero.


* **`tilt_mrad`**: Current tilt position in milliradians relative to home zero.


* **`firing_active`**: `1` if the solenoid is actively in a firing pulse or cooldown cycle, otherwise `0`.


* **`moving`**: `1` if either stepper axis is actively stepping, otherwise `0`.



---

## 🕹️ Xbox Controller Mapping & Manual Override

When manual input is detected, the ESP32 automatically locks out incoming autonomous serial velocity commands for $400\text{ ms}$ post-input.

| Control | Action | Function |
| --- | --- | --- |
| **Left Stick (Horizontal)** | Pan Axis | Dynamic velocity slewing (quadratic precision curve) |
| **Left Stick (Vertical)** | Tilt Axis | Dynamic velocity slewing (quadratic precision curve) |
| **Right Trigger (RT)** | Solenoid Fire | Engages trigger pulse state machine |
| **A Button** | Re-Zero Home | Instantly halts motion and resets coordinates to $(0, 0)\text{ mrad}$<br> |

---

## 🖥️ Web Diagnostic Console

The Python harness provides a lightweight web streaming HUD on port `5000`:

* **Live Stream:** Displays the real-time $640 \times 360$ annotated video feed with optical center crosshairs, target bounding boxes, and projected impact markers.


* **Boresight Alignment:** Renders the dynamic point of aim (red crosshair) converged to match target distance, accounting for camera parallax.


* **Live Telemetry Banner:** Displays streaming sensor FPS, live physical coordinates, serial packet counters, and storage status.


* **Event Recording:** Pre-roll buffer captures and stores MP4 video recordings of target engagement events automatically.



Access the dashboard by navigating to:
`http://<raspberry-pi-ip>:5000/`

---

## ⚠️ Safety Disclaimer

This project is designed and intended strictly for non-lethal wildlife deterrence and agricultural property management. Always verify physical travel limits, confirm the solenoid cycle parameters, ensure lines of sight are completely clear of bystanders, and comply with all applicable local regulations regarding autonomous and non-lethal deterrent devices.