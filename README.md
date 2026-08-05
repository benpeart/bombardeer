# 🎯 Bombadeer: Autonomous Vision-Guided Deterrent Turret

**Bombadeer** is an open-source, vision-guided autonomous deterrent turret designed to protect outdoor spaces, gardens, orchards, and agricultural property from intrusive wildlife (such as deer). 

Powered by a **Raspberry Pi 5** with a **26 TOPS Hailo-8 AI accelerator** for real-time visual tracking and an **ESP32** dedicated microcontroller running `FastAccelStepper` and `TMCStepper` for precision pan/tilt kinematic execution.

---

## 🚀 Features

* **Real-Time Edge AI Tracking:** Raspberry Pi 5 + Hailo-8 AI accelerator runs high-FPS YOLO-based inference pipelines for target detection.
* **Precision Pan/Tilt Motion:** ESP32 hardware-timer motor controller utilizing microstepped TMC2209 silent stepper drivers.
* **Heavy Payload Mechanics:** Built to handle a full paintball assembly (~4.5–5.0 kg total mass including marker, CO2 tank, and hopper) via a self-locking 30:1 worm gear on tilt and gear reduction on pan.
* **Dual Control Modes:**
  * **Autonomous Mode:** High-speed UART targeting stream (`P:<pan>,T:<tilt>`) driven by Pi 5 AI inference.
  * **Manual Override:** Wireless Xbox Series X/S controller integration via Bluetooth with velocity scaling, exponential response curves, and D-Pad precision jogging.
* **TMC2209 UART Integration:** Dynamically manages current limits, microstepping (1/16 interpolated to 1/256), and native `ihold` power saving over UART.
* **Smart Power & Battery Protection:** Built-in battery voltage monitoring with low-voltage alert states and auto-deep sleep power cutoff.

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
|  Raspberry Pi 5 +     |--------------------------->|    ESP32 Controller   |
|  Hailo-8 AI (26 TOPS) |   Target Commands (P, T)   |   (FastAccelStepper)  |
+-----------------------+                            +-----------+-----------+
|
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
| **Power Supply** | 12V–24V LiFePO4 / Battery Bank | Voltage divider on ADC pin for voltage monitoring |

---

## ⚡ ESP32 Firmware Setup

### Software Dependencies

The ESP32 firmware requires the following Arduino libraries:

* [TeemuAtlut/TMCStepper](https://github.com/teemuatlut/TMCStepper) — TMC2209 UART configuration
* [Gin66/FastAccelStepper](https://github.com/Gin66/FastAccelStepper) — Hardware-timer step pulse engine
* [XboxSeriesXControllerESP32_asukiaaa](https://github.com/asukiaaa/XboxSeriesXControllerESP32_asukiaaa) — Xbox controller Bluetooth library

---

## 🕹️ Xbox Controller Mapping

| Control | Action | Function |
| :--- | :--- | :--- |
| **Left Stick (Horizontal)** | Pan Axis | Dynamic velocity panning (quadratic response curve) |
| **Left Stick (Vertical)** | Tilt Axis | Dynamic velocity tilting (quadratic response curve) |
| **Right Trigger (RT)** | Actuation | Payload trigger signal |

---

## 📡 Serial Protocol (Pi 5 $\rightarrow$ ESP32)

Commands are transmitted over high-speed UART (`115200` baud, `8N1`) formatted as string frames terminated by `\n`:

```
P:<pan_steps>,T:<tilt_steps>\n
```

## Example:

```
P:1200,T:-450
```

* P:1200 — Absolute step position target for Pan axis
* T:-450 — Absolute step position target for Tilt axis

Note: If an Xbox controller is actively connected and generating manual joystick/button input, manual override automatically gates serial commands to prevent trajectory conflict.

## ⚙️ Configuration & Hardware Pinout
Adjust configuration parameters in globals.h:

```
// TMC2209 UART Driver Addresses (Configured via MS1 / MS2 Pins)
#define PAN_DRIVER_ADDR   0b00 // MS1 = LOW,  MS2 = LOW
#define TILT_DRIVER_ADDR  0b01 // MS1 = HIGH, MS2 = LOW

// Motor Current Calibration
#define PAN_RMS_CURRENT_MA   1300 // mA RMS (~85% rated capacity)
#define TILT_RMS_CURRENT_MA  1200 // mA RMS

// Hardware Sense Resistor
#define R_SENSE 0.11f // Standard 0.11 ohm for TMC2209 StepStick
```

## ⚠️ Safety Disclaimer
This project is intended strictly for agricultural property management and wildlife deterrence. Ensure all hardware deployment complies with local ordinances regarding non-lethal wildlife management. Always verify hardware safety stops and clear line-of-sight before arming the turret.
