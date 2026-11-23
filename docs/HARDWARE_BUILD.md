# Hardware Requirements & Physical Build Guide

This guide covers the complete Bill of Materials (BOM), mechanical drive ratios, camera-to-bore mounting geometry, electrical wiring schematics, and safe power-up verification for the **Bombardeer** autonomous turret.

---

## 1. Bill of Materials (BOM)

### 1.1 Compute, Vision & AI Hardware

| Component | Specification | Qty | Purpose & Notes |
| :--- | :--- | :---: | :--- |
| **Raspberry Pi 5** | 8 GB RAM + Official Active Cooler | 1 | Primary vision, Kalman tracking, ballistics, and web console host |
| **Hailo-8 AI Kit / M.2 HAT+** | Hailo-8 M.2 Module (26 TOPS) | 1 | PCIe Gen 3 neural inference (`MDV6-yolov9-c-1280.hef`) |
| **Raspberry Pi Camera Module 3** | **Wide NoIR** (`IMX708_wide_noir`) | 1 | `120°` FOV sensor without IR-cut filter for day/dusk wildlife detection |
| **CSI Camera Ribbon Cable** | Pi 5 22-pin to 15-pin CSI cable | 1 | Connects IMX708 to `CAM/DISP 0` or `1` with slack for full pan/tilt travel |
| **microSD Card / NVMe SSD** | 64 GB+ High-Endurance | 1 | Raspberry Pi OS (64-bit) + local MP4 event recordings (`10 GB` quota) |

### 1.2 Motion Control, Actuators & Electronics

| Component | Specification | Qty | Purpose & Notes |
| :--- | :--- | :---: | :--- |
| **ESP32 Development Board** | ESP32-WROOM-32 (`30-pin` or `38-pin`) + CP2102 USB-UART | 1 | Hardware-timer stepper pulse generation, solenoid timing & Bluetooth Xbox host |
| **Silent Stepper Drivers** | TMC2209 v1.2+ (`R_SENSE = 0.11 Ω`) | 2 | UART-configured microstepping (`1/16 -> 1/256`) & StealthChop/SpreadCycle |
| **NEMA 17 Stepper Motors** | STEPPERONLINE `17HS19-2004S1` (`2.0 A`, `59 N·cm`, `1.8°`) | 2 | Pan and tilt axis prime movers (`200` full steps/rev) |
| **Firing Solenoid** | Heschen `HS-1564B` or `HS-4564B` (24V DC Push-Pull) | 1 | Mechanical trigger actuation (`60 ms` pulse / `190 ms` cooldown) |
| **Solenoid Driver Stage** | Logic-level N-Channel MOSFET (`IRLZ44N`) + `1N5408` flyback diode + `10 kΩ` pull-down | 1 | Isolates `GPIO 13` (`3.3V`) from inductive 24V solenoid coil switching |
| **UART Bus Resistor** | `1 kΩ` 1/4W resistor | 1 | Couples `ESP32 TX2 (GPIO 17)` to the shared single-wire TMC2209 `PDN_UART` bus |
| **Power Supply & Buck Converters** | 24V DC LiFePO4 / PSU + `24V -> 5V (5A)` USB-C PD buck + `24V -> 5V/3.3V` logic buck | 1 | Powers steppers & solenoid at 24V while supplying clean `5.1V / 5A` to Pi 5 |
| **Manual Override Controller** | Xbox Series X/S Wireless Controller (BLE) | 1 | Optional Bluetooth manual jog, trigger test, and `'A'` button home zeroing |

### 1.3 Mechanical Drive & Frame Hardware

| Subsystem | Specification | Notes |
| :--- | :--- | :--- |
| **Pan Reduction (`6.6486:1`)** | `246T` printed ring gear + `37T` motor pinion + GT2 closed-loop belt | OpenSCAD `pan_timing_gears.scad`; `3.3863 steps/mrad` at `16×` microstepping |
| **Tilt Reduction (`50:1`)** | `50T` worm wheel + `1T` single-start worm gear | OpenSCAD `tilt_gears.scad`; self-locking against gravity (`25.4648 steps/mrad`) |
| **Main Azimuth Bearing** | Lazy Susan / slewing ring bearing (`150–200 mm` OD) | Supports upper turret carriage and isolates axial/radial loads from motor shaft |
| **Cable Management** | Drag chain / flexible spiral wrap with service loop | Prevents CSI ribbon and motor harnesses from snagging across `±180°` pan sweep |

---

## 2. Camera-to-Bore Mounting Geometry

The ballistics and parallax solver (`bombardeer_pkg/control/ballistics.py` and `config.yaml`) expects the IMX708 camera optical center to be rigidly mounted at fixed offsets relative to the barrel bore centerline:

* **Horizontal Offset (`camera_offset_x_mm`):** **`-170.0 mm`** (`170 mm` to the **left** of the bore when looking downrange).
* **Vertical Offset (`camera_offset_y_mm`):** **`-60.0 mm`** (`60 mm` **below** the bore centerline).
* **Mount Height (`camera_mount_height_m`):** **`1.15 m`** above level ground (used by the trigonometric ground-plane range estimator).
* **Sensor Orientation:** Mounted inverted (`hflip=True, vflip=True` applied in `_build_camera_config()`), with manual focus locked to tree-line hyperfocal infinity (`manual_lens_position: 0.0`).

> **Note:** If your physical bracket places the camera at different offsets or mount height, update `optics.camera_offset_x_mm`, `optics.camera_offset_y_mm`, and `optics.camera_mount_height_m` in `config.yaml` before running `1-SHOT` calibration.

---

## 3. Electrical Wiring & Pinout Reference

### 3.1 ESP32 GPIO Pin Map (`ESP32/src/main.cpp`)

| Signal / Function | ESP32 GPIO | Connected To | Notes |
| :--- | :---: | :--- | :--- |
| **Pan `STEP`** | `GPIO 26` | Pan TMC2209 `STEP` | Hardware timer pulse output (`FastAccelStepper`) |
| **Pan `DIR`** | `GPIO 25` | Pan TMC2209 `DIR` | Direction control |
| **Pan `MS1` (Addr Bit 0)** | `GPIO 4` | Pan TMC2209 `MS1` | Driven `LOW` at boot -> UART slave address **`0b00`** |
| **Pan `MS2` (Addr Bit 1)** | `GPIO 27` | Pan TMC2209 `MS2` | Driven `LOW` at boot -> UART slave address **`0b00`** |
| **Tilt `STEP`** | `GPIO 33` | Tilt TMC2209 `STEP` | Hardware timer pulse output (`FastAccelStepper`) |
| **Tilt `DIR`** | `GPIO 32` | Tilt TMC2209 `DIR` | Direction control |
| **Tilt `MS1` (Addr Bit 0)** | `GPIO 18` | Tilt TMC2209 `MS1` | Driven `HIGH` at boot -> UART slave address **`0b01`** |
| **Tilt `MS2` (Addr Bit 1)** | `GPIO 5` | Tilt TMC2209 `MS2` | Driven `LOW` at boot -> UART slave address **`0b01`** |
| **Shared Driver `ENN`** | `GPIO 19` | Both TMC2209 `ENN` | Active-low enable (held `HIGH` until UART init completes) |
| **TMC2209 `UART2 RX`** | `GPIO 16` | Shared `PDN_UART` bus | Direct connection to `PDN_UART` on both TMC2209 boards |
| **TMC2209 `UART2 TX`** | `GPIO 17` | Shared `PDN_UART` bus via **`1 kΩ`** | `1 kΩ` series resistor prevents bus contention on single-wire UART |
| **Solenoid Trigger** | `GPIO 13` | MOSFET gate (`100 Ω` series, `10 kΩ` to GND) | Switches 24V low-side path to Heschen solenoid (`1N5408` flyback across coil) |
| **Host USB Serial** | `UART0` (USB) | Raspberry Pi 5 USB port | `921,600` baud (`CP2102` `/dev/serial/by-id/...`) |

### 3.2 TMC2209 Single-Wire UART Wiring

```text
ESP32 GPIO 16 (RX2) ─────────────────────┬───────────────────────┐
                                         │                       │
ESP32 GPIO 17 (TX2) ────[ 1 kΩ ]─────────┤                       │
                                         ▼                       ▼
                                  [Pan TMC2209]           [Tilt TMC2209]
                                    PDN_UART                PDN_UART
                                  (MS1=0, MS2=0)          (MS1=1, MS2=0)
                                    Addr = 0b00             Addr = 0b01
```

* **Logic Voltage (`VIO`):** Connect `VIO` on both TMC2209 boards to **`3.3V`** (never `5V`), matching the ESP32's `3.3V` GPIO logic levels.
* **Motor Power (`VM`):** Connect `VM` and motor ground to the `24V` supply with a `100 µF` (`35V+`) electrolytic decoupling capacitor mounted close to each driver's `VM`/`GND` pins.

---

## 4. Mechanical Assembly & Safe Power-Up Sequence

1. **Bench-Test Electronics Before Belt/Worm Engagement:**
   * Leave the GT2 pan belt off and disengage the tilt worm gear during initial firmware flashing.
   * Power the ESP32 over USB (`5V`) first and confirm `[TMC2209] PAN OK` and `[TMC2209] TILT OK` in the debug serial log before energizing the `24V` rail.
2. **Verify Axis Polarity:**
   * Positive Pan (`+mrad`) must rotate the turret **clockwise (right)** when viewed from above, matching `camera_offset_x_mm = -170.0` and image `+X` right.
   * Positive Tilt (`+mrad`) must elevate the barrel **upward**, matching image `-Y` up.
   * If either axis moves opposite to convention, power down `24V` and reverse one stepper coil pair (`A1/A2`) or invert `setDirectionPin(..., true)` in `ESP32/src/main.cpp`.
3. **Install Mechanical Hard Stops & Home Alignment:**
   * Assemble the `246T/37T` pan timing ring and `50:1` tilt worm drive, verifying smooth travel across `±180°` pan and `±22.5°` tilt without cable binding.
   * Level the barrel horizontally (`0 mrad` tilt) and point downrange (`0 mrad` pan), then send `H` (or press `'A'` on the Xbox controller) to latch `(0, 0) mrad`.