# Bombardeer — Autonomous Vision-Guided Wildlife Deterrent Turret

**Bombardeer** is an open-source, vision-guided autonomous deterrent turret designed to protect yards, gardens, orchards, and agricultural property from intrusive wildlife (such as deer) using non-lethal air/paintball deterrence.

<img src="images/bombardeer.jpg" alt="Bombardeer: Autonomous Vision-Guided Deterrent Turret" width="480" />

The system splits high-level perception and ballistic trajectory planning from real-time motor pulse generation across two specialized processors:

1. **Raspberry Pi 5 + Hailo-8 NPU (26 TOPS) + IMX708 Wide NoIR Camera:** Captures dual-resolution `12 MP` Day / `3 MP` Dusk video, runs horizon-tiled MegaDetector v6 (`MDV6-yolov9-c-1280`) neural inference, enforces multi-layer human/vehicle safety vetoes, estimates 3D range and aerodynamic drag ballistics, and hosts a password-protected `1280×720` live Heads-Up Display (HUD) web console with progressive web app icons.
2. **ESP32 Motion Controller + Dual TMC2209 Stepper Drivers:** Executes hardware-timer trapezoidal step profiles (`FastAccelStepper`) over a `921,600` baud milliradian (`mrad`) UART link, drives the 24V Heschen firing solenoid with hardware pulse/cooldown cutoffs, and supports instant manual override via a Bluetooth Xbox Series X/S controller.

---

## High-Level System Architecture

```text
+-----------------------------------------------------------------------+
|                           Raspberry Pi 5                              |
|  - VisionMain (Core 0): PiSP 12MP/3MP Capture, ROI Scratch, ISP Ctrl  |
|  - HudEncoder (Core 1): Client-Aware 1280x720 HUD Overlay & JPEG      |
|  - HailoNPUProc (Core 2): Forked MDv6 1280x1280 Inference via SHM     |
|  - VidRecorder (Core 2): 1280x720 Pre/Post-Roll Event Archiver        |
|  - MotionServo & TurretSerial (Core 3): 50 Hz Loop & 921.6k UART      |
|  - Gunicorn + Flask Web Console: Basic Auth, /api/mode, /video_feed   |
+-----------------------------------------------------------------------+
                                    |
                                    |  Bidirectional UART (921,600 Baud, 8N1)
                                    |  Milliradian Commands & State Telemetry
                                    v
+-----------------------------------------------+     +-----------------------------+
|            ESP32 Microcontroller              |<----|  Xbox Series X Controller   |
|  - FastAccelStepper Hardware Pulse Timers     |     |     (Bluetooth Manual)      |
|  - TMC2209 Single-Wire UART Driver Control    |     +-----------------------------+
|  - Solenoid Timing & Hard Cutoff Watchdogs    |
+-----------------------------------------------+
         /              |              \
        v               v               v
+--------------+ +--------------+ +--------------------+
| TMC2209 Pan  | | TMC2209 Tilt | | 24V Firing Circuit |
| Stepper Axis | | Stepper Axis | | Heschen Solenoid   |
+--------------+ +--------------+ +--------------------+
```

---

## Repository Layout

```text
bombardeer/
├── bombardeer.py                  # Gunicorn WSGI entrypoint & CLI runner
├── config.yaml                    # Camera, optics, ballistics, control, wildlife & storage config
├── MDV6-yolov9-c-1280.hef         # Compiled Hailo-8 MegaDetector v6 model (gitignored)
├── static/                        # Multi-format favicons (ICO/SVG/PNG) & site.webmanifest
├── systemd/
│   └── bombardeer.service         # systemd unit (Type=notify, WatchdogSec=15, [::]:5000)
├── docs/                          # Full hardware, protocol, ESP32 & Raspberry Pi documentation
└── bombardeer_pkg/
    ├── config.py                  # Typed YAML + environment loader
    ├── state.py                   # SystemState (4 segregated lock planes) & CommandBus
    ├── utils.py                   # CPU core affinity, sd_notify & audit snapshot writer
    ├── control/                   # Parallax, horizon projection, drag ballistics, Kalman & 50 Hz servo
    ├── hardware/                  # PiSP camera pipeline, HudEncoder, non-reset UART & MP4 recorder
    ├── perception/                # ArUco detector, HailoNPUProc IPC & multi-tile MDv6 decoder/vetoes
    └── web/                       # Flask Basic-Auth REST API, /static/* assets & 1280x720 MJPEG HUD
```

---

## Physical Build & Kinematic Summary

* **Pan Axis (`6.65:1` Timing Belt Reduction):** `246T / 37T` GT2 belt drive (`3.3863 steps/mrad` at `16×` microstepping), capable of slewing up to `~3,425 mrad/s` across a `±180°` (`±3,141 mrad`) envelope.
* **Tilt Axis (`50:1` Anti-Backdrive Worm Drive):** Single-start `50T / 1T` worm gear (`25.4648 steps/mrad` at `16×` microstepping), self-locking against gravity with software travel clamps at `-420 mrad` and `+400 mrad`.
* **Optical Baseline & Parallax Geometry:** The IMX708 Wide NoIR camera is mounted **`170 mm` left** and **`60 mm` below** the barrel centerline (`f = 540.4 px` in canonical `1280×720` space, `1.850 mrad/px`). Software dynamically converges pan and tilt angles for the fused target range (`3–75 m`) and compensates for projectile gravity drop and aerodynamic drag (`v0 = 85.0 m/s`, `k = 0.0128`).
* **Firing Mechanism:** 24V Heschen push-pull solenoid (`60 ms` pulse / `190 ms` cooldown, `4.0 BPS` max, `2,000 ms` hardware cutoff) gated by optical freshness (`<= 220 ms`), aim deadband (`<= 40 mrad`), effective engagement range (`<= 40.0 m`), and barrel slew speed (`< 180 mrad/s`).

---

## ESP32 vs. Raspberry Pi Responsibilities

| Concern | Raspberry Pi 5 (`bombardeer_pkg/`) | ESP32 Firmware (`ESP32/`) |
| :--- | :--- | :--- |
| **Primary Role** | Perception, world-space tracking, ballistics, safety vetoes, web HUD, video archiving | Deterministic pulse generation, driver current/microstep management, hardware cutoffs |
| **Control Loop Rate** | `14.4–30 Hz` vision pipeline + `50 Hz` (`20 ms`) motion servo | Hardware timer interrupts (`13.5k–50k steps/s²` trapezoidal ramps) + `20–50 Hz` telemetry |
| **Coordinate Frame** | World-space & canonical `1280×720` image coordinates (`mrad`, `mrad/s`) | Mechanical step coordinates translated from incoming `mrad` / `mrad/s` packets |
| **Safety & Overrides** | Person/vehicle zero-overlap veto, `BOMBARDEER_PASS` auth gate, range/slew firing gates | Tilt/pan step limits, `2,000 ms` solenoid hardware cutoff, Xbox Bluetooth manual override |
| **Link & Persistence** | Clears `DTR`/`RTS` and disables `HUPCL` on `/dev/serial/by-id/` so restarts never reset MCU | Holds mechanical `(0, 0)` home across Pi service restarts; `'A'` button re-zeros origin |

---

## Operational Modes & Safety Model

The Web Console (`http://<pi-ip>:5000`) enforces a four-stage state machine and physical-style flip cover before live firing is permitted:

1. **`OFF` (OFF / SAFE):** Turret motors halted (`X`), solenoid disarmed (`T 0`), passive perception and HUD monitoring active.
2. **`ON` (TRACKING / SAFE):** Closed-loop pursuit active (`P` / `V` commands), solenoid locked out (`T 0`).
3. **`CALIBRATE` (1-SHOT):** Single-shot calibration/test mode (`PRIMED` -> fires one pulse on alignment, then locks until manually re-primed).
4. **`ARMED` (ARMED / LIVE):** Autonomous live-fire mode. Requires:
   * A non-default `BOMBARDEER_PASS` environment variable set in `/etc/systemd/system/bombardeer.service.d/override.conf` (otherwise `/api/mode` refuses `ARMED` with `HTTP 403`).
   * Lifting the virtual safety cover in the UI and confirming `ARM LIVE` within `4.0 s`.
   * Zero overlap with any detected `PERSON` (`>= 0.48`) or `VEHICLE` (`>= 0.35`) expanded by `24 px`.

---

## Documentation Index

| Document | Scope |
| :--- | :--- |
| [`docs/HARDWARE_BUILD.md`](docs/HARDWARE_BUILD.md) | Complete Bill of Materials (BOM), mechanical assembly, wiring schematics, and power-up checks |
| [`docs/SERIAL_PROTOCOL.md`](docs/SERIAL_PROTOCOL.md) | Shared `921,600` baud ASCII UART protocol (`P`, `V`, `X`, `T`, `H`, `Q`, `S`, `R`), units, and non-reset serial rules |
| [`docs/ESP32_DESIGN.md`](docs/ESP32_DESIGN.md) | ESP32 firmware architecture, `FastAccelStepper` profiles, TMC2209 UART setup, solenoid state machine, and Xbox override |
| [`docs/ESP32_DEPLOY_DEBUG.md`](docs/ESP32_DEPLOY_DEBUG.md) | PlatformIO build/flash instructions, USB serial testing, TMC2209 troubleshooting, and polarity checks |
| [`docs/RPI_DESIGN.md`](docs/RPI_DESIGN.md) | Raspberry Pi 5 multi-core/multi-process architecture, lock segregation, Kalman filter, drag ballistics, and 50 Hz servo |
| [`docs/RPI_DEPLOY_DEBUG.md`](docs/RPI_DEPLOY_DEBUG.md) | Pi 5 OS/PCIe Gen 3/HailoRT installation, Gunicorn + `systemd` deployment, Wi-Fi AP fallback, and package debugging |
| [`docs/WILDLIFE_PERCEPTION.md`](docs/WILDLIFE_PERCEPTION.md) | PiSP `12 MP` Day / `3 MP` Dusk dual-stream capture, horizon-anchored multi-tile NPU scanning, and safety veto funnel |

---

## ⚠️ Safety Disclaimer

This project is intended strictly for agricultural property management and non-lethal wildlife deterrence. Ensure all hardware deployment complies with local laws and ordinances. Always verify mechanical endstops, configure a strong `BOMBARDEER_PASS` credential, and confirm a clear line of sight before arming the turret.