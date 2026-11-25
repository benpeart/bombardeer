# ESP32 Firmware Architecture & Control Design

The ESP32 firmware (`ESP32/src/main.cpp`) acts as the deterministic real-time motion and firing controller for **Bombardeer**. Built on the Arduino/PlatformIO framework for `ESP32-WROOM-32`, it isolates time-critical stepper pulse generation and solenoid timing from host serial parsing and Bluetooth gamepad polling.

For wire-level command definitions, see [`SERIAL_PROTOCOL.md`](SERIAL_PROTOCOL.md). For GPIO pinouts and mechanical assembly, see [`HARDWARE_BUILD.md`](HARDWARE_BUILD.md).

---

## 1. Core Libraries & Hardware Peripherals

* **`FastAccelStepper`:** Generates jitter-free step pulses using the ESP32's hardware MCPWM/RMT pulse timer peripherals (`PAN_STEP_PIN = 26`, `TILT_STEP_PIN = 33`), running completely independent of `loop()` execution time.
* **`TMCStepper` (`HardwareSerial Serial2`):** Configures both TMC2209 drivers over a shared `115,200` baud single-wire UART bus (`RX2 = GPIO 16`, `TX2 = GPIO 17` via `1 kΩ`) with register read-back validation and automatic brownout recovery.
* **`XboxSeriesXControllerESP32_asukiaaa`:** Connects to a paired Xbox Series X/S controller over Bluetooth Low Energy (BLE) to provide immediate manual turret override without needing the Raspberry Pi.

---

## 2. TMC2209 Driver Initialization & Brownout Recovery

At boot (`setup()`), `SHARED_ENABLE_PIN` (`GPIO 19`) is held `HIGH` (drivers disabled) while address-select pins are driven (`PAN`: `MS1=0, MS2=0 -> 0b00`; `TILT`: `MS1=1, MS2=0 -> 0b01`). `initTMC2209()` then configures each driver over `Serial2`:

| Setting | Pan Axis (`Addr 0b00`) | Tilt Axis (`Addr 0b01`) | Rationale |
| :--- | :---: | :---: | :--- |
| **Active RMS Current (`irun`)** | `1,200 mA` | `1,100 mA` | High dynamic torque for rapid snaps without thermal saturation |
| **Standstill Hold Multiplier** | `0.75×` (`~885 mA`) | `0.15×` (`~165 mA`) | Pan resists outdoor wind gusts; `50:1` tilt worm gear is self-locking |
| **Microstepping** | `1/16` -> `1/256` (`intpol=true`) | `1/16` -> `1/256` (`intpol=true`) | Matches `3,200 steps/rev` pulse math with smooth `256×` interpolation |
| **Chopper Mode (`TPWMTHRS=200`)** | Hybrid StealthChop / SpreadCycle | Hybrid StealthChop / SpreadCycle | Dead-silent StealthChop at low tracking speeds; SpreadCycle above `~3,000 Hz` |
| **Powerdown Timing** | `iholddelay=4`, `TPOWERDOWN=128` | `iholddelay=4`, `TPOWERDOWN=128` | Smoothly reduces coil current `~2.8 s` after motion stops |

* **Deterministic Read-Back Check:** `initTMC2209()` reads back `mstep_reg_select()`, `!I_scale_analog()`, and `microsteps()` over UART, returning `false` if any register failed to latch.
* **Runtime Brownout Watchdog:** Every `3,000 ms` while both motors are stationary, `loop()` checks whether either TMC2209 reset to default analog-VREF mode (e.g., after a `24V` voltage sag) and automatically re-applies `initTMC2209()`.

---

## 3. Motion Execution Engine (`MODE_POSITION` vs. `MODE_VELOCITY`)

### 3.1 Kinematic Limits & Acceleration Profiles

| Parameter | Pan Axis | Tilt Axis | Notes |
| :--- | :---: | :---: | :--- |
| **Max Speed (`MAXSPEEDHZ`)** | `11,600 Hz` (`~3,425 mrad/s`) | `14,800 Hz` (`~580 mrad/s`) | Benchmarked `80%` ceiling (Pi velocity loop caps to `±850` / `±300 mrad/s`) |
| **Min Speed (`MINSPEEDHZ`)** | `15 Hz` (`~4.4 mrad/s`) | `15 Hz` (`~0.59 mrad/s`) | Below threshold -> clean `stopMove()` |
| **Acceleration** | `13,500 steps/s²` | `50,000 steps/s²` | Trapezoidal velocity profile |
| **Software Travel Limits** | `±180.0°` (`±3,141 mrad`) | `±22.5°` (`±392 mrad`) | Enforced in both `P` and `V` modes |

### 3.2 Dual-Mode Actuation
* **Absolute Waypoint Mode (`commandAbsolutePositionMrad()` — `P` command):** Sets `currentMotionMode = MODE_POSITION`, programs `MAXSPEEDHZ`, clamps target steps to `[MIN_STEPS, MAX_STEPS]`, and issues `stepper->moveTo(targetSteps)`. Used for large ballistic slews, auto-homing, and the Raspberry Pi's 7-stop daylight site calibration sweep.
* **Continuous Velocity Slew Mode (`applyVelocityActuation()` — `V` command):** Runs every `loop()` iteration when `currentMotionMode == MODE_VELOCITY`:
  * Zeroes commanded speed immediately if the axis has reached or exceeded `MIN_STEPS` / `MAX_STEPS` in the direction of travel.
  * Applies a **`40 Hz` speed hysteresis** (`VELOCITY_SPEED_HYSTERESIS_HZ`) on upward speed ramps (while allowing immediate downward deceleration or direction reversals) so `FastAccelStepper` ramp planning is never thrashed by minor 1–2 Hz quantization jitter.

---

## 4. Solenoid Timing State Machine & Safety Cutoffs

`updateSolenoid()` implements a non-blocking 3-state hardware cycle on `PIN_SOLENOID` (`GPIO 13`):

```text
  [SOLENOID_IDLE] ──(triggerSolenoid())──► [SOLENOID_FIRING] (GPIO 13 = HIGH)
         ▲                                         │
         │                                (60 ms elapsed)
         │                                         ▼
  (190 ms elapsed) ◄───────────────────── [SOLENOID_COOLDOWN] (GPIO 13 = LOW)
```

* **Cadence Limit:** `60 ms` energized pulse + `190 ms` mandatory cooldown = `250 ms` cycle (`4.0` shots/sec maximum), preventing coil overheating and ensuring full mechanical plunger return.
* **Hard Autonomous Cutoff (`2,000 ms`):** Whenever `T 1` enables `autoFireRequested`, `autoFireStartTime` is latched. Even if the Raspberry Pi crashes or drops serial communication mid-burst, `loop()` forcibly clears `autoFireRequested` after `MAX_AUTONOMOUS_FIRE_DURATION_MS` (`2,000 ms`), and the `500 ms` serial link watchdog disarms firing within half a second of link loss.
* **Emergency Halt (`X` and `H`):** Immediately writes `digitalWrite(PIN_SOLENOID, LOW)`, clears `autoFireRequested`, and transitions any active pulse straight into `SOLENOID_COOLDOWN`.

---

## 5. Xbox Series X Bluetooth Manual Override (`processXboxOverride()`)

`processXboxOverride()` polls `xboxController.onLoop()` on every iteration and takes complete precedence over Raspberry Pi serial commands whenever manual input is detected:

1. **Input Mapping & Quadratic Shaping:**
   * Left stick horizontal (`joyLHori`, `DEADZONE_RADIUS = 0.30`) -> inverted Pan velocity (`-rawPan`) with quadratic curve (`powf(normPct, 2.0f)`) and `300 Hz` joystick hysteresis for fine low-speed aiming.
   * Left stick vertical (`joyLVert`, `DEADZONE_RADIUS = 0.30`) -> Tilt velocity (`rawTilt`) with quadratic curve shaping.
   * Right trigger (`trigRT > 0.15`) -> holds `autoFireRequested = true` while squeezed (releasing immediately cuts `autoFireRequested = false`).
   * **`'A'` Button (Single-Shot Edge):** Calls `forceStopAndNewPosition(0)` on both steppers to zero the mechanical `(0, 0) mrad` home coordinate on demand (ideal for leveling the turret before running the Raspberry Pi's site boot sweep and Walk-Test).
2. **Release Lockout (`400 ms`):** When manual inputs return to neutral, `manualOverrideActive` remains `true` for `MANUAL_OVERRIDE_TIMEOUT_MS` (`400 ms`) while draining stale serial bytes (`Serial.read()`) so queued Pi commands cannot jerk the turret immediately after stick release.
