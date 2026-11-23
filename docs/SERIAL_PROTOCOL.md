# Serial Communication Protocol Specification

The Raspberry Pi 5 (`bombardeer_pkg/hardware/serial_link.py`) and ESP32 (`ESP32/src/main.cpp`) communicate over a bidirectional USB-UART link at **`921,600` baud, 8N1**, exchanging newline-terminated (`\n`) ASCII packets in native **milliradians (`mrad`)** and **milliradians per second (`mrad/s`)**.

Decoupling the wire protocol into angular milliradians keeps all kinematics, gear ratios (`246/37` pan, `50/1` tilt), microstepping (`1/16`), and acceleration profiles encapsulated inside the ESP32 firmware.

---

## 1. Physical Link & Non-Reset Serial Discipline

* **Port Discovery (`config.yaml`):** The Raspberry Pi searches in priority order:
  1. `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
  2. `/dev/ttyUSB0`
  3. `/dev/ttyACM0`
  4. `/dev/ttyUSB1`
* **Preventing ESP32 Auto-Reset on Service Restart:** Standard serial opens assert `DTR` and `RTS`, which triggers the auto-reset transistor circuit on ESP32 dev boards and wipes the turret's mechanical `(0, 0)` home position whenever `bombardeer.service` restarts. `turret_serial_worker()` prevents this by:
  1. Instantiating `serial.Serial(..., dsrdtr=False, rtscts=False)` with `port=None`.
  2. Setting `ser.dtr = False` and `ser.rts = False` **before** calling `ser.open()`.
  3. Clearing the POSIX `HUPCL` flag via `termios.tcsetattr()` immediately after open so kernel `close()` never toggles `DTR` on shutdown.
* **Framing & Watchdogs:**
  * **ESP32 RX Parser:** Non-blocking 64-byte line buffer with a `250 ms` partial-frame timeout and strict `strtol()` validation (malformed or overflowing payloads are silently discarded).
  * **ESP32 Link Watchdog (`500 ms`):** If no valid command arrives for `500 ms` while autonomous mode is active, the ESP32 zeroes velocity commands, decelerates both steppers to a stop, and disarms `autoFireRequested`.
  * **Pi Link Watchdog (`2.0 s` / `5.0 s`):** Marks `serial_connected = False` if no telemetry arrives for `2.0 s` (clearing any active target lock) and forces a port reconnect after `5.0 s` of silence.

---

## 2. Commands: Raspberry Pi -> ESP32

| Command | Format | Units | Rate / Priority | Description | Example |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`P`** | `P <pan> <tilt>\n` | `mrad` (int32) | Max 1 per `400 ms`, min `60 mrad` delta | **Atomic Ballistic Setpoint:** Switches ESP32 to `MODE_POSITION` and executes a hardware-timed trapezoidal move to `<pan>, <tilt>` clamped to physical limits (`±3,141` pan, `±392` tilt). | `P 280 -65` |
| **`V`** | `V <pan_s> <tilt_s>\n` | `mrad/s` (int32) | Up to `25 Hz` (`40 ms` min interval) | **Continuous Velocity Pursuit:** Switches ESP32 to `MODE_VELOCITY` and slews axes at `<pan_s>, <tilt_s>` (clamped by Pi to `±850` pan, `±450` tilt). | `V 320 -85` |
| **`X`** | `X\n` | — | Immediate (High Priority) | **Emergency Halt:** Cuts `PIN_SOLENOID` `LOW` immediately, cancels `autoFireRequested`, zeroes velocity setpoints, and calls `forceStop()` on both axes. | `X` |
| **`T`** | `T <0\|1>\n` | flag (`0` or `1`) | Immediate (High Priority) | **Autonomous Trigger Control:** `1` arms the solenoid pulse/cooldown state machine (subject to a `2,000 ms` hardware cutoff); `0` disarms firing after the active `60 ms` pulse finishes. | `T 1` |
| **`H`** | `H\n` | — | On-demand | **Zero Home Coordinates:** Disarms solenoid, decelerates both axes to a stop (`1,500 ms` bounded wait), and sets current motor step positions to `(0, 0) mrad`. | `H` |
| **`Y`** | `Y <pi_ms>\n` | `ms` (uint32) | Optional diagnostic | **Clock Sync Ping:** ESP32 immediately echoes `Y <pi_ms> <esp32_ms>\n` for round-trip latency measurement. | `Y 10482910` |

### CommandBus Coalescing (`bombardeer_pkg/state.py`)
To ensure high-rate `25 Hz` `V` packets never queue up behind slow I/O or delay an emergency stop:
* **`X` (Halt)** and **`T` (Trigger)** are stored in dedicated high-priority slots and drained ahead of motion commands.
* **`P` (Setpoint)** and **`V` (Velocity)** overwrite any pending unsent motion command (`latest-wins` semantics) so the ESP32 always receives the freshest trajectory vector.

---

## 3. Telemetry: ESP32 -> Raspberry Pi

| Packet | Format | Units | Cadence | Description | Example |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`S`** | `S <time_ms> <pan> <tilt> <firing> <moving>\n` | `ms`, `mrad`, `mrad`, `0\|1`, `0\|1` | **20 Hz** (`50 ms`) | **Periodic State Broadcast:** Reports ESP32 `millis()`, current pan/tilt angles in `mrad`, whether solenoid firing is active, and whether either stepper is currently stepping. | `S 1840291 195 -45 0 1` |
| **`Y`** | `Y <pi_ms> <esp32_ms>\n` | `ms`, `ms` | Immediate reply | **Time Sync Echo:** Returned in response to `Y <pi_ms>`. | `Y 10482910 1840312` |

### Telemetry Consumption on the Raspberry Pi
1. `turret_serial_worker()` parses each `S` line and calls `state.update_telemetry(pan_mrad, tilt_mrad, is_moving, is_firing)`.
2. `SystemState` updates instantaneous telemetry and appends `(time.monotonic(), pan_mrad, tilt_mrad)` to a 60-sample (`~3.0 s`) circular `telemetry_history` deque.
3. When `VisionMain` confirms a detection captured at mid-exposure timestamp $t_{\text{capture}}$, `state.get_turret_pos_at_time(t_capture)` linearly interpolates `(pan_mrad, tilt_mrad)` from `telemetry_history` so turret motion during the `110–345 ms` NPU inference window introduces zero angular error.

---

## 4. Milliradian-to-Step Conversion Constants (`ESP32/src/main.cpp`)

| Parameter | Pan Axis (`246T / 37T`) | Tilt Axis (`50T / 1T`) | Formula |
| :--- | :---: | :---: | :--- |
| **Motor Steps / Rev (`16×` ustep)** | `3,200.0` | `3,200.0` | `200.0 × 16` |
| **Gear Reduction Ratio** | `6.648649:1` | `50.000000:1` | `Ring / Pinion` or `Wheel / Worm` |
| **Axis Steps / Revolution** | `21,275.676` | `160,000.0` | `3,200 × Gear Ratio` |
| **Steps per Milliradian** | **`3.386129`** | **`25.464791`** | `Axis Steps / (2000 × π)` |
| **Milliradians per Step** | `0.295322 mrad` | `0.039270 mrad` | `1 / Steps per mrad` |
| **Software Travel Envelope** | `±3,141 mrad` (`±180.0°`) | `±392 mrad` (`±22.5°`) | Enforced in `P` and `V` handlers |