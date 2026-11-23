# ESP32 Build, Flash & Debugging Guide

This guide covers building and flashing the ESP32 turret controller firmware (`ESP32/src/main.cpp`), testing commands over USB serial, pairing the Xbox Series X manual override controller, and diagnosing common TMC2209 or motion issues.

---

## 1. PlatformIO Build & Flash Instructions

### 1.1 Required Libraries (`platformio.ini`)
Ensure your PlatformIO environment targets `esp32dev` at `921600` monitor/upload baud and includes:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 921600
upload_speed = 921600
lib_deps =
    teemuatlut/TMCStepper @ ^0.7.3
    gin66/FastAccelStepper @ ^0.30.15
    asukiaaa/XboxSeriesXControllerESP32_asukiaaa @ ^1.0.9
```

### 1.2 Compile & Upload
From the `ESP32/` project directory (disconnect the USB cable from the Raspberry Pi or stop `bombardeer.service` first if flashing directly from the Pi):

```bash
# Build firmware
pio run

# Upload firmware over USB
pio run --target upload
```

*(Note: If your ESP32 board lacks an auto-boot capacitor on `EN`/`IO0`, hold the **BOOT** button on the ESP32 until `Connecting...` transitions to `Writing at 0x00010000...`.)*

---

## 2. Debug Logging Configuration (`debug.h`)

The `921,600` baud USB `Serial` port carries the live `20 Hz` `S` telemetry stream consumed by the Raspberry Pi.

* **Production Builds:** Keep `DB_PRINT` / `DB_PRINTLN` / `DB_PRINTF` disabled (or routed only during bench testing) so human-readable `[TMC2209]` / `[SOLENOID]` messages do not interleave with `S` telemetry lines while `bombardeer.service` is running.
* **Bench Diagnostics:** Enable `DB_PRINTF` in `ESP32/include/debug.h` when inspecting TMC2209 register read-backs, solenoid pulse timing, or Xbox controller pairing over a serial terminal.

---

## 3. Standalone USB Serial Verification

Before connecting the ESP32 to the Raspberry Pi service, verify all motion and telemetry packets from a serial terminal at **`921600` baud** (with **Newline `\n`** line endings enabled):

1. **Verify `20 Hz` Telemetry Stream:**
   Immediately after boot, the ESP32 emits state packets every `50 ms`:
   ```text
   S 1250 0 0 0 0
   S 1300 0 0 0 0
   ```
   Fields: `S <uptime_ms> <pan_mrad> <tilt_mrad> <firing> <moving>`.
2. **Test Zero-Home (`H`):**
   Send `H\n` -> confirms `pan_mrad` and `tilt_mrad` report `0 0`.
3. **Test Absolute Waypoint Snap (`P`):**
   Send `P 100 50\n` -> pan slews `+100 mrad` (`~5.7°`) and tilt slews `+50 mrad` (`~2.9°`), with `<moving>` transitioning `1 -> 0` and final telemetry settling at `S <ms> 100 50 0 0`.
   Send `P 0 0\n` to return to home.
4. **Test Velocity Slew & `500 ms` Watchdog (`V`):**
   Send `V 200 0\n` -> pan begins slewing at `200 mrad/s`, then automatically decelerates to a stop `500 ms` later when no follow-up `V` packet arrives (verifying the host-disconnect watchdog).
5. **Test Emergency Halt (`X`):**
   Send `P 500 0\n` followed immediately by `X\n` -> motion halts mid-sweep.
6. **Test Solenoid Single-Cycle (`T`):**
   With `24V` energized and barrel clear, send `T 1\n` followed within `100 ms` by `T 0\n` -> solenoid fires a single `60 ms` pulse and returns to idle.

---

## 4. Pairing the Xbox Series X/S Controller

1. Power on the ESP32 (`xboxController.begin()` starts active BLE scanning automatically).
2. Turn on the Xbox Series X/S controller and hold its top **Pair** button (`((( `) until the Xbox logo flashes rapidly.
3. Within `5–10 seconds`, the Xbox logo turns solid white, indicating BLE connection.
4. **Verify Manual Override:**
   * Move the **Left Stick** -> pan and tilt axes slew smoothly with quadratic speed response.
   * Press **'A'** -> current turret position zeroes to `0 0` in the `S` telemetry stream.
   * Pull **Right Trigger (RT)** -> solenoid pulses at `4.0 BPS` (`60 ms` on / `190 ms` off) while held and stops immediately on release.

---

## 5. Troubleshooting Guide

| Symptom | Likely Cause | Fix |
| :--- | :--- | :--- |
| **`[TMC2209] ERROR: PAN/TILT axis connection failed!`** | Missing `3.3V` on `VIO`, missing `1 kΩ` resistor on `TX2 (GPIO 17)`, or swapped `MS1`/`MS2` pins | Verify `VIO = 3.3V`, check continuity from `GPIO 16` and `GPIO 17 -> 1 kΩ` to `PDN_UART` on both drivers, and confirm `PAN` (`MS1=0, MS2=0`) vs `TILT` (`MS1=1, MS2=0`). |
| **`[TMC2209] ERROR: register read-back failed!`** | TMC2209 V2 board with `PDN_UART` routed to a different pin header, or `VM` (`24V`) absent | On certain BigTreeTech TMC2209 V2 boards, verify which header pin connects to the TMC2209 `PDN_UART` IC pin (pin 14) and ensure `24V` `VM` is energized. |
| **Motors hum or vibrate without rotating** | Stepper coil pairs crossed (`A1/B1` swapped) or `SHARED_ENABLE_PIN (GPIO 19)` floating | Power off `24V`, verify coil continuity with a multimeter (`A1-A2` pair, `B1-B2` pair), and check `GPIO 19` is `LOW` after setup. |
| **ESP32 reboots every time `bombardeer.service` restarts** | Custom serial adapter asserting `DTR`/`RTS` or older kernel `HUPCL` behavior | Confirm CP2102 adapter is used and verify `turret_serial_worker()` in `bombardeer_pkg/hardware/serial_link.py` is up to date. |
| **Xbox controller connects then immediately disconnects** | Outdated Xbox controller firmware (BLE pairing incompatibility) | Update the Xbox Series X/S controller firmware via the Xbox Accessories app on Windows/Xbox, then re-pair. |
| **Solenoid stays energized or fails to fire** | Missing `10 kΩ` gate pull-down resistor on `GPIO 13` or reversed `1N5408` flyback diode | Verify `GPIO 13` reads `0V` at idle, check `1N5408` cathode stripe faces `+24V`, and test with `T 1` / `T 0`. |