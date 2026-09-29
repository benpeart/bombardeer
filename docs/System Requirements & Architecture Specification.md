# System Requirements & Architecture Specification: Bombardeer Autonomous Turret

## 1. System Summary

Build a two-tier, fault-tolerant autonomous target-tracking and deterrent turret system called **Bombardeer**. The system distributes compute tasks across a **Raspberry Pi (Host Vision & Target Policy)** and an **ESP32 (Hardware Real-Time Motion Control & Actuation)** linked via high-speed USB-UART.

The system tracks visual targets (ArUco markers / future object models), maps image-space coordinate errors into real-world milliradians, and commands atomic waypoints (`P`) for the ESP32 to execute using smooth hardware-timed trapezoidal acceleration profiles.

---

## 2. Hardware Architecture & Pinout Requirements

### 2.1 Microcontrollers & Connectivity

* **Host Processor:** Raspberry Pi 5 running Linux.


* **Motion Controller:** ESP32 NodeMCU / DevKit V1.


* **Host-Controller Bus:** USB-UART via `/dev/ttyUSB0` (or `/dev/ttyACM0`) operating at **921,600 baud, 8N1**.


* **Driver Bus:** Dedicated ESP32 `Serial2` hardware UART for TMC2209 driver communication operating at **115,200 baud**.



### 2.2 ESP32 Pin Allocation

* **Pan Stepper Driver (TMC2209 Address `0b00`):**
* `STEP`: GPIO 26


* `DIR`: GPIO 25


* `USTEP1` (MS1): GPIO 04 (Tied to GND/LOW)


* `USTEP2` (MS2): GPIO 27 (Tied to GND/LOW)




* **Tilt Stepper Driver (TMC2209 Address `0b01`):**
* `STEP`: GPIO 33


* `DIR`: GPIO 32


* `USTEP1` (MS1): GPIO 18 (Tied to VCC/HIGH)


* `USTEP2` (MS2): GPIO 05 (Tied to GND/LOW)




* **Shared Stepper Enable:** GPIO 19 (Active LOW).


* **TMC2209 Single-Wire Half-Duplex Bus:**
* `TMC_RX`: GPIO 16 (ESP32 RX2 connected directly to TMC2209 UART pin).


* `TMC_TX`: GPIO 17 (ESP32 TX2 connected through a $1\text{ k}\Omega$ series resistor to the same single-wire line).




* **Solenoid Actuator Output:** GPIO 13 (Drives logic-level MOSFET / relay circuit).



### 2.3 Stepper Motors & Mechanical Transmission

* **Motors:** 1.8° NEMA 17 steppers (200 full steps/rev, 16 microsteps $\to$ 3,200 steps/motor rev).


* **Angular Units:** Native milliradians ($1\text{ full rev} = 2000\pi \approx 6283.1853\text{ mrad}$).


* **Pan Axis:**
* Ring Gear: 246 teeth; Pinion Gear: 37 teeth (Gear ratio $\approx 6.6486:1$).


* Resolution: $\approx 3.386\text{ steps/mrad}$ ($\approx 0.295\text{ mrad/step}$).


* Software Limits: $\pm 180.0^\circ$ ($\pm 3141\text{ mrad}$).




* **Tilt Axis:**
* Worm Gear: 50 teeth; Worm: 1 start (Gear ratio: $50.0:1$).


* Resolution: $\approx 25.465\text{ steps/mrad}$ ($\approx 0.039\text{ mrad/step}$).


* Software Limits: $\pm 22.5^\circ$ ($\pm 392\text{ mrad}$).





### 2.4 Sensor & Optical Alignment

* **Camera:** Raspberry Pi Camera Module 3 Wide (IMX708 NoIR).


* **Intrinsics:** Native frame resolution 1280x720, focal length $f \approx 540.4\text{ px}$.


* **Bore-to-Sensor Parallax Offsets:**
* Camera X Offset: $-170.0\text{ mm}$ (Camera is mounted 170 mm left of bore axis).


* Camera Y Offset: $-60.0\text{ mm}$ (Camera is mounted 60 mm below bore axis).





---

## 3. ESP32 Firmware Requirements (`main.cpp`)

### 3.1 Motion Engine

* Must use `FastAccelStepper` for jitter-free, hardware-timer step generation.


* Configure Pan acceleration to 13,500 steps/s² and max speed to 11,600 Hz.


* Configure Tilt acceleration to 50,000 steps/s² and max speed to 14,800 Hz.


* Support atomic setpoint target updates via `moveTo()`. Calling `moveTo()` mid-transit must dynamically update the destination without stopping or interrupting motor velocity curves.



### 3.2 TMC2209 Configuration

* Initialize via `TMCStepper` library over `Serial2`.


* Configure Pan for 1200 mA RMS active current, 0.25 hold multiplier.


* Configure Tilt for 1100 mA RMS active current, 0.50 hold multiplier (critical to avoid gravity-induced cantilever droop when motors stop).


* Enable Hybrid StealthChop/SpreadCycle mode with `TPWMTHRS = 200` to maintain silent standstill while guaranteeing high-speed torque.



### 3.3 Actuator Control State Machine

* Non-blocking Heschen solenoid pulse state machine:
* `SOLENOID_PULSE_LENGTH`: 60 ms ON.


* `SOLENOID_COOLDOWN_LENGTH`: 190 ms OFF (enforces 4.0 BPS max cyclic rate).


* Hard failsafe timeout: Autonomous firing cannot exceed 2000 ms continuous duration.





### 3.4 Fallback & Overrides

* **Xbox Series X Bluetooth Controller:** Support wireless manual override via `XboxSeriesXControllerESP32_asukiaaa`. Left stick controls velocity, right trigger actuates solenoid, 'A' button sets current physical position as Home zero (`0, 0`). When manual input is detected, host serial motion commands are locked out for 400 ms.


* **Host Watchdog:** If no valid serial byte is received from the Raspberry Pi for 500 ms, automatically ramp down velocity to zero and halt all motion.



---

## 4. Communication Protocol Requirements

### 4.1 Host-to-Controller Commands (Pi $\to$ ESP32)

Lines must be ASCII strings terminated with `\n`:

* `P <pan_mrad:long> <tilt_mrad:long>\n` — Absolute position waypoint command.


* `T <state:0|1>\n` — Solenoid trigger state command.


* `X\n` — Emergency/controlled halt.


* `H\n` — Re-zero current motor positions in place.


* `Y <timestamp_pi:unsigned long>\n` — Clock-sync ping request.



### 4.2 Controller-to-Host Telemetry (ESP32 $\to$ Pi)

Broadcast continuously at **20 Hz (every 50 ms)**:

```text
S <timestamp_ms> <pan_mrad> <tilt_mrad> <firing_active> <is_moving>\n

```

* Response to `Y` must immediately echo: `Y <timestamp_pi> <esp32_now_ms>\n`.



---

## 5. Raspberry Pi Software Requirements (`bombardeer.py`)

### 5.1 Vision & Perception Pipeline

* **Engine:** `picamera2` capturing dual streams:
* Main stream: 1280x720 RGB888 native.


* Lores stream: 640x360 RGB888 for low-latency web HUD visualization.


* Preserve full 33.3 ms exposure integration window for low-light dawn/dusk sensitivity.




* **Detector:** OpenCV ArUco detector (Dictionary 4x4_50, ID 0) with CLAHE contrast enhancement.


* **Optical Mapping:** Must use **true pinhole ray projection** via `math.atan2` rather than linear pixel scaling to eliminate peripheral lens distortion:



$$\Delta x = -(x_{\text{center}} - \text{OPTICAL\_CENTER}_x) \text{[cite: 2]}$$


$$\Delta y = -(y_{\text{center}} - \text{OPTICAL\_CENTER}_y) \text{[cite: 2]}$$


$$\text{Error}_{\text{pan}} = \arctan\left(\frac{\Delta x}{f_x}\right) \times 1000 + \text{Parallax}_{\text{pan}} \text{[cite: 2]}$$


$$\text{Error}_{\text{tilt}} = \arctan\left(\frac{\Delta y}{f_y}\right) \times 1000 + \text{Parallax}_{\text{tilt}} \text{[cite: 2]}$$


* **Parallax Range Compensation:**

$$\text{Parallax}_{\text{pan}} = \left(\frac{-\text{CAMERA\_OFFSET\_X\_MM}}{\text{range\_m} \times 1000}\right) \times 1000 \text{[cite: 2]}$$


$$\text{Parallax}_{\text{tilt}} = \left(\frac{\text{CAMERA\_OFFSET\_Y\_MM}}{\text{range\_m} \times 1000}\right) \times 1000 \text{[cite: 2]}$$



### 5.2 State Estimation & Target Filtering

* **World-Space Kalman Filter:** A 4-state linear Kalman Filter tracking the target in **Global Turret Coordinates (mrad)** with dynamic $\Delta t$:



$$\vec{x} = [\text{pan}, \text{tilt}, v_{\text{pan}}, v_{\text{tilt}}]^T \text{[cite: 2]}$$


* State transition matrix adapts dynamically to frame interval $\Delta t$.


* Damps high-frequency image sensor noise ($\pm 3\text{ to }7\text{ mrad}$ baseline pixel jitter) without lagging behind target moves.




* **Motion Deadbands & Firing Policy:**
* `DEADBAND_MRAD`: $30.0\text{ mrad}$ (Once inside this error, suppress waypoint dispatching to eliminate hunting).


* `FIRE_DEADBAND_MRAD`: $40.0\text{ mrad}$ (Permit solenoid actuation only when radial error is strictly within this radius).


* Stream setpoint updates at a stable 20 Hz rate whenever radial error exceeds `DEADBAND_MRAD`.





### 5.3 Zero-Latency Serial Ingestion & Supervision

* **Buffer Drain Engine:** Must use non-blocking chunk reading (`ser.read(ser.in_waiting)`) to drain operating system serial buffers completely on each iteration, preventing queue latency buildup.


* **Hardware Reset Suppression:** On serial port initialization, explicitly clear `dtr = False` and `rts = False` to prevent spurious DTR toggling that forces the ESP32 into reset loops.


* **Non-Destructive Error Handling:** Do not call `ser.close()` on transient framing errors or string conversion exceptions; isolate parsing errors on a per-line basis.


* **Kinematic Ring Buffer:** Maintain a sliding history of physical turret positions indexed by monotonic capture timestamps to compute historical alignment during exposure.



### 5.4 Diagnostic Web Console & Auto-Archiver

* Provide a multi-threaded Flask web server on port 5000 streaming an MJPEG feed with targeting crosshairs, parallax convergence indicators, and active system telemetry.


* Automatically trigger MP4 video recording (H.264 / MP4V) upon target lock with 2.0 seconds of pre-roll and 3.0 seconds of post-roll. Ensure automated disk management purges oldest recordings if free space drops below 5.0 GB.



---

## 6. Verification Criteria & Acceptance Tests

1. **Low-Latency Serial Link:** Telemetry round-trip time (RTT) measured on the host must remain below 10.0 ms during full-speed motion.


2. **Single-Stroke Alignment:** When an off-axis target appears at $>400\text{ mrad}$ offset, the turret must calculate physical coordinates, accelerate, slew, and land inside the $30\text{ mrad}$ deadband in a single continuous movement without stuttering, overshoot, or intermediate stop-and-go steps.


3. **No Standstill Hunting:** When stationary on target, optical pixel noise must be absorbed by the Kalman filter and deadband; the turret must generate zero motor steps while holding position.


4. **Boot Stability:** The Raspberry Pi script must be capable of launching before or after the ESP32 powers on without triggering serial reset loops or unhandled exceptions.