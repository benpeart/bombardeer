# Waypoint Servoing & Low-Latency Link Architecture

## 1. System Overview

The Bombardeer autonomous tracking turret distributes computing tasks across two specialized microprocessors connected over an asynchronous USB-UART link:

```
[ Raspberry Pi 5 (picamera2) ] 
       │  High-resolution optical tracking (IMX708 NoIR)
       │  Pinhole distortion correction (math.atan2)
       │  World-space target estimation & Kalman filtering
       │  Command scheduling & trigger policy
       ▼  UART: 921,600 Baud ('P <pan> <tilt>', 'T <0|1>')
[ ESP32 Motor Controller ]
       │  FastAccelStepper deterministic pulse generation
       │  TMC2209 silent stepper drivers (StealthChop/SpreadCycle)
       │  Heschen solenoid trigger pulse state machine
       │  20 Hz deterministic telemetry stream ('S <t> <p> <t> <f> <m>')
       ▼
[ Pan/Tilt Gimbals & Solenoid Actuator ]

```

The system uses absolute waypoint servoing (`P`) rather than continuous velocity nudging (`V`). The host vision system estimates the target's physical coordinates in real-world milliradians and transmits an atomic destination point. The ESP32's hardware timers execute smooth S-curve/trapezoidal acceleration profiles directly to the coordinate, eliminating transport jitter from the motion trajectory.

---

## 2. Hardware Configuration & Kinematics

### Coordinate Units & Physical Ratios

All rotational angles and errors are computed natively in **milliradians (mrad)**:

* Full Circle = $2000\pi \approx 6283.1853\text{ mrad}$

* Motors: 1.8° NEMA 17 steppers (200 full steps/rev $\times$ 16 microsteps = 3,200 steps/rev)



$$\text{Pan Ratio} = \frac{246\text{ ring teeth}}{37\text{ pinion teeth}} \approx 6.6486 \implies \text{PAN\_STEPS\_PER\_MRAD} \approx 3.386 \text{ steps/mrad} \text{[cite: 1]}$$

$$\text{Tilt Ratio} = \frac{50\text{ worm wheel teeth}}{1\text{ start}} = 50.0 \implies \text{TILT\_STEPS\_PER\_MRAD} \approx 25.465 \text{ steps/mrad} \text{[cite: 1]}$$

### Stepper Driver & Actuator Settings

* **Pan Driver (TMC2209 Address `0b00`):** 1200 mA RMS run current, 0.25–0.35 hold current multiplier.


* **Tilt Driver (TMC2209 Address `0b01`):** 1100 mA RMS run current, 0.50 hold current multiplier (prevents vertical cantilever sag on standstill).


* **UART Bus:** Single-wire half-duplex on ESP32 `Serial2` (GPIO 16 RX, GPIO 17 TX via a $1\text{ k}\Omega$ series resistor) operating at **115,200 baud**.


* **Solenoid Actuator (GPIO 13):** Heschen pull-type solenoid with a 60 ms active fire pulse followed by a 190 ms cooldown window (maximum 4.0 BPS cycle rate).



---

## 3. Communication Protocol

The host-to-controller link operates over `/dev/ttyUSB0` at **921,600 baud, 8N1**.

### Host Commands (Raspberry Pi $\to$ ESP32)

* `P <pan_mrad> <tilt_mrad>\n` — Command absolute physical waypoint targets.


* `T <0|1>\n` — Set autonomous solenoid trigger state.


* `X\n` — Command immediate dynamic controlled halt and lock current steps.


* `H\n` — Zero current motor coordinates in place.



### Controller Telemetry (ESP32 $\to$ Raspberry Pi)

Broadcast deterministically at **20 Hz (every 50 ms)**:

```text
S <timestamp_ms> <pan_mrad> <tilt_mrad> <firing_active> <is_moving>\n

```

* `<timestamp_ms>`: ESP32 local clock timestamp via `millis()`.


* `<pan_mrad>`, `<tilt_mrad>`: Current physical motor coordinates calculated from step registers.


* `<firing_active>`: `1` if solenoid is energized or in fire cycle, else `0`.


* `<is_moving>`: `1` if either `FastAccelStepper` channel is actively generating step pulses, else `0`.



---

## 4. Key Engineering Challenges & Solutions

### A. OS Serial Buffer Bloat (989 ms $\to$ 2 ms Latency)

* **Problem:** In early tests, round-trip latency climbed to nearly 1,000 ms. Standard line-by-line reads (`readline()`) combined with thread sleep intervals were slower than incoming 20 Hz telemetry, causing stale serial frames to accumulate in Linux kernel buffers. The vision loop was calculating errors based on where the turret was one second in the past.
* **Solution:** Replaced line-by-line reading with a non-blocking chunk drainer (`ser.read(ser.in_waiting)`). On every worker tick, the entire hardware buffer is ingested, split into complete newline-delimited tokens, and only the newest telemetry state is latched. Telemetry round-trip latency dropped to **2.2 ms**.

### B. High-Speed Baud Mismatch & USB Auto-Reset Traps

* **Problem:** When increasing link performance, `main.cpp` was compiled at 921,600 baud while `bombardeer.py` was set to 115,200 baud. The baud rate mismatch caused telemetry frames to arrive as framing-error noise. An aggressive watchdog script misinterpreted the missing telemetry as a link drop and rapidly closed and reopened the port. On Linux, cycling `/dev/ttyUSB0` toggles the CP2102/CH340 DTR/RTS lines, grounding the ESP32 `EN` pin and throwing the hardware into an infinite bootloader reset loop.


* **Solution:** Matched the serial configuration on both ends to 921,600 baud. In Python, `ser.dtr = False` and `ser.rts = False` are asserted immediately upon opening, and the watchdog policy was converted to a soft state change rather than closing and tearing down the physical connection.



### C. Wide-Angle Peripheral Optical Distortion

* **Problem:** Using a simple linear scaling constant ($\text{error} = \Delta \text{px} \times K$) worked near the crosshairs, but introduced a 60+ mrad offset when targets appeared near the edges of the IMX708 Wide lens. The initial slew command would launch toward $+245\text{ mrad}$ when the true physical target was at $+184\text{ mrad}$, requiring multiple corrective moves to settle.
* **Solution:** Replaced small-angle linear approximations with exact pinhole ray projections:

$$\Delta x = -(x_{\text{center}} - x_{\text{optical\_center}})$$


$$\text{Error}_{\text{pan}} = \arctan\left(\frac{\Delta x}{f_x}\right) \times 1000 + \text{Parallax}_{\text{pan}}$$



This eliminated off-axis geometric distortion, allowing the primary ballistic slew to land within the target deadband on the first attempt.

### D. Sub-Pixel Noise & Deadband Hysteresis

* **Problem:** Normal camera shot noise and ArUco corner extraction jitter cause stationary targets to fluctuate by $\pm 2\text{ to }4\text{ pixels}$ ($\pm 3\text{ to }7\text{ mrad}$). Without filtering, these fluctuations repeatedly crossed the motion threshold, re-triggering small stepper adjustments and hunting.
* **Solution:** Built a 4-state constant-velocity Kalman Filter (`TargetKalmanFilter`) running in **World Milliradian Space** ($[\text{pan}, \text{tilt}, v_{\text{pan}}, v_{\text{tilt}}]^T$). Because a stationary target has an invariant world coordinate regardless of turret motion, the filter dampens high-frequency optical noise while preserving rapid response. Paired with a $30.0\text{ mrad}$ tracking deadband and a $40.0\text{ mrad}$ fire deadband, post-arrival jitter is completely ignored.

---

## 5. End-to-End Execution Trace

The following production log illustrates an acquisition, slew, deceleration, and firing sequence:

```text
12:21:52.800 [INFO] [TRACK] Cur:(  +11,+228) | Err:(-433.1,-252.2) Rad:501.2 | Goal:( -422, -24) | Mv:0
12:21:52.801 [INFO] [SERIAL TX] -> P -422 -24
12:21:52.867 [INFO] [SERIAL TX] -> P -423 -26
12:21:52.929 [INFO] [SERIAL TX] -> P -429 -32
12:21:52.952 [INFO] [TRACK] Cur:(  -20,+214) | Err:(-425.1,-257.7) Rad:497.1 | Goal:( -433, -38) | Mv:1
12:21:53.901 [INFO] [TRACK] Cur:( -429, -32) | Err:(-14.2,+11.7) Rad:18.5 | Goal:( -443, -20) | Mv:0
12:21:53.901 [INFO] [SERIAL] Solenoid trigger pulse FIRED
12:21:54.066 [INFO] [TRACK] Cur:( -429, -32) | Err:(-23.2,-1.7) Rad:23.3 | Goal:( -452, -35) | Mv:0
12:21:54.231 [INFO] [TRACK] Cur:( -429, -32) | Err:(-15.4,+7.0) Rad:16.9 | Goal:( -444, -24) | Mv:0
12:21:55.231 [INFO] [TRACK] Cur:( -429, -32) | Err:(-20.6,+1.9) Rad:20.6 | Goal:( -450, -30) | Mv:0

```

### Trace Breakdown:

1. **Target Sighting (`12:21:52.800`):** Turret is resting at `(+11, +228) mrad` with motors stationary (`Mv: 0`). Vision detects a target at $(-433.1, -252.2)\text{ mrad}$ displacement. Goal is computed as `(-422, -24) mrad`. Initial waypoint `P -422 -24` is dispatched over serial.


2. **Acceleration & Slew (`12:21:52.867 – 12:21:52.952`):** ESP32 initiates step acceleration curves. As the initial movement begins, subsequent frames refine the world-space target to `(-429, -32) mrad`. `FastAccelStepper` updates its active destination mid-stroke without interrupting motor acceleration. ESP32 reports `Mv: 1`.


3. **Ballistic Transit Window (`12:21:52.952 – 12:21:53.901`):** Turret sweeps through $409\text{ mrad}$ Pan and $246\text{ mrad}$ Tilt in ~950 ms. High-speed angular motion creates camera motion blur. The fresh-detection gate suppresses command generation during the brief optical dropout, allowing the ESP32 to track cleanly toward the designated waypoint.
4. **Touchdown & Deceleration (`12:21:53.901`):** ESP32 arrives at the target steps, halts pulse generation, and reports `Cur: (-429, -32) | Mv: 0`. Camera sharpness returns immediately. Residual optical error is measured at $(-14.2, +11.7)\text{ mrad}$ ($\text{Radial} = 18.5\text{ mrad}$).


5. **Authorization to Fire (`12:21:53.901`):** Because radial error ($18.5\text{ mrad}$) is inside `FIRE_DEADBAND_MRAD` ($40.0\text{ mrad}$), the Pi transmits the fire pulse command. The ESP32 pulls GPIO 13 high to actuate the solenoid.


6. **Stationary Lock (`12:21:54.066 – 12:21:55.231`):** Subsequent frames report optical errors fluctuating between $16.9\text{ mrad}$ and $23.3\text{ mrad}$ due to sensor noise. Because all values remain safely within `DEADBAND_MRAD` ($30.0\text{ mrad}$), no further setpoints are dispatched. The turret remains firmly on target without re-triggering corrective moves.