### System Architecture & Operational Pipeline

The Bombardeer tracking system coordinates optical perception, predictive state estimation, and physical motor actuation across three distinct layers.

---

### 1. Asynchronous Perception and Motion Decoupling

Rather than running in a single sequential loop, the system decouples computer vision from motor control into independent threads:

* **Vision Pipeline (Sensor-Rate, ~30 Hz):** Captures video frames, detects optical markers, estimates target range, and translates pixel positions into physical milliradian coordinates relative to the bore axis.


* **Motion Servoing Loop (Deterministic 50 Hz, 20 ms ticks):** Operates on an uninterrupted hardware timer, calculating real-time error vectors and streaming velocity commands to the ESP32 microcontroller.



Decoupling ensures that camera frame drops, auto-exposure adjustments, or vision processing latency never starve the motor servo or introduce trajectory jitter.

---

### 2. Latency Compensation via Kalman State Estimation

Image sensor exposure, image transfer, and computer vision processing introduce a measurement lag of approximately 30–50 ms. Steering directly toward raw vision coordinates causes steady-state pursuit lag behind moving targets.

* A 4-state Kalman filter models target position and velocity in world space.


* On every 20 ms tick, the filter projects the target's trajectory forward to the exact instant of actuation.


* Feed-forward velocity terms allow the turret to match target speed directly, driving positional error toward zero even during fast lateral sweeps.



---

### 3. Dual-Mode Control Strategy

The motion servo switches between two control modes based on angular displacement:

* **Mode 1: Ballistic Snap (`P` Command):** When the angular error exceeds 120 mrad (~6.9°), the controller commands an open-loop acceleration trajectory directly to the predicted coordinate. Setpoint rate-limiting locks out redundant commands during transit, preventing trajectory replanning overhead on the stepper driver.


* **Mode 2: Continuous Pursuit (`V` Command):** Once the target enters the tracking envelope, the system transitions to continuous proportional-derivative velocity control. The motor speeds are modulated at 50 Hz to smoothly track the target without stopping and starting.



---

### 4. Dead-Reckoning Coasting Horizon

To handle visual occlusions, motion blur, and lighting dropouts, the system maintains a 1.5-second dead-reckoning window:

* If the visual target is lost, the Kalman filter continues projecting target coordinates based on its last known velocity.


* After 350 ms of sustained dropout, commanded motor speeds decay exponentially toward zero over the remainder of the 1.5-second window.


* If optical detection resumes before timeout, tracking continues smoothly; otherwise, the system zeroes velocity and idles in standby.



---

### 5. Hardware Safety & Firing Enforcement

Physical constraints and firing logic are governed by strict operational gates:

* **Travel Limits:** Real-time telemetry monitoring prevents velocity commands from driving the elevation axis past mechanical limits.


* **Dynamic Firing Criteria:** Solenoid trigger pulses require three conditions simultaneously: optical data must be fresh (<150 ms), radial aim error must fall inside the 40 mrad engagement envelope, and overall turret slew speed must be below 180 mrad/s to avoid firing during rapid slewing.