# Bombardeer Bounded Kinematic Calibration Guide

This utility benchmarks the maximum stable acceleration and top speed for both the Pan and Tilt axes within their physical travel limits.

---

## 1. Hardware & Mechanical Setup

1. **Clear Turret Path:** Ensure all wiring harness slack, camera cables, and payload equipment can swing freely through the full travel envelope without binding.
2. **Center Turret:** Manually align the turret to mechanical center ($0^\circ$ pan, $0^\circ$ tilt) before powering up the ESP32.
3. **Power Up:** Connect the motor DC power supply and attach the ESP32 via USB.

---

## 2. Running the Benchmark

1. Open the project in your IDE (e.g., PlatformIO or Arduino IDE) and upload `calibrate_stepper_limits.cpp` to the ESP32.
2. Open the **Serial Monitor** set to **115200 baud**.
3. Follow the serial prompts:
   * Press **Enter** to begin the **PAN** axis calibration.
   * The motor will run an out-and-back sweep (`0 -> +MAX -> -MAX -> 0`) across the envelope.

---

## 3. Evaluating Each Test Pass

After each sweep, the console prompts:
`Did the axis move smoothly without missing steps? (y/n)`

* **Type `y` + Enter (Pass):** If the sweep was smooth with no stalling, grinding, or belt/gear skipping. The script will increment the parameter and run the next test.
* **Type `n` + Enter (Fail/Skip):** If the motor stalled, skipped steps, or audibly vibrated without moving. This halts Phase 1 or Phase 2 and locks in the last verified value.
* **Emergency Abort:** Press **Space** or **Enter** at any moment during a live move to trigger an immediate halt.

---

## 4. Test Phases per Axis

Each axis runs through two automated phases:

1. **Phase 1 (Max Acceleration):** The motor sweeps at a fixed cruise speed of $10,000\text{ steps/s}$ while increasing acceleration in $+5,000\text{ steps/s}^2$ increments to test pure inertial load.
2. **Phase 2 (Max Speed):** Using your confirmed acceleration, the speed increases in $+2,500\text{ steps/s}$ steps up to the calculated physical stroke limit.

Once Pan is complete, repeat the process for **TILT**.

---

## 5. Applying Values to Production (`main.cpp`)

At the conclusion of each test, the script outputs recommended production values using conservative engineering safety margins (80% of max acceleration, 85% of max speed):

```text
CALIBRATION SUMMARY: PAN AXIS
  RECOMMENDED PROD ACCEL (80%) : 20000 steps/s^2
  RECOMMENDED PROD SPEED (85%) : 25500 steps/s
```

Copy these values into your `main.cpp` configuration:

```cpp
#define PAN_STEPPER_ACCELERATION   <RECOMMENDED ACCEL PROD>
#define PAN_STEPPER_MAXSPEEDHZ     <RECOMMENDED PROD SPEED>

#define TILT_STEPPER_ACCELERATION  <RECOMMENDED ACCEL PROD>
#define TILT_STEPPER_MAXSPEEDHZ    <RECOMMENDED PROD SPEED>
```