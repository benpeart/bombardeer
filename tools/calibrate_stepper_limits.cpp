/*
    Bombardeer Bounded Kinematic Benchmark Suite
    -------------------------------------------
    - Phase 1: Benchmark Top Velocity first using adaptive minimal acceleration
               to isolate pull-out speed/back-EMF from inertial ramp stall.
    - Phase 2: Benchmark Maximum Acceleration at 75% of confirmed top speed.
    - Diagnoses whether aborts occur during acceleration or steady-state cruise.
    - Operator aborts stall anytime by pressing Space/Enter in Serial Monitor.
*/

#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

#define PAN_USTEP_PIN1 04
#define PAN_USTEP_PIN2 27
#define PAN_STEP_PIN 26
#define PAN_DIR_PIN 25

#define TILT_USTEP_PIN1 18
#define TILT_USTEP_PIN2 05
#define TILT_STEP_PIN 33
#define TILT_DIR_PIN 32

#define SHARED_ENABLE_PIN 19
#define TMC_RX_PIN 16
#define TMC_TX_PIN 17
#define SERIAL_PORT Serial2
#define R_SENSE 0.11f

#define PAN_DRIVER_ADDR 0b00
#define TILT_DRIVER_ADDR 0b01

// ============================================================================
// Kinematic Conversion Constants & Bounded Travel Envelopes
// ============================================================================
#define STEPPER_FULL_STEPS 200.0
#define STEPPER_MICROSTEPS 16.0
#define STEPS_PER_MOTOR_REV (STEPPER_FULL_STEPS * STEPPER_MICROSTEPS) // 3200.0 steps/rev

// Pan Axis: copied from pan_timing_gears.scad
#define PAN_RING_TEETH 246.0  // ring_teeth
#define PAN_PINION_TEETH 37.0 // pinion_teeth

// Tilt Axis: copied from tilt_gears.scad
#define TILT_GEAR_TEETH 50.0 // teeth
#define TILT_WORM_STARTS 1.0 // starts

constexpr double PAN_STEPS_PER_REV = STEPS_PER_MOTOR_REV * (PAN_RING_TEETH / PAN_PINION_TEETH);
constexpr double TILT_STEPS_PER_REV = STEPS_PER_MOTOR_REV * (TILT_GEAR_TEETH / TILT_WORM_STARTS);

// Pan: +-180.0 deg (+-0.5 revs = 360 deg total continuous sweep)
#define PAN_MAX_DEGREES 180.0
constexpr long PAN_MAX_STEPS = (long)((PAN_STEPS_PER_REV * (PAN_MAX_DEGREES / 360.0)) + 0.5);
constexpr long PAN_MIN_STEPS = -PAN_MAX_STEPS;

// Tilt: +-22.5 deg (+-1/16 revs = 45 deg total vertical envelope)
#define TILT_MAX_DEGREES 22.5
constexpr long TILT_MAX_STEPS = (long)((TILT_STEPS_PER_REV * (TILT_MAX_DEGREES / 360.0)) + 0.5);
constexpr long TILT_MIN_STEPS = -TILT_MAX_STEPS;

TMC2209Stepper panTMC(&SERIAL_PORT, R_SENSE, PAN_DRIVER_ADDR);
TMC2209Stepper tiltTMC(&SERIAL_PORT, R_SENSE, TILT_DRIVER_ADDR);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *panStepper = NULL;
FastAccelStepper *tiltStepper = NULL;

bool initTMC(TMC2209Stepper &driver, const char *axis, uint16_t mA, float holdMult)
{
    driver.begin();
    if (driver.test_connection() != 0)
        return false;
    driver.toff(0);
    driver.pdn_disable(true);
    driver.mstep_reg_select(true);
    driver.rms_current(mA, holdMult);
    driver.microsteps(16);
    driver.intpol(true);
    driver.iholddelay(4);
    driver.TPOWERDOWN(128);
    driver.freewheel(0);
    driver.blank_time(24);
    driver.en_spreadCycle(false);
    driver.pwm_autoscale(true);
    driver.TPWMTHRS(200);
    driver.toff(4);
    return (driver.mstep_reg_select() && driver.microsteps() == 16);
}

// Single-stroke test with real-time abort and ramp/cruise phase detection
bool runStroke(FastAccelStepper *stepper, long targetPos, uint32_t speedHz, uint32_t accelHz2, bool &abortedInRamp)
{
    abortedInRamp = false;
    stepper->setSpeedInHz(speedHz);
    stepper->setAcceleration(accelHz2);
    stepper->moveTo(targetPos);

    while (stepper->isRunning())
    {
        if (Serial.available() > 0)
        {
            // Convert milliHz to Hz (or use getCurrentSpeedInUs() -> 1000000UL / us)
            uint32_t liveSpeed = stepper->getCurrentSpeedInMilliHz() / 1000;
            if (liveSpeed < (uint32_t)(speedHz * 0.95f))
            {
                abortedInRamp = true;
            }

            while (Serial.available() > 0)
                Serial.read();
            stepper->forceStopAndNewPosition(stepper->getCurrentPosition());

            Serial.printf("\n[ABORTED] Stopped at %u steps/s (Target was %u steps/s)\n", liveSpeed, speedHz);
            return false;
        }
        delay(5);
    }
    return true;
}

// Executes out-and-back sweep across full travel envelope
bool runSweep(FastAccelStepper *stepper, long minPos, long maxPos, uint32_t speedHz, uint32_t accelHz2, bool &abortedInRamp)
{
    abortedInRamp = false;
    Serial.println("   (Press SPACE/ENTER at any moment to abort)");

    if (!runStroke(stepper, maxPos, speedHz, accelHz2, abortedInRamp))
        return false;
    delay(150);
    if (!runStroke(stepper, minPos, speedHz, accelHz2, abortedInRamp))
        return false;
    delay(150);
    if (!runStroke(stepper, 0, speedHz, accelHz2, abortedInRamp))
        return false;
    delay(250);
    return true;
}

bool getOperatorFeedback(bool abortedInRamp)
{
    Serial.println(">> Did the axis move cleanly without skipping steps?");
    Serial.print("   Type 'y' (Passed) or 'n' (Stalled/Skipped) and press Enter: ");

    while (Serial.available() > 0)
        Serial.read();

    while (true)
    {
        if (Serial.available() > 0)
        {
            char c = (char)Serial.read();
            if (c == 'y' || c == 'Y')
            {
                Serial.println("PASSED\n");
                return true;
            }
            if (c == 'n' || c == 'N')
            {
                Serial.println("FAILED/STALLED");
                if (abortedInRamp)
                {
                    Serial.println(">> [DIAGNOSTIC] Stall occurred during ACCELERATION ramp (Inertial Torque Stall).\n");
                }
                else
                {
                    Serial.println(">> [DIAGNOSTIC] Axis reached cruise speed before stall/abort (Velocity/Back-EMF Stall).\n");
                }
                return false;
            }
        }
        delay(10);
    }
}

void calibrateAxis(const char *axisName, FastAccelStepper *stepper, long minSteps, long maxSteps)
{
    long strokeSteps = (maxSteps - minSteps);
    Serial.println("\n==================================================");
    Serial.printf("STARTING CALIBRATION: %s AXIS\n", axisName);
    Serial.printf("Physical Stroke Window: %ld steps\n", strokeSteps);
    Serial.println("==================================================");

    // -------------------------------------------------------------
    // PHASE 1: Find Pure Top Speed with Stroke-Adaptive Acceleration
    // -------------------------------------------------------------
    Serial.println("\n--- PHASE 1: Determining Max Speed (Back-EMF / Steady Velocity Limit) ---");

    uint32_t currentSpeed = 2000;
    const uint32_t speedStep = 500;
    uint32_t confirmedMaxSpeed = currentSpeed;
    bool abortedInRamp = false;

    while (true)
    {
        // Require at least 20% cruise: available ramp budget is 80% of stroke
        // s_ramp = v^2 / a  =>  a_min = v^2 / (0.80 * strokeSteps)
        float minAccelNeeded = ((float)currentSpeed * currentSpeed) / (0.80f * (float)strokeSteps);

        // Start with a gentle 4000 steps/s^2, scaling up dynamically as speed demands
        uint32_t activeAccel = (uint32_t)max(4000.0f, minAccelNeeded);

        // Cap to prevent requesting an unreasonable jerk rate on stroke limits
        if (activeAccel > 22000)
        {
            Serial.printf("[PHYSICAL LIMIT REACHED] Reached maximum achievable speed (%u steps/s) within stroke envelope.\n", confirmedMaxSpeed);
            break;
        }

        float rampSteps = ((float)currentSpeed * currentSpeed) / (float)activeAccel;
        float cruiseSteps = (float)strokeSteps - rampSteps;
        float cruisePct = (cruiseSteps / (float)strokeSteps) * 100.0f;

        Serial.printf("[TEST SPEED] %s Speed: %u steps/s | Accel: %u steps/s^2 | Cruise: %.0f%% of stroke\n",
                      axisName, currentSpeed, activeAccel, cruisePct);

        bool completed = runSweep(stepper, minSteps, maxSteps, currentSpeed, activeAccel, abortedInRamp);

        if (!completed || !getOperatorFeedback(abortedInRamp))
        {
            if (abortedInRamp)
            {
                Serial.printf(">>> Note: Stall occurred during acceleration ramp at %u steps/s.\n", currentSpeed);
            }
            Serial.printf(">>> Confirmed Maximum Reliable Cruise Speed: %u steps/s\n", confirmedMaxSpeed);
            break;
        }

        confirmedMaxSpeed = currentSpeed;
        currentSpeed += speedStep;
    }

    // Baseline target for Phase 2: test acceleration at 75% of confirmed top speed
    uint32_t accelTestTargetSpeed = (uint32_t)(confirmedMaxSpeed * 0.75f);
    if (accelTestTargetSpeed < 2000)
        accelTestTargetSpeed = 2000;

    // -------------------------------------------------------------
    // PHASE 2: Benchmark Acceleration at Realistic Operating Speed
    // -------------------------------------------------------------
    Serial.printf("\n--- PHASE 2: Benchmarking Acceleration (Cruise target fixed at %u steps/s) ---\n", accelTestTargetSpeed);

    // Minimum acceleration required to achieve a flat-topped trapezoid (at least 15% cruise)
    // s_total = v^2 / a => a_min = v^2 / (0.85 * strokeSteps)
    uint32_t minAccelForTrapezoid = (uint32_t)(((float)accelTestTargetSpeed * accelTestTargetSpeed) / (0.85f * (float)strokeSteps));

    // Start at least at 2000, or the kinematic minimum for this stroke
    uint32_t currentAccel = max(2000UL, ((minAccelForTrapezoid / 1000UL) + 1) * 1000UL);
    const uint32_t accelStep = 1000;
    uint32_t confirmedMaxAccel = currentAccel;

    while (true)
    {
        float totalRampSteps = ((float)accelTestTargetSpeed * accelTestTargetSpeed) / (float)currentAccel;
        float cruiseSteps = (float)strokeSteps - totalRampSteps;
        float cruisePct = (cruiseSteps / (float)strokeSteps) * 100.0f;

        Serial.printf("[TEST ACCEL] %s Accel: %u steps/s^2 | Speed: %u steps/s | Cruise: %.0f%%\n",
                      axisName, currentAccel, accelTestTargetSpeed, cruisePct);

        bool completed = runSweep(stepper, minSteps, maxSteps, accelTestTargetSpeed, currentAccel, abortedInRamp);

        if (!completed || !getOperatorFeedback(abortedInRamp))
        {
            Serial.printf(">>> Stalled/Aborted at %u steps/s^2. Confirmed Max Accel: %u steps/s^2\n",
                          currentAccel, confirmedMaxAccel);
            break;
        }

        confirmedMaxAccel = currentAccel;
        currentAccel += accelStep;
    }

    // -------------------------------------------------------------
    // FINAL RECOMMENDED PRODUCTION PARAMETERS
    // -------------------------------------------------------------
    Serial.println("\n==================================================");
    Serial.printf("CALIBRATION SUMMARY: %s AXIS\n", axisName);
    Serial.printf("  Benchmarked Top Speed        : %u steps/s\n", confirmedMaxSpeed);
    Serial.printf("  Benchmarked Max Acceleration : %u steps/s^2 (tested at %u steps/s)\n", confirmedMaxAccel, accelTestTargetSpeed);
    Serial.printf("  RECOMMENDED PROD SPEED (80%%) : %u steps/s\n", (uint32_t)(confirmedMaxSpeed * 0.80f));
    Serial.printf("  RECOMMENDED PROD ACCEL (75%%) : %u steps/s^2\n", (uint32_t)(confirmedMaxAccel * 0.75f));
    Serial.println("==================================================");
}

void setup()
{
    Serial.begin(921600);
    delay(1500);

    pinMode(SHARED_ENABLE_PIN, OUTPUT);
    digitalWrite(SHARED_ENABLE_PIN, HIGH);

    pinMode(PAN_USTEP_PIN1, OUTPUT);
    pinMode(PAN_USTEP_PIN2, OUTPUT);
    pinMode(TILT_USTEP_PIN1, OUTPUT);
    pinMode(TILT_USTEP_PIN2, OUTPUT);

    digitalWrite(PAN_USTEP_PIN1, LOW);
    digitalWrite(PAN_USTEP_PIN2, LOW);
    digitalWrite(TILT_USTEP_PIN1, HIGH);
    digitalWrite(TILT_USTEP_PIN2, LOW);
    delay(10);

    SERIAL_PORT.begin(115200, SERIAL_8N1, TMC_RX_PIN, TMC_TX_PIN);

    initTMC(panTMC, "PAN", 1200, 0.25f);
    initTMC(tiltTMC, "TILT", 1100, 0.15f);

    digitalWrite(SHARED_ENABLE_PIN, LOW);

    engine.init();

    panStepper = engine.stepperConnectToPin(PAN_STEP_PIN);
    if (panStepper)
    {
        panStepper->setDirectionPin(PAN_DIR_PIN);
        panStepper->setCurrentPosition(0);
    }

    tiltStepper = engine.stepperConnectToPin(TILT_STEP_PIN);
    if (tiltStepper)
    {
        tiltStepper->setDirectionPin(TILT_DIR_PIN);
        tiltStepper->setCurrentPosition(0);
    }

    Serial.println("\nPosition turret at mechanical zero (0, 0).");
    Serial.println("Press Enter in the Serial Monitor to begin PAN axis calibration...");
    while (Serial.available() == 0)
    {
        delay(50);
    }
    while (Serial.available() > 0)
        Serial.read();

    calibrateAxis("PAN", panStepper, PAN_MIN_STEPS, PAN_MAX_STEPS);

    Serial.println("\nPress Enter in the Serial Monitor to begin TILT axis calibration...");
    while (Serial.available() == 0)
    {
        delay(50);
    }
    while (Serial.available() > 0)
        Serial.read();

    calibrateAxis("TILT", tiltStepper, TILT_MIN_STEPS, TILT_MAX_STEPS);

    Serial.println("\nCALIBRATION COMPLETE.");
}

void loop()
{
    delay(1000);
}