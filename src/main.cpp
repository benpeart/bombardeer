/*
    Bombardeer Autonomous Wildlife Deterrent Turret Controller
    ----------------------------------------------------------
    - Native Milliradian (mrad) Motion Protocol:
        * P <pan_mrad> <tilt_mrad> : Atomic absolute waypoint target (mrad)
        * V <pan_s> <tilt_s>       : Dynamic continuous velocity trim (mrad/s)
        * X                        : Immediate dynamic controlled halt
        * H                        : Zero current mechanical coordinates
        * T <0|1>                  : Autonomous solenoid trigger enable/disable
        * S <p> <t> <f> <m>        : 20 Hz deterministic telemetry stream
    - FastAccelStepper hardware-timed trapezoidal acceleration profiles.
    - TMC2209 silent stepping with dynamic torque transition.
    - Automated solenoid trigger pulse state machine.
    - Xbox Series X Bluetooth controller manual override with auto-timeout.
    - Non-blocking zero-allocation serial streaming parser with framing watchdog.
    - Failsafe serial timeout watchdog against host disconnect or reboot.
*/

#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include "debug.h"

// ============================================================================
// ESP32 Pin Assignments
// ============================================================================

// Driver 1: Pan Axis
#define PAN_USTEP_PIN1 04
#define PAN_USTEP_PIN2 27
#define PAN_STEP_PIN 26
#define PAN_DIR_PIN 25

// Driver 2: Tilt Axis
#define TILT_USTEP_PIN1 18
#define TILT_USTEP_PIN2 05
#define TILT_STEP_PIN 33
#define TILT_DIR_PIN 32

#define SHARED_ENABLE_PIN 19

// TMC2209 Single-Wire UART Pins (Shared UART2)
#define TMC_RX_PIN 16 // ESP32 RX2 connected to TMC2209 TX/RX pins
#define TMC_TX_PIN 17 // ESP32 TX2 connected through 1k ohm resistor
#define SERIAL_PORT Serial2
#define R_SENSE 0.11f // Standard sense resistor value for stepsticks

// Driver Addresses on the shared UART bus
#define PAN_DRIVER_ADDR 0b00  // MS1=GND, MS2=GND
#define TILT_DRIVER_ADDR 0b01 // MS1=VCC, MS2=GND

// Actuators
#define PIN_SOLENOID 13

// ============================================================================
// Milliradian Kinematic Conversion Constants
// ============================================================================
// Standard 1.8 deg NEMA 17 motor: 200 full steps/rev * 16 usteps = 3200 steps/rev
#define STEPPER_FULL_STEPS 200.0
#define STEPPER_MICROSTEPS 16.0
#define STEPS_PER_MOTOR_REV (STEPPER_FULL_STEPS * STEPPER_MICROSTEPS) // 3200.0 steps/rev

// Pan Axis: copied from pan_timing_gears.scad
#define PAN_RING_TEETH 246.0  // ring_teeth
#define PAN_PINION_TEETH 37.0 // pinion_teeth

// Tilt Axis: copied from tilt_gears.scad
#define TILT_GEAR_TEETH 50.0 // teeth
#define TILT_WORM_STARTS 1.0 // starts

// Mathematical constants
#define K_PI 3.14159265358979323846
#define MRAD_PER_REV (2000.0 * K_PI) // ~6283.1853 mrad

// Compile-time ratio calculations
constexpr double PAN_GEAR_RATIO = PAN_RING_TEETH / PAN_PINION_TEETH;
constexpr double PAN_STEPS_PER_REV = STEPS_PER_MOTOR_REV * PAN_GEAR_RATIO;
constexpr double PAN_STEPS_PER_MRAD = PAN_STEPS_PER_REV / MRAD_PER_REV;
constexpr double PAN_MRAD_PER_STEP = MRAD_PER_REV / PAN_STEPS_PER_REV;

constexpr double TILT_GEAR_RATIO = TILT_GEAR_TEETH / TILT_WORM_STARTS;
constexpr double TILT_STEPS_PER_REV = STEPS_PER_MOTOR_REV * TILT_GEAR_RATIO;
constexpr double TILT_STEPS_PER_MRAD = TILT_STEPS_PER_REV / MRAD_PER_REV;
constexpr double TILT_MRAD_PER_STEP = MRAD_PER_REV / TILT_STEPS_PER_REV;

inline long panMradToSteps(long mrad)
{
    return (mrad >= 0) ? (long)((mrad * PAN_STEPS_PER_MRAD) + 0.5)
                       : (long)((mrad * PAN_STEPS_PER_MRAD) - 0.5);
}

inline long panStepsToMrad(long steps)
{
    return (steps >= 0) ? (long)((steps * PAN_MRAD_PER_STEP) + 0.5)
                        : (long)((steps * PAN_MRAD_PER_STEP) - 0.5);
}

inline long tiltMradToSteps(long mrad)
{
    return (mrad >= 0) ? (long)((mrad * TILT_STEPS_PER_MRAD) + 0.5)
                       : (long)((mrad * TILT_STEPS_PER_MRAD) - 0.5);
}

inline long tiltStepsToMrad(long steps)
{
    return (steps >= 0) ? (long)((steps * TILT_MRAD_PER_STEP) + 0.5)
                        : (long)((steps * TILT_MRAD_PER_STEP) - 0.5);
}

// ============================================================================
// Physical Travel Boundaries (FastAccelStepper Steps)
// ============================================================================
// Pan: +-180.0 deg (+-0.5 revs = 360 deg total continuous sweep)
#define PAN_MAX_DEGREES 180.0
constexpr long PAN_MAX_STEPS = (long)((PAN_STEPS_PER_REV * (PAN_MAX_DEGREES / 360.0)) + 0.5);
constexpr long PAN_MIN_STEPS = -PAN_MAX_STEPS;

// Tilt: +-22.5 deg (+-1/16 revs = 45 deg total vertical envelope)
#define TILT_MAX_DEGREES 22.5
constexpr long TILT_MAX_STEPS = (long)((TILT_STEPS_PER_REV * (TILT_MAX_DEGREES / 360.0)) + 0.5);
constexpr long TILT_MIN_STEPS = -TILT_MAX_STEPS;

/*
    Timing for Heschen HS-1564B / HS-4564B Solenoid
    * SOLENOID_PULSE_LENGTH = 40-60 ms
    * SOLENOID_COOLDOWN_LENGTH = 190 ms
    * Total cycle = 250 ms (4.0 BPS)
*/
constexpr uint32_t SOLENOID_PULSE_LENGTH = 60;
constexpr uint32_t SOLENOID_COOLDOWN_LENGTH = 190;
constexpr uint32_t MAX_AUTONOMOUS_FIRE_DURATION_MS = 2000;

// Host Serial Link Watchdog Timeout (500ms without serial data -> halt autonomous motion)
constexpr uint32_t SERIAL_WATCHDOG_TIMEOUT_MS = 500;

// Xbox Controller Deadzone and Trigger Thresholds
constexpr float DEADZONE_RADIUS = 0.30f;
constexpr float TRIGGER_THRESHOLD = 0.15f;
constexpr unsigned long MANUAL_OVERRIDE_TIMEOUT_MS = 400;

// Bind to any Xbox controller
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

// ============================================================================
// Stepper Library Configuration & Instantiations
// ============================================================================

#define PAN_STEPPER_ACCELERATION 13500 // Benchmarked production acceleration (75% limit)
#define PAN_STEPPER_MAXSPEEDHZ 11600   // Benchmarked production max speed (80% limit)
#define PAN_STEPPER_MINSPEEDHZ 15

#define TILT_STEPPER_ACCELERATION 50000 // Benchmarked production acceleration (~72% limit)
#define TILT_STEPPER_MAXSPEEDHZ 14800   // Benchmarked production max speed (80% limit)
#define TILT_STEPPER_MINSPEEDHZ 15

#define STEPPER_HYSTERESISHZ 300
#define STEPPER_EXPONENT 2.0f // 1.0f = linear, 2.0f = quadratic, 3.0f = cubic

// TMCStepper Driver Configuration Objects
TMC2209Stepper panTMC(&SERIAL_PORT, R_SENSE, PAN_DRIVER_ADDR);
TMC2209Stepper tiltTMC(&SERIAL_PORT, R_SENSE, TILT_DRIVER_ADDR);

// FastAccelStepper Engine & Motor Handles
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *panStepper = NULL;
FastAccelStepper *tiltStepper = NULL;

static bool autoFireRequested = false;
static uint32_t autoFireStartTime = 0;
static bool manualOverrideActive = false;
static unsigned long lastSerialRxTime = 0;

// Control Modes
enum MotionMode
{
    MODE_VELOCITY,
    MODE_POSITION
};
static MotionMode currentMotionMode = MODE_VELOCITY;

// Autonomous Target Speeds (mrad/s)
static long commandedPanSpeed = 0;
static long commandedTiltSpeed = 0;

// Active Applied Stepper States
static long appliedPanSpeed = 0;
static long appliedTiltSpeed = 0;
static int activePanDir = 0; // -1 = backward, 0 = stopped, 1 = forward
static int activeTiltDir = 0;

// ============================================================================
// TMC2209 Driver Configuration
// ============================================================================

/*
 * Configure TMC2209 Driver Parameters deterministically over UART with read-back validation
 */
bool initTMC2209(TMC2209Stepper &driver, const char *axisName, uint16_t current_mA, float hold_multiplier = 0.25f, bool forceSpreadCycle = false)
{
    driver.begin();

    // 1. Verify basic UART Communication
    uint8_t result = driver.test_connection();
    if (result != 0)
    {
        DB_PRINTF("[TMC2209] ERROR: %s axis connection failed! Code: %d (Check wiring/addressing)\n", axisName, result);
        return false;
    }

    // 2. Clear fault flags & configure operating mode
    driver.toff(0);                // Disable driver output during register configuration
    driver.pdn_disable(true);      // Use PDN pin exclusively for UART communication
    driver.mstep_reg_select(true); // Microsteps configured via UART registers, not hardware pins

    // 3. Set RMS active current and reduced holding current to prevent standstill overheating
    driver.rms_current(current_mA, hold_multiplier);

    // 4. Microstepping & Internal Interpolation
    driver.microsteps(STEPPER_MICROSTEPS); // 1/16 Microstepping from FastAccelStepper
    driver.intpol(true);                   // Interpolate to 1/256 internally for smooth motion

    // 5. Standstill & Powerdown Timing (drops to hold current ~0.15s after motion stops)
    driver.iholddelay(4);   // Delay before current reduction begins
    driver.TPOWERDOWN(128); // Smooth transition window to hold current
    driver.freewheel(0);    // Normal current reduction during hold

    // 6. Chopper Configuration
    driver.blank_time(24); // Comparator blank time

    if (forceSpreadCycle)
    {
        // Pure SpreadCycle (Higher torque across all speeds, but hums at standstill)
        driver.en_spreadCycle(true);
        driver.pwm_autoscale(false);
        DB_PRINTF("[TMC2209] %s Configured: %d mA RMS, Pure SpreadCycle\n", axisName, current_mA);
    }
    else
    {
        // Hybrid Mode: Silent StealthChop at standstill and low speed,
        // automatically transitions to SpreadCycle at high speed to avoid step loss.
        driver.en_spreadCycle(false); // Enable StealthChop for dead-silent standstill
        driver.pwm_autoscale(true);   // Automatically scale StealthChop voltage

        // Threshold where driver switches from StealthChop to SpreadCycle:
        // Set so standstill and low/mid speeds remain silent, while top speeds (>3000 Hz) engage SpreadCycle
        driver.TPWMTHRS(200);
        DB_PRINTF("[TMC2209] %s Configured: %d mA RMS, StealthChop Standstill + Hybrid Switch\n", axisName, current_mA);
    }

    // 7. Enable chopper
    driver.toff(4);

    // 8. Read-back register validation to ensure UART writes were latched
    uint16_t readMicrosteps = driver.microsteps();
    uint16_t readCurrent = driver.cs2rms(driver.irun());
    bool regSelectActive = driver.mstep_reg_select();

    if (!regSelectActive || readMicrosteps != STEPPER_MICROSTEPS)
    {
        DB_PRINTF("[TMC2209] ERROR: %s register read-back failed! (Microsteps: %u, RegSelect: %d)\n",
                  axisName, readMicrosteps, regSelectActive ? 1 : 0);
        return false;
    }

    DB_PRINTF("[TMC2209] %s OK: %u mA Run, %u%% Hold, %u Microsteps, RegCtrl: Verified\n",
              axisName, readCurrent, (uint16_t)(hold_multiplier * 100.0f), readMicrosteps);
    return true;
}

// Persistent state tracker per axis to manage hysteresis and single-shot stops
struct AxisControlState
{
    int lastDir = 0;          // -1 = Reverse, 0 = Stopped, 1 = Forward
    uint32_t lastSpeedHz = 0; // Last speed commanded to the driver
};

/**
 * Controls a FastAccelStepper motor smoothly using analog joystick input.
 */
void updateAxisFromJoystick(
    FastAccelStepper *stepper,
    AxisControlState &state,
    float rawInput,
    uint32_t maxSpeedHz,
    uint32_t minSpeedHz,
    float deadzone = DEADZONE_RADIUS,
    uint32_t hysteresisHz = STEPPER_HYSTERESISHZ,
    float exponent = STEPPER_EXPONENT)
{
    if (!stepper)
        return;

    float absInput = fabs(rawInput);

    // Single-Shot Stop on Deadzone Release
    if (absInput <= deadzone)
    {
        if (state.lastDir != 0)
        {
            stepper->stopMove();
            state.lastDir = 0;
            state.lastSpeedHz = 0;
        }
        return;
    }

    // Percentage of Stick Travel Past Deadzone (0.0 to 1.0)
    float normPct = (absInput - deadzone) / (1.0f - deadzone);
    if (normPct > 1.0f)
        normPct = 1.0f;

    // Exponential Response Curve
    float curvedPct = powf(normPct, exponent);

    uint32_t targetSpeedHz = (uint32_t)(curvedPct * maxSpeedHz);
    if (targetSpeedHz < minSpeedHz)
        targetSpeedHz = minSpeedHz;

    int currentDir = (rawInput > 0.0f) ? 1 : -1;

    bool dirChanged = (currentDir != state.lastDir);
    bool speedChangedSignificantly = (abs((long)targetSpeedHz - (long)state.lastSpeedHz) > (long)hysteresisHz);

    if (dirChanged)
    {
        stepper->setSpeedInHz(targetSpeedHz);
        if (currentDir > 0)
            stepper->runForward();
        else
            stepper->runBackward();
        state.lastDir = currentDir;
        state.lastSpeedHz = targetSpeedHz;
    }
    else if (speedChangedSignificantly)
    {
        stepper->setSpeedInHz(targetSpeedHz);
        stepper->applySpeedAcceleration();
        state.lastSpeedHz = targetSpeedHz;
    }
}

// ============================================================================
// Solenoid State Machine
// ============================================================================

enum SolenoidState
{
    SOLENOID_IDLE,
    SOLENOID_FIRING,
    SOLENOID_COOLDOWN
};

static SolenoidState solenoidState = SOLENOID_IDLE;
static unsigned long solenoidTimer = 0;

void triggerSolenoid()
{
    if (solenoidState == SOLENOID_IDLE)
    {
        DB_PRINTLN("[SOLENOID] Firing!");
        digitalWrite(PIN_SOLENOID, HIGH);
        solenoidTimer = millis();
        solenoidState = SOLENOID_FIRING;
    }
}

void updateSolenoid()
{
    unsigned long now = millis();

    switch (solenoidState)
    {
    case SOLENOID_FIRING:
        if (now - solenoidTimer >= SOLENOID_PULSE_LENGTH)
        {
            DB_PRINTLN("[SOLENOID] Pulse expired, turning OFF.");
            digitalWrite(PIN_SOLENOID, LOW);
            solenoidTimer = now;
            solenoidState = SOLENOID_COOLDOWN;
        }
        break;

    case SOLENOID_COOLDOWN:
        if (now - solenoidTimer >= SOLENOID_COOLDOWN_LENGTH)
        {
            DB_PRINTLN("[SOLENOID] Cooldown expired, returning to IDLE.");
            solenoidState = SOLENOID_IDLE;
        }
        break;

    case SOLENOID_IDLE:
    default:
        break;
    }
}

// ============================================================================
// Xbox Controller Processing Engine
// ============================================================================

void processXboxOverride(AxisControlState &panState, AxisControlState &tiltState)
{
    static unsigned long lastManualInputTime = 0;
    static bool lastBtnA = false;

    float rawPan = 0.0f;
    float rawTilt = 0.0f;
    float trigger = 0.0f;
    bool stickActive = false;
    bool triggerActive = false;

    xboxController.onLoop();

    if (xboxController.isConnected() && !xboxController.isWaitingForFirstNotification())
    {
        constexpr float invHalfJoy = 2.0f / (float)XboxControllerNotificationParser::maxJoy;
        constexpr float invMaxTrig = 1.0f / (float)XboxControllerNotificationParser::maxTrig;

        rawPan = ((float)xboxController.xboxNotif.joyLHori * invHalfJoy) - 1.0f;
        rawTilt = ((float)xboxController.xboxNotif.joyLVert * invHalfJoy) - 1.0f;
        trigger = (float)xboxController.xboxNotif.trigRT * invMaxTrig;

        stickActive = (fabs(rawPan) > DEADZONE_RADIUS || fabs(rawTilt) > DEADZONE_RADIUS);
        triggerActive = (trigger > TRIGGER_THRESHOLD);

        // Single-Shot 'A' Button: Zero Current Mechanical Coordinates
        bool btnAPressed = xboxController.xboxNotif.btnA;
        if (btnAPressed && !lastBtnA)
        {
            if (panStepper)
                panStepper->forceStopAndNewPosition(0);
            if (tiltStepper)
                tiltStepper->forceStopAndNewPosition(0);

            panState = AxisControlState();
            tiltState = AxisControlState();
            commandedPanSpeed = 0;
            commandedTiltSpeed = 0;
            activePanDir = 0;
            activeTiltDir = 0;
            appliedPanSpeed = 0;
            appliedTiltSpeed = 0;
            currentMotionMode = MODE_VELOCITY;

            DB_PRINTLN("[XBOX] Re-zeroed home coordinates via 'A' button.");
        }
        lastBtnA = btnAPressed;

        if (stickActive || triggerActive || btnAPressed)
        {
            lastManualInputTime = millis();
            commandedPanSpeed = 0;
            commandedTiltSpeed = 0;
            currentMotionMode = MODE_VELOCITY;
        }
    }
    else
    {
        rawPan = 0.0f;
        rawTilt = 0.0f;
        trigger = 0.0f;
        lastBtnA = false;
    }

    manualOverrideActive = (millis() - lastManualInputTime < MANUAL_OVERRIDE_TIMEOUT_MS);

    if (manualOverrideActive)
    {
        updateAxisFromJoystick(panStepper, panState, rawPan, PAN_STEPPER_MAXSPEEDHZ, PAN_STEPPER_MINSPEEDHZ);
        updateAxisFromJoystick(tiltStepper, tiltState, rawTilt, TILT_STEPPER_MAXSPEEDHZ, TILT_STEPPER_MINSPEEDHZ);

        if (triggerActive)
        {
            triggerSolenoid();
        }
    }
    else
    {
        if (panState.lastDir != 0 || tiltState.lastDir != 0)
        {
            if (panStepper && panStepper->isRunning())
                panStepper->stopMove();
            if (tiltStepper && tiltStepper->isRunning())
                tiltStepper->stopMove();
            panState = AxisControlState();
            tiltState = AxisControlState();
        }
    }
}

// ============================================================================
// Atomic Absolute Waypoint Actuation (P Command)
// ============================================================================

void commandAbsolutePositionMrad(long targetPanMrad, long targetTiltMrad)
{
    currentMotionMode = MODE_POSITION;
    commandedPanSpeed = 0;
    commandedTiltSpeed = 0;
    activePanDir = 0;
    activeTiltDir = 0;
    appliedPanSpeed = 0;
    appliedTiltSpeed = 0;

    if (panStepper)
    {
        panStepper->setSpeedInHz(PAN_STEPPER_MAXSPEEDHZ);
        long targetSteps = constrain(panMradToSteps(targetPanMrad), PAN_MIN_STEPS, PAN_MAX_STEPS);
        panStepper->moveTo(targetSteps);
    }

    if (tiltStepper)
    {
        tiltStepper->setSpeedInHz(TILT_STEPPER_MAXSPEEDHZ);
        long targetSteps = constrain(tiltMradToSteps(targetTiltMrad), TILT_MIN_STEPS, TILT_MAX_STEPS);
        tiltStepper->moveTo(targetSteps);
    }
}

// ============================================================================
// Continuous Dynamic Velocity Slew Actuation (V Command)
// ============================================================================

void applyVelocityActuation()
{
    if (currentMotionMode != MODE_VELOCITY || !panStepper || !tiltStepper)
        return;

    long curPan = panStepper->getCurrentPosition();
    long curTilt = tiltStepper->getCurrentPosition();

    // --- 1. PAN AXIS VELOCITY ---
    long targetPan = commandedPanSpeed; // in mrad/s
    if (targetPan > 0 && curPan >= PAN_MAX_STEPS)
        targetPan = 0;
    if (targetPan < 0 && curPan <= PAN_MIN_STEPS)
        targetPan = 0;

    // Convert mrad/s directly into steps/s (Hz)
    long targetPanHz = (long)(fabs((double)targetPan) * PAN_STEPS_PER_MRAD);
    if (targetPanHz > PAN_STEPPER_MAXSPEEDHZ)
        targetPanHz = PAN_STEPPER_MAXSPEEDHZ;

    if (targetPanHz < PAN_STEPPER_MINSPEEDHZ)
    {
        if (activePanDir != 0)
        {
            panStepper->stopMove();
            activePanDir = 0;
            appliedPanSpeed = 0;
        }
    }
    else
    {
        int reqPanDir = (targetPan > 0) ? 1 : -1;
        if (reqPanDir != activePanDir)
        {
            panStepper->setSpeedInHz((uint32_t)targetPanHz);
            if (reqPanDir > 0)
                panStepper->runForward();
            else
                panStepper->runBackward();
            activePanDir = reqPanDir;
            appliedPanSpeed = targetPanHz;
        }
        else if (targetPanHz < appliedPanSpeed || abs(targetPanHz - appliedPanSpeed) >= 40)
        {
            panStepper->setSpeedInHz((uint32_t)targetPanHz);
            panStepper->applySpeedAcceleration();
            appliedPanSpeed = targetPanHz;
        }
    }

    // --- 2. TILT AXIS VELOCITY ---
    long targetTilt = commandedTiltSpeed; // in mrad/s
    if (targetTilt > 0 && curTilt >= TILT_MAX_STEPS)
        targetTilt = 0;
    if (targetTilt < 0 && curTilt <= TILT_MIN_STEPS)
        targetTilt = 0;

    // Convert mrad/s directly into steps/s (Hz)
    long targetTiltHz = (long)(fabs((double)targetTilt) * TILT_STEPS_PER_MRAD);
    if (targetTiltHz > TILT_STEPPER_MAXSPEEDHZ)
        targetTiltHz = TILT_STEPPER_MAXSPEEDHZ;

    if (targetTiltHz < TILT_STEPPER_MINSPEEDHZ)
    {
        if (activeTiltDir != 0)
        {
            tiltStepper->stopMove();
            activeTiltDir = 0;
            appliedTiltSpeed = 0;
        }
    }
    else
    {
        int reqTiltDir = (targetTilt > 0) ? 1 : -1;
        if (reqTiltDir != activeTiltDir)
        {
            tiltStepper->setSpeedInHz((uint32_t)targetTiltHz);
            if (reqTiltDir > 0)
                tiltStepper->runForward();
            else
                tiltStepper->runBackward();
            activeTiltDir = reqTiltDir;
            appliedTiltSpeed = targetTiltHz;
        }
        else if (targetTiltHz < appliedTiltSpeed || abs(targetTiltHz - appliedTiltSpeed) >= 40)
        {
            tiltStepper->setSpeedInHz((uint32_t)targetTiltHz);
            tiltStepper->applySpeedAcceleration();
            appliedTiltSpeed = targetTiltHz;
        }
    }
}

// ============================================================================
// Non-Blocking High-Speed Serial Protocol Parser
// ============================================================================

void processSerialCommands()
{
    if (manualOverrideActive)
    {
        while (Serial.available() > 0)
            Serial.read();
        return;
    }

    static char rxBuffer[64];
    static uint8_t rxIdx = 0;
    static unsigned long rxStartTime = 0;

    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();

        if (rxIdx == 0)
        {
            rxStartTime = millis();
        }
        else if (millis() - rxStartTime > 250)
        {
            rxIdx = 0;
            rxStartTime = millis();
        }

        if (c == '\n')
        {
            rxBuffer[rxIdx] = '\0';
            if (rxIdx > 0)
            {
                lastSerialRxTime = millis();
                char cmdType = rxBuffer[0];

                switch (cmdType)
                {
                case 'P': // Dynamic Absolute Waypoint Target (mrad)
                {
                    long targetPan = 0, targetTilt = 0;
                    if (sscanf(rxBuffer, "P %ld %ld", &targetPan, &targetTilt) == 2)
                    {
                        commandAbsolutePositionMrad(targetPan, targetTilt);
                    }
                    break;
                }

                case 'V': // Continuous Velocity Streaming (mrad/s)
                {
                    long pSpd = 0, tSpd = 0;
                    if (sscanf(rxBuffer, "V %ld %ld", &pSpd, &tSpd) == 2)
                    {
                        currentMotionMode = MODE_VELOCITY;
                        commandedPanSpeed = pSpd;
                        commandedTiltSpeed = tSpd;
                    }
                    break;
                }

                case 'X': // Emergency / Commanded Stop
                {
                    currentMotionMode = MODE_VELOCITY;
                    commandedPanSpeed = 0;
                    commandedTiltSpeed = 0;
                    activePanDir = 0;
                    activeTiltDir = 0;
                    appliedPanSpeed = 0;
                    appliedTiltSpeed = 0;
                    if (panStepper)
                        panStepper->forceStopAndNewPosition(panStepper->getCurrentPosition());
                    if (tiltStepper)
                        tiltStepper->forceStopAndNewPosition(tiltStepper->getCurrentPosition());
                    break;
                }

                case 'T': // Autonomous Solenoid Trigger
                {
                    int reqState = 0;
                    if (sscanf(rxBuffer, "T %d", &reqState) == 1)
                    {
                        if (reqState == 1)
                        {
                            if (!autoFireRequested)
                            {
                                autoFireRequested = true;
                                autoFireStartTime = millis();
                            }
                        }
                        else
                        {
                            autoFireRequested = false;
                        }
                    }
                    break;
                }

                case 'H': // Zero Current Coordinates
                {
                    currentMotionMode = MODE_VELOCITY;
                    commandedPanSpeed = 0;
                    commandedTiltSpeed = 0;
                    activePanDir = 0;
                    activeTiltDir = 0;
                    appliedPanSpeed = 0;
                    appliedTiltSpeed = 0;
                    if (panStepper)
                    {
                        panStepper->stopMove();
                        panStepper->setCurrentPosition(0);
                    }
                    if (tiltStepper)
                    {
                        tiltStepper->stopMove();
                        tiltStepper->setCurrentPosition(0);
                    }
                    break;
                }

                case 'Y': // Time Sync Ping: Echo Pi's timestamp immediately
                {
                    unsigned long piStamp = 0;
                    if (sscanf(rxBuffer, "Y %lu", &piStamp) == 1)
                    {
                        // Echo: Y <pi_timestamp_sent> <esp32_now_ms>
                        Serial.printf("Y %lu %lu\n", piStamp, millis());
                    }
                    break;
                }

                default:
                    break;
                }
            }
            rxIdx = 0;
        }
        else if (c != '\r')
        {
            if (rxIdx < (sizeof(rxBuffer) - 1))
            {
                rxBuffer[rxIdx++] = c;
            }
            else
            {
                rxIdx = 0;
            }
        }
    }
}

// ============================================================================
// Setup
// ============================================================================

void setup()
{
    Serial.begin(921600);
    Serial.setTimeout(2);

    unsigned long startWait = millis();
    while (!Serial && (millis() - startWait < 1500))
        ;

    DB_PRINTLN("\nStarting Bombardeer Turret Controller on " + String(ARDUINO_BOARD));

    xboxController.begin();

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

    initTMC2209(panTMC, "PAN", 1200, 0.25f);
    initTMC2209(tiltTMC, "TILT", 1100, 0.15f);

    digitalWrite(SHARED_ENABLE_PIN, LOW);

    engine.init();

    panStepper = engine.stepperConnectToPin(PAN_STEP_PIN);
    if (panStepper)
    {
        panStepper->setDirectionPin(PAN_DIR_PIN);
        panStepper->setSpeedInHz(PAN_STEPPER_MAXSPEEDHZ);
        panStepper->setAcceleration(PAN_STEPPER_ACCELERATION);
        panStepper->setCurrentPosition(0);
    }
    else
    {
        DB_PRINTLN("[ERROR] Failed to attach Pan stepper to hardware timer!");
    }

    tiltStepper = engine.stepperConnectToPin(TILT_STEP_PIN);
    if (tiltStepper)
    {
        tiltStepper->setDirectionPin(TILT_DIR_PIN);
        tiltStepper->setSpeedInHz(TILT_STEPPER_MAXSPEEDHZ);
        tiltStepper->setAcceleration(TILT_STEPPER_ACCELERATION);
        tiltStepper->setCurrentPosition(0);
    }
    else
    {
        DB_PRINTLN("[ERROR] Failed to attach Tilt stepper to hardware timer!");
    }

    pinMode(PIN_SOLENOID, OUTPUT);
    digitalWrite(PIN_SOLENOID, LOW);

    lastSerialRxTime = millis();
}

// ============================================================================
// Main Execution Loop
// ============================================================================

void loop()
{
    static AxisControlState panState;
    static AxisControlState tiltState;
    static unsigned long lastTelemetryTime = 0;

    // Periodic check to restore register configuration if TMC2209 brownout reset occurred
    static unsigned long lastDriverCheck = 0;
    if (millis() - lastDriverCheck >= 3000)
    {
        lastDriverCheck = millis();

        if (!panTMC.mstep_reg_select() || panTMC.microsteps() != 16 ||
            !tiltTMC.mstep_reg_select() || tiltTMC.microsteps() != 16)
        {
            DB_PRINTLN("[TMC2209] Driver reset detected! Re-applying UART parameters...");
            initTMC2209(panTMC, "PAN", 1200, 0.25f);
            initTMC2209(tiltTMC, "TILT", 1100, 0.15f);
        }
    }

    // 1. Process Xbox Controller Input & Manual Overrides
    processXboxOverride(panState, tiltState);

    // 2. Process Incoming Host Serial Commands
    processSerialCommands();

    // 3. Failsafe Watchdog: Prevent runaway motion if serial link is lost during autonomous tracking
    if ((millis() - lastSerialRxTime > SERIAL_WATCHDOG_TIMEOUT_MS) && !manualOverrideActive)
    {
        if (currentMotionMode == MODE_VELOCITY && (commandedPanSpeed != 0 || commandedTiltSpeed != 0))
        {
            commandedPanSpeed = 0;
            commandedTiltSpeed = 0;
            if (panStepper)
                panStepper->stopMove();
            if (tiltStepper)
                tiltStepper->stopMove();
        }
        else if (currentMotionMode == MODE_POSITION)
        {
            if (panStepper && panStepper->isRunning())
                panStepper->stopMove();
            if (tiltStepper && tiltStepper->isRunning())
                tiltStepper->stopMove();
        }

        if (autoFireRequested)
        {
            autoFireRequested = false;
        }
    }

    // 4. Autonomous Tracking Actuation
    if (!manualOverrideActive)
    {
        applyVelocityActuation();
    }

    // 5. Autonomous Firing Logic
    if (autoFireRequested)
    {
        if (millis() - autoFireStartTime >= MAX_AUTONOMOUS_FIRE_DURATION_MS)
            autoFireRequested = false;
        else
            triggerSolenoid();
    }

    // 6. Update Solenoid Actuator Timing
    updateSolenoid();

    // 7. Broadcast Telemetry to Raspberry Pi at 20 Hz (every 50 ms) in milliradians
    unsigned long now = millis();
    if (now - lastTelemetryTime >= 50)
    {
        lastTelemetryTime = now;
        long currentPanSteps = panStepper ? panStepper->getCurrentPosition() : 0;
        long currentTiltSteps = tiltStepper ? tiltStepper->getCurrentPosition() : 0;
        bool isMoving = (panStepper && panStepper->isRunning()) || (tiltStepper && tiltStepper->isRunning());
        bool firingActive = (autoFireRequested || solenoidState != SOLENOID_IDLE);

        Serial.printf("S %lu %ld %ld %d %d\n",
                      now,
                      panStepsToMrad(currentPanSteps),
                      tiltStepsToMrad(currentTiltSteps),
                      firingActive ? 1 : 0,
                      isMoving ? 1 : 0);
    }
}