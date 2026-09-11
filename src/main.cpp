/*
    Bombardeer Autonomous Wildlife Deterrent Turret Controller
    ----------------------------------------------------------
    - Native Milliradian (mrad) Motion Protocol:
        * M <d_pan> <d_tilt> : Relative waypoint move (mrad)
        * V <pan_s> <tilt_s> : Dynamic velocity trim (mrad/s)
        * X                  : Controlled dynamic halt
        * H                  : Zero current mechanical coordinates
        * Q                  : Immediate state & motion query
        * T <0|1>            : Autonomous solenoid trigger
        * S <p> <t> <f> <m>  : Periodic 20 Hz telemetry broadcast (mrad)
    - FastAccelStepper hardware-timed trapezoidal acceleration profiles.
    - TMC2209 silent stepping with dynamic torque transition.
    - Automated solenoid trigger pulse state machine.
    - Xbox Series X Bluetooth controller manual override with auto-timeout.
    - Non-blocking zero-allocation serial streaming parser.
*/

#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
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
// Milliradian Kinematic Conversion Constants (Standard 200 step / 1.8 deg motor)
// ============================================================================
// Motor: 200 full steps/rev * 16 usteps = 3200 steps/rev
// Pan Gearing: 100T ring / 15T pinion = 20/3 (~6.6667:1) -> 21,333.333 steps/rev
// 1 Full Revolution = 2000 * PI milliradians (~6283.1853 mrad)
constexpr double PAN_STEPS_PER_MRAD = 32.0 / (3.0 * 3.14159265358979323846); // ~3.395305 steps/mrad
constexpr double PAN_MRAD_PER_STEP = (3.0 * 3.14159265358979323846) / 32.0;  // ~0.294524 mrad/step

constexpr double TILT_STEPS_PER_MRAD = 80.0 / 3.14159265358979323846; // ~25.464791 steps/mrad
constexpr double TILT_MRAD_PER_STEP = 3.14159265358979323846 / 80.0;  // ~0.039270 mrad/step

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
// Pan:  +-180 deg (+-1000*PI mrad) = +-21,334 steps (360 deg continuous sweep)
// Tilt: +-22.5 deg (+-125*PI mrad) = +-10,000 steps (45 deg total vertical envelope)
const long PAN_MIN_STEPS = -10667;
const long PAN_MAX_STEPS = 10667;

const long TILT_MIN_STEPS = -10000;
const long TILT_MAX_STEPS = 10000;

// Xbox Controller Deadzone and Trigger Thresholds
#define DEADZONE_RADIUS 0.30f
#define TRIGGER_THRESHOLD 0.15f

/*
    Timing for Heschen HS-1564B / HS-4564B Solenoid
    * SOLENOID_PULSE_LENGTH = 60 ms
    * SOLENOID_COOLDOWN_LENGTH = 190 ms
    * Total cycle = 250 ms (4.0 BPS)
*/
#define SOLENOID_PULSE_LENGTH 60     // 40-60ms is a good range for the Heschen HS-1564B
#define SOLENOID_COOLDOWN_LENGTH 190 // Time (ms) solenoid rests before next allowed cycle
                                     // Total cycle = 250ms (4 shots per second)
const uint32_t MAX_AUTONOMOUS_FIRE_DURATION_MS = 2000;

// Bind to any Xbox controller
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

// ============================================================================
// Stepper Library Configuration & Instantiations
// ============================================================================

#define PAN_STEPPER_ACCELERATION 15000 // 15,000 steps/s^2 for zero-lag trajectory tracking
#define PAN_STEPPER_MAXSPEEDHZ 30000
#define PAN_STEPPER_MINSPEEDHZ 15

#define TILT_STEPPER_ACCELERATION 20000 // 20,000 steps/s^2
#define TILT_STEPPER_MAXSPEEDHZ 30000
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

// Autonomous Target Speeds
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
    driver.microsteps(16); // 1/16 Microstepping from FastAccelStepper
    driver.intpol(true);   // Interpolate to 1/256 internally for smooth motion

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

    if (!regSelectActive || readMicrosteps != 16)
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
 *
 * @param stepper      Pointer to FastAccelStepper instance
 * @param state        Reference to the persistent AxisControlState tracker
 * @param rawInput     Normalized joystick axis (-1.0 to +1.0)
 * @param maxSpeedHz   Target top speed at 100% stick deflection (e.g., 16000)
 * @param minSpeedHz   Minimum smooth starting speed in Hz (e.g., 250)
 * @param deadzone     Joystick deadzone radius (e.g., 0.30f)
 * @param hysteresisHz Noise threshold before updating speed mid-flight (e.g., 300)
 * @param exponent     Exponential curve factor (1.0 = linear, 2.0 = quadratic, 3.0 = cubic)
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

    // =========================================================================
    // Single-Shot Stop on Deadzone Release
    // =========================================================================
    if (absInput <= deadzone)
    {
        if (state.lastDir != 0)
        {
            stepper->stopMove(); // Trigger smooth deceleration ONCE
            state.lastDir = 0;
            state.lastSpeedHz = 0;
        }
        return; // Exit early so no speed/accel commands disrupt deceleration
    }

    // =========================================================================
    // Percentage of Stick Travel Past Deadzone (0.0 to 1.0)
    // =========================================================================
    float normPct = (absInput - deadzone) / (1.0f - deadzone);
    if (normPct > 1.0f)
        normPct = 1.0f;

    // =========================================================================
    // Exponential Response Curve for Low-Speed Precision
    // =========================================================================
    float curvedPct = powf(normPct, exponent);

    // Map percentage to target frequency (Hz)
    uint32_t targetSpeedHz = (uint32_t)(curvedPct * maxSpeedHz);
    if (targetSpeedHz < minSpeedHz)
        targetSpeedHz = minSpeedHz;

    int currentDir = (rawInput > 0.0f) ? 1 : -1;

    // =========================================================================
    // Hysteresis Filtering & Minimal Driver Updates
    // =========================================================================
    bool dirChanged = (currentDir != state.lastDir);
    bool speedChangedSignificantly = (abs((long)targetSpeedHz - (long)state.lastSpeedHz) > (long)hysteresisHz);

    if (dirChanged)
    {
        // Direction changed OR starting from a stop
        stepper->setSpeedInHz(targetSpeedHz);
        if (currentDir > 0)
        {
            stepper->runForward();
        }
        else
        {
            stepper->runBackward();
        }
        state.lastDir = currentDir;
        state.lastSpeedHz = targetSpeedHz;
    }
    else if (speedChangedSignificantly)
    {
        // Same direction: update speed dynamically ONLY if stick moved past hysteresis band
        stepper->setSpeedInHz(targetSpeedHz);
        stepper->applySpeedAcceleration(); // Signal FastAccelStepper to update speed mid-flight
        state.lastSpeedHz = targetSpeedHz;
    }
}

// ============================================================================
// Solenoid State Machine
// ============================================================================

enum SolenoidState
{
    SOLENOID_IDLE,    // Ready to fire
    SOLENOID_FIRING,  // Active HIGH (pulling trigger)
    SOLENOID_COOLDOWN // Active LOW (waiting to reset sear & prevent rapid-fire stall)
};

static SolenoidState solenoidState = SOLENOID_IDLE;
static unsigned long solenoidTimer = 0;

void triggerSolenoid()
{
    // Only fire if completely idle and cooldown has elapsed
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
        // Pulse time expired -> Turn OFF and begin cooldown/reset window
        if (now - solenoidTimer >= SOLENOID_PULSE_LENGTH)
        {
            DB_PRINTLN("[SOLENOID] Pulse expired, turning OFF.");
            digitalWrite(PIN_SOLENOID, LOW);
            solenoidTimer = now;
            solenoidState = SOLENOID_COOLDOWN;
        }
        break;

    case SOLENOID_COOLDOWN:
        // Cooldown expired -> Return to IDLE so next cycle or held trigger can fire
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
// Hardware Positional Waypoint Actuation (Milliradians)
// ============================================================================

void commandRelativeMoveMrad(long dPanMrad, long dTiltMrad)
{
    commandedPanSpeed = 0;
    commandedTiltSpeed = 0;
    activePanDir = 0;
    activeTiltDir = 0;
    appliedPanSpeed = 0;
    appliedTiltSpeed = 0;

    if (panStepper)
    {
        panStepper->setSpeedInHz(PAN_STEPPER_MAXSPEEDHZ); // Restore full slew speed
        long dPanSteps = panMradToSteps(dPanMrad);
        long curPan = panStepper->getCurrentPosition();
        long targetPan = constrain(curPan + dPanSteps, PAN_MIN_STEPS, PAN_MAX_STEPS);
        panStepper->moveTo(targetPan);
    }

    if (tiltStepper)
    {
        tiltStepper->setSpeedInHz(TILT_STEPPER_MAXSPEEDHZ); // Restore full slew speed
        long dTiltSteps = tiltMradToSteps(dTiltMrad);
        long curTilt = tiltStepper->getCurrentPosition();
        long targetTilt = constrain(curTilt + dTiltSteps, TILT_MIN_STEPS, TILT_MAX_STEPS);
        tiltStepper->moveTo(targetTilt);
    }
}

// ============================================================================
// Continuous Dynamic Velocity Slew Actuation (mrad/s)
// ============================================================================

void applyVelocityActuation()
{
    if (!panStepper || !tiltStepper)
        return;

    long curPan = panStepper->getCurrentPosition();
    long curTilt = tiltStepper->getCurrentPosition();

    // --- 1. PAN AXIS VELOCITY ---
    long targetPan = commandedPanSpeed;
    if (targetPan > 0 && curPan >= PAN_MAX_STEPS)
        targetPan = 0;
    if (targetPan < 0 && curPan <= PAN_MIN_STEPS)
        targetPan = 0;

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
    long targetTilt = commandedTiltSpeed;
    if (targetTilt > 0 && curTilt >= TILT_MAX_STEPS)
        targetTilt = 0;
    if (targetTilt < 0 && curTilt <= TILT_MIN_STEPS)
        targetTilt = 0;

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
    static char rxBuffer[64];
    static uint8_t rxIdx = 0;

    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();
        if (c == '\n')
        {
            rxBuffer[rxIdx] = '\0';
            if (rxIdx > 0)
            {
                char cmdType = rxBuffer[0];

                switch (cmdType)
                {
                // Move Relative in milliradians: M <d_pan_mrad> <d_tilt_mrad>
                case 'M':
                {
                    long dPan = 0, dTilt = 0;
                    if (sscanf(rxBuffer, "M %ld %ld", &dPan, &dTilt) == 2)
                    {
                        commandRelativeMoveMrad(dPan, dTilt);
                    }
                    break;
                }

                // Dynamic Velocity Trim in milliradians/second: V <pan_spd> <tilt_spd>
                case 'V':
                {
                    long pSpd = 0, tSpd = 0;
                    if (sscanf(rxBuffer, "V %ld %ld", &pSpd, &tSpd) == 2)
                    {
                        commandedPanSpeed = pSpd;
                        commandedTiltSpeed = tSpd;
                    }
                    break;
                }

                // Controlled dynamic halt
                case 'X':
                {
                    commandedPanSpeed = 0;
                    commandedTiltSpeed = 0;
                    activePanDir = 0;
                    activeTiltDir = 0;
                    appliedPanSpeed = 0;
                    appliedTiltSpeed = 0;
                    if (panStepper)
                        panStepper->stopMove();
                    if (tiltStepper)
                        tiltStepper->stopMove();
                    break;
                }

                // Autonomous trigger control: T <0|1>
                case 'T':
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

                // Home: set current mechanical positions to coordinate zero
                case 'H':
                {
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

                // Query: Immediate motion and position status in milliradians
                case 'Q':
                {
                    bool isMoving = (panStepper && panStepper->isRunning()) ||
                                    (tiltStepper && tiltStepper->isRunning());
                    long cPanSteps = panStepper ? panStepper->getCurrentPosition() : 0;
                    long cTiltSteps = tiltStepper ? tiltStepper->getCurrentPosition() : 0;

                    Serial.printf("R %d %ld %ld\n",
                                  isMoving ? 1 : 0,
                                  panStepsToMrad(cPanSteps),
                                  tiltStepsToMrad(cTiltSteps));
                    break;
                }

                // Unknown command — ignore or handle as needed
                default:
                    break;
                } // end switch
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
                rxIdx = 0; // Prevent buffer overrun
            }
        }
    }
}

// ============================================================================
// Setup
// ============================================================================

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(2); // Fail-safe: prevent any blocking stream reads
    while (!Serial)
        ; // wait for serial port to connect. Needed for native USB port only
    DB_PRINTLN("\nStarting Bombardeer Turret Controller on " + String(ARDUINO_BOARD));

    // Debug info about the ESP32 we are running on
    DB_PRINTLN("ESP32 Chip Model: " + String(ESP.getChipModel()));
    DB_PRINTLN("ESP32 Chip Revision: " + String(ESP.getChipRevision()));
    DB_PRINTLN("ESP32 Chip Cores: " + String(ESP.getChipCores()));
    DB_PRINTLN("ESP32 CPU Frequency: " + String(ESP.getCpuFreqMHz()) + " MHz");
    DB_PRINTLN("ESP32 Flash Size: " + String(ESP.getFlashChipSize() / (1024 * 1024)) + " MB");
    DB_PRINTLN("ESP32 Flash Speed: " + String(ESP.getFlashChipSpeed() / 1000000) + " MHz");
    DB_PRINTLN("ESP32 PSRAM Size: " + String(ESP.getPsramSize()));
    DB_PRINTLN("ESP32 Free PSRAM: " + String(ESP.getFreePsram()));

    //
    // Setup Xbox controller
    //
    xboxController.begin();

    //
    // Setup the TMC2209 Stepper Drivers and FastAccelStepper Engine
    //

    // Hardware Enable Pin Setup (Shared between Pan and Tilt)
    pinMode(SHARED_ENABLE_PIN, OUTPUT);
    digitalWrite(SHARED_ENABLE_PIN, HIGH); // Drive HIGH initially (Keep drivers disabled during UART config)

    // Explicitly lock in TMC2209 UART Addresses via MS1 / MS2
    // This can be overriden using physical jumpers next to the TMC2209 driver
    pinMode(PAN_USTEP_PIN1, OUTPUT);  // MS1
    pinMode(PAN_USTEP_PIN2, OUTPUT);  // MS2
    pinMode(TILT_USTEP_PIN1, OUTPUT); // MS1
    pinMode(TILT_USTEP_PIN2, OUTPUT); // MS2

    // --- PAN AXIS ADDRESS: 0 (MS1=LOW, MS2=LOW) ---
    digitalWrite(PAN_USTEP_PIN1, LOW);
    digitalWrite(PAN_USTEP_PIN2, LOW);

    // --- TILT AXIS ADDRESS: 1 (MS1=HIGH, MS2=LOW) ---
    digitalWrite(TILT_USTEP_PIN1, HIGH);
    digitalWrite(TILT_USTEP_PIN2, LOW);

    // Give hardware pins 10ms to settle to solid voltage levels
    delay(10);

    // Initialize Hardware UART2 for TMC2209 Drivers
    SERIAL_PORT.begin(115200, SERIAL_8N1, TMC_RX_PIN, TMC_TX_PIN);

    // Configure TMC2209 Registers over UART
    // Passing 'false' runs StealthChop at standstill (eliminates the hum completely)
    // while switching over to SpreadCycle dynamically when moving.
    initTMC2209(panTMC, "PAN", 1200, 0.25f, false);
    initTMC2209(tiltTMC, "TILT", 1100, 0.15f, false);

    // Enable both drivers in hardware permanently
    digitalWrite(SHARED_ENABLE_PIN, LOW); // LOW = Drivers Enabled

    // Initialize FastAccelStepper Engine
    engine.init();

    // Connect Pan Stepper to Hardware Timers
    panStepper = engine.stepperConnectToPin(PAN_STEP_PIN);
    if (panStepper)
    {
        panStepper->setDirectionPin(PAN_DIR_PIN);

        // Set Kinematics (Steps / sec)
        panStepper->setSpeedInHz(PAN_STEPPER_MAXSPEEDHZ);
        panStepper->setAcceleration(PAN_STEPPER_ACCELERATION);
        panStepper->setCurrentPosition(0);
    }
    else
    {
        DB_PRINTLN("[ERROR] Failed to attach Pan stepper to hardware timer!");
    }

    // Connect Tilt Stepper to Hardware Timers
    tiltStepper = engine.stepperConnectToPin(TILT_STEP_PIN);
    if (tiltStepper)
    {
        tiltStepper->setDirectionPin(TILT_DIR_PIN);

        // Set Kinematics
        tiltStepper->setSpeedInHz(TILT_STEPPER_MAXSPEEDHZ);
        tiltStepper->setAcceleration(TILT_STEPPER_ACCELERATION);
        tiltStepper->setCurrentPosition(0);
    }
    else
    {
        DB_PRINTLN("[ERROR] Failed to attach Tilt stepper to hardware timer!");
    }

    //
    // Setup trigger solenoid
    //
    pinMode(PIN_SOLENOID, OUTPUT);
    digitalWrite(PIN_SOLENOID, LOW);
}

// ============================================================================
// Main Execution Loop
// ============================================================================

void loop()
{
    static AxisControlState panState;
    static AxisControlState tiltState;
    static unsigned long lastTelemetryTime = 0;

    // Check every 3 seconds whether the driver brownout reset and lost register config
    static unsigned long lastDriverCheck = 0;
    if (millis() - lastDriverCheck >= 3000)
    {
        lastDriverCheck = millis();

        // If mstep_reg_select dropped to 0 or microsteps reset to 8, re-initialize
        if (!panTMC.mstep_reg_select() || panTMC.microsteps() != 16 ||
            !tiltTMC.mstep_reg_select() || tiltTMC.microsteps() != 16)
        {
            DB_PRINTLN("[TMC2209] Driver reset detected! Re-applying UART parameters...");
            initTMC2209(panTMC, "PAN", 1200, 0.25f, false);
            initTMC2209(tiltTMC, "TILT", 1100, 0.15f, false);
        }
    }

    // 1. Process Autonomous Commands
    processSerialCommands();

    // 2. Xbox Controller Handling (Manual Override with Auto-Timeout)
    static unsigned long lastManualInputTime = 0;
    const unsigned long MANUAL_OVERRIDE_TIMEOUT_MS = 400; // Return to auto 400ms after stick release

    xboxController.onLoop();
    bool manualOverrideActive = false;

    if (xboxController.isConnected() && !xboxController.isWaitingForFirstNotification())
    {
        const float halfJoy = (float)XboxControllerNotificationParser::maxJoy / 2.0f;
        float rawPan = ((float)xboxController.xboxNotif.joyLHori - halfJoy) / halfJoy;
        float rawTilt = ((float)xboxController.xboxNotif.joyLVert - halfJoy) / halfJoy;
        float trigger = ((float)xboxController.xboxNotif.trigRT / XboxControllerNotificationParser::maxTrig);

        bool stickDeflected = (fabs(rawPan) > 0.30f || fabs(rawTilt) > 0.30f);
        bool triggerPressed = (trigger > TRIGGER_THRESHOLD);

        if (stickDeflected || triggerPressed)
        {
            lastManualInputTime = millis();
        }

        // Active manual override window
        if (millis() - lastManualInputTime < MANUAL_OVERRIDE_TIMEOUT_MS)
        {
            manualOverrideActive = true;

            // Direct joystick velocity control
            updateAxisFromJoystick(panStepper, panState, rawPan, PAN_STEPPER_MAXSPEEDHZ, PAN_STEPPER_MINSPEEDHZ, 0.30f);
            updateAxisFromJoystick(tiltStepper, tiltState, rawTilt, TILT_STEPPER_MAXSPEEDHZ, TILT_STEPPER_MINSPEEDHZ, 0.30f);

            if (triggerPressed)
            {
                triggerSolenoid();
            }
        }
        else
        {
            // Human let go and timeout elapsed: ensure deceleration completed and reset state
            if (panState.lastDir != 0 || tiltState.lastDir != 0)
            {
                panStepper->stopMove();
                tiltStepper->stopMove();
                panState.lastDir = 0;
                panState.lastSpeedHz = 0;
                tiltState.lastDir = 0;
                tiltState.lastSpeedHz = 0;
            }
        }
    }

    // 3. Autonomous Tracking (Executes whenever human is not touching the controller)
    if (!manualOverrideActive)
    {
        applyVelocityActuation();
    }

    // 4. Autonomous Firing Logic
    if (autoFireRequested)
    {
        unsigned long currentMillis = millis();
        if (currentMillis - autoFireStartTime >= MAX_AUTONOMOUS_FIRE_DURATION_MS)
        {
            autoFireRequested = false;
        }
        else
        {
            triggerSolenoid();
        }
    }

    // 5. Update Solenoid Actuator Timing
    updateSolenoid();

    // 6. Broadcast Telemetry to Raspberry Pi at 20 Hz (every 50 ms) in milliradians
    unsigned long now = millis();
    if (now - lastTelemetryTime >= 50)
    {
        lastTelemetryTime = now;
        long currentPanSteps = panStepper ? panStepper->getCurrentPosition() : 0;
        long currentTiltSteps = tiltStepper ? tiltStepper->getCurrentPosition() : 0;
        bool isMoving = (panStepper && panStepper->isRunning()) || (tiltStepper && tiltStepper->isRunning());
        int firingActive = (autoFireRequested || solenoidState != SOLENOID_IDLE) ? 1 : 0;

        Serial.printf("S %ld %ld %d %d\n", panStepsToMrad(currentPanSteps), tiltStepsToMrad(currentTiltSteps), firingActive, isMoving ? 1 : 0);
    }
}