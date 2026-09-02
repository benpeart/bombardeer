/*
    Bombadeer is an autonomous, vision-guided deterrent turret designed to protect outdoor spaces
    from intrusive wildlife (such as deer). Powered by a **Raspberry Pi 5** with a **26 TOPS Hailo AI accelerator**
    for real-time target detection and an **ESP32** running **AccelStepper** for precision pan/tilt motor control.
    Bombadeer detects targets in real-time and actuates a paintball deterrent payload over high-speed UART or manual
    Xbox controller override.
*/

#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include "debug.h"

// ESP32 Pin Assignments

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

// -- Others
#define PIN_SOLENOID 13      // pin to control the solenoid
#define PIN_I2C_SDA 21       // MPU SDA pin
#define PIN_I2C_SCL 22       // MPU SCL pin
#define PIN_MPU_INTERRUPT 23 // MPU interrupt pin, RISING triggers interrupt

// Xbox Controller Deadzone and Trigger Thresholds
#define DEADZONE_RADIUS 0.25f
#define TRIGGER_THRESHOLD 0.15f

/*
    Recommended Timing for Auto-Fire Code

    To tune your rapid-fire code to the 5-7 BPS sweet spot with a Gravity Hopper:
    * SOLENOID_PULSE_LENGTH = 70 ms
    * SOLENOID_COOLDOWN_LENGTH = 130 ms
    * Result: 5.0 BPS (Extremely reliable, zero chopped paint)
*/
#define SOLENOID_PULSE_LENGTH 60     // 40-60ms is a good range for the For the Heschen HS-1564B
#define SOLENOID_COOLDOWN_LENGTH 190 // Time (ms) solenoid rests before next allowed cycle
                                     // Total cycle = 250ms (4 shots per second)

// bind to any xbox controller
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

// ============================================================================
// Stepper LIBRARY INSTANTIATIONS
// ============================================================================

#define PAN_STEPPER_ACCELERATION 8000 // 8000 steps/sec^2
#define PAN_STEPPER_MAXSPEEDHZ 8000   // 8000 steps/sec max
#define PAN_STEPPER_MINSPEEDHZ 250
#define TILT_STEPPER_ACCELERATION 16000 // 16000 steps/sec^2
#define TILT_STEPPER_MAXSPEEDHZ 16000   // 16000 steps/sec max
#define TILT_STEPPER_MINSPEEDHZ 250
#define STEPPER_HYSTERESISHZ 300
#define STEPPER_EXPONENT 2.0f // 1.0f = linear, 2.0f = quadratic, 3.0f = cubic

// TMCStepper Driver Configuration Objects
TMC2209Stepper panTMC(&SERIAL_PORT, R_SENSE, PAN_DRIVER_ADDR);
TMC2209Stepper tiltTMC(&SERIAL_PORT, R_SENSE, TILT_DRIVER_ADDR);

// FastAccelStepper Engine & Motor Handles
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *panStepper = NULL;
FastAccelStepper *tiltStepper = NULL;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/*
 * Configure TMC2209 Driver Parameters deterministically over UART
 */
void initTMC2209(TMC2209Stepper &driver, const char *axisName, uint16_t current_mA, bool forceSpreadCycle = false)
{
    driver.begin();

    // 1. Verify UART Communication
    uint8_t result = driver.test_connection();
    if (result != 0)
    {
        DB_PRINTF("[TMC2209] ERROR: %s axis connection failed! Code: %d (Check wiring/addressing)\n", axisName, result);
        return;
    }

    // 2. Clear fault flags & configure operating mode
    driver.toff(0);                // Disable driver during initial register writes
    driver.pdn_disable(true);      // Use PDN pin exclusively for UART communication
    driver.mstep_reg_select(true); // Microsteps configured via UART registers, not hardware pins

    // 3. Set RMS active current and 50% holding current
    driver.rms_current(current_mA, 0.5); // 50% hold current drops heat & hum at standstill

    // 4. Microstepping & Internal Interpolation
    driver.microsteps(16); // 1/16 Microstepping from FastAccelStepper
    driver.intpol(true);   // Interpolate to 1/256 internally for smooth motion

    // 5. Standstill & Powerdown Timing
    driver.iholddelay(8);  // ~0.25s delay after motion stops before dropping to hold current
    driver.TPOWERDOWN(20); // Standstill to powerdown delay
    driver.freewheel(0);   // Normal current reduction during hold (0 = standard hold current)

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

    // 7. Enable chopper with TOFF = 4
    driver.toff(4);
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
 * @param deadzone     Joystick deadzone radius (e.g., 0.15f)
 * @param maxSpeedHz   Target top speed at 100% stick deflection (e.g., 15000)
 * @param minSpeedHz   Minimum smooth starting speed in Hz (e.g., 250)
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

/**
 * --- Full-Auto Fire State Machine ---
 */
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

/*
 * Parse Incoming Serial ASCII Targeting Commands from Raspberry Pi 5
 * Example Frame: "P:1200,T:-450"
 */
void processSerialCommands()
{
    static String inputBuffer = "";

    while (Serial.available() > 0)
    {
        char c = Serial.read();
        if (c == '\n')
        {
            long panTarget = 0, tiltTarget = 0;
            if (sscanf(inputBuffer.c_str(), "P:%ld,T:%ld", &panTarget, &tiltTarget) == 2)
            {
                if (panStepper)
                    panStepper->moveTo(panTarget);
                if (tiltStepper)
                    tiltStepper->moveTo(tiltTarget);
            }
            else
            {
                DB_PRINTLN("[ERROR] Invalid command format received: " + inputBuffer);
            }

            inputBuffer = "";
        }
        else if (c != '\r')
        {
            inputBuffer += c;
        }
    }
}

// ============================================================================
// Main code
// ============================================================================

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // wait for serial port to connect. Needed for native USB port only
    DB_PRINTLN("\nStarting Bombardeer Turret Controller on " + String(ARDUINO_BOARD));

    // debug info about the ESP32 we are running on
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
    initTMC2209(panTMC, "PAN", 1300, false);   // Pan Axis: 1300 mA RMS
    initTMC2209(tiltTMC, "TILT", 1200, false); // Tilt Axis: 1200 mA RMS

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

        // 12,000 steps/s^2 reaches 15,000 Hz in 1.25 seconds (well under the 2-second limit)
        // and brings the motor to a full stop from top speed in 1.25 seconds.
        panStepper->setAcceleration(PAN_STEPPER_ACCELERATION);
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

        // 12,000 steps/s^2 reaches 15,000 Hz in 1.25 seconds (well under the 2-second limit)
        // and brings the motor to a full stop from top speed in 1.25 seconds.
        tiltStepper->setAcceleration(TILT_STEPPER_ACCELERATION);
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

void loop()
{
    static AxisControlState panState;
    static AxisControlState tiltState;

    // handle incoming serial commands from the Raspberry Pi 5
    processSerialCommands();

    // handle the Xbox controller input
    xboxController.onLoop();
    if (xboxController.isConnected() && !xboxController.isWaitingForFirstNotification())
    {
        // Dynamically calculate center offset and half-range using maxJoy
        const float halfJoy = (float)XboxControllerNotificationParser::maxJoy / 2.0f;

        // Normalize raw inputs to -1.0 to +1.0 range
        float rawPan = ((float)xboxController.xboxNotif.joyLHori - halfJoy) / halfJoy;
        float rawTilt = ((float)xboxController.xboxNotif.joyLVert - halfJoy) / halfJoy;

        // Process Pan Axis:
        updateAxisFromJoystick(panStepper, panState, rawPan, PAN_STEPPER_MAXSPEEDHZ, PAN_STEPPER_MINSPEEDHZ);

        // Process Tilt Axis:
        updateAxisFromJoystick(tiltStepper, tiltState, rawTilt, TILT_STEPPER_MAXSPEEDHZ, TILT_STEPPER_MINSPEEDHZ);

        // Process Right Trigger (Normalized to 0.0 to 1.0)
        float trigger = ((float)xboxController.xboxNotif.trigRT / XboxControllerNotificationParser::maxTrig);
        if (trigger > TRIGGER_THRESHOLD)
        {
            // When trigger is held down, triggerSolenoid() is called every loop,
            // but will safely fire only once every (PULSE_LENGTH + COOLDOWN_LENGTH) ms
            triggerSolenoid();
        }
    }

    // Non-blocking timer update for the solenoid states
    updateSolenoid();
}
