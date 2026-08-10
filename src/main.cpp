/*
    Bombadeer is an autonomous, vision-guided deterrent turret designed to protect outdoor spaces
    from intrusive wildlife (such as deer). Powered by a **Raspberry Pi 5** with a **26 TOPS Hailo AI accelerator**
    for real-time target detection and an **ESP32** running **AccelStepper** for precision pan/tilt motor control.
    Bombadeer detects targets in real-time and actuates a paintball deterrent payload over high-speed UART or manual
    Xbox controller override.
*/

#include <Arduino.h>
#include <Preferences.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include "globals.h"
#include "debug.h"

// -- EEPROM
Preferences preferences;
#define PREF_VERSION 1 // if setting structure has been changed, count this number up to delete all settings
#define PREF_NAMESPACE "pref"
#define PREF_KEY_VERSION "ver"

#define BATTERY_VOLTAGE_USB 5       // the voltage when running via USB instead of the battery
#define BATTERY_VOLTAGE_LOW 20      // the voltage we warn the user
#define BATTERY_VOLTAGE_SHUTDOWN 18 // the voltage we shutdown to prevent damaging the battery
#define BATTERY_VOLTAGE_FULL 22.3   // the voltage of a full battery

// bind to any xbox controller
#define DEADZONE_RADIUS 0.25f
#define TRIGGER_THRESHOLD 0.15f
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

#ifdef BATTERY_VOLTAGE
float BatteryVoltage()
{
    // Measure battery voltage
#ifdef BATTERY_VOLTAGE
    const float R1 = 100000.0;        // 100kΩ
    const float R2 = 10000.0;         // 10kΩ
    const float ADC_MAX = 4095.0;     // 12-bit ADC
    const float V_REF = 3.3;          // Reference voltage
    const float ALPHA = 0.05;         // low pass filter
    static float filteredBattery = 0; // use a low-pass filter to smooth battery readings

    int adcValue = analogRead(PIN_BATTERY_VOLTAGE);
    float voltage = (adcValue / ADC_MAX) * V_REF;
    float batteryVoltage = voltage * (R1 + R2) / R2;

    // take the first and filter the rest
    if (!filteredBattery)
        filteredBattery = batteryVoltage;
    else
        filteredBattery = (ALPHA * batteryVoltage) + ((1 - ALPHA) * filteredBattery);

    // return the battery voltage
    return filteredBattery;
#endif // BATTERY_VOLTAGE
}
#endif // BATTERY_VOLTAGE

// ============================================================================
// Stepper LIBRARY INSTANTIATIONS
// ============================================================================

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
 * Configure TMC2209 Driver Parameters over UART using teemuatlut/TMCStepper
 */
void initTMC2209(TMC2209Stepper &driver, const char *axisName, uint16_t current_mA, bool useSpreadCycle)
{
    driver.begin();

    // Verify UART Communication
    uint8_t result = driver.test_connection();
    if (result != 0)
    {
        DB_PRINTF("[TMC2209] ERROR: %s axis connection failed! Code: %d (Check wiring/addressing)\n", axisName, result);
        return;
    }

    driver.toff(4);        // Enable driver chopper (TOFF = 4 is recommended for 24V/12V systems)
    driver.blank_time(24); // Set comparator blank time

    // Set RMS current and holding current ratio (0.5 = 50% hold current)
    // 17HS19-2004S1 is rated 2.0A Peak (~1414mA RMS).
    // Setting 1200mA RMS is ~85% max capacity to keep drivers cool without active fans.
    driver.rms_current(current_mA, 0.5);

    driver.microsteps(16); // 1/16 Microstepping from controller
    driver.intpol(true);   // Interpolate to 1/256 microsteps internally (Ultra smooth)

    driver.iholddelay(10); // Delays ~0.3 seconds after last step before entering hold current
    driver.TPOWERDOWN(20); // Delay between standstill and powerdown

    if (useSpreadCycle)
    {
        // High-torque mode: Recommended for high speed/accel (15000 steps/sec)
        driver.en_spreadCycle(true);
        driver.pwm_autoscale(false);
        DB_PRINTF("[TMC2209] %s Configured: %d mA RMS, SpreadCycle (High Torque)\n", axisName, current_mA);
    }
    else
    {
        // Quiet mode with automatic StealthChop -> SpreadCycle velocity threshold
        driver.en_spreadCycle(false);
        driver.pwm_autoscale(true);
        // Automatically switch from StealthChop to SpreadCycle at higher RPM to prevent lost steps
        driver.TPWMTHRS(100);
        DB_PRINTF("[TMC2209] %s Configured: %d mA RMS, StealthChop + Hybrid Switch\n", axisName, current_mA);
    }
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
 * @param rawInput     Normalized joystick axis (-1.0 to +1.0)
 * @param deadzone     Joystick deadzone radius (e.g., 0.15f)
 * @param maxSpeedHz   Target top speed at 100% stick deflection (e.g., 15000)
 * @param minSpeedHz   Minimum smooth starting speed in Hz (e.g., 250)
 * @param hysteresisHz Noise threshold before updating speed mid-flight (e.g., 300)
 * @param exponent     Exponential curve factor (1.0 = linear, 2.0 = quadratic, 3.0 = cubic)
 * @param stepper      Pointer to FastAccelStepper instance
 * @param state        Reference to the persistent AxisControlState tracker
 */
void updateAxisFromJoystick(
    float rawInput,
    float deadzone,
    uint32_t maxSpeedHz,
    uint32_t minSpeedHz,
    uint32_t hysteresisHz,
    float exponent,
    FastAccelStepper *stepper,
    AxisControlState &state)
{
    if (!stepper)
        return;

    float absInput = fabs(rawInput);

    // =========================================================================
    // REQUIREMENT 4: Single-Shot Stop on Deadzone Release
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
    // REQUIREMENT 1: Percentage of Stick Travel Past Deadzone (0.0 to 1.0)
    // =========================================================================
    float normPct = (absInput - deadzone) / (1.0f - deadzone);
    if (normPct > 1.0f)
        normPct = 1.0f;

    // =========================================================================
    // REQUIREMENT 2: Exponential Response Curve for Low-Speed Precision
    // =========================================================================
    float curvedPct = powf(normPct, exponent);

    // Map percentage to target frequency (Hz)
    uint32_t targetSpeedHz = (uint32_t)(curvedPct * maxSpeedHz);
    if (targetSpeedHz < minSpeedHz)
        targetSpeedHz = minSpeedHz;

    int currentDir = (rawInput > 0.0f) ? 1 : -1;

    // =========================================================================
    // REQUIREMENT 3: Hysteresis Filtering & Minimal Driver Updates
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

    // Init preferences EEPROM, if not done before
    preferences.begin(PREF_NAMESPACE, false); // false = RW-mode
    if (preferences.getUInt(PREF_KEY_VERSION, 0) != PREF_VERSION)
    {
        preferences.clear(); // Remove all preferences under the opened namespace
        preferences.putUInt(PREF_KEY_VERSION, PREF_VERSION);
        DB_PRINTF("EEPROM init complete, all preferences deleted, new pref_version: %d\n", PREF_VERSION);
    }

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
    initTMC2209(panTMC, "PAN", 1300, true);   // Pan Axis: 1300 mA RMS, SpreadCycle enabled for rapid 180-degree pans
    initTMC2209(tiltTMC, "TILT", 1200, true); // Tilt Axis: 1200 mA RMS, SpreadCycle enabled to maintain torque against gravity/payload

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
        panStepper->setSpeedInHz(15000); // 15000 steps/sec max

        // 12,000 steps/s^2 reaches 15,000 Hz in 1.25 seconds (well under the 2-second limit)
        // and brings the motor to a full stop from top speed in 1.25 seconds.
        panStepper->setAcceleration(12000);
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
        tiltStepper->setSpeedInHz(15000); // 15000 steps/sec max

        // 12,000 steps/s^2 reaches 15,000 Hz in 1.25 seconds (well under the 2-second limit)
        // and brings the motor to a full stop from top speed in 1.25 seconds.
        tiltStepper->setAcceleration(12000); // 12000 steps/sec^2
    }
    else
    {
        DB_PRINTLN("[ERROR] Failed to attach Tilt stepper to hardware timer!");
    }
}

void loop()
{
    // Declare persistent state objects outside loop or as static
    static AxisControlState panState;
    static AxisControlState tiltState;

    // check the battery voltage and if necessary, inform the user
#ifdef BATTERY_VOLTAGE
    EVERY_N_MILLISECONDS(5000)
    {
        static float batteryVoltage = BATTERY_VOLTAGE_FULL;
        batteryVoltage = BatteryVoltage();

        // check to see if we're running via USB instead of the battery
        if (batteryVoltage < BATTERY_VOLTAGE_USB)
            batteryVoltage = BATTERY_VOLTAGE_FULL;

        if (batteryVoltage < BATTERY_VOLTAGE_LOW)
        {
            LED_set(LED_BATTERY, CRGB::Yellow);
        }
        if (batteryVoltage <= BATTERY_VOLTAGE_SHUTDOWN)
        {
            DB_PRINTF("Battery voltage is critically low (%.1f). Entering deep sleep mode...\n", batteryVoltage);
            LED_set(LED_BATTERY, CRGB::Red);

            // shut off the motors
            bc.setState(Disabled::GetInstance(), 0, NULL);
            LED_loop();
            esp_deep_sleep_start(); // Enter deep sleep mode
            return;
        }
    }
#endif // BATTERY_VOLTAGE

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
        // Deadzone: 0.15 | MaxSpeed: 15000Hz | MinSpeed: 250Hz | Hysteresis: 300Hz | Exponent: 2.0 (Quadratic)
        updateAxisFromJoystick(rawPan, DEADZONE_RADIUS, 15000, 250, 300, 2.0f, panStepper, panState);

        // Process Tilt Axis:
        updateAxisFromJoystick(rawTilt, DEADZONE_RADIUS, 15000, 250, 300, 2.0f, tiltStepper, tiltState);

        // Process right trigger:
        // normalize the controller input to the range of 0.0 to 1.0
        float trigger = ((float)xboxController.xboxNotif.trigRT / XboxControllerNotificationParser::maxTrig);
        if (trigger > TRIGGER_THRESHOLD)
            DB_PRINTF("trigger = %f\n", trigger);
    }
}
