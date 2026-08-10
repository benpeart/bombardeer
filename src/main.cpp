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
#define DEADZONE_RADIUS 0.15
#define TRIGGER_THRESHOLD 0.15
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
void initTMC2209(TMC2209Stepper &driver, const char *axisName, uint16_t current_mA)
{
    driver.begin();

    // Verify UART Communication
    uint8_t result = driver.test_connection();
    if (result != 0)
    {
        DB_PRINTF("[TMC2209] ERROR: %s axis connection failed! Code: %d (Check wiring/addressing)", axisName, result);
        return;
    }

    driver.toff(5);                 // Enable driver (Disable = 0)
    driver.rms_current(current_mA); // Set run current in mA RMS
    driver.iholddelay(10);          // Holding current delay
    driver.microsteps(16);          // Set microstepping to 1/16
    driver.en_spreadCycle(false);   // Enable StealthChop2 (Whisper quiet)
    driver.pwm_autoscale(true);     // Auto-tune motor current scale

    DB_PRINTF("[TMC2209] %s Driver Configured OK: %d mA RMS, 1/16 Microstepping, StealthChop ON", axisName, current_mA);
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

    // Setup Xbox controller
    xboxController.begin();

    // Disable steppers during startup
    pinMode(PAN_ENABLE_PIN, OUTPUT);
    digitalWrite(PAN_ENABLE_PIN, HIGH); // disable driver in hardware

    // setup micro stepping/serial address pins for output
    pinMode(PAN_USTEP_PIN1, OUTPUT);
    pinMode(PAN_USTEP_PIN2, OUTPUT);
    pinMode(PAN_STEP_PIN, OUTPUT);
    pinMode(PAN_DIR_PIN, OUTPUT);
    pinMode(TILT_USTEP_PIN1, OUTPUT);
    pinMode(TILT_USTEP_PIN2, OUTPUT);
    pinMode(TILT_STEP_PIN, OUTPUT);
    pinMode(TILT_DIR_PIN, OUTPUT);

    // 1. Initialize Hardware UART2 for TMC2209 Drivers
    SERIAL_PORT.begin(115200, SERIAL_8N1, TMC_RX_PIN, TMC_TX_PIN);

    // 2. Configure TMC2209 Registers over UART
    DB_PRINTLN("[TMC2209] Configuring Drivers...");
    initTMC2209(panTMC, "PAN", 1200);   // 1200 mA RMS run current
    initTMC2209(tiltTMC, "TILT", 1000); // 1000 mA RMS run current

    // 3. Initialize FastAccelStepper Engine
    engine.init();

    // 4. Connect Pan Stepper to Hardware Timers
    panStepper = engine.stepperConnectToPin(PAN_STEP_PIN);
    if (panStepper)
    {
        panStepper->setDirectionPin(PAN_DIR_PIN);
        panStepper->setEnablePin(PAN_ENABLE_PIN);
        panStepper->setAutoEnable(true); // Automatically disables driver on idle to save power

        // Set Kinematics (Steps / sec)
        panStepper->setSpeedInHz(15000);    // 15000 steps/sec max
        panStepper->setAcceleration(20000); // 20000 steps/sec^2
        DB_PRINTLN("[STEPPER] Pan axis hardware timer initialized successfully.");
    }
    else
    {
        DB_PRINTLN("[ERROR] Failed to attach Pan stepper to hardware timer!");
    }

    // 5. Connect Tilt Stepper to Hardware Timers
    tiltStepper = engine.stepperConnectToPin(TILT_STEP_PIN);
    if (tiltStepper)
    {
        tiltStepper->setDirectionPin(TILT_DIR_PIN);
        tiltStepper->setEnablePin(TILT_ENABLE_PIN);
        tiltStepper->setAutoEnable(true);

        // Set Kinematics
        tiltStepper->setSpeedInHz(15000);    // 15000 steps/sec max
        tiltStepper->setAcceleration(20000); // 20000 steps/sec^2
        DB_PRINTLN("[STEPPER] Tilt axis hardware timer initialized successfully.");
    }
    else
    {
        DB_PRINTLN("[ERROR] Failed to attach Tilt stepper to hardware timer!");
    }

    DB_PRINTLN("[SYSTEM] Bombadeer Controller Online. Ready for tracking commands.");
}

void loop()
{
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

    processSerialCommands();

    // Handle Xbox controller
    xboxController.onLoop();
    if (xboxController.isConnected())
    {
        if (xboxController.isWaitingForFirstNotification())
        {
#ifdef DEBUG
            static const char *spinner = "|/-\\";
            static int spinner_index = 0;

            DB_PRINTF("\r%c", spinner[spinner_index]);
            spinner_index = (spinner_index + 1) % sizeof(spinner);
#endif // DEBUG
        }
        else
        {
            //
            // Pan
            //
            // normalize the controller input to the range of -1.0 to 1.0
            float pan = (float)(xboxController.xboxNotif.joyLHori - (XboxControllerNotificationParser::maxJoy / 2)) / (XboxControllerNotificationParser::maxJoy / 2);

            // if within the dead zone, zero it out
            if (pan > -DEADZONE_RADIUS && pan < DEADZONE_RADIUS)
                pan = 0;

            // use a power of 2 (squared) response curve to dampen the response around center and ramp it up the further you go
            if (pan < 0)
                pan = -(pan * pan);
            else
                pan = pan * pan;

            // move in response to the pan input, scaled by a factor of 1000 steps per loop iteration
            if (panStepper && pan)
                panStepper->moveTo(panStepper->getCurrentPosition() + pan * 10000);

            //
            // Tilt
            //
            // normalize the controller input to the range of -1.0 to 1.0
            float tilt = (float)(xboxController.xboxNotif.joyLVert - (XboxControllerNotificationParser::maxJoy / 2)) / (XboxControllerNotificationParser::maxJoy / 2);

            // if within the dead zone, zero it out
            if (tilt > -DEADZONE_RADIUS && tilt < DEADZONE_RADIUS)
                tilt = 0;

            // use a power of 2 (squared) response curve to dampen the response around center and ramp it up the further you go
            if (tilt < 0)
                tilt = -(tilt * tilt);
            else
                tilt = tilt * tilt;

            // move in response to the tilt input, scaled by a factor of 1000 steps per loop iteration
            if (tiltStepper && tilt)
                tiltStepper->moveTo(tiltStepper->getCurrentPosition() + tilt * 10000);

            //
            // Trigger
            //
            // normalize the controller input to the range of 0.0 to 1.0
            float trigger = ((float)xboxController.xboxNotif.trigRT / XboxControllerNotificationParser::maxTrig);
            if (trigger > TRIGGER_THRESHOLD)
                DB_PRINTF("trigger = %f\n", trigger);

#if 0
            // normalize the controller input to the range of 0.0 to 1.0
            float car_speed_forward = ((float)xboxController.xboxNotif.trigRT / XboxControllerNotificationParser::maxTrig);
            float car_speed_reverse = ((float)xboxController.xboxNotif.trigLT / XboxControllerNotificationParser::maxTrig);

            // subtract the requested reverse speed from the requested forward speed in case both triggers are requesting different values
            remoteControl.speed = -(car_speed_forward - car_speed_reverse);

            // if within the dead zone, zero it out
            if (remoteControl.speed > -SPEED_DEADZONE_RADIUS && remoteControl.speed < SPEED_DEADZONE_RADIUS)
                remoteControl.speed = 0;

            // adjust the gain and scale the result according to the d-pad input
            remoteControl.speed = remoteControl.speed * remoteControl.speedGain + remoteControl.speedOffset;
            remoteControl.steer = remoteControl.steer * remoteControl.steerGain;

            // now scale the output from -1.0 to 1.0 to -100 to 100
            remoteControl.speed *= 100;
            remoteControl.steer *= 100;

            // handle other Xbox inputs
            xboxController.getReceiveNotificationAt();
            if (xboxController.xboxNotif.btnDirUp)
                ;
            if (xboxController.xboxNotif.btnDirDown)
                ;
            if (xboxController.xboxNotif.btnDirLeft)
                ;
            if (xboxController.xboxNotif.btnDirRight)
                ;
            if (xboxController.xboxNotif.btnA)
                ;
            if (xboxController.xboxNotif.btnB)
                ;
            if (xboxController.xboxNotif.btnLB)
                ;
            if (xboxController.xboxNotif.btnRB)
                ;
#endif
        }
    }
}
