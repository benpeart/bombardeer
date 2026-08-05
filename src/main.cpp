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
#define DEADZONE_RADIUS 0.08
#define TRIGGER_THRESHOLD 0.15
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

void onXboxConnect()
{
    DB_PRINTLN("Bluetooth MAC address: " + xboxController.buildDeviceAddressStr());
    DB_PRINT(xboxController.xboxNotif.toString());
    DB_PRINTLN("Xbox controller connected");
}

void onXboxDisconnect()
{
    DB_PRINTLN("Xbox controller disconnected");
}

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

// ----- Main code
void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // wait for serial port to connect. Needed for native USB port only
    DB_PRINTLN("\nStarting Bombardeer on " + String(ARDUINO_BOARD));

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

    // Disable steppers during startup
    pinMode(motEnablePin, OUTPUT);
    digitalWrite(motEnablePin, HIGH); // disable driver in hardware

    // setup micro stepping/serial address pins for output
    pinMode(motLeftUStepPin1, OUTPUT);
    pinMode(motLeftUStepPin2, OUTPUT);
    pinMode(motRightUStepPin1, OUTPUT);
    pinMode(motRightUStepPin2, OUTPUT);

    // Setup Xbox controller
    xboxController.begin();

    DB_PRINTLN("Booted, ready for action!");
}

void loop()
{
    static boolean firstNotification = true;

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

    // Handle Xbox controller
    xboxController.onLoop();
    if (xboxController.isConnected())
    {
        //DB_PRINTLN("Xbox controller connected");
        if (xboxController.isWaitingForFirstNotification())
        {
            DB_PRINTLN("waiting for first xbox controller notification");
#ifdef DEBUG
            static const char *spinner = "|/-\\";
            static int spinner_index = 0;

            DB_PRINTF("\r%c", spinner[spinner_index]);
            spinner_index = (spinner_index + 1) % sizeof(spinner);
#endif // DEBUG
        }
        else
        {
            if (firstNotification)
            {
                firstNotification = false;
                onXboxConnect();
            }

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
            DB_PRINTF("pan = %f\n", pan);

            //
            // Tilt
            //
            // normalize the controller input to the range of -1.0 to 1.0
            float tilt = (float)(xboxController.xboxNotif.joyLVert - (XboxControllerNotificationParser::maxJoy / 2)) / (XboxControllerNotificationParser::maxJoy / 2);

            // if within the dead zone, zero it out
            if (tilt > -DEADZONE_RADIUS && tilt < DEADZONE_RADIUS)
                tilt = 0;

            // use a power of 2 (squared) response curve to dampen the response around center and ramp it up the further you go
            if (tilt)
                tilt = -(tilt * tilt);
            else
                tilt = tilt * tilt;
            DB_PRINTF("tilt = %f\n", tilt);

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
    else
    {
        DB_PRINTLN("Xbox controller disconnected");
        if (!firstNotification)
            onXboxDisconnect();
        firstNotification = true;
    }
}
