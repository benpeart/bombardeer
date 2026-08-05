#ifndef GLOBALS_H
#define GLOBALS_H

//#define BATTERY_VOLTAGE

// ESP32 Pin Assignments

// Stepper motor pin assignments
#define motEnablePin 27

#define motLeftUStepPin1 19
#define motLeftUStepPin2 18
#define motLeftStepPin 26
#define motLeftDirPin 25

#define motRightUStepPin1 05
#define motRightUStepPin2 04
#define motRightStepPin 33
#define motRightDirPin 32

// TMC2209 Stepper driver
#define SERIAL2_RX_PIN 16    // Specify Serial2 RX pin as the default has changed
#define SERIAL2_TX_PIN 17    // Specify Serial2 TX pin as the default has changed

// -- Others
#define PIN_LED_DATA 02        // pin to the data line of WS2812 LEDs
#define PIN_BATTERY_VOLTAGE 36 // ADC pin connected to voltage divider

#define PIN_I2C_SDA 21       // MPU SDA pin
#define PIN_I2C_SCL 22       // MPU SCL pin
#define PIN_MPU_INTERRUPT 23 // MPU interrupt pin, RISING triggers interrupt

extern Preferences preferences;

#endif // GLOBALS_H
