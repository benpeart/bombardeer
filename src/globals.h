#ifndef GLOBALS_H
#define GLOBALS_H

//#define BATTERY_VOLTAGE

// ESP32 Pin Assignments

// Driver 1: Pan Axis
#define PAN_USTEP_PIN1 04
#define PAN_USTEP_PIN2 27
#define PAN_STEP_PIN   26
#define PAN_DIR_PIN    25

// Driver 2: Tilt Axis
#define TILT_USTEP_PIN1 18
#define TILT_USTEP_PIN2 05
#define TILT_STEP_PIN   33
#define TILT_DIR_PIN    32

#define SHARED_ENABLE_PIN 19

// TMC2209 Single-Wire UART Pins (Shared UART2)
#define TMC_RX_PIN       16  // ESP32 RX2 connected to TMC2209 TX/RX pins
#define TMC_TX_PIN       17  // ESP32 TX2 connected through 1k ohm resistor
#define SERIAL_PORT      Serial2
#define R_SENSE          0.11f // Standard sense resistor value for stepsticks

// Driver Addresses on the shared UART bus
#define PAN_DRIVER_ADDR  0b00  // MS1=GND, MS2=GND
#define TILT_DRIVER_ADDR 0b01  // MS1=VCC, MS2=GND

// -- Others
#define PIN_LED_DATA 02        // pin to the data line of WS2812 LEDs
#define PIN_BATTERY_VOLTAGE 36 // ADC pin connected to voltage divider

#define PIN_I2C_SDA 21       // MPU SDA pin
#define PIN_I2C_SCL 22       // MPU SCL pin
#define PIN_MPU_INTERRUPT 23 // MPU interrupt pin, RISING triggers interrupt

extern Preferences preferences;

#endif // GLOBALS_H
