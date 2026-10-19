#pragma once
#include <Arduino.h>
#include "driver/gpio.h"

#define LCD_H_RES 1024
#define LCD_V_RES 600
#define LCD_PIXEL_CLOCK_HZ (16 * 1000 * 1000)
#define LCD_BOUNCE_LINES 20
#define LCD_DATA_WIDTH 16
#define LCD_BITS_PER_PIXEL 16
#define LCD_FRAME_BUFFERS 2

#define LCD_GPIO_VSYNC GPIO_NUM_3
#define LCD_GPIO_HSYNC GPIO_NUM_46
#define LCD_GPIO_DE    GPIO_NUM_5
#define LCD_GPIO_PCLK  GPIO_NUM_7

#define LCD_GPIO_B0 GPIO_NUM_14
#define LCD_GPIO_B1 GPIO_NUM_38
#define LCD_GPIO_B2 GPIO_NUM_18
#define LCD_GPIO_B3 GPIO_NUM_17
#define LCD_GPIO_B4 GPIO_NUM_10
#define LCD_GPIO_G0 GPIO_NUM_39
#define LCD_GPIO_G1 GPIO_NUM_0
#define LCD_GPIO_G2 GPIO_NUM_45
#define LCD_GPIO_G3 GPIO_NUM_48
#define LCD_GPIO_G4 GPIO_NUM_47
#define LCD_GPIO_G5 GPIO_NUM_21
#define LCD_GPIO_R0 GPIO_NUM_1
#define LCD_GPIO_R1 GPIO_NUM_2
#define LCD_GPIO_R2 GPIO_NUM_42
#define LCD_GPIO_R3 GPIO_NUM_41
#define LCD_GPIO_R4 GPIO_NUM_40

#define I2C_SDA GPIO_NUM_8
#define I2C_SCL GPIO_NUM_9
#define I2C_FREQ_HZ 400000

#define IOX_ADDR 0x24
#define IOX_REG_MODE   0x02
#define IOX_REG_OUTPUT 0x03
#define IOX_TOUCH_RST  1
#define IOX_BACKLIGHT  2

#define GT911_ADDR 0x5D
#define GT911_INT GPIO_NUM_4

#define FLOW_PIN 6
#define FLOW_PULSES_PER_LITER 12
#define FLOW_CALC_INTERVAL_MS 500

#define SECTION_COUNT 6
static const uint8_t SECTION_WIDTH_DM[SECTION_COUNT] = {25, 40, 40, 40, 40, 25};

#define DEFAULT_RATE_L_HA 200
#define RATE_TOLERANCE_PERCENT 10
#define REGULATOR_MIN_TARGET_X10 10

#define AOG_RX_PORT 8888
#define AOG_TX_PORT 9999
#define AOG_TIMEOUT_MS 1500
#define SWITCH_TX_INTERVAL_MS 250

// Pressure: ADS1115 powered at 3.3 V, ADDR to GND, input A0.
// Sensor powered at 5 V; 10k series / 20k to GND divider.
#define PRESSURE_ADS_ADDR 0x48
#define PRESSURE_INTERVAL_MS 100
#define PRESSURE_DIVIDER_RATIO 1.5f
#define PRESSURE_ZERO_V 0.5f
#define PRESSURE_FULL_V 4.5f
#define PRESSURE_MAX_PSI 80.0f
#define PRESSURE_VALID_MIN_V 0.3f
#define PRESSURE_VALID_MAX_V 4.7f

// Manual L298N test via the external UART header, switch set to UART2.
// ENA must be held at GND during boot/reset/flashing. No automatic regulation.
#define VALVE_IN1_PIN 43
#define VALVE_IN2_PIN 44
#define VALVE_TEST_PULSE_MS 1000
