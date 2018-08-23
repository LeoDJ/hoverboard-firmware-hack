#pragma once
#include "stm32f1xx_hal.h"

// ################################################################################

#define PWM_FREQ         16000      // PWM frequency in Hz
#define DEAD_TIME        32         // PWM deadtime

#define DC_CUR_LIMIT     25         // Motor DC current limit in amps

#define BAT_LOW_LVL1     44.0       // gently beeps at this voltage level
#define BAT_LOW_LVL2     41.0       // your battery is almost empty. Charge now!
#define BAT_LOW_DEAD     39.0       // undervoltage lockout

// ################################################################################

//#define DEBUG_SERIAL_USART2
#define DEBUG_SERIAL_USART3
#define DEBUG_BAUD       115200     // UART baud rate
//#define DEBUG_SERIAL_SERVOTERM
#define DEBUG_SERIAL_ASCII
//#define DEBUG_I2C_LCD

#define CONTROL_USART2_RXTX
#define CONTROL_BAUD       250000

#define TIMEOUT          5000           //number of wrong / missing commands before emergency off

// ###### DRIVING BEHAVIOR ######
#define FILTER 0.1
