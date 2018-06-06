#pragma once
#include "stm32f1xx_hal.h"

// ################################################################################

#define PWM_FREQ         16000      // PWM frequency in Hz
#define DEAD_TIME        32         // PWM deadtime

#define DC_CUR_LIMIT     15         // Motor DC current limit in amps

#define BAT_LOW_LVL1     36.0       // gently beeps at this voltage level
#define BAT_LOW_LVL2     33.0       // your battery is almost empty. Charge now!
#define BAT_LOW_DEAD     31.0       // undervoltage lockout

// ################################################################################

//#define DEBUG_SERIAL_USART2
#define DEBUG_SERIAL_USART3
#define DEBUG_BAUD       115200     // UART baud rate
//#define DEBUG_SERIAL_SERVOTERM
#define DEBUG_SERIAL_ASCII
//#define DEBUG_I2C_LCD

#define CONTROL_USART2_RXTX
#define CONTROL_BAUD       57600

#define TIMEOUT          5           //number of wrong / missing commands before emergency off

// ###### DRIVING BEHAVIOR ######
#define FILTER              0.1 // lower value == softer filter. do not use values <0.01, you will get float precision issues.

#define IDLE        0
#define FOLLOW      1
#define STOP        2
#define FORWARD     3
#define BACKWARD    4
#define LEFT        5
#define RIGHT       6
#define GLIDE       7

#define DISTANCE         40 // set distance for following. No unit, 40 ^~ 2m
#define FORWARD_SPEED   100 // remote control forward speed
#define BACKWARD_SPEED -100 // remote control backward speed

#define LEFT_SPEED      120 // remote control turn left speed
#define RIGHT_SPEED    -120 // remote control turn right speed

#define FOLLOW_P         10 // proportional P for follow regulator
