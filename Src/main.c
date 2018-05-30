/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
//#include "hd44780.h"

void SystemClock_Config(void);

extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
extern volatile adc_buf_t adc_buffer;
//LCD_PCF8574_HandleTypeDef lcd;
extern I2C_HandleTypeDef hi2c2;

int cmd1;
int cmd2;
int cmd3;

volatile extern UART_data_struct commandsequence;

uint8_t button1, button2;

int steer; // global variable for steering. -1000 to 1000
int speed; // global variable for speed. -1000 to 1000

extern volatile int pwml;  // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;  // global variable for pwm right. -1000 to 1000
extern volatile int weakl; // global variable for field weakening left. -1000 to 1000
extern volatile int weakr; // global variable for field weakening right. -1000 to 1000

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable; // global variable for motor enable

extern volatile uint32_t timeout; // global variable for timeout
extern float batteryVoltage; // global variable for battery voltage

extern uint8_t nunchuck_data[6];
#ifdef CONTROL_PPM
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];
#endif

uint32_t buttonReleaseTime = 0;

DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;

#define UART2_RX_FIFO_SIZE 32

#define UART2_RX_DMA (DMA1_Channel6)

unsigned char uartCommandBuffer[UART2_RX_FIFO_SIZE];


int milli_vel_error_sum = 0;

uint8_t ASCIItoINT(char input) {
  uint8_t integer = 0;
  if (input < 58 && input > 47) {
    integer = input - 48;
  }
  if (input < 71 && input > 64) {
    integer = input - 55;
  }
  return integer;
}


uint32_t pwm_timeout = 0;
// SysTick executes once each ms
void PWM_SysTick_Callback() {
  pwm_timeout++;
  buttonReleaseTime++;
  // Stop after 500 ms without PPM signal
  if(pwm_timeout > 350) {
    enable = 0;
    pwm_timeout = 0;
  }
}

int main(void) {
  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
    UART_Init();
  #endif

  #ifdef CONTROL_USART2_RXTX)
    UART_Control_Init();
    //Telemetry_Init();
  #endif

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  for (int i = 8; i >= 0; i--) {
    buzzerFreq = i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  int lastSpeedL = 0, lastSpeedR = 0;
  int speedL = 0, speedR = 0;
  float direction = 1;



  enable = 1;
  //consoleLog("otter\n\r");

  UART2_RX_DMA->CNDTR = UART2_RX_FIFO_SIZE;
  uartCommandBuffer[0] = 0;

  int16_t rightreceive;
  int16_t leftreceive;

  HAL_UART_Receive_DMA(&huart2, (unsigned char*)&uartCommandBuffer, UART2_RX_FIFO_SIZE);


  #define IDLE        0
  #define FOLLOW      1
  #define STOP        2
  #define FORWARD     3
  #define BACKWARD    4
  #define LEFT        5
  #define RIGHT       6
  #define GLIDE       7

  uint8_t driveMode = 1;
  int16_t steerValue = 0;
  int16_t steerOverwrite = 0;
  while(1) {
    HAL_Delay(1);

    //buttonReleaseTime++;

    #define DISTANCE 40


    #ifdef CONTROL_USART2_RXTX

      //if (UART2_RX_DMA->CNDTR <= (32-15)) {
        //int32_t comm_index = 0;
        //while (comm_index != UART2_RX_DMA->CNDTR)
        //{
          //  comm_index = UART2_RX_DMA->CNDTR;
            //HAL_Delay(2);
        //}

        //HAL_Delay(10);
        //if (UART2_RX_DMA->CNDTR == 25) {

        uint8_t driveMode;
        uint8_t lastDriveMode;

        if (UART2_RX_DMA->CNDTR < (32-7)) {
          for (int i = 0; i < 8; i++) {
            if (uartCommandBuffer[i] == 97 && uartCommandBuffer[i+6] == 13) {
              rightreceive = ASCIItoINT(uartCommandBuffer[i+3]) + (ASCIItoINT(uartCommandBuffer[i+2])*16);
              leftreceive = ASCIItoINT(uartCommandBuffer[i+5]) + (ASCIItoINT(uartCommandBuffer[i+4])*16);

              if (uartCommandBuffer[i+1] == 63) {
                driveMode = FOLLOW;
              } else if (uartCommandBuffer[i+1] == 52  && driveMode != FORWARD && cmd1 > -20 && cmd1 < 20 && buttonReleaseTime > 200) {
                driveMode = FORWARD;
                cmd1 = 100;
                cmd2 = 100;
              }
              else if (uartCommandBuffer[i+1] == 56  && driveMode != BACKWARD && cmd1 > -20 && cmd1 < 20 && buttonReleaseTime > 200) {
                driveMode = BACKWARD;
                cmd1 = -100;
                cmd2 = -100;
              }
              else if (uartCommandBuffer[i+1] == 50  && driveMode != LEFT) {
                driveMode = LEFT;
                steerValue = 120;
              } else if (uartCommandBuffer[i+1] == 49  && driveMode != RIGHT) {
                driveMode = RIGHT;
                steerValue = -120;
              } else if (uartCommandBuffer[i+1] == 53  && driveMode != STOP) {
                driveMode = STOP;
                lastDriveMode = 0;
                cmd1 = 0;
                cmd2 = 0;
              } else if (uartCommandBuffer[i+1] == 57  && driveMode != GLIDE) {
                driveMode = GLIDE;
                enable = 0;
              }

              if (driveMode == LEFT && steerValue < 200 && uartCommandBuffer[i+1] == 50) {
                steerValue += 2;
                steerOverwrite = steerValue;
                //cmd1 += steerValue;
                //cmd2 -= steerValue;
              }

              if (driveMode == RIGHT && steerValue > -200 && uartCommandBuffer[i+1] == 49) {
                steerValue -= 2;
                steerOverwrite = steerValue;
                //cmd1 += steerValue;
                //cmd2 -= steerValue;
              }

              if (uartCommandBuffer[i+1] != 49 && uartCommandBuffer[i+1] != 50) {
                steerValue = 0;
                steerOverwrite = 0;
              }

              if (uartCommandBuffer[i+1] == 52 || uartCommandBuffer[i+1] == 56) {
                buttonReleaseTime = 0;
              }


              if (driveMode == FORWARD && cmd1 < 500 && uartCommandBuffer[i+1] == 52) {
                cmd1 += 10;
                cmd2 += 10;
              }

              if (driveMode == FORWARD && cmd1 > 0 && uartCommandBuffer[i+1] == 56) {
                cmd1 -= 10;
                cmd2 -= 10;
              }

              if (driveMode == BACKWARD && cmd1 > -300 && uartCommandBuffer[i+1] == 56) {
                cmd1 -= 5;
                cmd2 -= 5;
              }
              if (driveMode == BACKWARD && cmd1 < 0 && uartCommandBuffer[i+1] == 52) {
                cmd1 += 5;
                cmd2 += 5;
              }

              if (driveMode == FOLLOW) {
                if (rightreceive < DISTANCE) {
                  cmd1 = 0;
                } else {
                  cmd1 = ((rightreceive - DISTANCE) - 25) * 10;
                }

                if (leftreceive < DISTANCE) {
                  cmd2 = 0;
                } else {
                  cmd2 = ((leftreceive - DISTANCE) - 25) * 10;
                }
              }
              //setScopeChannel(1, rightreceive);
              //setScopeChannel(2, leftreceive);
              //setScopeChannel(3, 0);
              //consoleScope();
              HAL_UART_AbortReceive_IT(&huart2);
              UART2_RX_DMA->CNDTR = UART2_RX_FIFO_SIZE;
              HAL_UART_Receive_DMA(&huart2, (unsigned char*)&uartCommandBuffer, UART2_RX_FIFO_SIZE);
              if (driveMode != GLIDE) {
                enable = 1;
              }
              pwm_timeout = 0;
              break;
            }
          }
        }


        if (UART2_RX_DMA->CNDTR < (32 - 15)) {
          HAL_UART_AbortReceive_IT(&huart2);
          UART2_RX_DMA->CNDTR = UART2_RX_FIFO_SIZE;
          HAL_UART_Receive_DMA(&huart2, (unsigned char*)&uartCommandBuffer, UART2_RX_FIFO_SIZE);

        }


        //uartCommandBuffer[0] = 0;

    //  }
    #endif

    steer = steer * (1.0 - FILTER) + cmd1 * FILTER;
    speed = speed * (1.0 - FILTER) + cmd2 * FILTER;




    speedR = steer - steerOverwrite;
    speedL = speed + steerOverwrite;

    // ####### SET OUTPUTS #######
    if (rightreceive < 600 && leftreceive < 600) {
      if ((speedL < lastSpeedL + 50 && speedL > lastSpeedL - 50) && (speedR < lastSpeedR + 50 && speedR > lastSpeedR - 50) && timeout < TIMEOUT) {
        pwmr = -speedR;
        pwml = speedL;
      }
    }

    lastSpeedL = speedL;
    lastSpeedR = speedR;

    // ####### LOG TO CONSOLE #######


    if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      enable = 0;
      while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {}
      buzzerFreq = 0;
      buzzerPattern = 0;
      for (int i = 0; i < 8; i++) {
        buzzerFreq = i;
        HAL_Delay(100);
      }
      HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);
      while(1) {}
    }

    if (batteryVoltage < BAT_LOW_LVL1 && batteryVoltage > BAT_LOW_LVL2) {
      buzzerFreq = 5;
      buzzerPattern = 8;
    } else if  (batteryVoltage < BAT_LOW_LVL2 && batteryVoltage > BAT_LOW_DEAD) {
      buzzerFreq = 5;
      buzzerPattern = 1;
    } else if  (batteryVoltage < BAT_LOW_DEAD) {
      buzzerPattern = 0;
      enable = 0;
      for (int i = 0; i < 8; i++) {
        buzzerFreq = i;
        HAL_Delay(100);
      }
      HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);
      while(1) {}
    } else {
      buzzerFreq = 0;
      buzzerPattern = 0;
    }
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
