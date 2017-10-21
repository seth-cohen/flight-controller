/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "arm_math.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// Timing
#define CLOCK_MHZ 72
volatile uint32_t millis = 0; // Updated in handler 1ms interrupt
uint32_t lastMicros = 0, micros = 0;
float deltaMillis = 0, deltaSeconds = 0;

// Ring Buffer
char usbRXBuffer[RING_BUFFER_SIZE];
uint16_t usbRXBufferHead = 0;
uint16_t usbRXBufferTail = 0;

// USB Transceiver
char outbuffer[512];
char command[64];
uint8_t commandLength = 0;
volatile uint8_t hasUnprocessedCommand = 0;

// Servo Control
uint16_t counter = 0;
int pulseWidth = 1000;

// IMU
#define PITCH_INDEX 0
#define ROLL_INDEX 1
#define YAW_INDEX 2
#define X_AXIS_INDEX 0
#define Y_AXIS_INDEX 1
#define Z_AXIS_INDEX 2
#define AXIS_COUNT 3

float gyroBuffer[AXIS_COUNT], pitch, roll, yaw;
int16_t accBuffer[AXIS_COUNT], magBuffer[AXIS_COUNT];

// ACRO Mode - targets and PID
float targetRates[AXIS_COUNT];
#define MAX_YAW_RATE 100 // degrees per second
#define MAX_PITCH_RATE 100 // degrees per second
#define MAX_ROLL_RATE 100 // degrees per second
int16_t pitchOutput = 0, rollOutput = 0, yawOutput = 0, throttleOutput = 0;
Pid pids[AXIS_COUNT];

// PWM Input
#define PWM_INPUT_PERIOD 0xFFFF
#define THROTTLE_CHANNEL 0
#define PITCH_CHANNEL 1
#define YAW_CHANNEL 2
#define ROLL_CHANNEL 3
#define PWM_CHANNELS 4

typedef enum {
  false,
  true
} bool;

typedef struct {
  uint32_t rising;
  uint32_t falling;
  uint16_t capture;
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t channel;
} pwmInputType_t;
pwmInputType_t pwmInputs[PWM_CHANNELS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void GetGyroAngRates(float* pfData);
static void ComplementaryFilter(
  int16_t accData[AXIS_COUNT],
  float gyroData[AXIS_COUNT],
  float deltaTime,
  float *pitch,
  float *roll,
  float *yaw);

static int16_t mapRCInput(uint16_t input, uint16_t inMin, uint16_t inMax, int16_t outMin, int16_t outMax);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Initialize the pwmInputs
  // @TODO create config file for these
  pwmInputs[0].port = GPIOA;
  pwmInputs[0].pin = GPIO_PIN_15;
  pwmInputs[0].channel = TIM_CHANNEL_1;
  pwmInputs[1].port = GPIOA;
  pwmInputs[1].pin = GPIO_PIN_1;
  pwmInputs[1].channel = TIM_CHANNEL_2;
  pwmInputs[2].port = GPIOA;
  pwmInputs[2].pin = GPIO_PIN_2;
  pwmInputs[2].channel = TIM_CHANNEL_3;
  pwmInputs[3].port = GPIOA;
  pwmInputs[3].pin = GPIO_PIN_3;
  pwmInputs[3].channel = TIM_CHANNEL_4;

  // Initialize the Pids
  InitPid(&pids[PITCH_INDEX], 0.7f, 0.0f, 0.0f);
  InitPid(&pids[ROLL_INDEX], 0.7f, 0.0f, 0.0f);
  InitPid(&pids[YAW_INDEX], 2.5f, 0.0f, 0.0f);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

  // Initialize the IMU
  BSP_GYRO_Init();
  BSP_ACCELERO_Init();
  LSM303DLHC_MagInit(0x0);

  pitch = roll = yaw = 0.0f;

  while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    BSP_LED_Toggle(LED4);

    /* Wait for User button to be pressed */
    while (BSP_PB_GetState(BUTTON_USER) != KEY_PRESSED) {
      // Ensure that the Unit is level.

      // In case the loop is happening sub millisecond (which it is) let's increase
      // precision to microsecond scale.
      // micros = millis*1000+(SystemCoreClock/1000-SysTick->VAL)/SystemCoreClockInMHz;
      // ^^^ essentially Since Systick is a 24 bit counter counting down, we are adding the difference us
      //     to the number of micros that have passed since the milliSecond interrupt
      millis = HAL_GetTick();
      micros = millis * 1000 + 1000 - SysTick->VAL/CLOCK_MHZ;
      deltaMillis = (micros - lastMicros) / 1000.0f;
      deltaSeconds = deltaMillis / 1000.0f;
      lastMicros = micros;

      // Filter IMU Values
      GetGyroAngRates(gyroBuffer);
      BSP_ACCELERO_GetXYZ(accBuffer);
      LSM303DLHC_MagReadXYZ(magBuffer);
      ComplementaryFilter(accBuffer, gyroBuffer, deltaSeconds, &pitch, &roll, &yaw);

      // Get the Target rotational rates from the inputs then calculate the PID loops
      targetRates[PITCH_INDEX] = mapRCInput(pwmInputs[PITCH_CHANNEL].capture, 980, 1980, -MAX_PITCH_RATE, MAX_PITCH_RATE);
      targetRates[ROLL_INDEX] = mapRCInput(pwmInputs[ROLL_CHANNEL].capture, 980, 1980, -MAX_ROLL_RATE, MAX_ROLL_RATE);
      targetRates[YAW_INDEX] = mapRCInput(pwmInputs[YAW_CHANNEL].capture, 980, 1980, -MAX_YAW_RATE, MAX_YAW_RATE);

      // Provide some level of epsilon for Zero throttle
      throttleOutput = pwmInputs[THROTTLE_CHANNEL].capture;
      if (throttleOutput > 1020) {
        pitchOutput = (int16_t) RunPid(&pids[PITCH_INDEX], targetRates[PITCH_INDEX] - gyroBuffer[PITCH_INDEX], deltaMillis / 1000.0f);
        rollOutput = (int16_t) RunPid(&pids[ROLL_INDEX], targetRates[ROLL_INDEX] - gyroBuffer[ROLL_INDEX], deltaMillis / 1000.0f);
        yawOutput = (int16_t) RunPid(&pids[YAW_INDEX], targetRates[YAW_INDEX] - gyroBuffer[YAW_INDEX], deltaMillis / 1000.0f);

        // If pitch is negative then that is forward movement, increase pulse to rear motors
        // If roll is negative then that is left angle, increase pulse to the right motors
        // If yaw is negative then that is left rotation, increase  pulse to front left and back right motors
        // @TODO add in yaw and account for ESC offsets
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, throttleOutput - rollOutput - pitchOutput); // FR
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, throttleOutput + rollOutput - pitchOutput); // FL
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, throttleOutput + rollOutput + pitchOutput); // BL
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, throttleOutput - rollOutput + pitchOutput); // BR
      } else {
        // Zero throttle would be 1000 and max would be 2000 (one tick for every micro second)
        pulseWidth = MAX(pwmInputs[THROTTLE_CHANNEL].capture + 20, 1000);  // RX PWM output is 980 - 1980
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      }

      // Data packet of current state as requested over serial port
      while ((usbRXBufferHead - usbRXBufferTail) > 0U) { // buffer not empty
        command[commandLength++] = usbRXBuffer[usbRXBufferTail & (RING_BUFFER_SIZE - 1)];
        usbRXBufferTail++;
        if (command[commandLength - 1] == '\r') {
          command[commandLength++] = '\n';
          command[commandLength++] = '\0';
          hasUnprocessedCommand = 1;
          break;
        }
      }
      if (hasUnprocessedCommand == 1) {
        if (strcmp(command, "getPacket\r\n") == 0) {
          sprintf(
            outbuffer,
            "oPitch=%7ld, oRoll=%7ld, oYaw=%7ld,\r\n---Throt=%7d, Pitch=%7d, Yaw=%7d, Roll=%7d\r\n",
            (int32_t) (pitch * 1000.0f),
            (int32_t) (roll * 1000.0f),
            (int32_t) (yaw * 1000.0f),
            pwmInputs[0].capture,
            pwmInputs[1].capture,
            pwmInputs[2].capture,
            pwmInputs[3].capture
          );
          CDC_Transmit_FS((uint8_t*) outbuffer, strlen(outbuffer));
        }
        hasUnprocessedCommand = commandLength = 0;
      }
    }

    // Set Servo PWM outputs based on value set in serial port
    while (1) {
      counter++;
      while ((usbRXBufferHead - usbRXBufferTail) > 0U) { // buffer not empty
        command[commandLength++] = usbRXBuffer[usbRXBufferTail & (RING_BUFFER_SIZE - 1)];
        usbRXBufferTail++;
        if (command[commandLength - 1] == '\r') {
          command[commandLength++] = '\n';
          command[commandLength++] = '\0';
          hasUnprocessedCommand = 1;
          break;
        }
      }
      if (hasUnprocessedCommand == 1) {
        CDC_Transmit_FS((uint8_t*) command, strlen(command));
        pulseWidth = strtol(command, (char **) NULL, 10);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulseWidth);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulseWidth);
        hasUnprocessedCommand = commandLength = 0;
      }
    }
  }

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin 
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin 
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin 
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin 
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * @brief Linear forecasting functions. Maps input value from one range to another
 *
 * @param[in] input
 * @param[in] inMin
 * @param[in] inMax
 * @param[in] outMin
 * @param[in] outMax
 *
 * @return int16_t The value that the input maps to
 */
int16_t mapRCInput(uint16_t input, uint16_t inMin, uint16_t inMax, int16_t outMin, int16_t outMax) {
  return (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

/**
 * @brief Complementary Filter used to keep track of orientation from an IMU (accelerometer and gyroscope)
 *        This is much simpler than utilizing Kalman Filter.
 *
 * @link http://www.pieter-jan.com/node/11
 *
 * @param[in] accData   3 element float array, data from accelerometer
 * @param[in] gyroData  3 element int16_t array, data from the gyroscope
 * @param[in] deltaTime time, in seconds, since the last update
 * @param[out] pitch    pointer to float to receive the current angle of pitch
 * @param[out] roll     pointer to float to receive the current angle of roll
 * @param[out] yaw      pointer to float to receive the current angle of yaw
 *
 * @retval void
 */
void ComplementaryFilter(int16_t accData[AXIS_COUNT], float gyroData[AXIS_COUNT], float deltaTime, float *pitch, float *roll, float *yaw) {
  float pitchAcc, rollAcc;

  if (deltaTime > 5) return;

// Integrate the gyro data - integral(angularSpeed) = angle
// but careful because it drifts hence the need for the filter
  *pitch += (gyroData[PITCH_INDEX] * deltaTime);   // Angle around the X-axis
  *roll += (gyroData[ROLL_INDEX] * deltaTime);    // Angle around the Y-axis
  *yaw += (gyroData[YAW_INDEX] * deltaTime);     // Angle around the Z-axis

// Unfortunately, due to the fact that gravity is in the downward Z-axis there
// is no way to use the accelerometer to "fix" the gyro drift of Z-rotation

// Compensate for drift with accelerometer data if !bullshit
// Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
  int16_t absX = abs(accData[X_AXIS_INDEX]);
  int16_t absY = abs(accData[Y_AXIS_INDEX]);

  uint32_t forceMagnitudeApprox = absX + absY + abs(accData[Z_AXIS_INDEX]);
// @TODO - save these noise levels as consts somewhere
  if (absX < 200)
    accData[X_AXIS_INDEX] = 0; // ignore noise
  if (absY < 200)
    accData[Y_AXIS_INDEX] = 0;
  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768) {
    // Turning around the X axis results in a vector on the Y-axis
    pitchAcc = atan2f((float) accData[Y_AXIS_INDEX], (float) accData[Z_AXIS_INDEX]) * 180.0f / PI;
    *pitch = *pitch * 0.98 + pitchAcc * 0.02;

    // Turning around the Z axis results in a vector on the X-axis
    rollAcc = atan2f((float) accData[X_AXIS_INDEX], (float) accData[Z_AXIS_INDEX]) * 180.0f / PI;
    *roll = *roll * 0.98 + rollAcc * 0.02;

    // @TODO - combine magnetometer data with Gyro for filter yaw drift compensation
  }
}

/**
 * @brief  Calculate the angular Data rate Gyroscope. in dps
 * @param  pfData : Data out pointer
 * @retval None
 */
static void GetGyroAngRates(float* pfData) {
  uint8_t tmpbuffer[6] = {0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i = 0;

  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG4_ADDR, 1);
  GYRO_IO_Read(tmpbuffer, L3GD20_OUT_X_L_ADDR, 6);

  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if (!(tmpreg & L3GD20_BLE_MSB)) {
    for (i = 0; i < 3; i++) {
      RawData[i] = (int16_t) (((uint16_t) tmpbuffer[2 * i + 1] << 8) + tmpbuffer[2 * i]);
    }
  } else {
    for (i = 0; i < 3; i++) {
      RawData[i] = (int16_t) (((uint16_t) tmpbuffer[2 * i] << 8) + tmpbuffer[2 * i + 1]);
    }
  }

  /* Switch the sensitivity value set in the CRTL4 */
  switch (tmpreg & L3GD20_FULLSCALE_SELECTION) {
    case L3GD20_FULLSCALE_250:
      sensitivity = L3GD20_SENSITIVITY_250DPS;
      break;

    case L3GD20_FULLSCALE_500:
      sensitivity = L3GD20_SENSITIVITY_500DPS;
      break;

    case L3GD20_FULLSCALE_2000:
      sensitivity = L3GD20_SENSITIVITY_2000DPS;
      break;

    default:
      sensitivity = L3GD20_SENSITIVITY_250DPS;
  }
  sensitivity *= 0.001f;

  /* divide by sensitivity */
  for (i = 0; i < 3; i++) {
    pfData[i] = (float) (RawData[i] * sensitivity);
  }
}

/**
 * Interrupt handler for capture
 *
 * @param htim Timer handle
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
// @TODO - do some fancy mechanism to identify which pwm struct we have
  pwmInputType_t *instance = NULL;

  if (htim->Instance == TIM2) {
    switch (htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        instance = &pwmInputs[0];
        break;
      case HAL_TIM_ACTIVE_CHANNEL_2:
        instance = &pwmInputs[1];
        break;
      case HAL_TIM_ACTIVE_CHANNEL_3:
        instance = &pwmInputs[2];
        break;
      case HAL_TIM_ACTIVE_CHANNEL_4:
        instance = &pwmInputs[3];
        break;
      default:
        break;
    }
  }

  if (HAL_GPIO_ReadPin(instance->port, instance->pin)) {
    instance->rising = HAL_TIM_ReadCapturedValue(htim, instance->channel);
  } else {
    instance->falling = HAL_TIM_ReadCapturedValue(htim, instance->channel);
    if (instance->rising > instance->falling) {
      instance->rising += PWM_INPUT_PERIOD;
    }
    instance->capture = instance->falling - instance->rising;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
