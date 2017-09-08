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
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "hd44780.h"
#include "pcf8574.h"
#include "key.h"
#include "delay.h"
#include "myiic.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char kMsg[200];
char tempMsg1[100];
char tempMsg2[100];
char tempMsgStandby1[50];
char tempMsgStandby2[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

uint16_t ADCReadings_Bitshift = 0; //ADC Readings
uint16_t ADCReadings_Filter = 0; //ADC Readings

uint8_t  Key_mode = 0;
uint8_t  ADCReadings_Filter_ShowBit[8] = {0};
uint16_t ADCReadings_tmp = 0;
uint8_t  LCD_Reflash_flag = 0;
LCD_PCF8574_HandleTypeDef lcd;

// define timer clock setting
#define CLOCK_FREQ          48000000                            // 48M 
#define TIM_PRESCALER       48                                  // TIM_CLOCK = 48M/48 = 1M = 1000k
#define TIM_CLOCK           (CLOCK_FREQ/TIM_PRESCALER)          // 48M/48 = 1M = 1000k
#define TIM_20K_PERIOD      (TIM_CLOCK/20000)                   // 1000k/20k  = 50
#define TIM_10K_PERIOD      (TIM_CLOCK/10000)                   // 1000k/10k  = 100
#define TIM_5K_PERIOD       (TIM_CLOCK/5000)                    // 1000k/5k   = 200
#define TIM_1K_PERIOD       (TIM_CLOCK/1000)                    // 1000k/1k   = 1000
#define TIM_K5_PERIOD       (TIM_CLOCK/500)                     // 1000k/0.5k = 2000
uint32_t TIM_prescale_setting = TIM_20K_PERIOD;

#define FW_VERSION           "v1.0"
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* Variable containing ADC conversions results */
/* ADC parameters */
#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t)  1)    /* Size of array containing ADC converted values */
__IO uint16_t aADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];
__IO uint16_t uDAC_TEST[1] = {0};

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // TIM2  --> Callback
  // TIM3  --> ADC
  // TIM15 --> DAC


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_ADC_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  MX_TIM15_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
  lcd.pcf8574.PCF_I2C_ADDRESS = 7;
  lcd.pcf8574.PCF_I2C_TIMEOUT = 1000;
  lcd.pcf8574.i2c.Instance = I2C1;
  lcd.pcf8574.i2c.Init.Timing = 0x0000020B;
  lcd.NUMBER_OF_LINES = NUMBER_OF_LINES_2;
  lcd.type = TYPE0;
  if (LCD_Init(&lcd) != LCD_OK)
  {
    // error occured
    while (1);
  }
  // while (PCF8574_Init()) //Ê£?Êµã‰?çÂà∞ PCF8574
  // {
  //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  //   HAL_Delay(200);
  // }

  // default msg for LCD
//  sprintf(tempMsgStandby1, "ADC/DAC");
  sprintf(tempMsgStandby2, "Teach Module");

  // Start ADC DMA
  if (HAL_ADC_Start_DMA(&hadc, (uint32_t *)aADCxConvertedValues, ADCCONVERTEDVALUES_BUFFER_SIZE) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
  if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
  {
    /* Counter Enable Error */
    Error_Handler();
  }

  // Start DAC DMA
  //if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)aADCxConvertedValues, ADCCONVERTEDVALUES_BUFFER_SIZE, DAC_ALIGN_12B_R) != HAL_OK)
  if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)uDAC_TEST, 1, DAC_ALIGN_12B_R) != HAL_OK)
  {
    /* Start DMA Error */
    Error_Handler();
  }

  if (HAL_TIM_Base_Start(&htim15) != HAL_OK)
  {
    /* Counter Enable Error */
    Error_Handler();
  }

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  uint16_t i = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // USB display msg
    sprintf(kMsg, "A:%d, %d, %d, %d M: %d %d %d %d\n", uDAC_TEST[0], aADCxConvertedValues[0], ADCReadings_Bitshift, ADCReadings_Filter,
            Key_mode, ADC_Setting.ADC_clock, ADC_Setting.ADC_bit, ADC_Setting.ADC_filter);
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {CDC_Transmit_FS((uint8_t *)kMsg, strlen(kMsg));}

    // prase adc value from bit by bit
    ADCReadings_tmp = ADCReadings_Filter;
    for (i = 0; i < 8 ; i++)
    {
      ADCReadings_Filter_ShowBit[i] = (uint8_t) (ADCReadings_tmp >> i) & 1 ;
    }

    // LCD display msg
		sprintf(tempMsgStandby1, "ADC/DAC %d %d %d", ADC_Setting.ADC_clock, ADC_Setting.ADC_bit, ADC_Setting.ADC_filter);

    sprintf(tempMsg1, "DAC: %d", uDAC_TEST[0]);
    sprintf(tempMsg2, "ADC: %d%d%d%d%d%d%d%d", ADCReadings_Filter_ShowBit[7], ADCReadings_Filter_ShowBit[6], ADCReadings_Filter_ShowBit[5],
            ADCReadings_Filter_ShowBit[4], ADCReadings_Filter_ShowBit[3], ADCReadings_Filter_ShowBit[2], ADCReadings_Filter_ShowBit[1],
            ADCReadings_Filter_ShowBit[0]);
		
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(500);


    // scan gpio and change adc timer clock / bit / filter setting
    Key_mode = KEY_Scan_Clock(&ADC_Setting, 1);
    KEY_Scan_Bit(&ADC_Setting, 1);
    KEY_Scan_Filter(&ADC_Setting, 1);

    // if change adc timer clock, stop and restart timer setting
    if (Key_mode == KEY_CLOCK_PRES)
    {

      if (HAL_TIM_Base_Stop(&htim3) != HAL_OK)
      {
        Error_Handler();
      }

      if (HAL_TIM_Base_DeInit(&htim3) != HAL_OK)
      {
        Error_Handler();
      }
      // check ADC sample clock
      switch (ADC_Setting.ADC_clock)
      {
      case (CLOCK_20K):
        TIM_prescale_setting = TIM_20K_PERIOD;
        break;
      case (CLOCK_10K):
        TIM_prescale_setting = TIM_10K_PERIOD;
        break;
      case (CLOCK_5K):
        TIM_prescale_setting = TIM_5K_PERIOD;
        break;
      case (CLOCK_1K):
        TIM_prescale_setting = TIM_1K_PERIOD;
        break;
      case (CLOCK_K5):
        TIM_prescale_setting = TIM_K5_PERIOD;
        break;
      default:
        break;
      }

      MX_TIM3_ReInit((uint16_t)TIM_PRESCALER , (uint32_t)TIM_prescale_setting);

      if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
      {
        Error_Handler();
      }

    }


  } // while loop end

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14
                                     | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure the Systick interrupt time
  */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* I2C1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* ADC1_COMP_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USB_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USB_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USB_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * AdcHandle)
{
  // If enable DMA, just leave this commented.
  /* Get the converted value of regular channel */
  //uhADCxConvertedValue = HAL_ADC_GetValue(AdcHandle);

  ADCReadings_Bitshift = ADC_Bitshift(&ADC_Setting, (uint16_t)aADCxConvertedValues[0]);
  ADCReadings_Filter   = ADC_Filter_Output(&ADC_Setting , ADCReadings_Bitshift);
  uDAC_TEST[0] = DAC_Bitshift(&ADC_Setting, ADCReadings_Filter);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

  // only show LCD msg on clock = 20k and filter is none.
  if (ADC_Setting.ADC_clock == CLOCK_20K && ADC_Setting.ADC_filter == FILTER_None)
  {
    LCD_ClearDisplay(&lcd);
    LCD_SetLocation(&lcd, 0, 0);
    LCD_WriteString(&lcd, tempMsg1);
    LCD_SetLocation(&lcd, 0, 1);
    LCD_WriteString(&lcd, tempMsg2);
    LCD_Reflash_flag = 0;
  }
  else
  {
    //show at first time and refresh after counting 10 tims.
    if (LCD_Reflash_flag >= 10 || LCD_Reflash_flag == 0)
    {
      LCD_ClearDisplay(&lcd);
      LCD_SetLocation(&lcd, 0, 0);
      LCD_WriteString(&lcd, tempMsgStandby1);
      LCD_SetLocation(&lcd, 0, 1);
      LCD_WriteString(&lcd, tempMsgStandby2);
      LCD_Reflash_flag = 0;
    }
    LCD_Reflash_flag ++;
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
  while (1)
  {
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
