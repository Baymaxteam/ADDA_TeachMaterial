#ifndef _KEY_H
#define _KEY_H

#include "main.h"
#include "stdint.h"
#include "stm32f0xx_hal.h"
//下面的方式是通过直接操作HAL库函数方式读取IO
#define KEY_CLOCK_20K_Pin        HAL_GPIO_ReadPin(CLOCK_20K_GPIO_Port, CLOCK_20K_Pin)
#define KEY_CLOCK_10K_Pin        HAL_GPIO_ReadPin(CLOCK_10K_GPIO_Port, CLOCK_10K_Pin)
#define KEY_CLOCK_5K_Pin         HAL_GPIO_ReadPin(CLOCK_5K_GPIO_Port,  CLOCK_5K_Pin)
#define KEY_CLOCK_1K_Pin         HAL_GPIO_ReadPin(CLOCK_1K_GPIO_Port,  CLOCK_1K_Pin)
#define KEY_CLOCK_K5_Pin         HAL_GPIO_ReadPin(CLOCK_K5_GPIO_Port,  CLOCK_K5_Pin)

#define KEY_BIT8_Pin             HAL_GPIO_ReadPin(BIT8_GPIO_Port,BIT8_Pin)
#define KEY_BIT10_Pin            HAL_GPIO_ReadPin(BIT10_GPIO_Port,BIT10_Pin)
#define KEY_BIT12_Pin            HAL_GPIO_ReadPin(BIT12_GPIO_Port,BIT12_Pin)

#define KEY_FILTER_10K_Pin       HAL_GPIO_ReadPin(FILTER_10K_GPIO_Port,FILTER_10K_Pin)
#define KEY_FILTER_5K_Pin        HAL_GPIO_ReadPin(FILTER_5K_GPIO_Port,FILTER_5K_Pin)
#define KEY_FILTER_1K_Pin        HAL_GPIO_ReadPin(FILTER_1K_GPIO_Port,FILTER_1K_Pin)

#define KEY_CLOCK_PRES      1
#define KEY_BIT_PRES        2
#define KEY_FILTER_PRES     3

enum ADC_clock_setting
{
    CLOCK_20K = 0,
    CLOCK_10K = 1,
    CLOCK_5K  = 2,
    CLOCK_1K  = 3,
    CLOCK_K5  = 4
};

enum ADC_bit_setting
{
    BIT8  = 0,
    BIT10 = 1,
    BIT12 = 2
};

enum ADC_filter_setting
{
    FILTER_10K  = 0,
    FILTER_5K   = 1,
    FILTER_1K   = 2,
		FILTER_None = 3
};

typedef struct ADDA_Setting_s
{
    uint8_t ADC_clock;
		uint8_t ADC_pre_clock;
    uint8_t ADC_bit;
    uint8_t ADC_filter;
		uint8_t ADC_clock_change;

} ADDA_Setting_t;

static ADDA_Setting_t ADC_Setting =
{
    .ADC_clock  				= CLOCK_20K,
	  .ADC_pre_clock  		= CLOCK_20K,
    .ADC_bit    				= BIT12,
    .ADC_filter 				= FILTER_None,
	  .ADC_clock_change   = 0
};

uint8_t KEY_Scan(ADDA_Setting_t* ADCLab, uint8_t mode);
uint16_t ADC_Bitshift(ADDA_Setting_t* ADCLab, uint16_t adc_value);
uint16_t ADC_Filter_Output(ADDA_Setting_t* ADCLab, uint16_t adc_value);
float lowpass_filer(float input_X, float output_Y, float cutoff_frequency, float dt);


#endif
