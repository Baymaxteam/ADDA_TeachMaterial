#include "key.h"
#include "main.h"

//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//1，WKUP按下 WK_UP
//注意此函数有响应优先级,KEY0>KEY1>KEY2>WK_UP!!

uint8_t KEY_Scan_Clock(ADDA_Setting_t* ADCLab, uint8_t mode)
{
    static uint8_t key_up = 1;   //按键松开标志
    static uint8_t scan_type = 0;
    if (mode == 1) //支持连按
    {
        key_up = 1;
    }

    if (key_up && (KEY_CLOCK_20K_Pin == 1 || KEY_CLOCK_10K_Pin == 1 || KEY_CLOCK_5K_Pin == 1 || KEY_CLOCK_1K_Pin == 1 || KEY_CLOCK_K5_Pin == 1))
    {
        HAL_Delay(10);

        if (KEY_CLOCK_20K_Pin == 1)         ADCLab->ADC_clock = CLOCK_20K;
        else if (KEY_CLOCK_10K_Pin == 1)    ADCLab->ADC_clock = CLOCK_10K;
        else if (KEY_CLOCK_5K_Pin == 1)     ADCLab->ADC_clock = CLOCK_5K;
        else if (KEY_CLOCK_1K_Pin == 1)     ADCLab->ADC_clock = CLOCK_1K;
        else if (KEY_CLOCK_K5_Pin == 1)     ADCLab->ADC_clock = CLOCK_K5;

        // clock change , return 1
        if (ADCLab->ADC_clock != ADCLab->ADC_pre_clock)
        {
            ADCLab->ADC_clock_change = 1;
            ADCLab->ADC_pre_clock = ADCLab->ADC_clock;
            scan_type = KEY_CLOCK_PRES;
        }

    }
    return scan_type;   //无按键按下
}

uint8_t KEY_Scan_Bit(ADDA_Setting_t* ADCLab, uint8_t mode)
{
    static uint8_t key_up = 1;   //按键松开标志
    static uint8_t scan_type = 0;
    if (mode == 1) //支持连按
    {
        key_up = 1;
    }

    if (key_up && (KEY_BIT8_Pin == 1 || KEY_BIT10_Pin == 1 || KEY_BIT12_Pin == 1 ))
    {

        HAL_Delay(10);
        if (KEY_BIT8_Pin == 1)              ADCLab->ADC_bit = BIT8;
        else if (KEY_BIT10_Pin == 1)        ADCLab->ADC_bit = BIT10;
        else if (KEY_BIT12_Pin == 1)        ADCLab->ADC_bit = BIT12;

        scan_type = KEY_BIT_PRES;
    }
    return scan_type;   //无按键按下
}

uint8_t KEY_Scan_Filter(ADDA_Setting_t* ADCLab, uint8_t mode)
{
    static uint8_t key_up = 1;   //按键松开标志
    static uint8_t scan_type = 0;
    if (mode == 1) //支持连按
    {
        key_up = 1;
    }
    // anyway, filter is close status at first
    ADCLab->ADC_filter = FILTER_None;

    if (key_up && (KEY_FILTER_10K_Pin == 1 || KEY_FILTER_5K_Pin == 1 || KEY_FILTER_1K_Pin == 1))
    {
        HAL_Delay(10);

        if (KEY_FILTER_10K_Pin == 1)        ADCLab->ADC_filter = FILTER_10K;
        else if (KEY_FILTER_5K_Pin == 1)    ADCLab->ADC_filter = FILTER_5K;
        else if (KEY_FILTER_1K_Pin == 1)    ADCLab->ADC_filter = FILTER_1K;
        else                                ADCLab->ADC_filter = FILTER_None;

        scan_type = KEY_FILTER_PRES;
    }


    return scan_type;   //无按键按下
}

uint16_t ADC_Bitshift(ADDA_Setting_t* ADCLab, uint16_t adc_value)
{
    uint16_t tmp = 0;

    switch (ADCLab->ADC_bit)
    {
    case (BIT8):
        tmp = adc_value >> 4;
        break;
    case (BIT10):
        tmp = adc_value >> 2;
        break;
    case (BIT12):
        tmp = adc_value;
        break;
    default:
        break;
    }
    return tmp;
}

#define CUT_FRQUENCY_10K   10000.0f
#define CUT_FRQUENCY_5K    5000.0f
#define CUT_FRQUENCY_1K    1000.0f

#define ADC_CLOCK_20K      20000
#define ADC_CLOCK_10K      10000
#define ADC_CLOCK_5K       5000
#define ADC_CLOCK_1K       1000
#define ADC_CLOCK_K5       500
float adc_value_Y = 0;

#include "usbd_cdc_if.h"
#include "usb_device.h"
// char kMsg[200];

uint16_t ADC_Filter_Output(ADDA_Setting_t* ADCLab, uint16_t adc_value)
{
    uint16_t ADC_Output = 0;
    float    timestamp = 0;


    // check ADC sample clock
    switch (ADCLab->ADC_clock)
    {
    case (CLOCK_20K):
        timestamp = 1.0f / ADC_CLOCK_20K;
        break;
    case (CLOCK_10K):
        timestamp = 1.0f / ADC_CLOCK_10K;
        break;
    case (CLOCK_5K):
        timestamp = 1.0f / ADC_CLOCK_5K;
        break;
    case (CLOCK_1K):
        timestamp = 1.0f / ADC_CLOCK_1K;
        break;
    case (CLOCK_K5):
        timestamp = 1.0f / ADC_CLOCK_K5;
        break;
    default:
        break;

    }

    // check ADC filter cut off frequency
    switch (ADCLab->ADC_filter)
    {
    case (FILTER_10K):
        ADC_Output = lowpass_filer((float)adc_value, (float)adc_value_Y, CUT_FRQUENCY_10K, timestamp);
        break;
    case (FILTER_5K):
        ADC_Output = lowpass_filer((float)adc_value, (float)adc_value_Y, CUT_FRQUENCY_5K, timestamp);
        break;
    case (FILTER_1K):
        ADC_Output = lowpass_filer((float)adc_value, (float)adc_value_Y, CUT_FRQUENCY_1K, timestamp);
        break;
    case (FILTER_None):
        ADC_Output = adc_value;
        break;
    default:
        break;

    }
    adc_value_Y = ADC_Output;

//              sprintf(kMsg,"D:%f \n", timestamp);
//          if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {CDC_Transmit_FS((uint8_t *)kMsg, strlen(kMsg));}

    return (uint16_t)ADC_Output;
}


#define CURT_OFF_FREQUENCY_TO_RC(freq)    (1.0f / (2.0f * 3.14159f * freq))
float lowpass_filer(float input_X, float output_Y, float cutoff_frequency, float dt)
{
    float RC = CURT_OFF_FREQUENCY_TO_RC(cutoff_frequency);
    float alpha = dt / ( RC + dt );

//  sprintf(kMsg,"F: %f %f %f\n", RC, alpha, dt);
//          if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {CDC_Transmit_FS((uint8_t *)kMsg, strlen(kMsg));}

    return (alpha * input_X + (1 - alpha) * output_Y);
}



// DAC
uint16_t DAC_Bitshift(ADDA_Setting_t* ADCLab, uint16_t adc_value)
{
    uint16_t tmp = 0;

    switch (ADCLab->ADC_bit)
    {
    case (BIT8):
        tmp = adc_value << 4;
        break;
    case (BIT10):
        tmp = adc_value << 2;
        break;
    case (BIT12):
        tmp = adc_value;
        break;
    default:
        break;
    }
    return tmp;
}
