/**
 * @file    battery.c
 * @brief   ADC-based battery voltage sampling.
 */

#include "battery.h"

/* Resistor-divider ratio: Vbattery = Vadc_pin * BATTERY_DIVIDER_RATIO.
 * MUST be set to match your actual hardware divider -- this default of
 * 4.0 is only a placeholder (e.g. for a divider built from a 30k/10k
 * pair feeding a ~3.3V-max ADC pin from a ~13.2V max battery pack). */
#define BATTERY_DIVIDER_RATIO   4.0f
#define ADC_VREF_MILLIVOLTS     3300U
#define ADC_RESOLUTION_COUNTS   4096U   /* 12-bit ADC */
#define ADC_CONVERSION_TIMEOUT_MS 2U

void Battery_Init(void)
{
    /* MX_ADC1_Init() (CubeMX-generated) already performs HAL_ADCEx_Calibration_Start()
     * and configures the channel; nothing additional required here. */
}

uint16_t Battery_ReadMilliVolts(void)
{
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0;
    }

    if (HAL_ADC_PollForConversion(&hadc1, ADC_CONVERSION_TIMEOUT_MS) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return 0;
    }

    uint32_t counts = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    uint32_t adcPinMilliVolts = (counts * ADC_VREF_MILLIVOLTS) / ADC_RESOLUTION_COUNTS;
    uint32_t batteryMilliVolts = (uint32_t)((float)adcPinMilliVolts * BATTERY_DIVIDER_RATIO);

    return (uint16_t)batteryMilliVolts;
}
