/**
 * @file    battery.h
 * @brief   Battery voltage sampling via ADC1, used to populate
 *          g_systemState.batteryMilliVolts (read by Safety for the
 *          low-battery check and by Bluetooth for status reporting).
 *
 * ASSUMPTION: ADC1 channel is wired to a resistor-divider tapping the
 * battery rail, scaled to fit the ADC's 0-3.3V (or internal reference)
 * input range. BATTERY_DIVIDER_RATIO below must be set to match your
 * actual divider (e.g. a 1:4 divider for a ~13V max pack would use
 * 4.0f).
 */

#ifndef BATTERY_H
#define BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "project_config.h"

/**
 * @brief One-time init -- currently a no-op placeholder since MX_ADC1_Init()
 *        (CubeMX-generated) already configures and calibrates the ADC;
 *        present for symmetry and as a hook for ADC calibration re-runs
 *        if your H5 ADC setup requires periodic recalibration.
 */
void Battery_Init(void);

/**
 * @brief Performs one blocking single-conversion ADC read and converts
 *        it to battery millivolts using BATTERY_DIVIDER_RATIO. This is
 *        only ever called from the Safety task's 1ms cycle context as a
 *        single polled conversion (not continuous/DMA-driven), since
 *        H5's ADC conversion time at typical sampling settings is on the
 *        order of a few microseconds -- negligible against a 1ms budget.
 *
 * @return Battery voltage in millivolts, or 0 if the ADC conversion
 *         timed out (treated by Safety as "unknown", not as "low" --
 *         see safety.c's battery check, which only flags FAULT_BATTERY_LOW
 *         when batteryMilliVolts > 0).
 */
uint16_t Battery_ReadMilliVolts(void);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
