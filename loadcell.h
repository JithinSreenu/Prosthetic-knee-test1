/**
 * @file    loadcell.h
 * @brief   HX711 load-cell amplifier driver and gait-phase inference.
 *
 * The HX711 is read with a simple bit-banged 2-wire protocol (it is not
 * SPI or I2C): we pulse SCK and sample DOUT 24 times to shift out a 24-bit
 * signed reading, then pulse SCK 1-3 more times to select the next gain/
 * channel setting before the chip goes back to converting. Because this
 * requires tight timing (each SCK pulse must be 1-50us), it is done with
 * direct GPIO toggling inside the Load Cell task itself rather than via
 * any DMA/interrupt peripheral -- there is no hardware peripheral on
 * STM32H585 that speaks "HX711 protocol" natively.
 */

#ifndef LOADCELL_H
#define LOADCELL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "project_config.h"

/**
 * @brief Initializes GPIOs for the HX711 interface. Call once at startup.
 */
void LoadCell_Init(void);

/**
 * @brief Blocking read of one raw 24-bit sample from the HX711.
 *
 * This function busy-waits for the HX711 DOUT line to go low (indicating
 * a conversion is ready), which can take up to ~100ms at the HX711's 10Hz
 * data rate, or as little as ~10ms at its 80Hz rate. Because of this
 * variable wait, this function must only be called from the Load Cell
 * task (which has nothing else time-critical to do while waiting) and
 * never from a task sharing a deadline with other work.
 *
 * @param out_raw  Receives the sign-extended 24-bit conversion result.
 * @param timeout_ms Maximum time to wait for DOUT to go low.
 * @return true if a sample was read, false on timeout (HX711 not
 *         responding -- treated as a load cell failure by the caller).
 */
bool LoadCell_ReadRaw(int32_t *out_raw, uint32_t timeout_ms);

/**
 * @brief Converts a raw HX711 reading to Newtons using the calibration
 *        scale/offset currently stored (see LoadCell_Calibrate).
 */
float LoadCell_RawToNewtons(int32_t raw);

/**
 * @brief Sets the zero-offset (tare) and scale factor used by
 *        LoadCell_RawToNewtons(). In a real bring-up this would be
 *        derived from CMD_CALIBRATION_MODE sequence: read raw at zero
 *        load, then raw at a known reference weight, then compute scale.
 */
void LoadCell_Calibrate(int32_t zeroOffsetRaw, float newtonsPerRawCount);

/**
 * @brief Simple exponential moving-average filter, applied inside the
 *        Load Cell task to the raw force value before it's used for gait
 *        detection -- raw HX711 samples are noisy enough on a load-bearing
 *        prosthetic that an unfiltered reading would cause chattery state
 *        transitions.
 */
float LoadCell_FilterEMA(float previousFiltered, float newSample, float alpha);

/**
 * @brief Infers a coarse gait phase purely from filtered force and its
 *        rate of change. This is intentionally simple (threshold-based)
 *        -- a production system would likely fuse this with an IMU, but
 *        the spec only provides the load cell as a sensor input.
 *
 * @param forceNewtons      Current filtered force reading.
 * @param forceDeltaPerSec  Rate of change of force (Newtons/sec).
 * @param currentState      The state machine's current state, used as
 *                           context (e.g. to distinguish STANDING from
 *                           the stance phase of WALKING).
 * @return Suggested next KneeState_t. The caller (Load Cell task) still
 *         decides whether/how to apply this to g_systemState.
 */
KneeState_t LoadCell_InferGaitState(float forceNewtons, float forceDeltaPerSec, KneeState_t currentState);

#ifdef __cplusplus
}
#endif

#endif /* LOADCELL_H */
