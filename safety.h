/**
 * @file    safety.h
 * @brief   Highest-priority supervisory module. Monitors comm timeout,
 *          load cell failure, motor driver failure, and watchdog/battery
 *          conditions; drives entry into and recovery from Safe Mode.
 *
 * Safe Mode actions per spec: close hydraulic valve, disable motor
 * drive, transmit a fault packet, and activate the warning indicator.
 * This module owns that sequence; other modules only report raw
 * conditions (e.g. "no bytes received in X ms") into g_systemState or
 * via the functions below -- they do not decide Safe Mode entry/exit
 * themselves, keeping that single critical decision in one place.
 */

#ifndef SAFETY_H
#define SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "project_config.h"
#include "system_state.h"

/**
 * @brief One-time init: configures the warning indicator GPIO.
 */
void Safety_Init(void);

/**
 * @brief Runs one safety-check cycle: reads comm timestamps, load cell
 *        health, motor driver fault line, and battery voltage; updates
 *        g_systemState.faultBitmask; and enters/exits Safe Mode as
 *        needed. Intended to be called every SAFETY_TASK_PERIOD_MS by
 *        the Safety task -- it is the single highest-priority task in
 *        the system precisely because nothing should be able to delay
 *        a fault from being caught and acted on.
 */
void Safety_RunCheck(void);

/**
 * @brief Explicitly reports a load-cell read failure (called by the Load
 *        Cell task when LoadCell_ReadRaw() times out), so Safety can
 *        latch FAULT_LOADCELL_FAILURE even though the Safety task itself
 *        doesn't talk to the HX711 directly.
 */
void Safety_ReportLoadCellFailure(bool failed);

/**
 * @brief Returns true if the system is currently in Safe Mode.
 *        Other tasks (e.g. Motor Control) should consult this before
 *        applying any normal control output, since Safety's emergency
 *        stop must not be immediately overridden by a task that hasn't
 *        yet noticed the fault.
 */
bool Safety_IsSafeModeActive(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_H */
