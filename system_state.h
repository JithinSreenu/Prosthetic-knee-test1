/**
 * @file    system_state.h
 * @brief   Centralized, mutex-protected shared state for the whole system.
 *
 * Rather than have tasks reach into each other's internals, every task
 * publishes its findings (load reading, knee angle, fault flags, current
 * gait state, battery voltage...) into this single struct, guarded by one
 * mutex. Readers/writers take the mutex for the brief copy in/out and
 * release it immediately -- nobody holds it across blocking calls.
 *
 * This keeps coupling low: e.g. the Bluetooth task does not need to know
 * *how* the Load Cell task derived a gait phase, only that it can read
 * g_systemState.gaitState.
 */

#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"

/* ======================================================================
 *  Gait / Knee State Machine
 * ====================================================================== */
typedef enum
{
    STATE_IDLE = 0,
    STATE_STANDING,
    STATE_WALKING,
    STATE_SITTING,
    STATE_STAIR_ASCENT,
    STATE_STAIR_DESCENT,
    STATE_FAULT
} KneeState_t;

/* ======================================================================
 *  Fault bitmask -- each bit independently settable/clearable by the
 *  Safety task so multiple simultaneous faults can be reported at once
 *  in a single CMD_FAULT_REPORT packet Value field.
 * ====================================================================== */
#define FAULT_NONE                  0x0000U
#define FAULT_COMM_TIMEOUT          0x0001U
#define FAULT_LOADCELL_FAILURE      0x0002U
#define FAULT_MOTOR_DRIVER_FAILURE  0x0004U
#define FAULT_WATCHDOG_TIMEOUT      0x0008U
#define FAULT_BATTERY_LOW           0x0010U

typedef struct
{
    /* --- Load Cell Task outputs --- */
    float    forceNewtons;
    bool     groundContact;
    float    weightDistributionPct;  /* 0-100%, this leg's share of total */

    /* --- Motor / knee geometry --- */
    int16_t  kneeAngleDeciDeg;       /* tenths of a degree, signed */
    int16_t  targetKneeAngleDeciDeg;

    /* --- Battery --- */
    uint16_t batteryMilliVolts;

    /* --- Gait state machine --- */
    KneeState_t gaitState;

    /* --- Safety / fault status --- */
    uint16_t faultBitmask;
    bool     safeModeActive;

    /* --- Calibration --- */
    bool     calibrationModeActive;
} SystemState_t;

extern SystemState_t   g_systemState;
extern SemaphoreHandle_t g_systemStateMutex;

/**
 * @brief Must be called once during system init (before scheduler start)
 *        to create the mutex and zero-initialize the shared struct.
 */
void SystemState_Init(void);

/**
 * @brief Convenience helper: take the mutex, copy out the whole struct,
 *        release the mutex. Use this in tasks that need several fields
 *        at once (e.g. Bluetooth status packets) to get a consistent
 *        snapshot rather than reading fields one at a time across
 *        possibly-interrupted reads.
 */
void SystemState_GetSnapshot(SystemState_t *out);

/**
 * @brief Take the mutex, run a small in-place mutation via the provided
 *        function pointer, release the mutex. Keeps the critical section
 *        as short as possible since the mutation itself is just memory
 *        writes, no I/O.
 */
typedef void (*SystemStateMutator_t)(SystemState_t *state, void *ctx);
void SystemState_Mutate(SystemStateMutator_t mutator, void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_STATE_H */
