/**
 * @file    freertos.c
 * @brief   Implementation of all six FreeRTOS tasks and system init.
 *
 * Each task follows the same basic shape:
 *   for(;;) {
 *       ... do work ...
 *       vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD));
 *   }
 * vTaskDelayUntil (rather than vTaskDelay) is used everywhere so each
 * task's period is measured from its *previous wake time*, not from
 * "whenever it finished work" -- this keeps periods accurate even if a
 * given cycle's work took a variable amount of time, which matters most
 * for Motor Control (1ms) and Safety (1ms).
 */

#include "freertos.h"
#include "project_config.h"
#include "system_state.h"
#include "uart_dma.h"
#include "packet.h"
#include "loadcell.h"
#include "motor.h"
#include "bluetooth.h"
#include "safety.h"

static TaskHandle_t s_loadCellTaskHandle;
static TaskHandle_t s_motorTaskHandle;
static TaskHandle_t s_commTaskHandle;
static TaskHandle_t s_bluetoothTaskHandle;
static TaskHandle_t s_safetyTaskHandle;
static TaskHandle_t s_watchdogTaskHandle;

/* ======================================================================
 *  Task 1: Load Cell Task -- 5ms period, Above Normal priority
 * ====================================================================== */

typedef struct
{
    float       force;
    bool        groundContact;
    KneeState_t proposedState;
} LoadCellUpdate_t;

static void ApplyLoadCellUpdate(SystemState_t *state, void *ctxVoid)
{
    LoadCellUpdate_t *ctx = (LoadCellUpdate_t *)ctxVoid;

    state->forceNewtons   = ctx->force;
    state->groundContact  = ctx->groundContact;

    /* Only adopt the proposed gait state if Safety hasn't put us in
     * FAULT -- LoadCell_InferGaitState() already refuses to propose a
     * way *out* of FAULT, but we double-check here too since this is
     * the actual point where state is committed. */
    if (state->gaitState != STATE_FAULT)
    {
        state->gaitState = ctx->proposedState;
    }
}

void LoadCellTask(void *argument)
{
    (void)argument;

    LoadCell_Init();

    float filteredForce = 0.0f;
    float previousForce  = 0.0f;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        int32_t raw = 0;
        /* Timeout shorter than our own period: if the HX711 hasn't
         * produced a sample within 4ms, something is wrong (disconnected
         * cell, dead chip) and we should report failure and move on
         * rather than overrun our 5ms budget waiting longer. */
        bool ok = LoadCell_ReadRaw(&raw, 4);

        if (!ok)
        {
            Safety_ReportLoadCellFailure(true);
        }
        else
        {
            Safety_ReportLoadCellFailure(false);

            float instantForce = LoadCell_RawToNewtons(raw);
            filteredForce = LoadCell_FilterEMA(filteredForce, instantForce, 0.35f);

            /* Rate of change in N/sec: delta over one task period
             * (5ms = 0.005s). */
            float forceDeltaPerSec = (filteredForce - previousForce) / (LOADCELL_TASK_PERIOD_MS / 1000.0f);
            previousForce = filteredForce;

            bool groundContact = (filteredForce > 10.0f); /* small threshold above pure noise floor */

            SystemState_t snap;
            SystemState_GetSnapshot(&snap);

            KneeState_t proposed = LoadCell_InferGaitState(filteredForce, forceDeltaPerSec, snap.gaitState);

            LoadCellUpdate_t update = {
                .force         = filteredForce,
                .groundContact = groundContact,
                .proposedState = proposed
            };
            SystemState_Mutate(ApplyLoadCellUpdate, &update);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(LOADCELL_TASK_PERIOD_MS));
    }
}

/* ======================================================================
 *  Task 2: Motor Control Task -- 1ms period, High priority
 * ====================================================================== */
void MotorControlTask(void *argument)
{
    (void)argument;

    Motor_Init();

    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        if (Safety_IsSafeModeActive())
        {
            /* Safety already issued Motor_EmergencyStop() on Safe Mode
             * entry; we simply refrain from issuing any new control
             * output while it remains active rather than re-stopping
             * every cycle (cheap, but pointless extra GPIO/timer writes
             * at a 1ms rate). */
        }
        else
        {
            SystemState_t snap;
            SystemState_GetSnapshot(&snap);
            Motor_UpdateValveControl(snap.kneeAngleDeciDeg, snap.targetKneeAngleDeciDeg);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));
    }
}

/* ======================================================================
 *  Task 3: Communication Task -- 10ms period, Above Normal priority
 *  Processes USART2 (Main Controller) packets: verifies CRC (handled
 *  inside Packet_Decode), decodes commands, applies them.
 * ====================================================================== */

typedef struct
{
    bool    setTargetAngle;
    int16_t newTargetAngle;
    bool    enterCalibration;
    bool    openValve;
    bool    closeValve;
} CommCommandEffect_t;

static void ApplyCommCommand(SystemState_t *state, void *ctxVoid)
{
    CommCommandEffect_t *ctx = (CommCommandEffect_t *)ctxVoid;

    if (ctx->setTargetAngle)
    {
        state->targetKneeAngleDeciDeg = ctx->newTargetAngle;
    }
    if (ctx->enterCalibration)
    {
        state->calibrationModeActive = true;
    }
    if (ctx->openValve)
    {
        /* Per the spec's command table, 0x01/0x02 are direct valve
         * open/close commands from the main controller (unlike the
         * Bluetooth path, this link IS trusted for direct actuation).
         * We translate that into an aggressive target-angle bias rather
         * than calling Motor_Drive() straight from this task, so the
         * Motor Control task remains the single place that ever writes
         * to the PWM/direction outputs -- avoids two tasks racing to
         * drive the same timer channel. */
        state->targetKneeAngleDeciDeg = 900; /* example: drive toward full flexion-open; tune per mechanism */
    }
    if (ctx->closeValve)
    {
        state->targetKneeAngleDeciDeg = 0; /* drive toward fully closed/extended */
    }
}

void CommunicationTask(void *argument)
{
    (void)argument;

    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        UartRxChunk_t chunk;
        /* Short timeout: we want to spend most of our 10ms period idle/
         * blocked (yielding CPU to lower or equal priority tasks), only
         * actually consuming time when there's real data. */
        if (UART_DMA_Receive(UART_PORT_CONTROLLER, &chunk, 8))
        {
            Packet_t pkt;
            if (Packet_Decode(chunk.data, chunk.length, &pkt))
            {
                CommCommandEffect_t effect = {0};

                switch ((PacketCommand_t)pkt.Command)
                {
                    case CMD_OPEN_VALVE:
                        effect.openValve = true;
                        break;
                    case CMD_CLOSE_VALVE:
                        effect.closeValve = true;
                        break;
                    case CMD_SET_KNEE_ANGLE:
                        effect.setTargetAngle = true;
                        effect.newTargetAngle = (int16_t)pkt.Value;
                        break;
                    case CMD_CALIBRATION_MODE:
                        effect.enterCalibration = true;
                        break;
                    default:
                        /* Unknown command on the trusted controller
                         * link: still don't fault for it -- could be a
                         * newer firmware's status/echo opcode we don't
                         * recognize -- just ignore. */
                        break;
                }

                if (effect.setTargetAngle || effect.enterCalibration || effect.openValve || effect.closeValve)
                {
                    SystemState_Mutate(ApplyCommCommand, &effect);
                }
            }
            /* else: CRC failed or malformed -- Packet_Decode already
             * scanned for a valid header+CRC match; silently drop. */
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(COMM_TASK_PERIOD_MS));
    }
}

/* ======================================================================
 *  Task 4: Bluetooth Task -- 50ms period, Normal priority
 * ====================================================================== */
void BluetoothTask(void *argument)
{
    (void)argument;

    Bluetooth_Init();

    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        /* Process any inbound config/calibration/diagnostic packets
         * first, then publish our periodic status -- so a command that
         * just arrived (e.g. a new target angle) is reflected in the
         * status we send out the same cycle. */
        Bluetooth_ProcessIncoming(10);
        Bluetooth_SendStatus();

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BLE_TASK_PERIOD_MS));
    }
}

/* ======================================================================
 *  Task 5: Safety Task -- 1ms period, Highest priority
 * ====================================================================== */
void SafetyTask(void *argument)
{
    (void)argument;

    Safety_Init();

    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        Safety_RunCheck();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAFETY_TASK_PERIOD_MS));
    }
}

/* ======================================================================
 *  Task 6: Watchdog Task -- 100ms period, Low priority
 *
 *  Refreshes the hardware watchdog (WWDG) every cycle. Because this task
 *  runs at the *lowest* priority, a successful refresh is actually a
 *  meaningful liveness signal: it only happens if the scheduler was able
 *  to get around to running every higher-priority task AND still reach
 *  this one inside the watchdog's timeout window. If a higher-priority
 *  task were stuck in an infinite loop or deadlock, this task would
 *  starve, the watchdog would not be refreshed, and the MCU would reset
 *  -- which is the intended fail-safe behavior.
 * ====================================================================== */

typedef struct
{
    bool nearMiss;
} WatchdogUpdate_t;

static void ApplyWatchdogUpdate(SystemState_t *state, void *ctxVoid)
{
    WatchdogUpdate_t *ctx = (WatchdogUpdate_t *)ctxVoid;
    if (ctx->nearMiss)
    {
        state->faultBitmask |= FAULT_WATCHDOG_TIMEOUT;
    }
    else
    {
        state->faultBitmask &= ~FAULT_WATCHDOG_TIMEOUT;
    }
}

void WatchdogTask(void *argument)
{
    (void)argument;

    TickType_t lastWake = xTaskGetTickCount();
    uint32_t previousRefreshTick = HAL_GetTick();

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        uint32_t elapsedSincePrevious = now - previousRefreshTick;

        /* If we're cutting it close to our own intended period (which
         * would mean we got delayed by higher-priority task load), flag
         * a near-miss into the fault bitmask for Safety/Bluetooth to
         * see, even though we're still making the actual hardware
         * refresh in time this cycle. Threshold: 150% of our nominal
         * period. */
        WatchdogUpdate_t update = {
            .nearMiss = (elapsedSincePrevious > (WATCHDOG_REFRESH_MS + (WATCHDOG_REFRESH_MS / 2)))
        };
        SystemState_Mutate(ApplyWatchdogUpdate, &update);

        HAL_WWDG_Refresh(&hwwdg1);
        previousRefreshTick = now;

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(WATCHDOG_REFRESH_MS));
    }
}

/* ======================================================================
 *  System init: create shared objects, then all six tasks.
 * ====================================================================== */
void App_FreeRTOS_Init(void)
{
    SystemState_Init();
    UART_DMA_Init();

    xTaskCreate(SafetyTask,        "Safety",  TASK_STACK_SAFETY,    NULL, TASK_PRIO_SAFETY,    &s_safetyTaskHandle);
    xTaskCreate(MotorControlTask,  "Motor",   TASK_STACK_MOTOR,     NULL, TASK_PRIO_MOTOR,     &s_motorTaskHandle);
    xTaskCreate(LoadCellTask,      "LoadCell",TASK_STACK_LOADCELL,  NULL, TASK_PRIO_LOADCELL,  &s_loadCellTaskHandle);
    xTaskCreate(CommunicationTask, "Comm",    TASK_STACK_COMM,      NULL, TASK_PRIO_COMM,      &s_commTaskHandle);
    xTaskCreate(BluetoothTask,     "BLE",     TASK_STACK_BLUETOOTH, NULL, TASK_PRIO_BLUETOOTH, &s_bluetoothTaskHandle);
    xTaskCreate(WatchdogTask,      "Watchdog",TASK_STACK_WATCHDOG,  NULL, TASK_PRIO_WATCHDOG,  &s_watchdogTaskHandle);
}
