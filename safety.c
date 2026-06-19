/**
 * @file    safety.c
 * @brief   Implementation of the safety supervisor: fault detection,
 *          Safe Mode entry/exit, and the Safe Mode action sequence.
 */

#include "safety.h"
#include "motor.h"
#include "bluetooth.h"
#include "uart_dma.h"
#include "packet.h"
#include "battery.h"

static bool s_loadCellFailureReported = false;

void Safety_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = WARNING_LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WARNING_LED_PORT, &gpio);
    HAL_GPIO_WritePin(WARNING_LED_PORT, WARNING_LED_PIN, GPIO_PIN_RESET);
}

void Safety_ReportLoadCellFailure(bool failed)
{
    s_loadCellFailureReported = failed;
}

bool Safety_IsSafeModeActive(void)
{
    SystemState_t snap;
    SystemState_GetSnapshot(&snap);
    return snap.safeModeActive;
}

typedef struct
{
    uint16_t newFaultBitmask;
    bool     enterSafeMode;
    bool     exitSafeMode;
} SafetyMutation_t;

static void ApplySafetyMutation(SystemState_t *state, void *ctxVoid)
{
    SafetyMutation_t *ctx = (SafetyMutation_t *)ctxVoid;

    state->faultBitmask = ctx->newFaultBitmask;

    if (ctx->enterSafeMode)
    {
        state->safeModeActive = true;
        state->gaitState = STATE_FAULT;
    }
    else if (ctx->exitSafeMode)
    {
        state->safeModeActive = false;
        state->gaitState = STATE_IDLE; /* re-enter cleanly, do not assume prior gait state is still valid */
    }
}

/**
 * @brief Performs the spec's mandated Safe Mode action sequence:
 *        close hydraulic valve, disable motor drive, transmit fault
 *        packet, activate warning indicator.
 */
static void EnterSafeModeActions(uint16_t faultBitmask)
{
    /* 1) Close hydraulic valve: drive Motor 2 (close) briefly, then... */
    Motor_Drive(MOTOR_2_CLOSE_VALVE, MAX_DUTY_PCT_SAFE_CLOSE, true);
    HAL_Delay(SAFE_CLOSE_VALVE_PULSE_MS); /* short bounded pulse, see header comment below */

    /* 2) ...then disable all motor drive entirely so nothing is left
     *    energized once the valve is closed. */
    Motor_EmergencyStop();

    /* 3) Transmit fault packet on both links: the main controller link
     *    (so any host system reacts immediately) and Bluetooth (so a
     *    paired diagnostic app sees it too). */
    Packet_t pkt = { .Header = PACKET_HEADER_BYTE, .Command = CMD_FAULT_REPORT, .Value = faultBitmask, .CRC = 0 };
    uint8_t wire[PACKET_WIRE_SIZE];
    Packet_Encode(&pkt, wire);
    UART_DMA_Transmit(UART_PORT_CONTROLLER, wire, PACKET_WIRE_SIZE, 20);
    Bluetooth_SendFaultReport(faultBitmask);

    /* 4) Activate warning indicator. */
    HAL_GPIO_WritePin(WARNING_LED_PORT, WARNING_LED_PIN, GPIO_PIN_SET);
}

static void ExitSafeModeActions(void)
{
    HAL_GPIO_WritePin(WARNING_LED_PORT, WARNING_LED_PIN, GPIO_PIN_RESET);
    /* Motors remain at 0 duty (as left by EmergencyStop) until the Motor
     * Control task resumes normal angle-tracking on its own next cycle;
     * Safety does not re-energize anything itself on recovery. */
}

typedef struct
{
    uint16_t milliVolts;
} BatteryUpdate_t;

static void ApplyBatteryUpdate(SystemState_t *state, void *ctxVoid)
{
    BatteryUpdate_t *ctx = (BatteryUpdate_t *)ctxVoid;
    /* Only overwrite if we got a real reading -- a 0 from
     * Battery_ReadMilliVolts() means the ADC conversion timed out this
     * cycle, and we'd rather keep the last known-good value than briefly
     * report "0V" (which downstream could misread as a battery fault or
     * a momentarily-bogus status packet over Bluetooth). */
    if (ctx->milliVolts > 0)
    {
        state->batteryMilliVolts = ctx->milliVolts;
    }
}

void Safety_RunCheck(void)
{
    uint16_t faults = FAULT_NONE;

    uint32_t now = HAL_GetTick();

    /* --- Condition 1: controller comm timeout --- */
    uint32_t lastControllerRx = UART_DMA_GetLastRxTick(UART_PORT_CONTROLLER);
    if ((now - lastControllerRx) >= COMM_TIMEOUT_MS)
    {
        faults |= FAULT_COMM_TIMEOUT;
    }

    /* --- Condition 2: load cell failure (reported by Load Cell task) --- */
    if (s_loadCellFailureReported)
    {
        faults |= FAULT_LOADCELL_FAILURE;
    }

    /* --- Condition 3: motor driver failure --- */
    if (Motor_HasDriverFault())
    {
        faults |= FAULT_MOTOR_DRIVER_FAILURE;
    }

    /* --- Condition 4: watchdog timeout ---
     * A *true* watchdog timeout would reset the MCU before this code
     * could ever run, so this flag is set instead by the Watchdog task
     * if it ever detects it nearly missed its refresh deadline (i.e. a
     * near-miss / early-warning condition), giving Safety a chance to
     * react before an actual hardware reset would occur. See
     * freertos.c's WatchdogTask for where FAULT_WATCHDOG_TIMEOUT is
     * raised; we just read its already-published value here, OR'd in
     * below from the previous fault bitmask so we don't clobber it. */
    SystemState_t snap;
    SystemState_GetSnapshot(&snap);
    faults |= (snap.faultBitmask & FAULT_WATCHDOG_TIMEOUT);

    /* --- Battery: sample now and publish into shared state so
     *     Bluetooth's status reports stay current. Safety is the only
     *     task that touches the ADC, avoiding any cross-task contention
     *     over the single ADC1 peripheral. --- */
    uint16_t batteryMv = Battery_ReadMilliVolts();
    BatteryUpdate_t battUpdate = { .milliVolts = batteryMv };
    SystemState_Mutate(ApplyBatteryUpdate, &battUpdate);

    /* Use the freshly-sampled reading (not the pre-update snapshot) for
     * the low-battery decision, but still guard against a 0 (timed-out
     * conversion) being misread as "critically low". */
    uint16_t batteryForCheck = (batteryMv > 0) ? batteryMv : snap.batteryMilliVolts;
    if (batteryForCheck > 0 && batteryForCheck < BATTERY_LOW_VOLTAGE_MV)
    {
        faults |= FAULT_BATTERY_LOW;
    }

    bool shouldBeSafe = (faults != FAULT_NONE);
    bool currentlySafe = snap.safeModeActive;

    SafetyMutation_t mutation = {
        .newFaultBitmask = faults,
        .enterSafeMode   = (shouldBeSafe && !currentlySafe),
        .exitSafeMode    = (!shouldBeSafe && currentlySafe)
    };

    SystemState_Mutate(ApplySafetyMutation, &mutation);

    if (mutation.enterSafeMode)
    {
        EnterSafeModeActions(faults);
    }
    else if (mutation.exitSafeMode)
    {
        ExitSafeModeActions();
    }
}
