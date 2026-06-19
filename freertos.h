/**
 * @file    freertos.h
 * @brief   Task declarations, priorities, and stack sizes for all six
 *          FreeRTOS tasks in the prosthetic knee controller, plus the
 *          single entry point (App_FreeRTOS_Init) called from main().
 *
 * Priority scheme (FreeRTOS: higher number = higher priority):
 *   Safety            -> configMAX_PRIORITIES - 1   (Highest, per spec)
 *   Motor Control      -> configMAX_PRIORITIES - 2   (High)
 *   Load Cell           -> configMAX_PRIORITIES - 3   (Above Normal)
 *   Communication       -> configMAX_PRIORITIES - 3   (Above Normal)
 *   Bluetooth           -> configMAX_PRIORITIES - 4   (Normal)
 *   Watchdog            -> configMAX_PRIORITIES - 5   (Low)
 *
 * Load Cell and Communication share a priority level: they are both
 * "Above Normal" per spec, and since they block on independent
 * resources (a GPIO bit-bang wait vs. a queue receive) there's no risk
 * of one starving the other at the same priority -- the scheduler will
 * round-robin if both happen to be ready simultaneously.
 */

#ifndef FREERTOS_H
#define FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"

#define TASK_PRIO_SAFETY        (configMAX_PRIORITIES - 1)
#define TASK_PRIO_MOTOR         (configMAX_PRIORITIES - 2)
#define TASK_PRIO_LOADCELL      (configMAX_PRIORITIES - 3)
#define TASK_PRIO_COMM          (configMAX_PRIORITIES - 3)
#define TASK_PRIO_BLUETOOTH     (configMAX_PRIORITIES - 4)
#define TASK_PRIO_WATCHDOG      (configMAX_PRIORITIES - 5)

/* Stack sizes in words (FreeRTOS convention), not bytes. Sized generously
 * for headroom during development; revisit with
 * uxTaskGetStackHighWaterMark() once the application is stable and trim
 * to actual usage + ~20% margin to save RAM. */
#define TASK_STACK_SAFETY       256
#define TASK_STACK_MOTOR        256
#define TASK_STACK_LOADCELL     320
#define TASK_STACK_COMM         320
#define TASK_STACK_BLUETOOTH    320
#define TASK_STACK_WATCHDOG     128

/**
 * @brief Creates all six tasks and the shared synchronization objects
 *        they depend on (via SystemState_Init / UART_DMA_Init, which
 *        this function also calls). Does NOT start the scheduler --
 *        call vTaskStartScheduler() yourself from main() afterward, in
 *        case you need to create additional CubeMX-generated tasks
 *        first.
 */
void App_FreeRTOS_Init(void);

/* Task entry points -- exposed in the header mainly so unit tests or
 * alternate init paths could reference them directly if needed; normal
 * usage only needs App_FreeRTOS_Init(). */
void LoadCellTask(void *argument);
void MotorControlTask(void *argument);
void CommunicationTask(void *argument);
void BluetoothTask(void *argument);
void SafetyTask(void *argument);
void WatchdogTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_H */
