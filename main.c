/**
 * @file    main.c
 * @brief   Application entry point.
 *
 * ASSUMPTION: This file shows the application-specific additions to a
 * standard CubeMX-generated main.c. The HAL_Init(), SystemClock_Config(),
 * and MX_xxx_Init() calls below are exactly what CubeMX would generate
 * for a project with GPIO, USART1, USART2, TIM3, ADC1, GPDMA1, and WWDG
 * enabled -- if your actual .ioc generates different function names
 * (e.g. MX_ICACHE_Init for H5's instruction cache, which CubeMX adds by
 * default), keep those calls; they are orthogonal to this application
 * code and just omitted here for brevity/focus.
 *
 * The only application-specific addition is the App_FreeRTOS_Init() call
 * and starting the scheduler, both placed after all CubeMX peripheral
 * init and before the (normally infinite, now unreachable) while(1) loop
 * that CubeMX generates -- vTaskStartScheduler() never returns under
 * normal operation.
 */

#include "stm32h5xx_hal.h"
#include "freertos.h"
#include "FreeRTOS.h"
#include "task.h"

/* CubeMX-generated peripheral handles -- defined here, declared extern
 * in project_config.h for use by the driver modules. */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef  htim3;
ADC_HandleTypeDef  hadc1;
WWDG_HandleTypeDef hwwdg1;
DMA_HandleTypeDef  hdma_usart1_rx;
DMA_HandleTypeDef  hdma_usart1_tx;
DMA_HandleTypeDef  hdma_usart2_rx;
DMA_HandleTypeDef  hdma_usart2_tx;

/* --- CubeMX-style forward declarations for peripheral init functions ---
 * Bodies are NOT included here: they are exactly what STM32CubeMX
 * generates from your .ioc clock/pin configuration and are specific to
 * your exact board wiring, clock tree, and chosen PWM frequency/ADC
 * channel -- copy them from your generated project rather than from
 * this listing, since hand-written register values here could silently
 * mismatch your actual crystal/HSE frequency or pin assignment. */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_WWDG_Init(void);

int main(void)
{
    /* --- Standard CubeMX-generated bring-up sequence --- */
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_GPDMA1_Init();     /* must precede UART init so DMA handles exist to link */
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_TIM3_Init();
    MX_ADC1_Init();
    MX_WWDG_Init();

    /* --- Application init: creates shared state, UART/DMA queues+sems,
     *     and all six FreeRTOS tasks. Must run after every peripheral
     *     the tasks depend on (UART handles, TIM3, WWDG) is already
     *     initialized above, and before the scheduler starts. --- */
    App_FreeRTOS_Init();

    /* Hands control to FreeRTOS. Does not return under normal operation;
     * if it ever does return, that indicates heap exhaustion in
     * vTaskStartScheduler() itself (e.g. configTOTAL_HEAP_SIZE too small
     * for the idle/timer tasks), which is unrecoverable here. */
    vTaskStartScheduler();

    while (1)
    {
        /* Unreachable under normal operation -- see comment above. */
    }
}

/**
 * @brief HAL's SysTick callback. FreeRTOS, when configured with
 *        configUSE_TICK_HOOK / using its own SysTick handler (the usual
 *        CubeMX FreeRTOS integration), takes over the SysTick interrupt
 *        for its own tick once the scheduler starts. HAL_GetTick() then
 *        rides on FreeRTOS's tick count via the standard CubeMX
 *        freertos.c/port glue -- no extra code needed here beyond what
 *        CubeMX already generates for "Timebase Source = TIM" or the
 *        FreeRTOS-takes-SysTick option, depending on your .ioc setting.
 */

/**
 * @brief Stack overflow hook -- required when configCHECK_FOR_STACK_OVERFLOW
 *        is enabled (recommended for this project, given Safety's
 *        critical role). On a real prosthetic device, the safest
 *        recoverable action for a stack overflow (a memory-corruption
 *        condition where "safe" can no longer be guaranteed) is an
 *        immediate controlled reset rather than trying to continue --
 *        the watchdog WWDG is not refreshed here, so if NVIC_SystemReset
 *        somehow failed to take effect, WWDG would catch it shortly
 *        after as a backstop.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    __disable_irq();
    NVIC_SystemReset();
}

/**
 * @brief Malloc-failure hook -- required when configUSE_MALLOC_FAILED_HOOK
 *        is enabled. Same reasoning as stack overflow: an embedded
 *        system that has exhausted its heap has lost a safety-relevant
 *        guarantee (a task or queue it expected to create did not get
 *        created), so we reset rather than continue in an unknown state.
 */
void vApplicationMallocFailedHook(void)
{
    __disable_irq();
    NVIC_SystemReset();
}
