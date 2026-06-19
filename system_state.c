/**
 * @file    system_state.c
 * @brief   Implementation of the mutex-protected shared system state.
 */

#include "system_state.h"
#include <string.h>

SystemState_t    g_systemState;
SemaphoreHandle_t g_systemStateMutex;

void SystemState_Init(void)
{
    memset(&g_systemState, 0, sizeof(g_systemState));
    g_systemState.gaitState = STATE_IDLE;
    g_systemStateMutex = xSemaphoreCreateMutex();
}

void SystemState_GetSnapshot(SystemState_t *out)
{
    if (out == NULL)
    {
        return;
    }

    /* Wait forever for the mutex: this is only ever called from task
     * context (never ISR), and the mutex is only ever held briefly by
     * any holder, so a real deadlock here would indicate a logic bug
     * elsewhere worth catching loudly rather than silently timing out. */
    if (xSemaphoreTake(g_systemStateMutex, portMAX_DELAY) == pdTRUE)
    {
        memcpy(out, &g_systemState, sizeof(SystemState_t));
        xSemaphoreGive(g_systemStateMutex);
    }
}

void SystemState_Mutate(SystemStateMutator_t mutator, void *ctx)
{
    if (mutator == NULL)
    {
        return;
    }

    if (xSemaphoreTake(g_systemStateMutex, portMAX_DELAY) == pdTRUE)
    {
        mutator(&g_systemState, ctx);
        xSemaphoreGive(g_systemStateMutex);
    }
}
