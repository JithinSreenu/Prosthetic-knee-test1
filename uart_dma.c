/**
 * @file    uart_dma.c
 * @brief   Implementation of the DMA/IDLE-line UART transport for both
 *          USART1 (Bluetooth) and USART2 (Main Controller).
 *
 * How RX works end-to-end:
 *   1. UART_DMA_Init() arms HAL_UARTEx_ReceiveToIdle_DMA() on each port.
 *      This tells the DMA controller to continuously fill a buffer with
 *      incoming bytes; the UART peripheral's IDLE-line detector watches
 *      the RX line and raises an interrupt as soon as the line goes quiet
 *      (i.e. the sender paused, which in a packet protocol normally means
 *      "end of packet/burst").
 *   2. On that interrupt, HAL calls HAL_UARTEx_RxEventCallback() with the
 *      number of bytes actually received (Size). We override that weak
 *      callback (see UART_DMA_RxEventCallback below) and copy the bytes
 *      into a UartRxChunk_t, which we push onto a FreeRTOS queue.
 *   3. We then immediately re-arm ReceiveToIdle_DMA so the line is never
 *      "deaf" waiting for a task to catch up.
 *   4. A task (Comm or Bluetooth) calls UART_DMA_Receive(), which is just
 *      xQueueReceive() -- the task blocks (sleeps, costs no CPU) until a
 *      chunk shows up or the timeout elapses.
 *
 * How TX works:
 *   1. UART_DMA_Transmit() takes the TX semaphore for that port (so only
 *      one transmission is in flight at a time), then calls
 *      HAL_UART_Transmit_DMA() and returns immediately -- the calling
 *      task is not blocked while the bytes physically go out the wire.
 *   2. When DMA finishes shifting the last byte, HAL fires
 *      HAL_UART_TxCpltCallback(), which we override to give back the
 *      semaphore, allowing the next transmission to start.
 */

#include "uart_dma.h"
#include <string.h>

/* ----------------------------------------------------------------------
 * Per-port runtime state
 * -------------------------------------------------------------------- */
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t              rxRawBuffer[UART_RX_BUFFER_SIZE]; /* DMA target buffer */
    QueueHandle_t         rxQueue;       /* holds UartRxChunk_t items        */
    SemaphoreHandle_t      txDoneSem;     /* released on TX complete           */
    volatile uint32_t      lastRxTick;    /* HAL_GetTick() at last RX event    */
} UartPortState_t;

static UartPortState_t s_ports[UART_PORT_COUNT];

/* Map a HAL UART handle pointer back to our port index/state. With only
 * two ports a linear scan is simpler and fast enough than a hash. */
static UartPortState_t *PortStateFromHandle(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < UART_PORT_COUNT; i++)
    {
        if (s_ports[i].huart == huart)
        {
            return &s_ports[i];
        }
    }
    return NULL;
}

void UART_DMA_Init(void)
{
    s_ports[UART_PORT_BLUETOOTH].huart  = &huart1;
    s_ports[UART_PORT_CONTROLLER].huart = &huart2;

    for (int i = 0; i < UART_PORT_COUNT; i++)
    {
        /* Queue depth of 8: even at a 10ms comm task period and bursty
         * Bluetooth traffic, 8 outstanding chunks gives generous headroom
         * before we'd ever drop data due to a slow consumer task. */
        s_ports[i].rxQueue   = xQueueCreate(8, sizeof(UartRxChunk_t));
        s_ports[i].txDoneSem = xSemaphoreCreateBinary();
        s_ports[i].lastRxTick = HAL_GetTick();

        /* Semaphore starts "available" (as if a previous TX already
         * completed) so the first UART_DMA_Transmit() call doesn't block. */
        xSemaphoreGive(s_ports[i].txDoneSem);

        /* Arm reception. HAL will DMA bytes into rxRawBuffer and notify us
         * via UART_DMA_RxEventCallback on IDLE-line or buffer-full. */
        HAL_UARTEx_ReceiveToIdle_DMA(s_ports[i].huart,
                                      s_ports[i].rxRawBuffer,
                                      UART_RX_BUFFER_SIZE);
        /* Disable the half-transfer-complete interrupt: we only care about
         * the IDLE event / full event, not every half-buffer tick. */
        __HAL_DMA_DISABLE_IT(s_ports[i].huart->hdmarx, DMA_IT_HT);
    }
}

bool UART_DMA_Transmit(UartPortId_t port, const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    if (port >= UART_PORT_COUNT || data == NULL || length == 0)
    {
        return false;
    }

    UartPortState_t *st = &s_ports[port];

    /* Wait for any prior transmission on this port to finish. This call
     * blocks the *task*, not the CPU -- the scheduler runs other tasks. */
    if (xSemaphoreTake(st->txDoneSem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        return false; /* previous TX still in flight past our timeout */
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(st->huart, data, length);
    if (status != HAL_OK)
    {
        /* Could not start DMA TX -- release the semaphore back since no
         * completion callback will ever fire for this attempt. */
        xSemaphoreGive(st->txDoneSem);
        return false;
    }

    return true;
}

bool UART_DMA_Receive(UartPortId_t port, UartRxChunk_t *out_chunk, uint32_t timeout_ms)
{
    if (port >= UART_PORT_COUNT || out_chunk == NULL)
    {
        return false;
    }

    return xQueueReceive(s_ports[port].rxQueue, out_chunk, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

uint32_t UART_DMA_GetLastRxTick(UartPortId_t port)
{
    if (port >= UART_PORT_COUNT)
    {
        return 0;
    }
    return s_ports[port].lastRxTick;
}

/* ----------------------------------------------------------------------
 * HAL callback overrides
 *
 * These have the exact signatures of HAL's weak callbacks. Because the
 * weak originals are defined with __weak in the HAL UART driver, simply
 * providing these strong definitions anywhere in the link causes the
 * linker to use ours instead. CubeMX does NOT generate these for you when
 * "Register Callback" mode is off, so they must live in exactly one .c
 * file project-wide -- here.
 * -------------------------------------------------------------------- */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    UART_DMA_RxEventCallback(huart, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_DMA_TxCompleteCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART_DMA_ErrorCallback(huart);
}

void UART_DMA_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    UartPortState_t *st = PortStateFromHandle(huart);
    if (st == NULL || Size == 0 || Size > UART_RX_BUFFER_SIZE)
    {
        return;
    }

    UartRxChunk_t chunk;
    chunk.length = Size;
    memcpy(chunk.data, st->rxRawBuffer, Size);

    st->lastRxTick = HAL_GetTick();

    /* From ISR context: queue send must use the FromISR variant. We do not
     * block; if the queue is genuinely full it means the consumer task has
     * stalled badly, and dropping a chunk is preferable to wedging the
     * UART interrupt. */
    BaseType_t higherPrioTaskWoken = pdFALSE;
    xQueueSendFromISR(st->rxQueue, &chunk, &higherPrioTaskWoken);

    /* Re-arm reception immediately so no bytes are missed while we were
     * servicing this event. */
    HAL_UARTEx_ReceiveToIdle_DMA(huart, st->rxRawBuffer, UART_RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    portYIELD_FROM_ISR(higherPrioTaskWoken);
}

void UART_DMA_TxCompleteCallback(UART_HandleTypeDef *huart)
{
    UartPortState_t *st = PortStateFromHandle(huart);
    if (st == NULL)
    {
        return;
    }

    BaseType_t higherPrioTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(st->txDoneSem, &higherPrioTaskWoken);
    portYIELD_FROM_ISR(higherPrioTaskWoken);
}

void UART_DMA_ErrorCallback(UART_HandleTypeDef *huart)
{
    UartPortState_t *st = PortStateFromHandle(huart);
    if (st == NULL)
    {
        return;
    }

    /* A UART error (framing/overrun/noise) can leave DMA reception
     * stopped. Re-arm it so the link self-heals without needing a task
     * to notice and intervene. Also free the TX semaphore in case the
     * error occurred mid-transmission, so we don't deadlock future sends. */
    HAL_UARTEx_ReceiveToIdle_DMA(huart, st->rxRawBuffer, UART_RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    BaseType_t higherPrioTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(st->txDoneSem, &higherPrioTaskWoken);
    portYIELD_FROM_ISR(higherPrioTaskWoken);
}
