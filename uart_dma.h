/**
 * @file    uart_dma.h
 * @brief   Non-blocking, DMA-driven UART transport layer used by both
 *          USART1 (Bluetooth) and USART2 (Main Controller).
 *
 * Design:
 *  - TX uses HAL_UART_Transmit_DMA(): fire-and-forget, completion signalled
 *    via HAL callback -> a binary semaphore the task can wait on if needed.
 *  - RX uses HAL_UARTEx_ReceiveToIdle_DMA() into a circular-style buffer.
 *    The IDLE line detection means we get a callback as soon as a burst of
 *    bytes stops arriving, without needing a fixed-length read or blocking.
 *  - No task ever calls a blocking HAL_UART_Receive/Transmit function.
 */

#ifndef UART_DMA_H
#define UART_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "project_config.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

typedef enum
{
    UART_PORT_BLUETOOTH = 0,   /* USART1 */
    UART_PORT_CONTROLLER = 1,  /* USART2 */
    UART_PORT_COUNT
} UartPortId_t;

/**
 * One received "chunk" pushed into the port's queue every time the IDLE
 * line interrupt fires after a DMA reception. The Comm/Bluetooth tasks
 * pull from here and run packet framing/CRC on top.
 */
typedef struct
{
    uint8_t  data[UART_RX_BUFFER_SIZE];
    uint16_t length;
} UartRxChunk_t;

/**
 * @brief Initialize both UART/DMA channels, their RX queues, and TX
 *        completion semaphores. Must be called once after CubeMX's
 *        MX_USARTx_UART_Init() and MX_GPDMA1_Init(), before the scheduler
 *        starts.
 */
void UART_DMA_Init(void);

/**
 * @brief Queue a non-blocking DMA transmission on the given port.
 *        Safe to call from any task. Will block the *calling task* (not
 *        the CPU) for up to timeout_ms if a previous TX on that port is
 *        still in flight, waiting on the TX-complete semaphore.
 *
 * @return true if the DMA transmission was started, false on timeout/busy.
 */
bool UART_DMA_Transmit(UartPortId_t port, const uint8_t *data, uint16_t length, uint32_t timeout_ms);

/**
 * @brief Blocking (task-level, not CPU-level) read of the next received
 *        chunk for a port. Internally this is a FreeRTOS queue receive,
 *        so the calling task simply sleeps until data arrives or the
 *        timeout expires -- the CPU is free for other tasks the whole time.
 *
 * @return true if a chunk was received before timeout, false otherwise.
 */
bool UART_DMA_Receive(UartPortId_t port, UartRxChunk_t *out_chunk, uint32_t timeout_ms);

/**
 * @brief Returns the millisecond timestamp (HAL tick) of the last byte
 *        successfully received on the given port. Used by the Safety task
 *        to detect comm-link timeouts.
 */
uint32_t UART_DMA_GetLastRxTick(UartPortId_t port);

/* --- Callbacks: wired up from stm32h5xx_it.c / HAL weak callback overrides --- */
void UART_DMA_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void UART_DMA_TxCompleteCallback(UART_HandleTypeDef *huart);
void UART_DMA_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* UART_DMA_H */
