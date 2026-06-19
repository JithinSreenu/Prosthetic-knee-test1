/**
 * @file    bluetooth.h
 * @brief   Bluetooth status/telemetry and command-receive handling over
 *          USART1. Transmits knee angle, load, battery voltage, mode,
 *          and error codes; receives configuration/calibration/
 *          diagnostic commands using the same Packet_t framing as the
 *          main controller link.
 */

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "project_config.h"
#include "packet.h"

/**
 * @brief One-time init. Currently a thin wrapper -- UART/DMA bring-up is
 *        shared with the controller link via UART_DMA_Init(), called
 *        once from main(). Present for symmetry/future expansion (e.g.
 *        BLE module reset sequence, AT-command setup for some modules).
 */
void Bluetooth_Init(void);

/**
 * @brief Builds and transmits a CMD_STATUS_REPORT packet over USART1
 *        summarizing current knee angle, load, battery, mode, and fault
 *        bits. Because a single 16-bit Value field can't carry all of
 *        that, status is sent as a short *sequence* of packets each
 *        cycle (angle, then battery+mode packed, then fault bits) -- see
 *        implementation for the exact sub-packet layout.
 */
void Bluetooth_SendStatus(void);

/**
 * @brief Sends an explicit fault-report packet immediately (used by the
 *        Safety task on Safe Mode entry) rather than waiting for the
 *        next periodic status cycle.
 */
void Bluetooth_SendFaultReport(uint16_t faultBitmask);

/**
 * @brief Polls for and processes any received Bluetooth packets
 *        (configuration, calibration, diagnostic requests), applying
 *        them to g_systemState or dispatching calibration as needed.
 *        Non-blocking beyond the given timeout.
 */
void Bluetooth_ProcessIncoming(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BLUETOOTH_H */
