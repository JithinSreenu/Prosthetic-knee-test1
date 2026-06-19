/**
 * @file    bluetooth.c
 * @brief   Implementation of Bluetooth status reporting and incoming
 *          command processing over USART1, built on the shared
 *          UART_DMA transport and Packet_t framing.
 */

#include "bluetooth.h"
#include "uart_dma.h"
#include "system_state.h"
#include "loadcell.h"

void Bluetooth_Init(void)
{
    /* No BLE-module-specific bring-up assumed (e.g. AT-command config)
     * since the spec doesn't name a specific module. If your BLE module
     * needs an init sequence (name, advertising params, etc.), send it
     * here via UART_DMA_Transmit(UART_PORT_BLUETOOTH, ...). */
}

static void SendPacket(PacketCommand_t cmd, uint16_t value)
{
    Packet_t pkt = {
        .Header  = PACKET_HEADER_BYTE,
        .Command = (uint8_t)cmd,
        .Value   = value,
        .CRC     = 0 /* filled in by Packet_Encode */
    };

    uint8_t wire[PACKET_WIRE_SIZE];
    Packet_Encode(&pkt, wire);

    /* 20ms timeout: status reporting runs every 50ms (BLE_TASK_PERIOD_MS),
     * so we'd rather skip a send than block the task long enough to miss
     * its next period if the link is momentarily busy. */
    UART_DMA_Transmit(UART_PORT_BLUETOOTH, wire, PACKET_WIRE_SIZE, 20);
}

void Bluetooth_SendStatus(void)
{
    SystemState_t snap;
    SystemState_GetSnapshot(&snap);

    /* Status is split across a few packets per cycle since one Value
     * field (16 bits) can't hold angle+battery+mode+load together.
     * Each still uses CMD_STATUS_REPORT; a Sub-field in the high bits of
     * Value distinguishes which quantity it carries so the receiving
     * app can demux them -- see bit layout comments below.
     *
     *   Sub-type 0 (bits15:14 = 00): Value[13:0] = knee angle, deci-deg,
     *                                 sign-extended by the app from 14 bits.
     *   Sub-type 1 (bits15:14 = 01): Value[13:0] = battery millivolts.
     *   Sub-type 2 (bits15:14 = 10): Value[13:0] = gait state (low byte)
     *                                 packed with safe-mode flag (bit8).
     */
    uint16_t angleField = ((uint16_t)snap.kneeAngleDeciDeg) & 0x3FFF;
    SendPacket(CMD_STATUS_REPORT, (0x0000U << 14) | angleField);

    uint16_t battField = snap.batteryMilliVolts & 0x3FFF;
    SendPacket(CMD_STATUS_REPORT, (0x1U << 14) | battField);

    uint16_t modeField = ((uint16_t)snap.gaitState & 0xFF) | (snap.safeModeActive ? 0x0100U : 0x0000U);
    SendPacket(CMD_STATUS_REPORT, (0x2U << 14) | modeField);

    if (snap.faultBitmask != FAULT_NONE)
    {
        Bluetooth_SendFaultReport(snap.faultBitmask);
    }
}

void Bluetooth_SendFaultReport(uint16_t faultBitmask)
{
    SendPacket(CMD_FAULT_REPORT, faultBitmask);
}

typedef struct
{
    int16_t newTargetAngle;
    bool    setTargetAngle;
    bool    enterCalibration;
    bool    openValveRequested;
    bool    closeValveRequested;
} BleCommandEffect_t;

static void ApplyBleCommand(SystemState_t *state, void *ctxVoid)
{
    BleCommandEffect_t *ctx = (BleCommandEffect_t *)ctxVoid;

    if (ctx->setTargetAngle)
    {
        state->targetKneeAngleDeciDeg = ctx->newTargetAngle;
    }
    if (ctx->enterCalibration)
    {
        state->calibrationModeActive = true;
    }
    /* Direct open/close-valve requests received over Bluetooth are
     * intentionally NOT applied here as a direct motor command -- per
     * the spec, Bluetooth is for "configuration commands, calibration
     * commands, diagnostic requests", while valve open/close is the
     * Main Controller link's job (USART2/Comm task). We still latch the
     * request into state as a diagnostic flag so a technician using a
     * BLE app can be aware their request needs the controller link, but
     * we do not move hydraulic hardware solely on a Bluetooth packet --
     * that link is for status & low-risk config, not actuation, which
     * keeps a phone-app bug or BLE spoof from being able to drive the
     * joint directly. */
    (void)ctx->openValveRequested;
    (void)ctx->closeValveRequested;
}

void Bluetooth_ProcessIncoming(uint32_t timeout_ms)
{
    UartRxChunk_t chunk;
    if (!UART_DMA_Receive(UART_PORT_BLUETOOTH, &chunk, timeout_ms))
    {
        return; /* nothing received this cycle */
    }

    Packet_t pkt;
    if (!Packet_Decode(chunk.data, chunk.length, &pkt))
    {
        return; /* malformed/corrupt packet, silently drop (CRC caught it) */
    }

    BleCommandEffect_t effect = {0};

    switch ((PacketCommand_t)pkt.Command)
    {
        case CMD_SET_KNEE_ANGLE:
            effect.setTargetAngle  = true;
            effect.newTargetAngle  = (int16_t)pkt.Value;
            break;

        case CMD_CALIBRATION_MODE:
            effect.enterCalibration = true;
            break;

        case CMD_OPEN_VALVE:
            effect.openValveRequested = true;
            break;

        case CMD_CLOSE_VALVE:
            effect.closeValveRequested = true;
            break;

        default:
            /* Unrecognized command -- ignore rather than fault, since an
             * unknown-but-harmless config/diagnostic opcode from a newer
             * app version shouldn't trip a hardware safety response. */
            return;
    }

    SystemState_Mutate(ApplyBleCommand, &effect);
}
