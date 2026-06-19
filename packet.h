/**
 * @file    packet.h
 * @brief   Wire packet format shared by USART1 (Bluetooth) and USART2
 *          (Main Controller) links.
 *
 * Packet layout (5 bytes, matches the spec's example AA 01 0064 CF):
 *
 *   byte0: Header   (always 0xAA)
 *   byte1: Command  (see PacketCommand_t)
 *   byte2: Value_H  (big-endian high byte of uint16_t Value)
 *   byte3: Value_L  (big-endian low byte of uint16_t Value)
 *   byte4: CRC      (CRC-8/MAXIM over bytes 0..3)
 *
 * NOTE: The struct Packet_t below is the *logical* representation used in
 * C code. PACKET_WIRE_SIZE bytes are what actually travels the wire --
 * we do not memcpy the struct directly onto the wire because struct
 * padding/alignment on Cortex-M33 could differ from the 5-byte format and
 * because byte order must be explicit (big-endian) regardless of host
 * endianness.
 */

#ifndef PACKET_H
#define PACKET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define PACKET_HEADER_BYTE   0xAAU
#define PACKET_WIRE_SIZE     5U   /* Header + Command + Value(2) + CRC */

typedef struct
{
    uint8_t  Header;
    uint8_t  Command;
    uint16_t Value;
    uint8_t  CRC;
} Packet_t;

typedef enum
{
    CMD_OPEN_VALVE       = 0x01,  /* Open hydraulic valve              */
    CMD_CLOSE_VALVE      = 0x02,  /* Close hydraulic valve              */
    CMD_SET_KNEE_ANGLE   = 0x03,  /* Value = target angle (deci-degrees) */
    CMD_CALIBRATION_MODE = 0x04,  /* Enter calibration mode             */

    /* Outbound-only status/fault commands (controller/BLE <- device) */
    CMD_STATUS_REPORT    = 0x10,  /* Value = packed status, see bluetooth.c */
    CMD_FAULT_REPORT     = 0x11,  /* Value = fault bitmask, see safety.h    */
} PacketCommand_t;

/**
 * @brief Compute CRC-8/MAXIM (poly 0x31, init 0x00, reflected) over a byte
 *        buffer. Chosen for being cheap on a Cortex-M33 (no table needed,
 *        though we use one for speed) and adequate for a 4-byte payload
 *        on a short, low-noise wired/BLE link.
 */
uint8_t Packet_CRC8(const uint8_t *data, uint16_t length);

/**
 * @brief Serialize a Packet_t into a PACKET_WIRE_SIZE-byte buffer in the
 *        wire format described above, computing and filling in the CRC.
 */
void Packet_Encode(const Packet_t *pkt, uint8_t out_buf[PACKET_WIRE_SIZE]);

/**
 * @brief Attempt to parse one packet out of a raw byte buffer (as received
 *        from UART_DMA_Receive). Scans for the header byte, checks the
 *        CRC, and on success fills *out_pkt.
 *
 * @return true if a valid packet was found and decoded, false if the
 *         buffer was too short, no header was found, or CRC failed.
 */
bool Packet_Decode(const uint8_t *buf, uint16_t buf_len, Packet_t *out_pkt);

#ifdef __cplusplus
}
#endif

#endif /* PACKET_H */
