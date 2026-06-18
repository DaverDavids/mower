#ifndef USB_CDC_COMMS_H
#define USB_CDC_COMMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* ============================================================
 * USB CDC Communications Layer
 *
 * Provides:
 *   - Debug printf over USB virtual COM port
 *   - Bidirectional command protocol with Atomic Pi Linux host
 *
 * Protocol (ASCII, newline-terminated):
 *   Host -> STM32 commands:
 *     "E<m>\n"          Enable motor m (0/1/2)
 *     "D<m>\n"          Disable motor m
 *     "DA\n"            Disable ALL motors (emergency stop)
 *     "S<m>,<duty>\n"   Set duty for motor m (0-3599)
 *     "R<m>,<dir>\n"    Set direction (0=fwd, 1=rev)
 *     "C<m>\n"          Reset commutation counter for motor m
 *     "?\n"             Query: returns status of all motors
 *     "P\n"             Ping: returns "PONG\n"
 *
 *   STM32 -> Host responses:
 *     "OK\n"            Command accepted
 *     "ERR:<msg>\n"     Error
 *     "STATUS:<m>,<en>,<duty>,<dir>,<count>,<errors>\n"  per motor
 *     "PONG\n"          Ping response
 *     "DBG:<msg>\n"     Debug print messages
 * ============================================================ */

/* Max receive buffer size */
#define COMMS_RX_BUF_SIZE   128U
#define COMMS_TX_BUF_SIZE   256U

/**
 * @brief  Initialise the comms layer. Call after USB init.
 */
void COMMS_Init(void);

/**
 * @brief  Call this from the main loop to process any pending received data.
 *         Parses complete lines and dispatches commands.
 */
void COMMS_Tick(void);

/**
 * @brief  Send a formatted debug string over USB CDC.
 *         Safe to call before USB enumeration (drops data if not connected).
 */
void COMMS_Debug(const char *fmt, ...);

/**
 * @brief  Send a raw string over USB CDC.
 */
void COMMS_Send(const char *str);

/**
 * @brief  Called by USB CDC receive callback to buffer incoming data.
 *         Wire this into usbd_cdc_if.c CDC_Receive_FS().
 */
void COMMS_RxCallback(uint8_t *buf, uint32_t len);

/**
 * @brief  Returns 1 if USB is enumerated and ready.
 */
uint8_t COMMS_IsConnected(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_COMMS_H */
