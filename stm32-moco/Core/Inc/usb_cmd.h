#ifndef USB_CMD_H
#define USB_CMD_H

#include <stdint.h>

/* -------------------------------------------------------------------------
 * USB CDC Command Interface
 *
 * ASCII line-based protocol (newline-terminated).
 * Commands from Atomic Pi Linux host → STM32.
 * Responses / feedback from STM32 → host.
 *
 * Command set:
 *   SET <motor> <duty>           – set duty 0-1440
 *   DIR <motor> <0|1>            – set direction (0=fwd, 1=rev)
 *   EN  <motor>                  – enable motor
 *   DIS <motor>                  – disable motor
 *   STOP                         – disable all motors immediately
 *   STATUS                       – dump status of all motors
 *   HALL                         – print raw Hall readings for all motors
 *   TICKS                        – print Hall tick counters
 *   RESETTICKS [<motor>]         – reset tick counter(s)
 *   MAP <motor> <p0> <p1> <p2>   – remap phase outputs for pin-swap debug
 *   GETMAP <motor>               – read current phase map
 *   PING                         – replies "PONG"
 *   HELP                         – print command list
 *
 * All responses are prefixed with "OK ", "ERR ", or "INFO ".
 * -------------------------------------------------------------------------
 */

void USBCMD_Init(void);
void USBCMD_Process(void);          /* Call every main-loop iteration    */
void USBCMD_OnReceive(uint8_t *buf, uint32_t len);  /* Called by CDC ISR */

/* Convenience transmit helper (null-terminated string) */
void USBCMD_Send(const char *msg);

#endif /* USB_CMD_H */
