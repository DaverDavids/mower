#ifndef USB_CDC_COMMS_H
#define USB_CDC_COMMS_H

/* -----------------------------------------------------------------------
 * usb_cdc_comms.h
 * Command/response layer over USB CDC virtual COM port.
 *
 * Atomic Pi (Linux) sends ASCII commands over /dev/ttyACM0.
 * STM32 replies with ASCII responses.
 *
 * Command set:
 *   P             → PONG\n                    (ping/alive check)
 *   ?             → STATUS:<m>,<en>,<duty>,<dir>,<count>,<errors>\n  x3
 *   E<m>          → enable motor m (0/1/2)
 *   D<m>          → disable motor m
 *   DA            → disable ALL motors (emergency stop)
 *   S<m>,<duty>   → set duty 0-3599 on motor m
 *   R<m>,<0|1>    → set direction (0=fwd, 1=rev) on motor m
 *   C<m>          → clear fault on motor m
 *   Z<m>          → zero commutation counter on motor m
 *   DBG           → send one debug status dump
 * ----------------------------------------------------------------------- */

#include <stdint.h>

void COMMS_Init(void);
void COMMS_Tick(void);           /* Call every main loop iteration */
void COMMS_Send(const char *str);
void COMMS_Debug(const char *msg);
int  COMMS_IsConnected(void);    /* Returns 1 if USB CDC host is connected */

#endif /* USB_CDC_COMMS_H */
