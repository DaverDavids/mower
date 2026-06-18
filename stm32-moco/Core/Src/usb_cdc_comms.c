/**
 * usb_cdc_comms.c
 *
 * USB CDC communications and debug layer.
 *
 * WIRING TO USB CDC MIDDLEWARE:
 *   After adding USB CDC via CubeMX (Middleware -> USB_DEVICE -> CDC),
 *   open USB_DEVICE/App/usbd_cdc_if.c and in CDC_Receive_FS() add:
 *
 *     #include "usb_cdc_comms.h"
 *     COMMS_RxCallback(Buf, *Len);
 *
 *   And in usbd_cdc_if.h or usb_cdc_comms.c, reference:
 *     extern USBD_HandleTypeDef hUsbDeviceFS;
 *     CDC_Transmit_FS(buf, len)  -- declared in usbd_cdc_if.h
 *
 * Until CDC middleware is added, COMMS_Send() is a no-op stub.
 */

#include "usb_cdc_comms.h"
#include "motor_controller.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

/* ============================================================
 * CDC transmit - weak stub, overridden when CDC middleware present
 * ============================================================ */

/* Forward declaration - implemented in usbd_cdc_if.c by CubeMX.
 * Declared weak here so the project compiles before CDC is added. */
__attribute__((weak)) uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
    (void)Buf; (void)Len;
    return 1; /* USBD_FAIL stub */
}

/* ============================================================
 * Internal RX ring buffer
 * ============================================================ */

static uint8_t  rx_buf[COMMS_RX_BUF_SIZE];
static uint32_t rx_head = 0;
static uint32_t rx_tail = 0;
static uint8_t  line_buf[COMMS_RX_BUF_SIZE];
static uint32_t line_pos = 0;

static uint8_t  g_connected = 0;

/* ============================================================
 * Init
 * ============================================================ */

void COMMS_Init(void)
{
    rx_head   = 0;
    rx_tail   = 0;
    line_pos  = 0;
    g_connected = 0;
    memset(rx_buf,  0, sizeof(rx_buf));
    memset(line_buf, 0, sizeof(line_buf));
}

/* ============================================================
 * RX callback (called from USB ISR context)
 * ============================================================ */

void COMMS_RxCallback(uint8_t *buf, uint32_t len)
{
    g_connected = 1;
    for (uint32_t i = 0; i < len; i++) {
        uint32_t next = (rx_head + 1) % COMMS_RX_BUF_SIZE;
        if (next != rx_tail) {
            rx_buf[rx_head] = buf[i];
            rx_head = next;
        }
        /* Drop byte if buffer full */
    }
}

/* ============================================================
 * Send helpers
 * ============================================================ */

void COMMS_Send(const char *str)
{
    if (!str) return;
    uint16_t len = (uint16_t)strlen(str);
    if (len == 0) return;
    CDC_Transmit_FS((uint8_t *)str, len);
}

void COMMS_Debug(const char *fmt, ...)
{
    char buf[COMMS_TX_BUF_SIZE];
    char msg[COMMS_TX_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf) - 1, fmt, args);
    va_end(args);
    snprintf(msg, sizeof(msg) - 1, "DBG:%s\n", buf);
    COMMS_Send(msg);
}

uint8_t COMMS_IsConnected(void)
{
    return g_connected;
}

/* ============================================================
 * Command parser
 * ============================================================ */

static void dispatch_command(char *line)
{
    if (!line || line[0] == '\0') return;

    char resp[COMMS_TX_BUF_SIZE];

    /* P - Ping */
    if (line[0] == 'P' && line[1] == '\0') {
        COMMS_Send("PONG\n");
        return;
    }

    /* ? - Status query */
    if (line[0] == '?') {
        for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
            MotorState_t s = MC_GetState(m);
            snprintf(resp, sizeof(resp) - 1,
                     "STATUS:%u,%u,%lu,%u,%lu,%lu\n",
                     m, s.enabled, s.duty,
                     (unsigned)s.direction,
                     s.commut_count, s.hall_errors);
            COMMS_Send(resp);
        }
        return;
    }

    /* DA - Disable All (emergency stop) */
    if (line[0] == 'D' && line[1] == 'A') {
        MC_DisableAll();
        COMMS_Send("OK\n");
        return;
    }

    /* Single-char commands with motor index: E<m>, D<m>, C<m> */
    if (line[1] >= '0' && line[1] <= '2') {
        uint8_t m = (uint8_t)(line[1] - '0');
        switch (line[0]) {
            case 'E':
                MC_Enable(m);
                COMMS_Send("OK\n");
                return;
            case 'D':
                MC_Disable(m);
                COMMS_Send("OK\n");
                return;
            case 'C':
                MC_ResetCommutCount(m);
                COMMS_Send("OK\n");
                return;
            default:
                break;
        }
    }

    /* S<m>,<duty> - Set duty */
    if (line[0] == 'S' && line[1] >= '0' && line[1] <= '2' && line[2] == ',') {
        uint8_t  m    = (uint8_t)(line[1] - '0');
        uint32_t duty = (uint32_t)atoi(&line[3]);
        MC_SetDuty(m, duty);
        COMMS_Send("OK\n");
        return;
    }

    /* R<m>,<dir> - Set direction */
    if (line[0] == 'R' && line[1] >= '0' && line[1] <= '2' && line[2] == ',') {
        uint8_t    m   = (uint8_t)(line[1] - '0');
        MotorDir_t dir = (atoi(&line[3]) == 0) ? DIR_FORWARD : DIR_REVERSE;
        MC_SetDirection(m, dir);
        COMMS_Send("OK\n");
        return;
    }

    /* Unknown command */
    snprintf(resp, sizeof(resp) - 1, "ERR:unknown cmd '%s'\n", line);
    COMMS_Send(resp);
}

/* ============================================================
 * Main tick - call from main loop
 * ============================================================ */

void COMMS_Tick(void)
{
    /* Drain ring buffer into line buffer, dispatch on newline */
    while (rx_tail != rx_head) {
        uint8_t c = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % COMMS_RX_BUF_SIZE;

        if (c == '\r') continue; /* ignore CR */

        if (c == '\n') {
            line_buf[line_pos] = '\0';
            dispatch_command((char *)line_buf);
            line_pos = 0;
        } else {
            if (line_pos < COMMS_RX_BUF_SIZE - 1) {
                line_buf[line_pos++] = c;
            } else {
                /* Line too long - reset */
                line_pos = 0;
                COMMS_Send("ERR:line overflow\n");
            }
        }
    }
}
