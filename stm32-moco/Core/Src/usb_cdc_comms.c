/* -----------------------------------------------------------------------
 * usb_cdc_comms.c
 * ASCII command processor over USB CDC (virtual COM port).
 * Hooks into usbd_cdc_if.c's UserRxBufferFS via a simple ring buffer.
 * ----------------------------------------------------------------------- */

#include "usb_cdc_comms.h"
#include "usbd_cdc_if.h"
#include "motor_controller.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ---- RX ring buffer ---- */
#define RX_BUF_SIZE     256U
static char     g_rx_buf[RX_BUF_SIZE];
static uint16_t g_rx_head = 0;
static uint16_t g_rx_tail = 0;

/* ---- TX staging buffer ---- */
#define TX_BUF_SIZE     128U
static uint8_t g_tx_buf[TX_BUF_SIZE];

/* ---- Connection flag (set when first byte arrives from host) ---- */
static uint8_t g_connected = 0;

/* ======================================================================
 * COMMS_Init – call after MX_USB_DEVICE_Init()
 * ====================================================================== */
void COMMS_Init(void)
{
    g_rx_head  = 0;
    g_rx_tail  = 0;
    g_connected = 0;
    memset(g_rx_buf, 0, sizeof(g_rx_buf));
}

/* ======================================================================
 * COMMS_PushByte – called from CDC_Receive_FS (usbd_cdc_if.c)
 * Feed received bytes into the ring buffer.
 * ====================================================================== */
void COMMS_PushByte(uint8_t byte)
{
    g_connected = 1;
    uint16_t next = (g_rx_head + 1) % RX_BUF_SIZE;
    if (next != g_rx_tail) {
        g_rx_buf[g_rx_head] = (char)byte;
        g_rx_head = next;
    }
    /* If buffer full, silently drop – host should slow down */
}

/* ---- Process one complete newline-terminated command ---- */
static void ProcessCommand(const char *cmd)
{
    char resp[128];

    if (cmd[0] == 'P' && cmd[1] == '\0') {
        COMMS_Send("PONG\n");
        return;
    }

    if (cmd[0] == '?') {
        /* Status dump for all 3 motors */
        for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
            MotorState_t s = MC_GetState(m);
            snprintf(resp, sizeof(resp),
                     "STATUS:%u,%u,%lu,%u,%lu,%lu\n",
                     m, s.enabled, s.duty,
                     (unsigned)s.direction,
                     s.commut_count, s.hall_errors);
            COMMS_Send(resp);
        }
        return;
    }

    if (cmd[0] == 'D' && cmd[1] == 'A') {
        MC_DisableAll();
        COMMS_Send("OK:DA\n");
        return;
    }

    if (cmd[0] == 'D' && cmd[1] == 'B' && cmd[2] == 'G') {
        COMMS_Debug("Manual DBG dump");
        return;
    }

    /* Single-motor commands – cmd[1] is motor index digit */
    uint8_t idx = (uint8_t)(cmd[1] - '0');
    if (idx >= MOTOR_COUNT) {
        COMMS_Send("ERR:BADIDX\n");
        return;
    }

    switch (cmd[0]) {
        case 'E':
            MC_Enable(idx);
            snprintf(resp, sizeof(resp), "OK:E%u\n", idx);
            COMMS_Send(resp);
            break;

        case 'D':
            MC_Disable(idx);
            snprintf(resp, sizeof(resp), "OK:D%u\n", idx);
            COMMS_Send(resp);
            break;

        case 'S': {
            /* S<m>,<duty>  e.g. "S0,1800" */
            const char *comma = strchr(cmd + 2, ',');
            if (!comma) { COMMS_Send("ERR:FMT\n"); break; }
            uint32_t duty = (uint32_t)atoi(comma + 1);
            MC_SetDuty(idx, duty);
            snprintf(resp, sizeof(resp), "OK:S%u,%lu\n", idx, duty);
            COMMS_Send(resp);
            break;
        }

        case 'R': {
            /* R<m>,<0|1>  e.g. "R0,1" */
            const char *comma = strchr(cmd + 2, ',');
            if (!comma) { COMMS_Send("ERR:FMT\n"); break; }
            MotorDir_t dir = (atoi(comma + 1) != 0) ? DIR_REVERSE : DIR_FORWARD;
            MC_SetDirection(idx, dir);
            snprintf(resp, sizeof(resp), "OK:R%u,%u\n", idx, (unsigned)dir);
            COMMS_Send(resp);
            break;
        }

        case 'C':
            MC_ClearFault(idx);
            snprintf(resp, sizeof(resp), "OK:C%u\n", idx);
            COMMS_Send(resp);
            break;

        case 'Z':
            MC_ResetCommutCount(idx);
            snprintf(resp, sizeof(resp), "OK:Z%u\n", idx);
            COMMS_Send(resp);
            break;

        default:
            COMMS_Send("ERR:UNK\n");
            break;
    }
}

/* ======================================================================
 * COMMS_Tick – call from main loop
 * Drains the ring buffer, assembles lines, dispatches commands.
 * ====================================================================== */
void COMMS_Tick(void)
{
    static char line_buf[64];
    static uint8_t line_len = 0;

    while (g_rx_tail != g_rx_head) {
        char c = g_rx_buf[g_rx_tail];
        g_rx_tail = (g_rx_tail + 1) % RX_BUF_SIZE;

        if (c == '\n' || c == '\r') {
            if (line_len > 0) {
                line_buf[line_len] = '\0';
                ProcessCommand(line_buf);
                line_len = 0;
            }
        } else if (line_len < (sizeof(line_buf) - 1)) {
            line_buf[line_len++] = c;
        }
    }
}

/* ======================================================================
 * COMMS_Send – send a null-terminated ASCII string over CDC
 * ====================================================================== */
void COMMS_Send(const char *str)
{
    uint16_t len = (uint16_t)strlen(str);
    if (len == 0 || len > TX_BUF_SIZE) return;
    memcpy(g_tx_buf, str, len);
    /* CDC_Transmit_FS handles busy/retry internally */
    CDC_Transmit_FS(g_tx_buf, len);
}

/* ======================================================================
 * COMMS_Debug – send a prefixed debug line
 * Format: "DBG:<msg>\n"
 * ====================================================================== */
void COMMS_Debug(const char *msg)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "DBG:%s\n", msg);
    COMMS_Send(buf);
}

/* ======================================================================
 * COMMS_IsConnected – returns 1 after first byte from host
 * ====================================================================== */
int COMMS_IsConnected(void)
{
    return (int)g_connected;
}
