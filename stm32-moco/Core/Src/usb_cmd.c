/* =========================================================================
 * usb_cmd.c  –  USB CDC command parser and response handler
 *
 * Protocol: ASCII, newline-terminated lines (\n or \r\n).
 * All incoming bytes are buffered here; parsing happens in USBCMD_Process()
 * called from the main loop – no blocking in the ISR callback.
 *
 * This file also serves as the interactive debug shell.  Use any serial
 * terminal at any baud (CDC ignores baud) on the STM32 USB port.
 *
 * Command reference – see usb_cmd.h
 * =========================================================================
 */

#include "usb_cmd.h"
#include "motor_ctrl.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/* -------------------------------------------------------------------------
 * RX ring buffer (filled by CDC ISR, drained by USBCMD_Process)
 * -------------------------------------------------------------------------
 */
#define RX_BUF_SIZE  256U

static volatile uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;   /* Write index (ISR side)  */
static volatile uint16_t rx_tail = 0;   /* Read  index (main loop) */

/* Line assembly buffer */
#define LINE_BUF_SIZE 128U
static char line_buf[LINE_BUF_SIZE];
static uint16_t line_pos = 0;

/* TX scratch buffer */
static char tx_scratch[256];

/* -------------------------------------------------------------------------
 * Pin lookup table
 * Covers all named GPIOs in the project:
 *   - Hall sensor inputs:        M1=PA0-PA2, M2=PA3-PA5, M3=PB10-PB12
 *   - M1 high-side PWM outputs:  PA8, PA9, PA10 (TIM1 CH1-3)   → M1HSA/B/C
 *   - M1 low-side PWM outputs:   PB13, PB14, PB15 (TIM1 CH1N-3N) → M1LSA/B/C
 *   - M2 high-side PWM outputs:  PA6, PA7, PB0 (TIM3 CH1-3)    → M2HSA/B/C
 *   - M2 low-side enables:       PA15, PB3, PB5 (GPIO output)   → M2LSA/B/C
 *   - M3 high-side PWM outputs:  PB6, PB7, PB8 (TIM4 CH1-3)    → M3HSA/B/C
 *   - M3 low-side enables:       PB9, PC13, PC14 (GPIO output)  → M3LSA/B/C
 *
 * SETPIN is blocked on PWM-controlled pins (M1/M2/M3 high-side and M1
 * low-side) to avoid fighting the timer hardware.  Use EN/DIS/SET
 * commands for those.  Low-side enable GPIOs (M2/M3 LS) are freely settable.
 * -------------------------------------------------------------------------
 */
typedef struct {
    const char   *name;       /* uppercase name used in commands        */
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       output;     /* 1 = output pin, 0 = input pin          */
    uint8_t       pwm_only;   /* 1 = timer-controlled, block SETPIN     */
} PinEntry_t;

static const PinEntry_t pin_table[] = {
    /* Hall inputs – Motor 1 */
    { "M1HA",  GPIOA, GPIO_PIN_0,  0, 0 },
    { "M1HB",  GPIOA, GPIO_PIN_1,  0, 0 },
    { "M1HC",  GPIOA, GPIO_PIN_2,  0, 0 },
    /* Hall inputs – Motor 2 */
    { "M2HA",  GPIOA, GPIO_PIN_3,  0, 0 },
    { "M2HB",  GPIOA, GPIO_PIN_4,  0, 0 },
    { "M2HC",  GPIOA, GPIO_PIN_5,  0, 0 },
    /* Hall inputs – Motor 3 */
    { "M3HA",  GPIOB, GPIO_PIN_10, 0, 0 },
    { "M3HB",  GPIOB, GPIO_PIN_11, 0, 0 },
    { "M3HC",  GPIOB, GPIO_PIN_12, 0, 0 },
    /* M1 high-side PWM (TIM1 CH1-3) – read-only via READPIN */
    { "M1HSA", GPIOA, GPIO_PIN_8,  1, 1 },
    { "M1HSB", GPIOA, GPIO_PIN_9,  1, 1 },
    { "M1HSC", GPIOA, GPIO_PIN_10, 1, 1 },
    /* M1 low-side PWM (TIM1 CH1N-3N) – read-only via READPIN */
    { "M1LSA", GPIOB, GPIO_PIN_13, 1, 1 },
    { "M1LSB", GPIOB, GPIO_PIN_14, 1, 1 },
    { "M1LSC", GPIOB, GPIO_PIN_15, 1, 1 },
    /* M2 high-side PWM (TIM3 CH1-3) – read-only via READPIN */
    { "M2HSA", GPIOA, GPIO_PIN_6,  1, 1 },
    { "M2HSB", GPIOA, GPIO_PIN_7,  1, 1 },
    { "M2HSC", GPIOB, GPIO_PIN_0,  1, 1 },
    /* M2 low-side enables (GPIO output) – freely settable */
    { "M2LSA", GPIOA, GPIO_PIN_15, 1, 0 },
    { "M2LSB", GPIOB, GPIO_PIN_3,  1, 0 },
    { "M2LSC", GPIOB, GPIO_PIN_5,  1, 0 },
    /* M3 high-side PWM (TIM4 CH1-3) – read-only via READPIN */
    { "M3HSA", GPIOB, GPIO_PIN_6,  1, 1 },
    { "M3HSB", GPIOB, GPIO_PIN_7,  1, 1 },
    { "M3HSC", GPIOB, GPIO_PIN_8,  1, 1 },
    /* M3 low-side enables (GPIO output) – freely settable */
    { "M3LSA", GPIOB, GPIO_PIN_9,  1, 0 },
    { "M3LSB", GPIOC, GPIO_PIN_13, 1, 0 },
    { "M3LSC", GPIOC, GPIO_PIN_14, 1, 0 },
};

#define PIN_TABLE_SIZE (sizeof(pin_table) / sizeof(pin_table[0]))

static const PinEntry_t *find_pin(const char *name)
{
    for (uint32_t i = 0; i < PIN_TABLE_SIZE; i++) {
        if (strcmp(pin_table[i].name, name) == 0) return &pin_table[i];
    }
    return NULL;
}

/* -------------------------------------------------------------------------
 * ISR callback – called from usbd_cdc_if.c CDC_Receive_FS()
 * Just push bytes into the ring buffer; do NOT parse here.
 * -------------------------------------------------------------------------
 */
void USBCMD_OnReceive(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        uint16_t next = (rx_head + 1U) % RX_BUF_SIZE;
        if (next != rx_tail) {          /* drop if full */
            rx_buf[rx_head] = buf[i];
            rx_head = next;
        }
    }
}

/* -------------------------------------------------------------------------
 * Transmit helper
 * -------------------------------------------------------------------------
 */
void USBCMD_Send(const char *msg)
{
    uint16_t len = (uint16_t)strlen(msg);
    /* CDC_Transmit is non-blocking; retry once if busy */
    if (CDC_Transmit_FS((uint8_t *)msg, len) == USBD_BUSY) {
        HAL_Delay(1);
        CDC_Transmit_FS((uint8_t *)msg, len);
    }
}

/* -------------------------------------------------------------------------
 * Command dispatcher
 * -------------------------------------------------------------------------
 */
static void dispatch(char *line)
{
    /* Tokenise in-place */
    char *tok = strtok(line, " \t");
    if (tok == NULL) return;

    /* ------------------------------------------------------------------ */
    if (strcmp(tok, "PING") == 0) {
        USBCMD_Send("OK PONG\r\n");

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "STOP") == 0) {
        Motor_SafeAll();
        USBCMD_Send("OK ALL_STOPPED\r\n");

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "SET") == 0) {
        char *s_mid  = strtok(NULL, " \t");
        char *s_duty = strtok(NULL, " \t");
        if (!s_mid || !s_duty) {
            USBCMD_Send("ERR USAGE: SET <motor 1-3> <duty 0-1440>\r\n"); return;
        }
        int mid  = atoi(s_mid)  - 1;   /* user 1-based → 0-based */
        int duty = atoi(s_duty);
        if (mid < 0 || mid >= MOTOR_COUNT || duty < 0 || duty > (int)DUTY_MAX) {
            USBCMD_Send("ERR INVALID_ARG\r\n"); return;
        }
        Motor_SetDuty((uint8_t)mid, (uint16_t)duty);
        snprintf(tx_scratch, sizeof(tx_scratch), "OK MOTOR%d DUTY=%d\r\n", mid+1, duty);
        USBCMD_Send(tx_scratch);

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "DIR") == 0) {
        char *s_mid = strtok(NULL, " \t");
        char *s_dir = strtok(NULL, " \t");
        if (!s_mid || !s_dir) {
            USBCMD_Send("ERR USAGE: DIR <motor 1-3> <0=fwd|1=rev>\r\n"); return;
        }
        int mid = atoi(s_mid) - 1;
        int dir = atoi(s_dir);
        if (mid < 0 || mid >= MOTOR_COUNT || (dir != 0 && dir != 1)) {
            USBCMD_Send("ERR INVALID_ARG\r\n"); return;
        }
        Motor_SetDir((uint8_t)mid, (MotorDir_t)dir);
        snprintf(tx_scratch, sizeof(tx_scratch), "OK MOTOR%d DIR=%s\r\n",
                 mid+1, dir == 0 ? "FWD" : "REV");
        USBCMD_Send(tx_scratch);

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "EN") == 0) {
        char *s_mid = strtok(NULL, " \t");
        if (!s_mid) { USBCMD_Send("ERR USAGE: EN <motor 1-3>\r\n"); return; }
        int mid = atoi(s_mid) - 1;
        if (mid < 0 || mid >= MOTOR_COUNT) { USBCMD_Send("ERR INVALID_ARG\r\n"); return; }
        Motor_Enable((uint8_t)mid);
        snprintf(tx_scratch, sizeof(tx_scratch), "OK MOTOR%d ENABLED\r\n", mid+1);
        USBCMD_Send(tx_scratch);

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "DIS") == 0) {
        char *s_mid = strtok(NULL, " \t");
        if (!s_mid) { USBCMD_Send("ERR USAGE: DIS <motor 1-3>\r\n"); return; }
        int mid = atoi(s_mid) - 1;
        if (mid < 0 || mid >= MOTOR_COUNT) { USBCMD_Send("ERR INVALID_ARG\r\n"); return; }
        Motor_Disable((uint8_t)mid);
        snprintf(tx_scratch, sizeof(tx_scratch), "OK MOTOR%d DISABLED\r\n", mid+1);
        USBCMD_Send(tx_scratch);

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "STATUS") == 0) {
        USBCMD_Send("INFO --- Motor Status ---\r\n");
        for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
            MotorState_t *ms = &g_motor[m];
            snprintf(tx_scratch, sizeof(tx_scratch),
                "INFO M%d: en=%d dir=%s duty=%u hall=0x%X step=%u ticks=%ld map=[%d,%d,%d]\r\n",
                m+1,
                ms->enabled,
                ms->dir == DIR_FORWARD ? "FWD" : "REV",
                ms->duty,
                ms->hall_state,
                ms->commut_step,
                (long)ms->hall_ticks,
                ms->phase_map[0], ms->phase_map[1], ms->phase_map[2]);
            USBCMD_Send(tx_scratch);
        }

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "HALL") == 0) {
        for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
            uint8_t h = Motor_ReadHall(m);
            snprintf(tx_scratch, sizeof(tx_scratch),
                "INFO M%d HALL: raw=0x%X HA=%d HB=%d HC=%d\r\n",
                m+1, h, (h>>2)&1, (h>>1)&1, h&1);
            USBCMD_Send(tx_scratch);
        }

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "TICKS") == 0) {
        for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
            snprintf(tx_scratch, sizeof(tx_scratch),
                "INFO M%d TICKS: %ld\r\n", m+1, (long)Motor_GetTicks(m));
            USBCMD_Send(tx_scratch);
        }

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "RESETTICKS") == 0) {
        char *s_mid = strtok(NULL, " \t");
        if (s_mid) {
            int mid = atoi(s_mid) - 1;
            if (mid >= 0 && mid < MOTOR_COUNT) {
                Motor_ResetTicks((uint8_t)mid);
                snprintf(tx_scratch, sizeof(tx_scratch), "OK M%d TICKS_RESET\r\n", mid+1);
                USBCMD_Send(tx_scratch);
            } else {
                USBCMD_Send("ERR INVALID_ARG\r\n");
            }
        } else {
            for (uint8_t m = 0; m < MOTOR_COUNT; m++) Motor_ResetTicks(m);
            USBCMD_Send("OK ALL_TICKS_RESET\r\n");
        }

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "MAP") == 0) {
        char *s_mid = strtok(NULL, " \t");
        char *s_p0  = strtok(NULL, " \t");
        char *s_p1  = strtok(NULL, " \t");
        char *s_p2  = strtok(NULL, " \t");
        if (!s_mid || !s_p0 || !s_p1 || !s_p2) {
            USBCMD_Send("ERR USAGE: MAP <motor 1-3> <p0 0-2> <p1 0-2> <p2 0-2>\r\n"); return;
        }
        int mid = atoi(s_mid) - 1;
        int p0  = atoi(s_p0);
        int p1  = atoi(s_p1);
        int p2  = atoi(s_p2);
        if (mid < 0 || mid >= MOTOR_COUNT ||
            p0 < 0 || p0 > 2 || p1 < 0 || p1 > 2 || p2 < 0 || p2 > 2) {
            USBCMD_Send("ERR INVALID_ARG\r\n"); return;
        }
        Motor_SetPhaseMap((uint8_t)mid, (uint8_t)p0, (uint8_t)p1, (uint8_t)p2);
        snprintf(tx_scratch, sizeof(tx_scratch),
            "OK M%d MAP=[%d,%d,%d]\r\n", mid+1, p0, p1, p2);
        USBCMD_Send(tx_scratch);

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "GETMAP") == 0) {
        char *s_mid = strtok(NULL, " \t");
        if (!s_mid) { USBCMD_Send("ERR USAGE: GETMAP <motor 1-3>\r\n"); return; }
        int mid = atoi(s_mid) - 1;
        if (mid < 0 || mid >= MOTOR_COUNT) { USBCMD_Send("ERR INVALID_ARG\r\n"); return; }
        MotorState_t *ms = &g_motor[mid];
        snprintf(tx_scratch, sizeof(tx_scratch),
            "OK M%d MAP=[%d,%d,%d]\r\n", mid+1,
            ms->phase_map[0], ms->phase_map[1], ms->phase_map[2]);
        USBCMD_Send(tx_scratch);

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "SETPIN") == 0) {
        /* SETPIN <name> <0|1>  –  drive a named GPIO output high or low.
         * PWM-controlled pins are blocked; use EN/DIS/SET for those. */
        char *s_name = strtok(NULL, " \t");
        char *s_val  = strtok(NULL, " \t");
        if (!s_name || !s_val) {
            USBCMD_Send("ERR USAGE: SETPIN <name> <0|1>\r\n"); return;
        }
        int val = atoi(s_val);
        if (val != 0 && val != 1) {
            USBCMD_Send("ERR VALUE must be 0 or 1\r\n"); return;
        }
        const PinEntry_t *p = find_pin(s_name);
        if (!p) {
            snprintf(tx_scratch, sizeof(tx_scratch), "ERR UNKNOWN_PIN: %s\r\n", s_name);
            USBCMD_Send(tx_scratch); return;
        }
        if (!p->output) {
            snprintf(tx_scratch, sizeof(tx_scratch), "ERR PIN_IS_INPUT: %s\r\n", s_name);
            USBCMD_Send(tx_scratch); return;
        }
        if (p->pwm_only) {
            snprintf(tx_scratch, sizeof(tx_scratch),
                "ERR PIN_IS_PWM: %s – use EN/DIS/SET commands\r\n", s_name);
            USBCMD_Send(tx_scratch); return;
        }
        HAL_GPIO_WritePin(p->port, p->pin,
            val ? GPIO_PIN_SET : GPIO_PIN_RESET);
        snprintf(tx_scratch, sizeof(tx_scratch), "OK %s=%d\r\n", s_name, val);
        USBCMD_Send(tx_scratch);

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "READPIN") == 0) {
        /* READPIN <name>  –  read the current logic level of any named pin.
         * Works on both input and output pins. */
        char *s_name = strtok(NULL, " \t");
        if (!s_name) {
            USBCMD_Send("ERR USAGE: READPIN <name>\r\n"); return;
        }
        const PinEntry_t *p = find_pin(s_name);
        if (!p) {
            snprintf(tx_scratch, sizeof(tx_scratch), "ERR UNKNOWN_PIN: %s\r\n", s_name);
            USBCMD_Send(tx_scratch); return;
        }
        int level = (HAL_GPIO_ReadPin(p->port, p->pin) == GPIO_PIN_SET) ? 1 : 0;
        snprintf(tx_scratch, sizeof(tx_scratch),
            "OK %s=%d (%s)\r\n", s_name, level, p->output ? "OUT" : "IN");
        USBCMD_Send(tx_scratch);

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "PINS") == 0) {
        /* PINS  –  dump the current level of every pin in the table. */
        USBCMD_Send("INFO --- Pin States ---\r\n");
        for (uint32_t i = 0; i < PIN_TABLE_SIZE; i++) {
            const PinEntry_t *p = &pin_table[i];
            int level = (HAL_GPIO_ReadPin(p->port, p->pin) == GPIO_PIN_SET) ? 1 : 0;
            snprintf(tx_scratch, sizeof(tx_scratch),
                "INFO %s=%d (%s%s)\r\n",
                p->name, level,
                p->output ? "OUT" : "IN",
                p->pwm_only ? "/PWM" : "");
            USBCMD_Send(tx_scratch);
        }

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "HELP") == 0) {
        USBCMD_Send("INFO Commands:\r\n");
        USBCMD_Send("INFO   PING\r\n");
        USBCMD_Send("INFO   STOP\r\n");
        USBCMD_Send("INFO   SET  <motor 1-3> <duty 0-1440>\r\n");
        USBCMD_Send("INFO   DIR  <motor 1-3> <0=fwd|1=rev>\r\n");
        USBCMD_Send("INFO   EN   <motor 1-3>\r\n");
        USBCMD_Send("INFO   DIS  <motor 1-3>\r\n");
        USBCMD_Send("INFO   STATUS\r\n");
        USBCMD_Send("INFO   HALL\r\n");
        USBCMD_Send("INFO   TICKS\r\n");
        USBCMD_Send("INFO   RESETTICKS [<motor 1-3>]\r\n");
        USBCMD_Send("INFO   MAP  <motor 1-3> <p0> <p1> <p2>  (phase remap 0-2)\r\n");
        USBCMD_Send("INFO   GETMAP <motor 1-3>\r\n");
        USBCMD_Send("INFO   SETPIN <name> <0|1>  (output GPIO only, not PWM pins)\r\n");
        USBCMD_Send("INFO   READPIN <name>\r\n");
        USBCMD_Send("INFO   PINS  (dump all pin states)\r\n");
        USBCMD_Send("INFO   Pin names: M1HA/HB/HC  M2HA/HB/HC  M3HA/HB/HC\r\n");
        USBCMD_Send("INFO              M1HSA/HSB/HSC  M1LSA/LSB/LSC (PWM, read-only)\r\n");
        USBCMD_Send("INFO              M2HSA/HSB/HSC  (PWM, read-only)\r\n");
        USBCMD_Send("INFO              M2LSA/LSB/LSC  M3HSA/HSB/HSC (PWM, read-only)\r\n");
        USBCMD_Send("INFO              M3LSA/LSB/LSC  (settable output)\r\n");
        USBCMD_Send("INFO   HELP\r\n");

    /* ------------------------------------------------------------------ */
    } else {
        snprintf(tx_scratch, sizeof(tx_scratch), "ERR UNKNOWN_CMD: %s\r\n", tok);
        USBCMD_Send(tx_scratch);
    }
}

/* -------------------------------------------------------------------------
 * Init
 * -------------------------------------------------------------------------
 */
void USBCMD_Init(void)
{
    rx_head  = 0;
    rx_tail  = 0;
    line_pos = 0;
    memset(line_buf, 0, sizeof(line_buf));
    USBCMD_Send("INFO stm32-moco ready. Type HELP for commands.\r\n");
}

/* -------------------------------------------------------------------------
 * Main-loop process function
 * Drain ring buffer byte-by-byte, assemble lines, dispatch on \n.
 * -------------------------------------------------------------------------
 */
void USBCMD_Process(void)
{
    while (rx_tail != rx_head) {
        uint8_t c  = rx_buf[rx_tail];
        rx_tail    = (rx_tail + 1U) % RX_BUF_SIZE;

        if (c == '\r') continue;    /* ignore CR in CRLF pairs */

        if (c == '\n' || c == '\0') {
            if (line_pos > 0) {
                line_buf[line_pos] = '\0';
                /* Convert to uppercase for case-insensitive matching */
                for (uint16_t i = 0; i < line_pos; i++) {
                    if (line_buf[i] >= 'a' && line_buf[i] <= 'z') {
                        line_buf[i] -= 32;
                    }
                }
                dispatch(line_buf);
                line_pos = 0;
            }
        } else {
            if (line_pos < LINE_BUF_SIZE - 1U) {
                line_buf[line_pos++] = (char)c;
            }
            /* Silently drop overlong lines */
        }
    }
}
