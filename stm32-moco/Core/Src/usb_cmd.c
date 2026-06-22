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
 * Pin lookup table  (source of truth: project-and-pins.txt)
 *
 * Hall sensor inputs (GPIO_Input, pull-up):
 *   M1HA/HB/HC  = PA0, PA1, PA2
 *   M2HA/HB/HC  = PA3, PA4, PA5
 *   M3HA/HB/HC  = PA6, PA7, PB10
 *
 * Motor 1 – TIM1 (advanced timer, hardware dead-time):
 *   M1HSA/HSB/HSC  = PA8,  PA9,  PA10  (TIM1 CH1-3,  high-side PWM)
 *   M1LSA/LSB/LSC  = PB13, PB14, PB15  (TIM1 CH1N-3N, low-side PWM)
 *   All six are pwm_only – use EN/DIS/SET commands.
 *
 * Motor 2 – TIM4 (general-purpose timer):
 *   M2HSA/HSB/HSC  = PB6, PB7, PB8    (TIM4 CH1-3, high-side PWM)
 *   M2LSA/LSB/LSC  = PA15, PB3, PB5   (GPIO output, low-side enables)
 *
 * Motor 3 – TIM3 (general-purpose timer):
 *   M3HSA/HSB/HSC  = PB4, PB0, PB1    (TIM3 CH1/CH3/CH4, high-side PWM)
 *   M3LSA/LSB/LSC  = PB9, PB11, PB12  (GPIO output, low-side enables)
 *
 * SETPIN is blocked on pwm_only pins.  Use EN/DIS/SET for those.
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
    { "M3HA",  GPIOA, GPIO_PIN_6,  0, 0 },
    { "M3HB",  GPIOA, GPIO_PIN_7,  0, 0 },
    { "M3HC",  GPIOB, GPIO_PIN_10, 0, 0 },
    /* M1 high-side PWM (TIM1 CH1-3) – read-only via READPIN */
    { "M1HSA", GPIOA, GPIO_PIN_8,  1, 1 },
    { "M1HSB", GPIOA, GPIO_PIN_9,  1, 1 },
    { "M1HSC", GPIOA, GPIO_PIN_10, 1, 1 },
    /* M1 low-side PWM (TIM1 CH1N-3N) – read-only via READPIN */
    { "M1LSA", GPIOB, GPIO_PIN_13, 1, 1 },
    { "M1LSB", GPIOB, GPIO_PIN_14, 1, 1 },
    { "M1LSC", GPIOB, GPIO_PIN_15, 1, 1 },
    /* M2 high-side PWM (TIM4 CH1-3) – read-only via READPIN */
    { "M2HSA", GPIOB, GPIO_PIN_6,  1, 1 },
    { "M2HSB", GPIOB, GPIO_PIN_7,  1, 1 },
    { "M2HSC", GPIOB, GPIO_PIN_8,  1, 1 },
    /* M2 low-side enables (GPIO output) – freely settable */
    { "M2LSA", GPIOA, GPIO_PIN_15, 1, 0 },
    { "M2LSB", GPIOB, GPIO_PIN_3,  1, 0 },
    { "M2LSC", GPIOB, GPIO_PIN_5,  1, 0 },
    /* M3 high-side PWM (TIM3 CH1/CH3/CH4) – read-only via READPIN */
    { "M3HSA", GPIOB, GPIO_PIN_4,  1, 1 },
    { "M3HSB", GPIOB, GPIO_PIN_0,  1, 1 },
    { "M3HSC", GPIOB, GPIO_PIN_1,  1, 1 },
    /* M3 low-side enables (GPIO output) – freely settable */
    { "M3LSA", GPIOB, GPIO_PIN_9,  1, 0 },
    { "M3LSB", GPIOB, GPIO_PIN_11, 1, 0 },
    { "M3LSC", GPIOB, GPIO_PIN_12, 1, 0 },
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
 * Helper: get bit position from a GPIO_PIN_x mask
 * -------------------------------------------------------------------------
 */
static uint8_t pin_bit(uint16_t pin_mask)
{
    uint8_t bit = 0;
    while (bit < 16 && !((pin_mask >> bit) & 1U)) bit++;
    return bit;
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
        int mid  = atoi(s_mid)  - 1;   /* user 1-based -> 0-based */
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
                "ERR PIN_IS_PWM: %s - use EN/DIS/SET commands\r\n", s_name);
            USBCMD_Send(tx_scratch); return;
        }
        HAL_GPIO_WritePin(p->port, p->pin,
            val ? GPIO_PIN_SET : GPIO_PIN_RESET);
        snprintf(tx_scratch, sizeof(tx_scratch), "OK %s=%d\r\n", s_name, val);
        USBCMD_Send(tx_scratch);

    /* ------------------------------------------------------------------ */
    } else if (strcmp(tok, "READPIN") == 0) {
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

    /* ================================================================== */
    /* DEBUG COMMANDS                                                       */
    /* ================================================================== */

    /*
     * ODRDUMP
     * Read ODR (Output Data Register) directly for GPIOA and GPIOB.
     * ODR holds what we WROTE; IDR holds what the pin MEASURES.
     * If ODR bit is 1 but IDR bit is 0 -> pin is physically held low
     * despite the write succeeding (external load, AF conflict, etc).
     * If ODR bit is 0 -> the write never happened or was overwritten.
     */
    } else if (strcmp(tok, "ODRDUMP") == 0) {
        uint32_t odr_a = GPIOA->ODR;
        uint32_t idr_a = GPIOA->IDR;
        uint32_t odr_b = GPIOB->ODR;
        uint32_t idr_b = GPIOB->IDR;
        USBCMD_Send("INFO --- ODR vs IDR (write vs actual voltage) ---\r\n");
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO GPIOA ODR=0x%04lX  IDR=0x%04lX  DIFF=0x%04lX\r\n",
            odr_a, idr_a, odr_a ^ idr_a);
        USBCMD_Send(tx_scratch);
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO GPIOB ODR=0x%04lX  IDR=0x%04lX  DIFF=0x%04lX\r\n",
            odr_b, idr_b, odr_b ^ idr_b);
        USBCMD_Send(tx_scratch);
        /* Decode which LS pins have ODR=1 but IDR=0 (driven but read low) */
        USBCMD_Send("INFO Pins with ODR=1 but IDR=0 (write stuck low):\r\n");
        uint8_t found = 0;
        for (uint32_t i = 0; i < PIN_TABLE_SIZE; i++) {
            const PinEntry_t *p = &pin_table[i];
            if (!p->output || p->pwm_only) continue;
            uint32_t odr = (p->port == GPIOA) ? odr_a : odr_b;
            uint32_t idr = (p->port == GPIOA) ? idr_a : idr_b;
            uint8_t  bit = pin_bit(p->pin);
            uint8_t  odr_bit = (odr >> bit) & 1U;
            uint8_t  idr_bit = (idr >> bit) & 1U;
            if (odr_bit && !idr_bit) {
                snprintf(tx_scratch, sizeof(tx_scratch),
                    "INFO   %s (bit %u): ODR=1 IDR=0 <- STUCK LOW\r\n", p->name, bit);
                USBCMD_Send(tx_scratch);
                found = 1;
            }
        }
        if (!found) USBCMD_Send("INFO   (none)\r\n");

    /*
     * REGS <PA|PB>
     * Dump the full GPIO register block for port A or B.
     * Shows CRL, CRH (pin mode/config), ODR, IDR, LCKR.
     * CRL covers pins 0-7, CRH covers pins 8-15.
     * Each pin gets 4 bits: CNF[1:0] MODE[1:0]
     *   MODE=00 input; MODE=01/10/11 output (10MHz/2MHz/50MHz)
     *   CNF (input):  00=analog, 01=float, 10/11=pull
     *   CNF (output): 00=PP, 01=OD, 10=AF-PP, 11=AF-OD
     * Expected for plain output: MODE=10 (2MHz) CNF=00 -> nibble=0x2
     * If you see CNF=10 or 11 on an output pin -> AF is active (timer
     * or JTAG has the pin), not plain GPIO.
     */
    } else if (strcmp(tok, "REGS") == 0) {
        char *s_port = strtok(NULL, " \t");
        GPIO_TypeDef *port = NULL;
        const char   *pname = "";
        if (s_port && strcmp(s_port, "PA") == 0) { port = GPIOA; pname = "GPIOA"; }
        else if (s_port && strcmp(s_port, "PB") == 0) { port = GPIOB; pname = "GPIOB"; }
        else { USBCMD_Send("ERR USAGE: REGS <PA|PB>\r\n"); return; }

        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO --- %s Registers ---\r\n", pname);
        USBCMD_Send(tx_scratch);
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO CRL =0x%08lX  (pins 0-7  mode/cfg)\r\n", port->CRL);
        USBCMD_Send(tx_scratch);
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO CRH =0x%08lX  (pins 8-15 mode/cfg)\r\n", port->CRH);
        USBCMD_Send(tx_scratch);
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO IDR =0x%04lX        (actual pin voltage)\r\n", port->IDR);
        USBCMD_Send(tx_scratch);
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO ODR =0x%04lX        (what we wrote)\r\n", port->ODR);
        USBCMD_Send(tx_scratch);
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO LCKR=0x%08lX  (lock status)\r\n", port->LCKR);
        USBCMD_Send(tx_scratch);

        /* Decode each pin's nibble from CRL/CRH */
        USBCMD_Send("INFO Pin-by-pin mode decode:\r\n");
        for (uint8_t bit = 0; bit < 16; bit++) {
            uint32_t cr   = (bit < 8) ? port->CRL : port->CRH;
            uint8_t  shift = (bit % 8) * 4;
            uint8_t  nibble = (cr >> shift) & 0xFU;
            uint8_t  mode = nibble & 0x3U;
            uint8_t  cnf  = (nibble >> 2) & 0x3U;
            const char *mode_str = (mode == 0) ? "IN" :
                                   (mode == 1) ? "OUT_10MHz" :
                                   (mode == 2) ? "OUT_2MHz"  : "OUT_50MHz";
            const char *cnf_str;
            if (mode == 0) {
                cnf_str = (cnf == 0) ? "analog" :
                          (cnf == 1) ? "float"  :
                          (cnf == 2) ? "pull"   : "pull(rsv)";
            } else {
                cnf_str = (cnf == 0) ? "PP"    :
                          (cnf == 1) ? "OD"    :
                          (cnf == 2) ? "AF-PP" : "AF-OD";
            }
            uint8_t idr_bit = (port->IDR >> bit) & 1U;
            uint8_t odr_bit = (port->ODR >> bit) & 1U;
            snprintf(tx_scratch, sizeof(tx_scratch),
                "INFO   P%s%02u nibble=0x%X mode=%-9s cnf=%-8s ODR=%u IDR=%u\r\n",
                pname + 4, bit, nibble, mode_str, cnf_str, odr_bit, idr_bit);
            USBCMD_Send(tx_scratch);
        }

    /*
     * CRLCONF <PA|PB> <bit 0-15>
     * Read the 4-bit CRL/CRH nibble for a single pin and decode it.
     * Quick check: is a specific pin configured as AF instead of GPIO output?
     */
    } else if (strcmp(tok, "CRLCONF") == 0) {
        char *s_port = strtok(NULL, " \t");
        char *s_bit  = strtok(NULL, " \t");
        if (!s_port || !s_bit) {
            USBCMD_Send("ERR USAGE: CRLCONF <PA|PB> <bit 0-15>\r\n"); return;
        }
        GPIO_TypeDef *port = NULL;
        const char   *pname = "";
        if      (strcmp(s_port, "PA") == 0) { port = GPIOA; pname = "PA"; }
        else if (strcmp(s_port, "PB") == 0) { port = GPIOB; pname = "PB"; }
        else { USBCMD_Send("ERR USAGE: REGS <PA|PB>\r\n"); return; }
        int bit = atoi(s_bit);
        if (bit < 0 || bit > 15) { USBCMD_Send("ERR bit must be 0-15\r\n"); return; }
        uint32_t cr    = (bit < 8) ? port->CRL : port->CRH;
        uint8_t  shift = (uint8_t)((bit % 8) * 4);
        uint8_t  nibble = (cr >> shift) & 0xFU;
        uint8_t  mode   = nibble & 0x3U;
        uint8_t  cnf    = (nibble >> 2) & 0x3U;
        const char *mode_str = (mode == 0) ? "INPUT" :
                               (mode == 1) ? "OUT_10MHz" :
                               (mode == 2) ? "OUT_2MHz"  : "OUT_50MHz";
        const char *cnf_str;
        if (mode == 0) {
            cnf_str = (cnf == 0) ? "analog" :
                      (cnf == 1) ? "floating" :
                      (cnf == 2) ? "pull-up/dn" : "pull(rsv)";
        } else {
            cnf_str = (cnf == 0) ? "push-pull (GPIO)" :
                      (cnf == 1) ? "open-drain (GPIO)" :
                      (cnf == 2) ? "AF push-pull" : "AF open-drain";
        }
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO %s%d nibble=0x%X mode=%s cnf=%s ODR=%u IDR=%u\r\n",
            pname, bit, nibble, mode_str, cnf_str,
            (unsigned)((port->ODR >> bit) & 1U),
            (unsigned)((port->IDR >> bit) & 1U));
        USBCMD_Send(tx_scratch);

    /*
     * SETBSRR <PA|PB> <bit 0-15> <0|1>
     * Write directly to the BSRR (Bit Set/Reset Register) bypassing HAL.
     * BSRR is the atomic hardware mechanism – writing here is the lowest
     * level write possible without a debugger.
     * If ODR changes but IDR still reads 0 -> the pin is loaded externally.
     * If ODR does NOT change -> the pin is AF-controlled (timer/JTAG owns it).
     */
    } else if (strcmp(tok, "SETBSRR") == 0) {
        char *s_port = strtok(NULL, " \t");
        char *s_bit  = strtok(NULL, " \t");
        char *s_val  = strtok(NULL, " \t");
        if (!s_port || !s_bit || !s_val) {
            USBCMD_Send("ERR USAGE: SETBSRR <PA|PB> <bit 0-15> <0|1>\r\n"); return;
        }
        GPIO_TypeDef *port = NULL;
        const char   *pname = "";
        if      (strcmp(s_port, "PA") == 0) { port = GPIOA; pname = "PA"; }
        else if (strcmp(s_port, "PB") == 0) { port = GPIOB; pname = "PB"; }
        else { USBCMD_Send("ERR port must be PA or PB\r\n"); return; }
        int bit = atoi(s_bit);
        int val = atoi(s_val);
        if (bit < 0 || bit > 15) { USBCMD_Send("ERR bit must be 0-15\r\n"); return; }
        if (val != 0 && val != 1) { USBCMD_Send("ERR val must be 0 or 1\r\n"); return; }
        if (val == 1) {
            port->BSRR = (1U << bit);           /* set */
        } else {
            port->BSRR = (1U << (bit + 16));    /* reset */
        }
        /* Read back immediately */
        uint8_t odr_bit = (port->ODR >> bit) & 1U;
        uint8_t idr_bit = (port->IDR >> bit) & 1U;
        snprintf(tx_scratch, sizeof(tx_scratch),
            "OK BSRR %s%d=%d -> ODR=%u IDR=%u%s\r\n",
            pname, bit, val, odr_bit, idr_bit,
            (odr_bit != idr_bit) ? "  ** ODR!=IDR: pin loaded or AF conflict **" : "");
        USBCMD_Send(tx_scratch);

    /*
     * SETPINRAW <name> <0|1>
     * Like SETPIN but reads back ODR and IDR immediately and reports both.
     * Tells you in one command: did the write land in ODR, and does IDR agree?
     */
    } else if (strcmp(tok, "SETPINRAW") == 0) {
        char *s_name = strtok(NULL, " \t");
        char *s_val  = strtok(NULL, " \t");
        if (!s_name || !s_val) {
            USBCMD_Send("ERR USAGE: SETPINRAW <name> <0|1>\r\n"); return;
        }
        int val = atoi(s_val);
        if (val != 0 && val != 1) { USBCMD_Send("ERR val must be 0 or 1\r\n"); return; }
        const PinEntry_t *p = find_pin(s_name);
        if (!p) {
            snprintf(tx_scratch, sizeof(tx_scratch), "ERR UNKNOWN_PIN: %s\r\n", s_name);
            USBCMD_Send(tx_scratch); return;
        }
        uint8_t bit = pin_bit(p->pin);
        /* Write via BSRR (atomic, bypasses HAL) */
        if (val == 1) {
            p->port->BSRR = (1U << bit);
        } else {
            p->port->BSRR = (1U << (bit + 16));
        }
        uint8_t odr_bit = (p->port->ODR >> bit) & 1U;
        uint8_t idr_bit = (p->port->IDR >> bit) & 1U;
        snprintf(tx_scratch, sizeof(tx_scratch),
            "OK %s bit=%u wrote=%d ODR=%u IDR=%u%s\r\n",
            s_name, bit, val, odr_bit, idr_bit,
            (odr_bit != idr_bit) ? "  ** MISMATCH **" : "  OK");
        USBCMD_Send(tx_scratch);

    /*
     * AFIO
     * Dump the AFIO (Alternate Function I/O) remapping registers.
     * MAPR bits tell us if JTAG pins (PA15/PB3/PB4) have been released
     * to GPIO, and which timers have been remapped.
     * SWJ_CFG field (bits [26:24]):
     *   000 = full JTAG+SWD (PA15/PB3/PB4 locked to JTAG)
     *   010 = JTAG disabled, SWD only (PA15/PB3/PB4 free as GPIO) <- want this
     *   100 = all disabled (PA13/PA14 also released)
     */
    } else if (strcmp(tok, "AFIO") == 0) {
        uint32_t mapr  = AFIO->MAPR;
        uint32_t mapr2 = AFIO->MAPR2;
        uint8_t  swj   = (mapr >> 24) & 0x7U;
        const char *swj_str =
            (swj == 0) ? "FULL_JTAG (PA15/PB3/PB4 locked!)" :
            (swj == 1) ? "JTAG_NO_NJTRST" :
            (swj == 2) ? "SWD_ONLY (PA15/PB3/PB4 free)" :
            (swj == 4) ? "ALL_DISABLED" : "reserved";
        USBCMD_Send("INFO --- AFIO Remap Registers ---\r\n");
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO MAPR =0x%08lX\r\n", mapr);
        USBCMD_Send(tx_scratch);
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO MAPR2=0x%08lX\r\n", mapr2);
        USBCMD_Send(tx_scratch);
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO SWJ_CFG [26:24]=%u -> %s\r\n", swj, swj_str);
        USBCMD_Send(tx_scratch);
        /* Decode timer remaps that affect our pins */
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO TIM1_REMAP [7:6]=%lu  (0=no remap; 3=full remap)\r\n",
            (mapr >> 6) & 0x3U);
        USBCMD_Send(tx_scratch);
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO TIM2_REMAP [9:8]=%lu  (0=no remap)\r\n",
            (mapr >> 8) & 0x3U);
        USBCMD_Send(tx_scratch);
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO TIM3_REMAP [11:10]=%lu  (0=no; 2=partial PB4->PB0; 3=full)\r\n",
            (mapr >> 10) & 0x3U);
        USBCMD_Send(tx_scratch);
        snprintf(tx_scratch, sizeof(tx_scratch),
            "INFO TIM4_REMAP [12]=%lu  (0=no remap)\r\n",
            (mapr >> 12) & 0x1U);
        USBCMD_Send(tx_scratch);

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
        USBCMD_Send("INFO   SETPIN <name> <0|1>  (GPIO output only, not PWM pins)\r\n");
        USBCMD_Send("INFO   READPIN <name>\r\n");
        USBCMD_Send("INFO   PINS  (dump all pin states via IDR)\r\n");
        USBCMD_Send("INFO --- Debug commands ---\r\n");
        USBCMD_Send("INFO   ODRDUMP          ODR vs IDR for GPIOA+B; flags mismatches\r\n");
        USBCMD_Send("INFO   REGS <PA|PB>     Full register dump + per-pin mode decode\r\n");
        USBCMD_Send("INFO   CRLCONF <PA|PB> <bit>   Decode one pin's CRL/CRH nibble\r\n");
        USBCMD_Send("INFO   SETBSRR <PA|PB> <bit> <0|1>  Raw BSRR write + readback\r\n");
        USBCMD_Send("INFO   SETPINRAW <name> <0|1>  BSRR write by name + ODR/IDR readback\r\n");
        USBCMD_Send("INFO   AFIO             Dump AFIO remap regs (JTAG/SWD/timer remap)\r\n");
        USBCMD_Send("INFO   Pin names: M1HA/HB/HC  M2HA/HB/HC  M3HA/HB/HC\r\n");
        USBCMD_Send("INFO              M1HSA/B/C  M1LSA/B/C  (PWM, read-only)\r\n");
        USBCMD_Send("INFO              M2HSA/B/C  (PWM read-only)  M2LSA/B/C (settable)\r\n");
        USBCMD_Send("INFO              M3HSA/B/C  (PWM read-only)  M3LSA/B/C (settable)\r\n");
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

        if (c == '\r') continue;

        if (c == '\n' || c == '\0') {
            if (line_pos > 0) {
                line_buf[line_pos] = '\0';
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
        }
    }
}
