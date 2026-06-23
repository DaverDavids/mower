/* =========================================================================
 * usb_cmd.c  -  USB CDC command parser + VT100 TUI dashboard
 *
 * Two modes share the same USB CDC connection:
 *
 *   TUI mode  (default on boot) - VT100 live dashboard.  Single-key
 *             control, auto-refresh every ~100 ms.  Exit with Q.
 *
 *   CMD mode  - classic ASCII line protocol for scripting / the Python
 *             controller.  Enter with the RAW command, or Q from TUI.
 *             Every response is prefixed OK / ERR / INFO so the host
 *             can parse it reliably.
 *
 * VT100 sequences used:
 *   \x1b[2J\x1b[H   clear screen + home
 *   \x1b[r;cH       move cursor to row r, col c  (1-based)
 *   \x1b[0m         reset attributes
 *   \x1b[1m         bold
 *   \x1b[7m         reverse video
 *   \x1b[32m        green fg
 *   \x1b[31m        red fg
 *   \x1b[33m        yellow fg
 *   \x1b[36m        cyan fg
 *   \x1b[2K         erase line
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
 * RX ring buffer
 * -------------------------------------------------------------------------
 */
#define RX_BUF_SIZE  256U
static volatile uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* Line assembly (CMD mode) */
#define LINE_BUF_SIZE 128U
static char     line_buf[LINE_BUF_SIZE];
static uint16_t line_pos = 0;

/* TX scratch */
static char tx_scratch[512];

/* -------------------------------------------------------------------------
 * Mode state
 * -------------------------------------------------------------------------
 */
typedef enum { MODE_TUI = 0, MODE_CMD } AppMode_t;
static AppMode_t g_mode = MODE_TUI;

/* TUI state */
static uint8_t  tui_selected_motor = 0;   /* 0-based */
static uint8_t  tui_needs_full_redraw = 1;
static uint32_t tui_last_refresh_ms = 0;
#define TUI_REFRESH_MS  100U

/* Rate-limit for arrow-key duty changes: minimum ms between increments */
#define DUTY_KEY_RATE_MS  80U
static uint32_t tui_last_duty_key_ms = 0;

/* Escape-sequence parser for arrow / function keys */
typedef enum { ESC_IDLE, ESC_GOT_ESC, ESC_GOT_BRACKET } EscState_t;
static EscState_t esc_state = ESC_IDLE;

/* -------------------------------------------------------------------------
 * RX flush helper - discard all pending bytes in the ring buffer.
 * -------------------------------------------------------------------------
 */
static void rx_flush(void)
{
    rx_tail = rx_head;
    esc_state = ESC_IDLE;
}

/* -------------------------------------------------------------------------
 * Pin table
 * -------------------------------------------------------------------------
 */
typedef struct {
    const char   *name;
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       output;
    uint8_t       pwm_only;
} PinEntry_t;

static const PinEntry_t pin_table[] = {
    { "M1HA",  GPIOA, GPIO_PIN_0,  0, 0 },
    { "M1HB",  GPIOA, GPIO_PIN_1,  0, 0 },
    { "M1HC",  GPIOA, GPIO_PIN_2,  0, 0 },
    { "M2HA",  GPIOA, GPIO_PIN_3,  0, 0 },
    { "M2HB",  GPIOA, GPIO_PIN_4,  0, 0 },
    { "M2HC",  GPIOA, GPIO_PIN_5,  0, 0 },
    { "M3HA",  GPIOA, GPIO_PIN_6,  0, 0 },
    { "M3HB",  GPIOA, GPIO_PIN_7,  0, 0 },
    { "M3HC",  GPIOB, GPIO_PIN_10, 0, 0 },
    { "M1HSA", GPIOA, GPIO_PIN_8,  1, 1 },
    { "M1HSB", GPIOA, GPIO_PIN_9,  1, 1 },
    { "M1HSC", GPIOA, GPIO_PIN_10, 1, 1 },
    { "M1LSA", GPIOB, GPIO_PIN_13, 1, 1 },
    { "M1LSB", GPIOB, GPIO_PIN_14, 1, 1 },
    { "M1LSC", GPIOB, GPIO_PIN_15, 1, 1 },
    { "M2HSA", GPIOB, GPIO_PIN_6,  1, 1 },
    { "M2HSB", GPIOB, GPIO_PIN_7,  1, 1 },
    { "M2HSC", GPIOB, GPIO_PIN_8,  1, 1 },
    { "M2LSA", GPIOA, GPIO_PIN_15, 1, 0 },
    { "M2LSB", GPIOB, GPIO_PIN_3,  1, 0 },
    { "M2LSC", GPIOB, GPIO_PIN_5,  1, 0 },
    { "M3HSA", GPIOB, GPIO_PIN_4,  1, 1 },
    { "M3HSB", GPIOB, GPIO_PIN_0,  1, 1 },
    { "M3HSC", GPIOB, GPIO_PIN_1,  1, 1 },
    { "M3LSA", GPIOB, GPIO_PIN_9,  1, 0 },
    { "M3LSB", GPIOB, GPIO_PIN_11, 1, 0 },
    { "M3LSC", GPIOB, GPIO_PIN_12, 1, 0 },
};
#define PIN_TABLE_SIZE (sizeof(pin_table)/sizeof(pin_table[0]))

static const PinEntry_t *find_pin(const char *name)
{
    for (uint32_t i = 0; i < PIN_TABLE_SIZE; i++)
        if (strcmp(pin_table[i].name, name) == 0) return &pin_table[i];
    return NULL;
}

static uint8_t pin_bit(uint16_t mask)
{
    uint8_t b = 0;
    while (b < 16 && !((mask >> b) & 1U)) b++;
    return b;
}

/* -------------------------------------------------------------------------
 * ISR callback
 * -------------------------------------------------------------------------
 */
void USBCMD_OnReceive(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        uint16_t next = (rx_head + 1U) % RX_BUF_SIZE;
        if (next != rx_tail) { rx_buf[rx_head] = buf[i]; rx_head = next; }
    }
}

/* -------------------------------------------------------------------------
 * TX helper  (retries once if CDC busy)
 * -------------------------------------------------------------------------
 */
void USBCMD_Send(const char *msg)
{
    uint16_t len = (uint16_t)strlen(msg);
    if (CDC_Transmit_FS((uint8_t *)msg, len) == USBD_BUSY) {
        HAL_Delay(1);
        CDC_Transmit_FS((uint8_t *)msg, len);
    }
}

static void send_raw(const char *s) { USBCMD_Send(s); }

/* =========================================================================
 * TUI RENDERING
 * =========================================================================
 *
 * Fixed layout (80-column, 24-row VT100):
 *
 *  Row 1    Title bar
 *  Row 2    Blank
 *  Row 3    Motor selector tabs  [M1] [M2] [M3]
 *  Row 4    -------------------------------------------------------
 *  Row 5    Enable status + keys
 *  Row 6    Direction + key
 *  Row 7    Duty bar + up/down keys
 *  Row 8    Hall state (live)
 *  Row 9    Hall sequence log  (last TUI_HALLSEQ_SHOW transitions)
 *  Row 10   CommStep debug
 *  Row 11   Phase map + swap keys
 *  Row 12   -------------------------------------------------------
 *  Row 13   All-motors summary (compact)
 *  Row 14   GPIOB ODR/IDR raw
 *  Row 15   Blank
 *  Row 16   Key help line 1
 *  Row 17   Key help line 2
 *  Row 18   Status / last action
 */

#define TUI_ROW_TITLE    1
#define TUI_ROW_TABS     3
#define TUI_ROW_SEP1     4
#define TUI_ROW_ENABLE   5
#define TUI_ROW_DIR      6
#define TUI_ROW_DUTY     7
#define TUI_ROW_HALL     8
#define TUI_ROW_HALLMON  9
#define TUI_ROW_TICKS   10
#define TUI_ROW_MAP     11
#define TUI_ROW_SEP2    12
#define TUI_ROW_SUMMARY 13
#define TUI_ROW_GPIO    14
#define TUI_ROW_HELP1   16
#define TUI_ROW_HELP2   17
#define TUI_ROW_STATUS  18

/* How many ring entries to show in the TUI hall-seq row */
#define TUI_HALLSEQ_SHOW  20U

static void tui_goto(uint8_t row, uint8_t col)
{
    snprintf(tx_scratch, sizeof(tx_scratch), "\x1b[%u;%uH", row, col);
    send_raw(tx_scratch);
}

static void tui_erase_line(void) { send_raw("\x1b[2K"); }

static void tui_draw_bar(uint16_t val, uint16_t max_val, uint8_t width)
{
    uint8_t filled = (uint8_t)((uint32_t)val * width / max_val);
    send_raw("\x1b[32m[");
    for (uint8_t i = 0; i < width; i++)
        send_raw(i < filled ? "#" : "-");
    send_raw("]\x1b[0m");
}

static void tui_full_clear(void)
{
    send_raw("\x1b[2J\x1b[H");
}

static void tui_draw_title(void)
{
    tui_goto(TUI_ROW_TITLE, 1);
    send_raw("\x1b[1;36m  MOCO Motor Controller  ");
    send_raw("\x1b[0;33m[Q]exit TUI  [RAW]cmd mode\x1b[0m");
}

static void tui_draw_tabs(void)
{
    tui_goto(TUI_ROW_TABS, 1);
    tui_erase_line();
    for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
        if (m == tui_selected_motor)
            send_raw("\x1b[1;7m");
        else
            send_raw("\x1b[0m");
        snprintf(tx_scratch, sizeof(tx_scratch), " M%u ", m+1);
        send_raw(tx_scratch);
        send_raw("\x1b[0m ");
    }
    send_raw("  \x1b[33m[1/2/3]\x1b[0m select motor");
}

static void tui_draw_sep(uint8_t row)
{
    tui_goto(row, 1);
    send_raw("\x1b[2;37m----------------------------------------------\x1b[0m");
}

static void tui_draw_motor_panel(void)
{
    uint8_t m = tui_selected_motor;
    MotorState_t *ms = &g_motor[m];

    /* Enable */
    tui_goto(TUI_ROW_ENABLE, 1); tui_erase_line();
    send_raw(" Status: ");
    if (ms->enabled) send_raw("\x1b[1;32mENABLED \x1b[0m");
    else             send_raw("\x1b[1;31mDISABLED\x1b[0m");
    send_raw("  \x1b[33m[E]\x1b[0menable  \x1b[33m[D]\x1b[0mdisable  \x1b[33m[S]\x1b[0mstop all");

    /* Direction */
    tui_goto(TUI_ROW_DIR, 1); tui_erase_line();
    send_raw(" Dir:    ");
    if (ms->dir == DIR_FORWARD) send_raw("\x1b[32mFWD\x1b[0m");
    else                        send_raw("\x1b[33mREV\x1b[0m");
    send_raw("  \x1b[33m[F]\x1b[0mforward  \x1b[33m[R]\x1b[0mreverse");

    /* Duty bar */
    tui_goto(TUI_ROW_DUTY, 1); tui_erase_line();
    send_raw(" Duty:   ");
    tui_draw_bar(ms->duty, DUTY_MAX, 20);
    snprintf(tx_scratch, sizeof(tx_scratch),
        "  %4u/%-4u  \x1b[33m[\xe2\x86\x91]\x1b[0m+10  \x1b[33m[\xe2\x86\x93]\x1b[0m-10  "
        "\x1b[33m[PgUp]\x1b[0m+100  \x1b[33m[PgDn]\x1b[0m-100  \x1b[33m[0]\x1b[0mzero",
        ms->duty, DUTY_MAX);
    send_raw(tx_scratch);

    /* Hall - live current state */
    tui_goto(TUI_ROW_HALL, 1); tui_erase_line();
    uint8_t h = Motor_ReadHall(m);
    snprintf(tx_scratch, sizeof(tx_scratch),
        " Hall:   \x1b[1;36m0x%X\x1b[0m  HA=%u HB=%u HC=%u  ticks=\x1b[36m%ld\x1b[0m  %s",
        h, (h>>2)&1, (h>>1)&1, h&1,
        (long)ms->hall_ticks,
        (h == 0 || h == 7) ? "\x1b[1;31m[FAULT]\x1b[0m" : "");
    send_raw(tx_scratch);

    /* Hall sequence - always show the last TUI_HALLSEQ_SHOW entries from
     * the ring so the row is live on every redraw.
     *
     * We do NOT use a persistent read cursor here.  A cursor that advances
     * each frame races to head==cursor in a single redraw (one frame can
     * consume all available entries) and then shows nothing forever after.
     * Instead, every frame we simply display the most-recent entries
     * relative to the current head - the display scrolls naturally as the
     * wheel turns and new transitions push the window forward. */
    tui_goto(TUI_ROW_HALLMON, 1); tui_erase_line();
    send_raw(" HallSeq:");
    {
        HallRing_t *r = &ms->hall_ring;
        uint32_t head  = r->head;   /* snapshot */
        uint32_t count = (head < HALL_RING_LEN) ? head : HALL_RING_LEN;
        uint32_t show  = (count < TUI_HALLSEQ_SHOW) ? count : TUI_HALLSEQ_SHOW;
        uint32_t start = head - show;   /* index of oldest entry to display */

        if (show == 0U) {
            send_raw(" \x1b[2;36m--\x1b[0m");
        } else {
            for (uint32_t i = 0; i < show; i++) {
                uint8_t v = r->buf[(start + i) & HALL_RING_MASK];
                snprintf(tx_scratch, sizeof(tx_scratch), " \x1b[36m%X\x1b[0m", v);
                send_raw(tx_scratch);
            }
        }

        snprintf(tx_scratch, sizeof(tx_scratch),
            "  \x1b[2m(%lu total)\x1b[0m  \x1b[33m[C]\x1b[0mclear",
            (unsigned long)head);
        send_raw(tx_scratch);
    }

    /* Phase map */
    tui_goto(TUI_ROW_MAP, 1); tui_erase_line();
    snprintf(tx_scratch, sizeof(tx_scratch),
        " PhaseMap:[%d,%d,%d]  "
        "\x1b[33m[A]\x1b[0mnext map  \x1b[33m[Z]\x1b[0mprev map",
        ms->phase_map[0], ms->phase_map[1], ms->phase_map[2]);
    send_raw(tx_scratch);

    /* CommStep debug row */
    tui_goto(TUI_ROW_TICKS, 1); tui_erase_line();
    snprintf(tx_scratch, sizeof(tx_scratch),
        " CommStep:%u  \x1b[33m[T]\x1b[0mreset ticks",
        ms->commut_step);
    send_raw(tx_scratch);
}

/* The 6 valid phase permutations */
static const uint8_t phase_perms[6][3] = {
    {0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}
};

static uint8_t current_perm_idx[MOTOR_COUNT] = {0,0,0};

static uint8_t find_perm_idx(uint8_t m)
{
    MotorState_t *ms = &g_motor[m];
    for (uint8_t i = 0; i < 6; i++) {
        if (phase_perms[i][0] == ms->phase_map[0] &&
            phase_perms[i][1] == ms->phase_map[1] &&
            phase_perms[i][2] == ms->phase_map[2]) return i;
    }
    return 0;
}

static void tui_draw_summary(void)
{
    tui_goto(TUI_ROW_SUMMARY, 1); tui_erase_line();
    send_raw(" All: ");
    for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
        MotorState_t *ms = &g_motor[m];
        snprintf(tx_scratch, sizeof(tx_scratch),
            "M%u:%s/%s/%u  ",
            m+1,
            ms->enabled ? "\x1b[32mEN\x1b[0m" : "\x1b[31mDIS\x1b[0m",
            ms->dir == DIR_FORWARD ? "F" : "R",
            ms->duty);
        send_raw(tx_scratch);
    }
}

static void tui_draw_gpio(void)
{
    tui_goto(TUI_ROW_GPIO, 1); tui_erase_line();
    snprintf(tx_scratch, sizeof(tx_scratch),
        " GPIO: PA ODR=\x1b[36m%04lX\x1b[0m IDR=\x1b[36m%04lX\x1b[0m  "
        "PB ODR=\x1b[36m%04lX\x1b[0m IDR=\x1b[36m%04lX\x1b[0m",
        GPIOA->ODR, GPIOA->IDR, GPIOB->ODR, GPIOB->IDR);
    send_raw(tx_scratch);
}

static void tui_draw_help(void)
{
    tui_goto(TUI_ROW_HELP1, 1); tui_erase_line();
    send_raw(" \x1b[33m[1/2/3]\x1b[0mmotor  "
             "\x1b[33m[E/D]\x1b[0men/dis  "
             "\x1b[33m[F/R]\x1b[0mfwd/rev  "
             "\x1b[33m[\xe2\x86\x91\xe2\x86\x93]\x1b[0mduty  "
             "\x1b[33m[PgU/PgD]\x1b[0mduty x10");
    tui_goto(TUI_ROW_HELP2, 1); tui_erase_line();
    send_raw(" \x1b[33m[A/Z]\x1b[0mphase map  "
             "\x1b[33m[T]\x1b[0mreset ticks  "
             "\x1b[33m[C]\x1b[0mclear hall seq  "
             "\x1b[33m[S]\x1b[0mstop all  "
             "\x1b[33m[Q]\x1b[0mexit");
}

static char tui_status_msg[80] = "Ready.";

static void tui_set_status(const char *msg)
{
    strncpy(tui_status_msg, msg, sizeof(tui_status_msg)-1);
    tui_status_msg[sizeof(tui_status_msg)-1] = '\0';
}

static void tui_draw_status(void)
{
    tui_goto(TUI_ROW_STATUS, 1); tui_erase_line();
    snprintf(tx_scratch, sizeof(tx_scratch),
        " \x1b[2m%s\x1b[0m", tui_status_msg);
    send_raw(tx_scratch);
}

static void tui_full_redraw(void)
{
    tui_full_clear();
    tui_draw_title();
    tui_draw_tabs();
    tui_draw_sep(TUI_ROW_SEP1);
    tui_draw_motor_panel();
    tui_draw_sep(TUI_ROW_SEP2);
    tui_draw_summary();
    tui_draw_gpio();
    tui_draw_help();
    tui_draw_status();
    tui_goto(TUI_ROW_STATUS + 1, 1);
}

static void tui_partial_redraw(void)
{
    tui_draw_tabs();
    tui_draw_motor_panel();
    tui_draw_summary();
    tui_draw_gpio();
    tui_draw_status();
    tui_goto(TUI_ROW_STATUS + 1, 1);
}

/* =========================================================================
 * TUI keypress handler
 * =========================================================================
 */
static void tui_handle_key(uint8_t key)
{
    uint8_t m = tui_selected_motor;
    MotorState_t *ms = &g_motor[m];

    switch (key) {
    case '1': case '2': case '3':
        tui_selected_motor = key - '1';
        current_perm_idx[tui_selected_motor] = find_perm_idx(tui_selected_motor);
        tui_set_status("Motor selected.");
        tui_needs_full_redraw = 1;
        break;

    case 'e': case 'E':
        rx_flush();
        Motor_Enable(m);
        tui_set_status("Motor enabled.");
        break;
    case 'd': case 'D':
        rx_flush();
        Motor_Disable(m);
        tui_set_status("Motor disabled.");
        break;

    case 's': case 'S':
        rx_flush();
        Motor_SafeAll();
        tui_set_status("ALL STOPPED.");
        break;

    case 'f': case 'F':
        Motor_SetDir(m, DIR_FORWARD);
        tui_set_status("Direction: FORWARD.");
        break;
    case 'r': case 'R':
        Motor_SetDir(m, DIR_REVERSE);
        tui_set_status("Direction: REVERSE.");
        break;

    case '+':
    {
        uint32_t now = HAL_GetTick();
        if (now - tui_last_duty_key_ms >= DUTY_KEY_RATE_MS) {
            tui_last_duty_key_ms = now;
            uint16_t d = ms->duty + 10; if (d > DUTY_MAX) d = DUTY_MAX;
            Motor_SetDuty(m, d); tui_set_status("Duty +10.");
        }
        break;
    }
    case '-':
    {
        uint32_t now = HAL_GetTick();
        if (now - tui_last_duty_key_ms >= DUTY_KEY_RATE_MS) {
            tui_last_duty_key_ms = now;
            uint16_t d = (ms->duty >= 10) ? ms->duty - 10 : 0;
            Motor_SetDuty(m, d); tui_set_status("Duty -10.");
        }
        break;
    }
    case '0':
        rx_flush();
        Motor_SetDuty(m, 0);
        tui_set_status("Duty zeroed.");
        break;

    case 'a': case 'A':
        current_perm_idx[m] = (current_perm_idx[m] + 1) % 6;
        Motor_SetPhaseMap(m,
            phase_perms[current_perm_idx[m]][0],
            phase_perms[current_perm_idx[m]][1],
            phase_perms[current_perm_idx[m]][2]);
        snprintf(tui_status_msg, sizeof(tui_status_msg),
            "Phase map -> [%d,%d,%d] (perm %u/6)",
            phase_perms[current_perm_idx[m]][0],
            phase_perms[current_perm_idx[m]][1],
            phase_perms[current_perm_idx[m]][2],
            current_perm_idx[m]+1);
        break;
    case 'z': case 'Z':
        current_perm_idx[m] = (current_perm_idx[m] + 5) % 6;
        Motor_SetPhaseMap(m,
            phase_perms[current_perm_idx[m]][0],
            phase_perms[current_perm_idx[m]][1],
            phase_perms[current_perm_idx[m]][2]);
        snprintf(tui_status_msg, sizeof(tui_status_msg),
            "Phase map -> [%d,%d,%d] (perm %u/6)",
            phase_perms[current_perm_idx[m]][0],
            phase_perms[current_perm_idx[m]][1],
            phase_perms[current_perm_idx[m]][2],
            current_perm_idx[m]+1);
        break;

    case 't': case 'T':
        Motor_ResetTicks(m);
        tui_set_status("Ticks reset.");
        break;

    case 'c': case 'C':
        Motor_ClearHallRing(m);
        tui_set_status("Hall sequence cleared.");
        break;

    case 'q': case 'Q':
        rx_flush();
        g_mode = MODE_CMD;
        tui_full_clear();
        USBCMD_Send("INFO Entered CMD mode. Type TUI to return.\r\n");
        return;

    default: break;
    }
}

static void tui_handle_escape_final(uint8_t final_byte)
{
    uint8_t m = tui_selected_motor;
    MotorState_t *ms = &g_motor[m];
    uint32_t now = HAL_GetTick();

    switch (final_byte) {
    case 'A':
        if (now - tui_last_duty_key_ms >= DUTY_KEY_RATE_MS) {
            tui_last_duty_key_ms = now;
            uint16_t d = ms->duty + 10; if (d > DUTY_MAX) d = DUTY_MAX;
            Motor_SetDuty(m, d); tui_set_status("Duty +10.");
        }
        break;
    case 'B':
        if (now - tui_last_duty_key_ms >= DUTY_KEY_RATE_MS) {
            tui_last_duty_key_ms = now;
            uint16_t d = (ms->duty >= 10) ? ms->duty - 10 : 0;
            Motor_SetDuty(m, d); tui_set_status("Duty -10.");
        }
        break;
    case '5':
        if (now - tui_last_duty_key_ms >= DUTY_KEY_RATE_MS) {
            tui_last_duty_key_ms = now;
            uint16_t d = ms->duty + 100; if (d > DUTY_MAX) d = DUTY_MAX;
            Motor_SetDuty(m, d); tui_set_status("Duty +100.");
        }
        break;
    case '6':
        if (now - tui_last_duty_key_ms >= DUTY_KEY_RATE_MS) {
            tui_last_duty_key_ms = now;
            uint16_t d = (ms->duty >= 100) ? ms->duty - 100 : 0;
            Motor_SetDuty(m, d); tui_set_status("Duty -100.");
        }
        break;
    default: break;
    }
}

/* =========================================================================
 * TUI process  (called every main-loop iteration)
 * =========================================================================
 */
static void TUI_Process(void)
{
    /* One keypress per loop iteration - prevents arrow-key backlog from
     * monopolising CPU and blocking safety keys */
    if (rx_tail != rx_head) {
        uint8_t c = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1U) % RX_BUF_SIZE;

        if (esc_state == ESC_IDLE) {
            if (c == 0x1b) { esc_state = ESC_GOT_ESC; }
            else { tui_handle_key(c); }
        } else if (esc_state == ESC_GOT_ESC) {
            if (c == '[') { esc_state = ESC_GOT_BRACKET; }
            else { esc_state = ESC_IDLE; tui_handle_key(c); }
        } else if (esc_state == ESC_GOT_BRACKET) {
            esc_state = ESC_IDLE;
            tui_handle_escape_final(c);
        }
    }

    /* Periodic redraw */
    uint32_t now = HAL_GetTick();
    if (now - tui_last_refresh_ms >= TUI_REFRESH_MS) {
        tui_last_refresh_ms = now;
        if (tui_needs_full_redraw) {
            tui_full_redraw();
            tui_needs_full_redraw = 0;
        } else {
            tui_partial_redraw();
        }
    }
}

/* =========================================================================
 * CMD mode dispatcher
 * =========================================================================
 */
static void dispatch(char *line)
{
    char *tok = strtok(line, " \t");
    if (tok == NULL) return;

    if (strcmp(tok, "PING") == 0) {
        USBCMD_Send("OK PONG\r\n");

    } else if (strcmp(tok, "TUI") == 0) {
        g_mode = MODE_TUI;
        tui_needs_full_redraw = 1;
        tui_last_refresh_ms = 0;
        return;

    } else if (strcmp(tok, "STOP") == 0) {
        Motor_SafeAll();
        USBCMD_Send("OK ALL_STOPPED\r\n");

    } else if (strcmp(tok, "SET") == 0) {
        char *s_mid = strtok(NULL," \t"), *s_duty = strtok(NULL," \t");
        if (!s_mid||!s_duty){USBCMD_Send("ERR USAGE: SET <motor 1-3> <duty 0-1440>\r\n");return;}
        int mid=atoi(s_mid)-1, duty=atoi(s_duty);
        if(mid<0||mid>=MOTOR_COUNT||duty<0||duty>(int)DUTY_MAX){USBCMD_Send("ERR INVALID_ARG\r\n");return;}
        Motor_SetDuty((uint8_t)mid,(uint16_t)duty);
        snprintf(tx_scratch,sizeof(tx_scratch),"OK MOTOR%d DUTY=%d\r\n",mid+1,duty);
        USBCMD_Send(tx_scratch);

    } else if (strcmp(tok, "DIR") == 0) {
        char *s_mid=strtok(NULL," \t"),*s_dir=strtok(NULL," \t");
        if(!s_mid||!s_dir){USBCMD_Send("ERR USAGE: DIR <motor 1-3> <0|1>\r\n");return;}
        int mid=atoi(s_mid)-1,dir=atoi(s_dir);
        if(mid<0||mid>=MOTOR_COUNT||(dir!=0&&dir!=1)){USBCMD_Send("ERR INVALID_ARG\r\n");return;}
        Motor_SetDir((uint8_t)mid,(MotorDir_t)dir);
        snprintf(tx_scratch,sizeof(tx_scratch),"OK MOTOR%d DIR=%s\r\n",mid+1,dir==0?"FWD":"REV");
        USBCMD_Send(tx_scratch);

    } else if (strcmp(tok, "EN") == 0) {
        char *s_mid=strtok(NULL," \t");
        if(!s_mid){USBCMD_Send("ERR USAGE: EN <motor 1-3>\r\n");return;}
        int mid=atoi(s_mid)-1;
        if(mid<0||mid>=MOTOR_COUNT){USBCMD_Send("ERR INVALID_ARG\r\n");return;}
        Motor_Enable((uint8_t)mid);
        snprintf(tx_scratch,sizeof(tx_scratch),"OK MOTOR%d ENABLED\r\n",mid+1);
        USBCMD_Send(tx_scratch);

    } else if (strcmp(tok, "DIS") == 0) {
        char *s_mid=strtok(NULL," \t");
        if(!s_mid){USBCMD_Send("ERR USAGE: DIS <motor 1-3>\r\n");return;}
        int mid=atoi(s_mid)-1;
        if(mid<0||mid>=MOTOR_COUNT){USBCMD_Send("ERR INVALID_ARG\r\n");return;}
        Motor_Disable((uint8_t)mid);
        snprintf(tx_scratch,sizeof(tx_scratch),"OK MOTOR%d DISABLED\r\n",mid+1);
        USBCMD_Send(tx_scratch);

    } else if (strcmp(tok, "STATUS") == 0) {
        USBCMD_Send("INFO --- Motor Status ---\r\n");
        for(uint8_t m=0;m<MOTOR_COUNT;m++){
            MotorState_t *ms=&g_motor[m];
            snprintf(tx_scratch,sizeof(tx_scratch),
                "INFO M%d: en=%d dir=%s duty=%u hall=0x%X step=%u ticks=%ld map=[%d,%d,%d]\r\n",
                m+1,ms->enabled,ms->dir==DIR_FORWARD?"FWD":"REV",ms->duty,
                ms->hall_state,ms->commut_step,(long)ms->hall_ticks,
                ms->phase_map[0],ms->phase_map[1],ms->phase_map[2]);
            USBCMD_Send(tx_scratch);
        }
        USBCMD_Send("OK STATUS\r\n");

    } else if (strcmp(tok, "HALL") == 0) {
        for(uint8_t m=0;m<MOTOR_COUNT;m++){
            uint8_t h=Motor_ReadHall(m);
            snprintf(tx_scratch,sizeof(tx_scratch),
                "INFO M%d HALL: raw=0x%X HA=%d HB=%d HC=%d\r\n",
                m+1,h,(h>>2)&1,(h>>1)&1,h&1);
            USBCMD_Send(tx_scratch);
        }
        USBCMD_Send("OK HALL\r\n");

    } else if (strcmp(tok, "HALLSEQ") == 0) {
        char *s_mid=strtok(NULL," \t");
        if(!s_mid){USBCMD_Send("ERR USAGE: HALLSEQ <motor 1-3>\r\n");return;}
        int mid=atoi(s_mid)-1;
        if(mid<0||mid>=MOTOR_COUNT){USBCMD_Send("ERR INVALID_ARG\r\n");return;}
        HallRing_t *r=&g_motor[mid].hall_ring;
        uint32_t head=r->head;
        uint32_t count=(head<HALL_RING_LEN)?head:HALL_RING_LEN;
        uint32_t start=head-count;
        snprintf(tx_scratch,sizeof(tx_scratch),
            "INFO M%d HALLSEQ (%lu transitions):",mid+1,(unsigned long)head);
        USBCMD_Send(tx_scratch);
        for(uint32_t i=0;i<count;i++){
            snprintf(tx_scratch,sizeof(tx_scratch)," %X",r->buf[(start+i)&HALL_RING_MASK]);
            USBCMD_Send(tx_scratch);
        }
        USBCMD_Send("\r\n");
        USBCMD_Send("OK HALLSEQ\r\n");

    } else if (strcmp(tok, "CLEARRING") == 0) {
        char *s_mid=strtok(NULL," \t");
        if(!s_mid){USBCMD_Send("ERR USAGE: CLEARRING <motor 1-3>\r\n");return;}
        if(strcmp(s_mid,"ALL")==0){
            for(uint8_t m=0;m<MOTOR_COUNT;m++) Motor_ClearHallRing(m);
            USBCMD_Send("OK CLEARRING ALL\r\n");
        } else {
            int mid=atoi(s_mid)-1;
            if(mid<0||mid>=MOTOR_COUNT){USBCMD_Send("ERR INVALID_ARG\r\n");return;}
            Motor_ClearHallRing((uint8_t)mid);
            snprintf(tx_scratch,sizeof(tx_scratch),"OK CLEARRING M%d\r\n",mid+1);
            USBCMD_Send(tx_scratch);
        }

    } else if (strcmp(tok, "HALLMONITOR") == 0) {
        char *s_mid=strtok(NULL," \t");
        if(!s_mid){USBCMD_Send("ERR USAGE: HALLMONITOR <motor 1-3>\r\n");return;}
        int mid=atoi(s_mid)-1;
        if(mid<0||mid>=MOTOR_COUNT){USBCMD_Send("ERR INVALID_ARG\r\n");return;}
        USBCMD_Send("INFO HALLMONITOR running - send any key to stop\r\n");
        uint32_t last_head=g_motor[mid].hall_ring.head;
        while(rx_tail==rx_head){
            HallRing_t *r=&g_motor[mid].hall_ring;
            uint32_t cur_head=r->head;
            while(last_head!=cur_head){
                uint8_t v=r->buf[last_head&HALL_RING_MASK];
                snprintf(tx_scratch,sizeof(tx_scratch),"INFO %X\r\n",v);
                USBCMD_Send(tx_scratch);
                last_head++;
            }
            HAL_Delay(2);
        }
        rx_tail=(rx_tail+1U)%RX_BUF_SIZE;
        USBCMD_Send("OK HALLMONITOR stopped\r\n");

    } else if (strcmp(tok, "TICKS") == 0) {
        for(uint8_t m=0;m<MOTOR_COUNT;m++){
            snprintf(tx_scratch,sizeof(tx_scratch),
                "INFO M%d TICKS: %ld\r\n",m+1,(long)Motor_GetTicks(m));
            USBCMD_Send(tx_scratch);
        }
        USBCMD_Send("OK TICKS\r\n");

    } else if (strcmp(tok, "RESETTICKS") == 0) {
        char *s_mid=strtok(NULL," \t");
        if(s_mid){
            int mid=atoi(s_mid)-1;
            if(mid>=0&&mid<MOTOR_COUNT){
                Motor_ResetTicks((uint8_t)mid);
                snprintf(tx_scratch,sizeof(tx_scratch),"OK M%d TICKS_RESET\r\n",mid+1);
                USBCMD_Send(tx_scratch);
            }else USBCMD_Send("ERR INVALID_ARG\r\n");
        }else{
            for(uint8_t m=0;m<MOTOR_COUNT;m++) Motor_ResetTicks(m);
            USBCMD_Send("OK ALL_TICKS_RESET\r\n");
        }

    } else if (strcmp(tok, "MAP") == 0) {
        char *s_mid=strtok(NULL," \t"),*s_p0=strtok(NULL," \t"),
             *s_p1=strtok(NULL," \t"),*s_p2=strtok(NULL," \t");
        if(!s_mid||!s_p0||!s_p1||!s_p2){USBCMD_Send("ERR USAGE: MAP <motor 1-3> <p0> <p1> <p2>\r\n");return;}
        int mid=atoi(s_mid)-1,p0=atoi(s_p0),p1=atoi(s_p1),p2=atoi(s_p2);
        if(mid<0||mid>=MOTOR_COUNT||p0<0||p0>2||p1<0||p1>2||p2<0||p2>2){USBCMD_Send("ERR INVALID_ARG\r\n");return;}
        Motor_SetPhaseMap((uint8_t)mid,(uint8_t)p0,(uint8_t)p1,(uint8_t)p2);
        snprintf(tx_scratch,sizeof(tx_scratch),"OK M%d MAP=[%d,%d,%d]\r\n",mid+1,p0,p1,p2);
        USBCMD_Send(tx_scratch);

    } else if (strcmp(tok, "GETMAP") == 0) {
        char *s_mid=strtok(NULL," \t");
        if(!s_mid){USBCMD_Send("ERR USAGE: GETMAP <motor 1-3>\r\n");return;}
        int mid=atoi(s_mid)-1;
        if(mid<0||mid>=MOTOR_COUNT){USBCMD_Send("ERR INVALID_ARG\r\n");return;}
        MotorState_t *ms=&g_motor[mid];
        snprintf(tx_scratch,sizeof(tx_scratch),"OK M%d MAP=[%d,%d,%d]\r\n",mid+1,
            ms->phase_map[0],ms->phase_map[1],ms->phase_map[2]);
        USBCMD_Send(tx_scratch);

    } else if (strcmp(tok, "SETPIN") == 0) {
        char *s_name=strtok(NULL," \t"),*s_val=strtok(NULL," \t");
        if(!s_name||!s_val){USBCMD_Send("ERR USAGE: SETPIN <name> <0|1>\r\n");return;}
        int val=atoi(s_val);
        if(val!=0&&val!=1){USBCMD_Send("ERR VALUE must be 0 or 1\r\n");return;}
        const PinEntry_t *p=find_pin(s_name);
        if(!p){snprintf(tx_scratch,sizeof(tx_scratch),"ERR UNKNOWN_PIN: %s\r\n",s_name);USBCMD_Send(tx_scratch);return;}
        if(!p->output){snprintf(tx_scratch,sizeof(tx_scratch),"ERR PIN_IS_INPUT: %s\r\n",s_name);USBCMD_Send(tx_scratch);return;}
        if(p->pwm_only){snprintf(tx_scratch,sizeof(tx_scratch),"ERR PIN_IS_PWM: %s\r\n",s_name);USBCMD_Send(tx_scratch);return;}
        HAL_GPIO_WritePin(p->port,p->pin,val?GPIO_PIN_SET:GPIO_PIN_RESET);
        snprintf(tx_scratch,sizeof(tx_scratch),"OK %s=%d\r\n",s_name,val);
        USBCMD_Send(tx_scratch);

    } else if (strcmp(tok, "READPIN") == 0) {
        char *s_name=strtok(NULL," \t");
        if(!s_name){USBCMD_Send("ERR USAGE: READPIN <name>\r\n");return;}
        const PinEntry_t *p=find_pin(s_name);
        if(!p){snprintf(tx_scratch,sizeof(tx_scratch),"ERR UNKNOWN_PIN: %s\r\n",s_name);USBCMD_Send(tx_scratch);return;}
        int level=(HAL_GPIO_ReadPin(p->port,p->pin)==GPIO_PIN_SET)?1:0;
        snprintf(tx_scratch,sizeof(tx_scratch),"OK %s=%d (%s)\r\n",s_name,level,p->output?"OUT":"IN");
        USBCMD_Send(tx_scratch);

    } else if (strcmp(tok, "PINS") == 0) {
        USBCMD_Send("INFO --- Pin States ---\r\n");
        for(uint32_t i=0;i<PIN_TABLE_SIZE;i++){
            const PinEntry_t *p=&pin_table[i];
            int level=(HAL_GPIO_ReadPin(p->port,p->pin)==GPIO_PIN_SET)?1:0;
            snprintf(tx_scratch,sizeof(tx_scratch),"INFO %s=%d (%s%s)\r\n",
                p->name,level,p->output?"OUT":"IN",p->pwm_only?"/PWM":"");
            USBCMD_Send(tx_scratch);
        }
        USBCMD_Send("OK PINS\r\n");

    } else if (strcmp(tok, "ODRDUMP") == 0) {
        uint32_t oa=GPIOA->ODR,ia=GPIOA->IDR,ob=GPIOB->ODR,ib=GPIOB->IDR;
        USBCMD_Send("INFO --- ODR vs IDR ---\r\n");
        snprintf(tx_scratch,sizeof(tx_scratch),"INFO GPIOA ODR=0x%04lX IDR=0x%04lX DIFF=0x%04lX\r\n",oa,ia,oa^ia);
        USBCMD_Send(tx_scratch);
        snprintf(tx_scratch,sizeof(tx_scratch),"INFO GPIOB ODR=0x%04lX IDR=0x%04lX DIFF=0x%04lX\r\n",ob,ib,ob^ib);
        USBCMD_Send(tx_scratch);
        uint8_t found=0;
        for(uint32_t i=0;i<PIN_TABLE_SIZE;i++){
            const PinEntry_t *p=&pin_table[i];
            if(!p->output||p->pwm_only) continue;
            uint32_t odr=(p->port==GPIOA)?oa:ob,idr=(p->port==GPIOA)?ia:ib;
            uint8_t bit=pin_bit(p->pin);
            if(((odr>>bit)&1)&&!((idr>>bit)&1)){
                snprintf(tx_scratch,sizeof(tx_scratch),"INFO   %s bit%u ODR=1 IDR=0 STUCK\r\n",p->name,bit);
                USBCMD_Send(tx_scratch); found=1;
            }
        }
        if(!found) USBCMD_Send("INFO   (no stuck pins)\r\n");
        USBCMD_Send("OK ODRDUMP\r\n");

    } else if (strcmp(tok, "REGS") == 0) {
        char *s_port=strtok(NULL," \t");
        GPIO_TypeDef *port=NULL; const char *pn="";
        if(s_port&&strcmp(s_port,"PA")==0){port=GPIOA;pn="GPIOA";}
        else if(s_port&&strcmp(s_port,"PB")==0){port=GPIOB;pn="GPIOB";}
        else{USBCMD_Send("ERR USAGE: REGS <PA|PB>\r\n");return;}
        snprintf(tx_scratch,sizeof(tx_scratch),"INFO %s CRL=0x%08lX CRH=0x%08lX IDR=0x%04lX ODR=0x%04lX\r\n",
            pn,port->CRL,port->CRH,port->IDR,port->ODR);
        USBCMD_Send(tx_scratch);
        for(uint8_t bit=0;bit<16;bit++){
            uint32_t cr=(bit<8)?port->CRL:port->CRH;
            uint8_t sh=(bit%8)*4,nib=(cr>>sh)&0xF,mode=nib&3,cnf=(nib>>2)&3;
            const char *ms_=(mode==0)?"IN":(mode==1)?"10MHz":(mode==2)?"2MHz":"50MHz";
            const char *cs_;
            if(mode==0) cs_=(cnf==0)?"analog":(cnf==1)?"float":(cnf==2)?"pull":"pull?";
            else        cs_=(cnf==0)?"PP":(cnf==1)?"OD":(cnf==2)?"AF-PP":"AF-OD";
            snprintf(tx_scratch,sizeof(tx_scratch),
                "INFO   P%s%02u 0x%X %s/%s ODR=%lu IDR=%lu\r\n",
                pn+4,bit,nib,ms_,cs_,(port->ODR>>bit)&1,(port->IDR>>bit)&1);
            USBCMD_Send(tx_scratch);
        }
        USBCMD_Send("OK REGS\r\n");

    } else if (strcmp(tok, "CRLCONF") == 0) {
        char *sp=strtok(NULL," \t"),*sb=strtok(NULL," \t");
        if(!sp||!sb){USBCMD_Send("ERR USAGE: CRLCONF <PA|PB> <bit>\r\n");return;}
        GPIO_TypeDef *port=NULL; const char *pn="";
        if(strcmp(sp,"PA")==0){port=GPIOA;pn="PA";}
        else if(strcmp(sp,"PB")==0){port=GPIOB;pn="PB";}
        else{USBCMD_Send("ERR port PA or PB\r\n");return;}
        int bit=atoi(sb); if(bit<0||bit>15){USBCMD_Send("ERR bit 0-15\r\n");return;}
        uint32_t cr=(bit<8)?port->CRL:port->CRH;
        uint8_t sh=(uint8_t)((bit%8)*4),nib=(cr>>sh)&0xF,mode=nib&3,cnf=(nib>>2)&3;
        snprintf(tx_scratch,sizeof(tx_scratch),
            "INFO %s%d nib=0x%X mode=%u cnf=%u ODR=%lu IDR=%lu\r\n",
            pn,bit,nib,mode,cnf,(port->ODR>>bit)&1,(port->IDR>>bit)&1);
        USBCMD_Send(tx_scratch);
        USBCMD_Send("OK CRLCONF\r\n");

    } else if (strcmp(tok, "SETBSRR") == 0) {
        char *sp=strtok(NULL," \t"),*sb=strtok(NULL," \t"),*sv=strtok(NULL," \t");
        if(!sp||!sb||!sv){USBCMD_Send("ERR USAGE: SETBSRR <PA|PB> <bit> <0|1>\r\n");return;}
        GPIO_TypeDef *port=NULL; const char *pn="";
        if(strcmp(sp,"PA")==0){port=GPIOA;pn="PA";}
        else if(strcmp(sp,"PB")==0){port=GPIOB;pn="PB";}
        else{USBCMD_Send("ERR port PA or PB\r\n");return;}
        int bit=atoi(sb),val=atoi(sv);
        if(bit<0||bit>15||val<0||val>1){USBCMD_Send("ERR INVALID_ARG\r\n");return;}
        if(val) port->BSRR=(1U<<bit); else port->BSRR=(1U<<(bit+16));
        snprintf(tx_scratch,sizeof(tx_scratch),
            "OK BSRR %s%d=%d ODR=%lu IDR=%lu%s\r\n",
            pn,bit,val,(port->ODR>>bit)&1,(port->IDR>>bit)&1,
            (((port->ODR>>bit)&1)!=((port->IDR>>bit)&1))?" MISMATCH":"");
        USBCMD_Send(tx_scratch);

    } else if (strcmp(tok, "SETPINRAW") == 0) {
        char *sn=strtok(NULL," \t"),*sv=strtok(NULL," \t");
        if(!sn||!sv){USBCMD_Send("ERR USAGE: SETPINRAW <name> <0|1>\r\n");return;}
        int val=atoi(sv); if(val!=0&&val!=1){USBCMD_Send("ERR val 0 or 1\r\n");return;}
        const PinEntry_t *p=find_pin(sn);
        if(!p){snprintf(tx_scratch,sizeof(tx_scratch),"ERR UNKNOWN_PIN: %s\r\n",sn);USBCMD_Send(tx_scratch);return;}
        uint8_t bit=pin_bit(p->pin);
        if(val) p->port->BSRR=(1U<<bit); else p->port->BSRR=(1U<<(bit+16));
        snprintf(tx_scratch,sizeof(tx_scratch),
            "OK %s bit=%u wrote=%d ODR=%lu IDR=%lu%s\r\n",
            sn,bit,val,(p->port->ODR>>bit)&1,(p->port->IDR>>bit)&1,
            (((p->port->ODR>>bit)&1)!=(unsigned)val)?" MISMATCH":"");
        USBCMD_Send(tx_scratch);

    } else if (strcmp(tok, "AFIO") == 0) {
        uint32_t mapr=AFIO->MAPR;
        uint8_t swj=(mapr>>24)&7;
        const char *ss=(swj==0)?"FULL_JTAG":(swj==2)?"SWD_ONLY":(swj==4)?"ALL_DIS":"other";
        snprintf(tx_scratch,sizeof(tx_scratch),
            "INFO MAPR=0x%08lX SWJ=%u(%s) TIM1=%lu TIM2=%lu TIM3=%lu TIM4=%lu\r\n",
            mapr,swj,ss,(mapr>>6)&3,(mapr>>8)&3,(mapr>>10)&3,(mapr>>12)&1);
        USBCMD_Send(tx_scratch);
        USBCMD_Send("OK AFIO\r\n");

    } else if (strcmp(tok, "RAW") == 0) {
        USBCMD_Send("OK already in CMD mode\r\n");

    } else if (strcmp(tok, "HELP") == 0) {
        USBCMD_Send("INFO Motor: SET DIR EN DIS STATUS HALL HALLSEQ HALLMONITOR CLEARRING TICKS RESETTICKS MAP GETMAP\r\n");
        USBCMD_Send("INFO GPIO:  SETPIN READPIN PINS ODRDUMP REGS CRLCONF SETBSRR SETPINRAW AFIO\r\n");
        USBCMD_Send("INFO Mode:  TUI (return to dashboard)  RAW (stay in cmd)  PING  STOP\r\n");
        USBCMD_Send("OK HELP\r\n");

    } else {
        snprintf(tx_scratch,sizeof(tx_scratch),"ERR UNKNOWN_CMD: %s\r\n",tok);
        USBCMD_Send(tx_scratch);
    }
}

/* =========================================================================
 * CMD mode line processor
 * =========================================================================
 */
static void CMD_Process(void)
{
    while (rx_tail != rx_head) {
        uint8_t c = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1U) % RX_BUF_SIZE;
        if (c == '\r') continue;
        if (c == '\n' || c == '\0') {
            if (line_pos > 0) {
                line_buf[line_pos] = '\0';
                for (uint16_t i = 0; i < line_pos; i++)
                    if (line_buf[i]>='a'&&line_buf[i]<='z') line_buf[i]-=32;
                dispatch(line_buf);
                line_pos = 0;
            }
        } else if (line_pos < LINE_BUF_SIZE-1U) {
            line_buf[line_pos++] = (char)c;
        }
    }
}

/* =========================================================================
 * Public API
 * =========================================================================
 */
void USBCMD_Init(void)
{
    rx_head = rx_tail = line_pos = 0;
    memset(line_buf, 0, sizeof(line_buf));
    g_mode = MODE_TUI;
    tui_needs_full_redraw = 1;
    tui_last_refresh_ms = 0;
    tui_last_duty_key_ms = 0;
    HAL_Delay(500);
    tui_full_redraw();
}

void USBCMD_Process(void)
{
    if (g_mode == MODE_TUI)
        TUI_Process();
    else
        CMD_Process();
}
