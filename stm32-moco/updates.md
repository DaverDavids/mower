<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# check the code and add any more debug we need to figure this out. add it to the serial command line as well as the serial UI which we connect to with moco.py

Now I have the full picture. Here are the exact changes needed:

## The Confirmed Bug

In `Motor_Commutate()`, after setting CCER bits, the low-side complementary output (CCxN) on TIM1 outputs the **inverse** of its paired CCx channel. You zero all CCRs first, then set only the high-side CCR to `duty`. The low-side CCR stays at 0 — CCx is LOW 100% → CCxN is HIGH 100%. That should work... **BUT** — looking at `all_off()`, it calls `__HAL_TIM_MOE_DISABLE()`.  `Motor_Enable()` calls `__HAL_TIM_MOE_ENABLE()`. However, `Motor_Commutate()` also calls `all_off()` internally when `step.high == 0xFF`, and **crucially: when `!ms->enabled || ms->duty == 0` and `ms->was_enabled == 1`**.

The issue: there's **no `REGS TIM1` command** to read raw TIM1 registers (BDTR, CCER, CCR1-3). That's the missing debug. Everything else exists but there's no way to see TIM1's actual register state live.

***

## Exact Code Changes

### `usb_cmd.c` — Add `TIM1REGS` command

In the `dispatch()` function, add this block **before** the final `else` (unknown command) block:

```c
} else if (strcmp(tok, "TIM1REGS") == 0) {
    uint32_t ccer  = TIM1->CCER;
    uint32_t bdtr  = TIM1->BDTR;
    uint32_t ccr1  = TIM1->CCR1;
    uint32_t ccr2  = TIM1->CCR2;
    uint32_t ccr3  = TIM1->CCR3;
    uint32_t cr1   = TIM1->CR1;
    uint8_t  moe   = (bdtr >> 15) & 1;
    uint8_t  ossr  = (bdtr >> 11) & 1;
    uint8_t  ossi  = (bdtr >> 10) & 1;
    uint8_t  aoe   = (bdtr >> 14) & 1;
    snprintf(tx_scratch, sizeof(tx_scratch),
        "INFO TIM1 CR1=0x%04lX CCER=0x%04lX BDTR=0x%04lX\r\n"
        "INFO   MOE=%u OSSR=%u OSSI=%u AOE=%u\r\n"
        "INFO   CCR1=%lu CCR2=%lu CCR3=%lu\r\n"
        "INFO   CC1E=%u CC1NE=%u CC2E=%u CC2NE=%u CC3E=%u CC3NE=%u\r\n",
        cr1, ccer, bdtr,
        moe, ossr, ossi, aoe,
        ccr1, ccr2, ccr3,
        (ccer>>0)&1, (ccer>>2)&1,
        (ccer>>4)&1, (ccer>>6)&1,
        (ccer>>8)&1, (ccer>>10)&1);
    USBCMD_Send(tx_scratch);
    USBCMD_Send("OK TIM1REGS\r\n");
```

Also update the `HELP` command's INFO line for GPIO to include `TIM1REGS`:

```c
// Change this line:
USBCMD_Send("INFO GPIO:  SETPIN READPIN PINS ODRDUMP REGS CRLCONF SETBSRR SETPINRAW AFIO\r\n");
// To:
USBCMD_Send("INFO GPIO:  SETPIN READPIN PINS ODRDUMP REGS CRLCONF SETBSRR SETPINRAW AFIO TIM1REGS\r\n");
```


***

### `usb_cmd.c` — Add TUI row for TIM1 BDTR/CCER

In the layout constants block, add one row and shift the ones below it:

```c
// After TUI_ROW_GPIO = 15, add:
#define TUI_ROW_TIM1    16
// Shift these up by 1:
#define TUI_ROW_HELP1   18   // was 17
#define TUI_ROW_HELP2   19   // was 18
#define TUI_ROW_STATUS  20   // was 19
```

Add a new `tui_draw_tim1()` function after `tui_draw_gpio()`:

```c
static void tui_draw_tim1(void)
{
    tui_goto(TUI_ROW_TIM1, 1); tui_erase_line();
    uint32_t ccer = TIM1->CCER, bdtr = TIM1->BDTR;
    uint8_t moe  = (bdtr >> 15) & 1;
    uint8_t ossr = (bdtr >> 11) & 1;
    uint8_t ossi = (bdtr >> 10) & 1;
    snprintf(tx_scratch, sizeof(tx_scratch),
        " TIM1: MOE=%u OSSR=%u OSSI=%u  CCER=0x%04lX"
        "  CC1E=%u CC1NE=%u CC2E=%u CC2NE=%u CC3E=%u CC3NE=%u"
        "  CCR1=%lu CCR2=%lu CCR3=%lu",
        moe, ossr, ossi, ccer,
        (ccer>>0)&1, (ccer>>2)&1,
        (ccer>>4)&1, (ccer>>6)&1,
        (ccer>>8)&1, (ccer>>10)&1,
        TIM1->CCR1, TIM1->CCR2, TIM1->CCR3);
    /* Highlight MOE=0 as a fault */
    if (!moe) send_raw("\x1b[1;31m");
    send_raw(tx_scratch);
    if (!moe) send_raw(" [MOE=0!]\x1b[0m");
    else send_raw("\x1b[0m");
}
```

Then call it in both `tui_full_redraw()` and `tui_partial_redraw()`, right after `tui_draw_gpio()`:

```c
// In tui_full_redraw() and tui_partial_redraw(), after tui_draw_gpio():
tui_draw_tim1();
```


***

### What to look for after flashing

Run `TIM1REGS` in CMD mode (or watch the new TIM1 row in TUI) while Motor 1 is enabled and duty > 0. The smoking gun will be one of:


| Symptom | Cause |
| :-- | :-- |
| `MOE=0` while enabled | `all_off()` disabled MOE and something prevented `Motor_Enable()` re-enabling it |
| `CC1NE=CC2NE=CC3NE=0` | CCER not being set — commutation not reaching the active step |
| `CCR1=CCR2=CCR3=0` | Duty not being applied — `duty == 0` guard firing |
| `OSSR=0` or `OSSI=0` | BDTR user code not running (CubeMX regenerated over it) |

