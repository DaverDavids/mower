<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# check mower repository in github and look in the stm32-moco folder. we are implementing a motor controller but I'm having issues. read the project-and-pins.txt file to first understand the architecture. our recent commits to fix motor driving have made it a bit worse... it pulses on startup every time now, so the fixes to start stall didn't help. the timing addition only sometimes works if I spam the backtick key... if I press it once it always seems to say "no data yet need 6+ transitions". but here is some of the data:

Status: ENABLED    [E]enable  [D]disable  [S]stop all
Dir:    FWD   [F]forward  [R]reverse
Duty:   [\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#----]  1200/1440  [Up/Dn]+/-10  [PgU/PgD]+/-100  [0]zero
Hall:   0x3  HA=0 HB=1 HC=1
HallSeq: 4 5 1 3 2 6 4 5 1 3 2 6 4 5 1 3 2 6 4 5 1 3 2 6 4 5 1 3 2 6 4 5 1 3 2 6 4 5 1 3 2 6 4 5 1 3 2 6 4 5 1 3 2 6 4 5 1 3 2 6 4 5 1 3   (9294 ticks)  [C]
Ticks:  9294   [T]reset
PhaseMap: [1,0,2]  perm 3/6  [A]next  [Z]prev
CommutOff: 4/5  [O]next  [I]prev
--------------------------------------------------
All: M1:DIS/F/0/off1  M2:EN/F/1200/off4  M3:DIS/F/0/off5
GPIO: PA ODR=20FF IDR=3049  PB ODR=0418 IDR=0508
TIM1: MOE=0 CH1 HS=off LS=off CCR=0     CH2 HS=off LS=off CCR=0     CH3 HS=off LS=off CCR=0
--------------------------------------------------
[1/2/3]motor  [E/D]en/dis  [F/R]fwd/rev  [Up/Dn]duty  [PgU/D]duty*10
[A/Z]phase map  [O/I]commut offset  [T]ticks  [`]timing  [~]clr timing  [C]hall  [M]monitor  [P]pin test  [S]stop  [Q]quit
M2 TIMING intervals(ms): 3 3 4 4 4 3

M2 TIMING intervals(ms): 3 4 3 4 4 4

M2 TIMING intervals(ms): 3 5 3 4 4 4

M2 TIMING intervals(ms): 3 4 4 4 3 5

I've now read all three source files thoroughly. Here's a detailed breakdown of the bugs I found:

***

## Issue 1: Startup Pulse (The Biggest Problem)

**Root cause: `force_steps` fires unconditionally on every `Motor_Enable()` call, even from stall recovery.**

In `Motor_Enable()` , `force_steps` is hard-set to 60 and `force_duty` is clamped to a minimum of 400. This means every time you press `E`, the motor gets 60ms of open-loop forced commutation at up to 400 duty regardless of rotor state — that's your pulse on startup.

The real problem is the force-start duty is **always** `min(duty, 400)` but with a floor of 400:

```c
ms->force_duty = (ms->duty > 0 && ms->duty < 400) ? ms->duty : 400;
```

If your duty is above 400 (yours is 1200), this clamps it to exactly 400, not the actual duty. So the motor lurches at 400 for 60ms, then jumps to 1200. That's the pulse.

**Fix:** Either remove the 400 floor so `force_duty` matches actual duty, or at startup use the actual duty directly:

```c
ms->force_duty = (ms->duty > 0) ? ms->duty : 200;
```


***

## Issue 2: `hall_moved` Check Is Always False During Forced Steps

This is a logic bug in `Motor_Commutate()`  in the force-step block:

```c
uint8_t hall_moved = (new_hall != ms->hall_state && new_hall != 0U && new_hall != 7U);
```

But `ms->hall_state` is **only updated in the ring buffer section above**, which only runs if `new_hall != ms->hall_state`. So by the time you reach `hall_moved`, `ms->hall_state` has already been updated — meaning `new_hall == ms->hall_state` is now always true, and `hall_moved` is **always 0**. The forced sequence never exits early when the Hall actually moves; it runs all 60 steps every single time.

**Fix:** Save the old hall state before the ring buffer update:

```c
uint8_t prev_hall = ms->hall_state;
// ... ring buffer update sets ms->hall_state ...
// then in the force block:
uint8_t hall_moved = (new_hall != prev_hall && new_hall != 0U && new_hall != 7U);
```


***

## Issue 3: Backtick Timing — "No data yet / need 6+ transitions"

The backtick key in TUI mode is not mapped in the TUI handler at all . Looking at `tui_handle_key()`, there's no `case '`':`. The backtick does nothing in TUI mode. You're getting the timing output because the motor is generating enough Hall transitions that when you spam it, occasionally you've already accumulated 6+ since the last clear — but the key isn't actually triggering a firmware call, it's just printing whatever the TUI dashboard refresh happens to show at the right moment.

The TUI has no inline timing display — the `M2 TIMING intervals(ms)` output you're seeing is coming from the periodic TUI partial redraw which apparently includes timing. But looking at `tui_draw_motor_panel()`, there's **no timing row rendered there**. So that output is leaking from somewhere else — possibly from `moco.py` , not from the firmware TUI itself.

**What to do for the backtick:** You either need to add a `case '`':`handler in`tui_handle_key()`that calls`Motor_GetTimingStats()`and sends the result, or switch to CMD mode with`Q`and use the`TIMING 2` command directly.

***

## Summary of Fixes

| Bug | Location | Fix |
| :-- | :-- | :-- |
| Startup pulse | `Motor_Enable()` in `motor_ctrl.c` | Remove the `force_duty=400` floor; use actual duty |
| force-step never exits on Hall move | `Motor_Commutate()` force block | Save `prev_hall` before ring buffer write; use it for `hall_moved` check |
| Backtick does nothing in TUI | `tui_handle_key()` in `usb_cmd.c` | Add `case '`':`handler or use CMD mode`TIMING 2` |

The timing data you shared (3–5ms intervals, ~3.6ms mean) looks healthy for M2 spinning — the commutation is well-aligned. The issue is purely the startup behavior described above.

