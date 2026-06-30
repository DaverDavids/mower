<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# okay I just now uploaded the changes... it is quite a bit better now. but still it sometimes stalls on startup and takes some pulses until it moves. do you know why?

Status: DISABLED   [E]enable  [D]disable  [S]stop all
Dir:    FWD   [F]forward  [R]reverse
Duty:   [\#\#\#\#\#\#\#\#\#\#\#---------]   800/1440  [Up/Dn]+/-10  [PgU/PgD]+/-100  [0]zero
Hall:   0x5  HA=1 HB=0 HC=1
HallSeq: 1 5 1 5 1 5 1 5 1 5 1 5 1 5 1 5 1 5 1 5 1 5 1 5 1 5   (3957 ticks)  [C]clear  [M]monitor
Ticks:  3957   [T]reset
PhaseMap: [1,0,2]  perm 3/6  [A]next  [Z]prev
CommutOff: 4/5  [O]next  [I]prev
--------------------------------------------------
All: M1:DIS/F/0/off1  M2:DIS/F/800/off4  M3:DIS/F/0/off5
GPIO: PA ODR=20FF IDR=3069  PB ODR=0410 IDR=0400
TIM1: MOE=0 CH1 HS=off LS=off CCR=0     CH2 HS=off LS=off CCR=0     CH3 HS=off LS=off CCR=0
--------------------------------------------------
[1/2/3]motor  [E/D]en/dis  [F/R]fwd/rev  [Up/Dn]duty  [PgU/D]duty*10
[A/Z]phase map  [O/I]commut offset  [T]ticks  [`]timing  [~]clr timing  [C]hall  [M]monitor  [P]pin test  [S]stop  [Q]quit
Duty 800

Good — I can see the updated code. The `prev_hall` fix is in  and `force_duty` no longer has the 400 floor. But the remaining startup stall issue is clear from a careful read. Here are the exact causes:

## Why It Still Pulses/Stalls on Startup

### Problem 1: `force_step_idx` always starts at 0, ignoring rotor position

In `Motor_Enable()` :

```c
ms->force_step_idx = 0;  // always HALL_ORDER[0] = state 1
```

`HALL_ORDER[0]` = hall state `1`. But your rotor is sitting at hall state `0x5` when you enable. The forced sequence starts energizing the wrong phase pair for that rotor position — it may be a braking step or a backwards step depending on where hall 1 is relative to hall 5 in your motor's winding geometry. It has to "walk" through potentially several wrong steps before it stumbles onto one that actually produces forward torque, which is why you see the pulses before movement.

**Fix:** Seed `force_step_idx` from the current Hall state at enable time, just like `Motor_CheckStall()` already does for recovery:

```c
uint8_t current_pos = 0;
uint8_t current_hall = Motor_ReadHall(motor_id);
for (uint8_t i = 0; i < 6; i++) {
    if (HALL_ORDER[i] == current_hall) { current_pos = i; break; }
}
// Start one step ahead in the forward direction
ms->force_step_idx = (current_pos + 1) % 6;
```

This way the very first forced step is already the correct *next* phase for the rotor's actual position.

***

### Problem 2: `step_timeout` is calculated from `force_duty` but the formula assumes full ARR

```c
uint16_t step_timeout = 20 + (uint16_t)((DUTY_MAX - ms->force_duty) / 72);
```

With `force_duty = 800` and `DUTY_MAX = 1440`, that's `20 + (640/72)` = `20 + 8` = **28ms per forced step**. At 60 steps that's up to 1.68 seconds of forced open-loop stepping before it gives up. Each wrong step is holding the wrong phase energized for 28ms, which can actually push the rotor backwards or lock it depending on position — multiple wrong steps in sequence cause the "pulse" feel.

This timeout is meant to handle the case where the rotor *doesn't move* (no Hall transition), but 28ms is a long time per wrong step. Reducing it to something like `10 + (DUTY_MAX - ms->force_duty) / 144` at higher duties would make bad steps shorter, though the real fix is Problem 1 above.

***

### Problem 3: The `hall_moved` exit from forced stepping doesn't account for commut_offset

When the Hall moves during forced stepping, the code jumps straight to hall-based commutation . But your motor has `commut_offset = 3` (displayed 4/5). The transition from forced-step indexing to offset-based commutation can cause a phase glitch at the handoff moment — the forced step is using the raw `COMMUT_FWD/REV` table directly, but the normal path applies the offset. These are different table lookups for the same hall state.

This isn't a major issue but it can cause a single hiccup on the transition. Not a fix needed immediately, but worth noting.

***

## The One-Line Core Fix

In `Motor_Enable()`, replace `ms->force_step_idx = 0;` with:

```c
uint8_t cur_hall = Motor_ReadHall(motor_id);
uint8_t cur_pos = 0;
for (uint8_t i = 0; i < 6; i++) {
    if (HALL_ORDER[i] == cur_hall) { cur_pos = i; break; }
}
MotorDir_t eff_dir = (motor_id == 1) ?
    (g_motor[motor_id].dir == DIR_FORWARD ? DIR_REVERSE : DIR_FORWARD) :
    g_motor[motor_id].dir;
ms->force_step_idx = (eff_dir == DIR_FORWARD) ?
    (cur_pos + 1) % 6 : (cur_pos + 5) % 6;
```

Note you need the same M2 direction flip that the commutate path already applies, otherwise for M2 the forced sequence walks the wrong way. That's the complete fix — the stall recovery code in `Motor_CheckStall()` already does this correctly; `Motor_Enable()` just needs to match it.

