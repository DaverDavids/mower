<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# The motor also stalls easily when I touch it when its rotating....  describe all code changes we need to make

The easy stall + startup issues together mean the commutation is **reactive only** — it only updates when Hall state changes, so at low speed or under load there's a long gap between commutation steps where the wrong phases stay energised, killing torque.

## All Required Code Changes

### 1. `motor_ctrl.c` — Force commutation on enable

In `Motor_Enable()`, after setting `enabled=1`, immediately apply the current Hall state so CCER/CCR are set before the first SysTick:

```c
void Motor_Enable(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) return;
    g_motor[motor_id].enabled = 1;
    g_motor[motor_id].was_enabled = 1;  // ADD: prevent all_off() race on first tick
    if (MOTOR_HW[motor_id].is_advanced)
        __HAL_TIM_MOE_ENABLE(MOTOR_HW[motor_id].htim);
    Motor_Commutate(motor_id);           // ADD: prime CCER/CCR immediately
}
```


### 2. `motor_ctrl.c` — Commutate every tick, not just on Hall change

This is the stall fix. Currently CCER/CCR are only rewritten when `new_hall != ms->hall_state`. Under load the rotor slows, Hall stays the same for many ticks, but the energisation is still correct — **except** `all_off()` is never called and CCER stays set from last transition. Actually the real problem is the opposite: the zero-all-CCR + set-active-pair runs every tick unconditionally, which is correct.

Re-reading `Motor_Commutate()` — the commutation block **does** run every tick regardless of Hall change. The Hall change block only updates the ring/ticks. So reactive-only is not the issue.

**The actual stall cause**: Motor 2 low-side is GPIO (always fully on during its step). At high duty the high-side PWM is near 100% — both high and low side near fully on simultaneously = shoot-through = current spike = voltage collapse = stall. You need to **cap effective duty for M2/M3** or add a minimum off-time.

### 3. `motor_ctrl.c` — Duty cap for non-advanced motors

```c
// In Motor_Commutate(), just before SET_COMPARE:
uint16_t effective_duty = ms->duty;
if (!hw->is_advanced && effective_duty > (DUTY_MAX * 85 / 100))
    effective_duty = DUTY_MAX * 85 / 100;  // cap at 85% to prevent shoot-through
__HAL_TIM_SET_COMPARE(hw->htim, TIM_CH[phys_high], effective_duty);
```


### 4. `motor_ctrl.c` — Fix `was_enabled` race

Change the commutation guard so `all_off()` doesn't fire spuriously on the first tick after enable:

```c
// Change:
if (!ms->enabled || ms->duty == 0) {
    if (ms->was_enabled) {
        all_off(mid);
        ms->was_enabled = 0;
    }
    return;
}
ms->was_enabled = 1;

// To:
if (!ms->enabled) {
    if (ms->was_enabled) {
        all_off(mid);
        ms->was_enabled = 0;
    }
    return;
}
if (ms->duty == 0) return;  // just return, don't call all_off
ms->was_enabled = 1;
```


### 5. `motor_ctrl.c` — Startup ramp

In `Motor_Enable()`, save the requested duty, start from a low value and ramp up over ~200ms in the commutation ISR:

Add to `MotorState_t` in `motor_ctrl.h`:

```c
uint16_t target_duty;    // requested duty
uint8_t  ramping;        // 1 = startup ramp in progress
```

In `Motor_Enable()`:

```c
g_motor[motor_id].target_duty = g_motor[motor_id].duty;
g_motor[motor_id].duty        = 100;  // start low
g_motor[motor_id].ramping     = 1;
```

In `Motor_Commutate()`, after `ms->was_enabled = 1`:

```c
if (ms->ramping) {
    if (ms->duty < ms->target_duty) {
        ms->duty += 2;  // ramp up 2 counts/ms = ~700ms to full
    } else {
        ms->duty    = ms->target_duty;
        ms->ramping = 0;
    }
}
```

And update `Motor_SetDuty()` to also update `target_duty` if already enabled:

```c
void Motor_SetDuty(uint8_t motor_id, uint16_t duty) {
    if (motor_id >= MOTOR_COUNT) return;
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    g_motor[motor_id].target_duty = duty;
    if (!g_motor[motor_id].ramping)
        g_motor[motor_id].duty = duty;
}
```


***

Want me to push all of these now?

