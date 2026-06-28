<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# looks like commutoff setting 4/5 works for both directions. still have some jitter at start though. often, after disabling the motor when it was running, it won't run again until it's manually moved. it stays still and draws large current until then

The offset 4/5 finding and the "won't restart after stop" behavior together tell us exactly what's happening.

## The Restart/High-Current Problem

When the motor stops, the rotor parks at a Hall state where the current commutation step is **directly opposing** the rotor's resting position — it's locked against the energised phases drawing max current but producing no torque (or negative torque). This is the classic **locked-rotor** condition.

The fix: when re-enabling after a stop, **force a commutation offset that pushes the rotor rather than locks it**. The simplest reliable fix is to briefly pulse through 2-3 forced commutation steps at low duty on enable, regardless of Hall state, to "unstick" the rotor before handing off to Hall-driven commutation.

## All Required Code Changes

### 1. `motor_ctrl.h` — Add forced-step startup fields

```c
uint8_t  force_steps;     // countdown of forced commutation steps remaining
uint8_t  force_step_idx;  // current index into HALL_ORDER for forced stepping
uint16_t force_duty;      // duty during forced stepping
```


### 2. `motor_ctrl.c` — Implement forced startup in `Motor_Enable()`

```c
void Motor_Enable(uint8_t motor_id) {
    if (motor_id >= MOTOR_COUNT) return;
    MotorState_t *ms = &g_motor[motor_id];
    ms->enabled     = 1;
    ms->was_enabled = 1;
    ms->force_steps    = 60;   // 60ms of forced stepping
    ms->force_step_idx = 0;
    ms->force_duty     = 200;  // low duty during forced phase
    if (MOTOR_HW[motor_id].is_advanced)
        __HAL_TIM_MOE_ENABLE(MOTOR_HW[motor_id].htim);
}
```


### 3. `motor_ctrl.c` — Forced stepping in `Motor_Commutate()`

Replace the `ms->was_enabled = 1` block with:

```c
ms->was_enabled = 1;

// Forced startup stepping — cycle through Hall order at low duty
// to unstick rotor before handing off to Hall-driven commutation
if (ms->force_steps > 0) {
    ms->force_steps--;
    // Advance one step every 10ms
    if ((ms->force_steps % 10) == 0)
        ms->force_step_idx = (ms->force_step_idx + 1) % 6;

    const CommutStep_t *table = (ms->dir == DIR_FORWARD) ? COMMUT_FWD : COMMUT_REV;
    uint8_t forced_hall = HALL_ORDER[ms->force_step_idx];
    CommutStep_t step   = table[forced_hall];
    uint8_t phys_high   = ms->phase_map[step.high];
    uint8_t phys_low    = ms->phase_map[step.low];

    for (uint8_t ch = 0; ch < 3; ch++) {
        __HAL_TIM_SET_COMPARE(hw->htim, TIM_CH[ch], 0);
        if (!hw->is_advanced && hw->ls_port[ch])
            HAL_GPIO_WritePin(hw->ls_port[ch], hw->ls_pin[ch], GPIO_PIN_RESET);
    }
    if (hw->is_advanced) {
        hw->htim->Instance->CCER &= ~(TIM1_CCER_CC1E|TIM1_CCER_CC1NE|
                                      TIM1_CCER_CC2E|TIM1_CCER_CC2NE|
                                      TIM1_CCER_CC3E|TIM1_CCER_CC3NE);
        hw->htim->Instance->CCER |= TIM1_CCER_CCE[phys_high] | TIM1_CCER_CCNE[phys_low];
    }
    __HAL_TIM_SET_COMPARE(hw->htim, TIM_CH[phys_high], ms->force_duty);
    if (!hw->is_advanced)
        HAL_GPIO_WritePin(hw->ls_port[phys_low], hw->ls_pin[phys_low], GPIO_PIN_SET);
    return;  // skip Hall-driven commutation during forced phase
}
```


### 4. `motor_ctrl.c` — Clear force_steps on disable

In `Motor_Disable()`:

```c
ms->force_steps = 0;
```


### 5. Keep commut_offset at 4

Since offset 4 works for both directions, hardcode it as the default in `Motor_Init()`:

```c
g_motor[m].commut_offset = 4;
```


***

The forced stepping acts like an open-loop kickstart — it rotates the magnetic field through one full electrical cycle at low current, which is enough to unstick the rotor from any parking position. Once `force_steps` hits 0, normal Hall-driven commutation takes over. Want me to push these changes?

