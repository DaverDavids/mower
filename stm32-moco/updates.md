<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# update stale comments and other sections, help me fix all other issues you see. Describe exact code changes to make, be very concise/terse but precise

Now I have everything I need. Here is the complete, precise change list.

***

## File-by-file fixes

### `project-and-pins.txt` — stale "Known bugs" section

The three bugs listed are already fixed. Replace the entire block :

```
Known bugs (unfixed as of 2026-06-24):
    1. Motor_CommutateAll() called from both SysTick ISR and main() ...
    2. TIM1 CCER not cleared for inactive phases ...
    3. Motor_Init() BDTR: OSSR/OSSI=DISABLE, AutomaticOutput=ENABLE ...
```

With:

```
Fixed bugs (resolved 2026-06-27):
    1. Motor_CommutateAll() is now called exclusively from SysTick_Handler.
       main() while-loop no longer calls it (race condition eliminated).
    2. TIM1 CCER: all CCxE/CCxNE bits are cleared each step before enabling
       only the active high/low pair (shoot-through eliminated).
    3. Motor_Init() BDTR now sets OSSR=ENABLE, OSSI=ENABLE,
       AutomaticOutput=DISABLE (Bug 3 fix applied in MX_TIM1_Init USER CODE).
```


***

### `main.c` — wrong GPIO init comments

**Change 1** — wrong low-side label in `WritePin` comment :

```c
// WRONG:
/*Configure GPIO pin Output Level - PB3=M3LSA, PB5=M2LSC, PB9=M3LSB, PB11=M3LSC, PB12=M2LSB */

// CORRECT:
/*Configure GPIO pin Output Level - PB3=M2LSB, PB5=M2LSC, PB9=M3LSA, PB11=M3LSB, PB12=M3LSC */
```

**Change 2** — wrong label in `GPIO_Init` comment below it :

```c
// WRONG:
  /*Configure GPIO pins : PB3 PB5 PB9 PB11 PB12 (low-side enables, plain GPIO output)
   * PB3=M3LSA, PB5=M2LSC (fix: was missing), PB9=M3LSB, PB11=M3LSC, PB12=M2LSB */

// CORRECT:
  /*Configure GPIO pins : PB3 PB5 PB9 PB11 PB12 (low-side enables, plain GPIO output)
   * PB3=M2LSB, PB5=M2LSC, PB9=M3LSA, PB11=M3LSB, PB12=M3LSC */
```

These are comment-only — the actual `GPIO_PIN_x` values and `MOTOR_HW` descriptor in `motor_ctrl.c` are already correct .

***

### `stm32f1xx_it.c` — `SysTick_Handler` ordering

`Motor_CommutateAll()` fires before `HAL_IncTick()`, which means any `HAL_Delay()` call in progress (e.g. boot delay) can produce a commutation tick with HAL time not yet incremented — irrelevant at runtime, but creates subtle ordering risk . Move it after:

```c
// BEFORE:
void SysTick_Handler(void) {
    Motor_CommutateAll();   // runs before HAL tick
    HAL_IncTick();

// AFTER:
void SysTick_Handler(void) {
    HAL_IncTick();          // increment HAL timebase first
    Motor_CommutateAll();   // then commutate (consistent 1 kHz, after tick is valid)
```

Also update the comment block above it — remove the "Bug 1 fix" wording since that bug is history:

```c
 * Motor_CommutateAll() runs exclusively here at 1 kHz.
 * HAL_IncTick() is called first so HAL_GetTick() reflects the
 * current tick when Motor_Commutate reads Hall state.
 * Must NOT also be called from the main loop.
```


***

### `motor_ctrl.h` — stale comment on `Motor_Commutate`

The public API comment says :

```c
/* ----- Commutation (call from main loop / TIM ISR) --------------------- */
```

Change to:

```c
/* ----- Commutation (call from SysTick ISR only, NOT main loop) --------- */
```


***

### `motor_ctrl.c` — `Motor_Commutate` doc block ordering note

The function doc block says :

```c
 * Call from SysTick ISR only (not main loop – see Bug 1 fix).
```

Change to:

```c
 * Call from SysTick ISR only (not main loop).
```


***

### `motor_ctrl.c` — `Motor_Init()` redundant BDTR call

`Motor_Init()` calls `HAL_TIMEx_ConfigBreakDeadTime()` with `OSSR_ENABLE/OSSI_ENABLE/AutomaticOutput_DISABLE` for Motor 1 . `MX_TIM1_Init()` in `main.c` already does the exact same thing in its `USER CODE END TIM1_Init 2` block , so `Motor_Init` is doing it twice. Remove it from `Motor_Init()` — the `main.c` USER CODE block is the right place since it survives CubeMX regeneration:

```c
// In Motor_Init(), REMOVE this entire block:
if (MOTOR_HW[m].is_advanced) {
    HAL_TIMEx_PWMN_Start(MOTOR_HW[m].htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(MOTOR_HW[m].htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(MOTOR_HW[m].htim, TIM_CHANNEL_3);
    TIM_BreakDeadTimeConfigTypeDef bdtConfig = {0};
    bdtConfig.OffStateRunMode   = TIM_OSSR_ENABLE;
    ...
    HAL_TIMEx_ConfigBreakDeadTime(MOTOR_HW[m].htim, &bdtConfig);
}
```

**Keep** the `HAL_TIMEx_PWMN_Start` calls — just move the BDTR config out. Final result for that block in `Motor_Init()`:

```c
if (MOTOR_HW[m].is_advanced) {
    HAL_TIMEx_PWMN_Start(MOTOR_HW[m].htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(MOTOR_HW[m].htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(MOTOR_HW[m].htim, TIM_CHANNEL_3);
    /* BDTR (OSSR/OSSI/AutomaticOutput) configured in MX_TIM1_Init USER CODE */
}
```


***

## Summary of what's left for hardware debugging

All code issues are now clean. The remaining vibration-without-rotation is almost certainly a **commutation offset problem** — use `HALLMONITOR` while spinning the wheel by hand and confirm you see all 6 distinct states (1,2,3,4,5,6) cycling. If any state is missing or you see 0/7, you still have a hall wire issue that no offset value can fix.

