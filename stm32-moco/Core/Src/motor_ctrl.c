/* =========================================================================
 * motor_ctrl.c  –  3-axis BLDC motor controller (Blue Pill / STM32F103C8)
 *
 * Safety contract:
 *   1. Motor_Init() is called after all HAL timer inits.
 *   2. All PWM channels start with Pulse=0 and timers are NOT started
 *      until Motor_Enable() is called – outputs remain LOW at power-on.
 *   3. Motor_SafeAll() can be called at any time to cut all outputs.
 *
 * Commutation tables (6-step, Hall → phase pattern):
 *   Index = 3-bit Hall state (HA<<2 | HB<<1 | HC).
 *   Entry = {high_ch, low_ch}  (channel index 0-2 → CH1/CH2/CH3).
 *   Invalid states 0b000 and 0b111 map to STOP (step 255).
 *
 * Commutation offset:
 *   commut_offset (0-5, displayed 1-6) rotates the lookup table by N
 *   steps relative to the hall state.  Use Motor_SetCommutOffset() /
 *   COMMUTOFFSET cmd to find the correct alignment for a given motor
 *   without reflashing.  The canonical forward sequence for the valid
 *   hall states is:
 *     index: 0  1  2  3  4  5
 *     hall:  1  3  2  6  4  5
 *   offset=0 means hall state drives its own table row (default).
 *   offset=1 means each hall state drives the pattern that would
 *   normally belong to the *next* state in the forward sequence, etc.
 *
 * Pin assignments (source of truth: project-and-pins.txt):
 *   Motor 1 – TIM1: HS=A8/A9/A10, LS=B13/B14/B15 (complementary)
 *   Motor 2 – TIM4: HS=B6/B7/B8,  LS enables=A15/B3/B5 (GPIO)
 *   Motor 3 – TIM3: HS=B4/B0/B1,  LS enables=B9/B11/B12 (GPIO)
 *   Hall M1: A0/A1/A2  Hall M2: A3/A4/A5  Hall M3: A6/A7/B10
 * =========================================================================
 */

#include "motor_ctrl.h"
#include "main.h"
#include <string.h>

/* -------------------------------------------------------------------------
 * External timer handles (declared in main.c)
 * -------------------------------------------------------------------------
 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* -------------------------------------------------------------------------
 * Commutation lookup table
 * hall_state (0-7) → {high_side_channel, low_side_channel}
 * channel 0 = CH1, 1 = CH2, 2 = CH3.  0xFF = invalid/stop.
 * Standard BLDC forward sequence – reverse is mirrored.
 * -------------------------------------------------------------------------
 */
typedef struct { uint8_t high; uint8_t low; } CommutStep_t;

static const CommutStep_t COMMUT_FWD[8] = {
    {0xFF, 0xFF},  /* 0b000 – invalid           */
    {1,    2   },  /* 0b001 – Hall=1: CH2H, CH3L */
    {0,    1   },  /* 0b010 – Hall=2: CH1H, CH2L */
    {0,    2   },  /* 0b011 – Hall=3: CH1H, CH3L */
    {2,    0   },  /* 0b100 – Hall=4: CH3H, CH1L */
    {2,    1   },  /* 0b101 – Hall=5: CH3H, CH2L */
    {1,    0   },  /* 0b110 – Hall=6: CH2H, CH1L */
    {0xFF, 0xFF},  /* 0b111 – invalid           */
};

static const CommutStep_t COMMUT_REV[8] = {
    {0xFF, 0xFF},  /* 0b000 – invalid           */
    {2,    1   },  /* 0b001 */
    {1,    0   },  /* 0b010 */
    {2,    0   },  /* 0b011 */
    {0,    2   },  /* 0b100 */
    {1,    2   },  /* 0b101 */
    {0,    1   },  /* 0b110 */
    {0xFF, 0xFF},  /* 0b111 – invalid           */
};

/*
 * Canonical forward hall-state order (the 6 valid states in sequence).
 * Used by Motor_Commutate to apply commut_offset: we find the position
 * of new_hall in this array, advance by offset, then look up the table
 * entry for the resulting hall state.
 */
static const uint8_t HALL_ORDER[6] = {1, 3, 2, 6, 4, 5};

/* Timer channel IDs indexed 0-2 */
static const uint32_t TIM_CH[3] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3
};

/*
 * TIM1 CCER bit masks for each channel pair.
 * CCxE  = main (high-side) output enable
 * CCxNE = complementary (low-side) output enable
 */
#define TIM1_CCER_CC1E   (1U << 0)
#define TIM1_CCER_CC1NE  (1U << 2)
#define TIM1_CCER_CC2E   (1U << 4)
#define TIM1_CCER_CC2NE  (1U << 6)
#define TIM1_CCER_CC3E   (1U << 8)
#define TIM1_CCER_CC3NE  (1U << 10)

static const uint16_t TIM1_CCER_CCE[3]  = { TIM1_CCER_CC1E,  TIM1_CCER_CC2E,  TIM1_CCER_CC3E  };
static const uint16_t TIM1_CCER_CCNE[3] = { TIM1_CCER_CC1NE, TIM1_CCER_CC2NE, TIM1_CCER_CC3NE };

/* -------------------------------------------------------------------------
 * Motor hardware descriptors
 * -------------------------------------------------------------------------
 */
typedef struct {
    TIM_HandleTypeDef *htim;
    GPIO_TypeDef *ls_port[3];
    uint16_t      ls_pin[3];
    GPIO_TypeDef *hall_port[3];
    uint16_t      hall_pin[3];
    uint8_t       is_advanced;
} MotorHW_t;

static const MotorHW_t MOTOR_HW[MOTOR_COUNT] = {
    {
        .htim       = &htim1,
        .ls_port    = {NULL, NULL, NULL},
        .ls_pin     = {0, 0, 0},
        .hall_port  = {GPIOA, GPIOA, GPIOA},
        .hall_pin   = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2},
        .is_advanced = 1
    },
    {
        .htim       = &htim4,
        .ls_port    = {GPIOA, GPIOB, GPIOB},
        .ls_pin     = {GPIO_PIN_15, GPIO_PIN_3, GPIO_PIN_5},
        .hall_port  = {GPIOA, GPIOA, GPIOA},
        .hall_pin   = {GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5},
        .is_advanced = 0
    },
    {
        .htim       = &htim3,
        .ls_port    = {GPIOB, GPIOB, GPIOB},
        .ls_pin     = {GPIO_PIN_9, GPIO_PIN_11, GPIO_PIN_12},
        .hall_port  = {GPIOA, GPIOA, GPIOB},
        .hall_pin   = {GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_10},
        .is_advanced = 0
    }
};

/* -------------------------------------------------------------------------
 * Global state
 * -------------------------------------------------------------------------
 */
MotorState_t g_motor[MOTOR_COUNT];

/* -------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------
 */
static void all_off(uint8_t mid)
{
    const MotorHW_t *hw = &MOTOR_HW[mid];
    for (uint8_t ch = 0; ch < 3; ch++) {
        __HAL_TIM_SET_COMPARE(hw->htim, TIM_CH[ch], 0);
        if (!hw->is_advanced && hw->ls_port[ch] != NULL) {
            HAL_GPIO_WritePin(hw->ls_port[ch], hw->ls_pin[ch], GPIO_PIN_RESET);
        }
    }
    if (hw->is_advanced) {
        hw->htim->Instance->CCER &= ~(TIM1_CCER_CC1E  | TIM1_CCER_CC1NE |
                                      TIM1_CCER_CC2E  | TIM1_CCER_CC2NE |
                                      TIM1_CCER_CC3E  | TIM1_CCER_CC3NE);
        __HAL_TIM_MOE_DISABLE(hw->htim);
    }
}

/* -------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------
 */

void Motor_Init(void)
{
    for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
        g_motor[m].enabled       = 0;
        g_motor[m].was_enabled   = 0;
        g_motor[m].dir           = DIR_FORWARD;
        g_motor[m].duty          = 0;
        g_motor[m].force_steps   = 0;
        g_motor[m].force_step_idx= 0;
        g_motor[m].force_duty    = 0;
        g_motor[m].hall_state    = 0;
        g_motor[m].commut_step   = 0;
        g_motor[m].commut_offset = 0;
        g_motor[m].hall_ticks    = 0;
        if (m == 1) {
            g_motor[m].phase_map[0] = 1;
            g_motor[m].phase_map[1] = 0;
            g_motor[m].phase_map[2] = 2;
            g_motor[m].commut_offset = 3;
        } else if (m == 2) {
            g_motor[m].phase_map[0] = 0;
            g_motor[m].phase_map[1] = 2;
            g_motor[m].phase_map[2] = 1;
            g_motor[m].commut_offset = 0;
        } else {
            g_motor[m].phase_map[0] = 0;
            g_motor[m].phase_map[1] = 1;
            g_motor[m].phase_map[2] = 2;
        }
        g_motor[m].force_step_ticks   = 0;
        g_motor[m].stall_count        = 0;
        g_motor[m].last_tick_snapshot = 0;
        g_motor[m].last_tick_time     = 0;
        g_motor[m].last_hall_time_ms  = 0;
        g_motor[m].hall_period_ms     = 0;
        g_motor[m].tick_time_idx      = 0;
        g_motor[m].in_stall_recovery  = 0;
        g_motor[m].estimated_ticks    = 0;
        memset(g_motor[m].tick_times, 0, sizeof(g_motor[m].tick_times));
        memset(&g_motor[m].hall_ring, 0, sizeof(HallRing_t));

        HAL_TIM_PWM_Start(MOTOR_HW[m].htim, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(MOTOR_HW[m].htim, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(MOTOR_HW[m].htim, TIM_CHANNEL_3);

        if (MOTOR_HW[m].is_advanced) {
            HAL_TIMEx_PWMN_Start(MOTOR_HW[m].htim, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Start(MOTOR_HW[m].htim, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Start(MOTOR_HW[m].htim, TIM_CHANNEL_3);
            /* BDTR (OSSR/OSSI/AutomaticOutput) configured in MX_TIM1_Init USER CODE */
        }

        all_off(m);
    }

}

void Motor_SafeAll(void)
{
    for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
        g_motor[m].enabled     = 0;
        g_motor[m].was_enabled = 0;
        g_motor[m].duty        = 0;
        all_off(m);
    }
}

void Motor_SetDuty(uint8_t motor_id, uint16_t duty)
{
    if (motor_id >= MOTOR_COUNT) return;
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    g_motor[motor_id].duty = duty;
}

void Motor_SetDir(uint8_t motor_id, MotorDir_t dir)
{
    if (motor_id >= MOTOR_COUNT) return;
    g_motor[motor_id].dir = dir;
}

void Motor_Enable(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) return;
    MotorState_t *ms = &g_motor[motor_id];
    ms->enabled     = 1;
    ms->was_enabled = 1;
    ms->force_steps     = 60;
    {
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
    }
    ms->force_step_ticks= 0;
    ms->force_duty      = (ms->duty > 0) ? ms->duty : 200;
    ms->stall_count        = 0;
    ms->in_stall_recovery  = 0;
    ms->last_tick_snapshot = ms->hall_ticks;
    ms->estimated_ticks    = ms->hall_ticks;
    ms->last_tick_time     = HAL_GetTick();
    if (MOTOR_HW[motor_id].is_advanced) {
        MOTOR_HW[motor_id].htim->Instance->CCER &= ~(
            TIM1_CCER_CC1E|TIM1_CCER_CC1NE|
            TIM1_CCER_CC2E|TIM1_CCER_CC2NE|
            TIM1_CCER_CC3E|TIM1_CCER_CC3NE);
        __HAL_TIM_MOE_ENABLE(MOTOR_HW[motor_id].htim);
    }
}

void Motor_Disable(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) return;
    g_motor[motor_id].enabled     = 0;
    g_motor[motor_id].was_enabled = 0;
    g_motor[motor_id].duty        = 0;
    g_motor[motor_id].force_steps = 0;
    all_off(motor_id);
}

void Motor_SetCommutOffset(uint8_t motor_id, uint8_t offset)
{
    if (motor_id >= MOTOR_COUNT) return;
    if (offset > 5) offset = 5;
    g_motor[motor_id].commut_offset = offset;
}

uint8_t Motor_ReadHall(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) return 0;
    const MotorHW_t *hw = &MOTOR_HW[motor_id];
    uint8_t ha = HAL_GPIO_ReadPin(hw->hall_port[0], hw->hall_pin[0]) ? 1U : 0U;
    uint8_t hb = HAL_GPIO_ReadPin(hw->hall_port[1], hw->hall_pin[1]) ? 1U : 0U;
    uint8_t hc = HAL_GPIO_ReadPin(hw->hall_port[2], hw->hall_pin[2]) ? 1U : 0U;
    return (uint8_t)((ha << 2) | (hb << 1) | hc);
}

int32_t Motor_GetTicks(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) return 0;
    return g_motor[motor_id].hall_ticks;
}

void Motor_ResetTicks(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) return;
    g_motor[motor_id].hall_ticks = 0;
}

void Motor_ClearHallRing(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) return;
    memset(&g_motor[motor_id].hall_ring, 0, sizeof(HallRing_t));
    g_motor[motor_id].hall_state = 0;
}

void Motor_SetPhaseMap(uint8_t motor_id, uint8_t p0, uint8_t p1, uint8_t p2)
{
    if (motor_id >= MOTOR_COUNT) return;
    if (p0 > 2 || p1 > 2 || p2 > 2) return;
    g_motor[motor_id].phase_map[0] = p0;
    g_motor[motor_id].phase_map[1] = p1;
    g_motor[motor_id].phase_map[2] = p2;
}

/* -------------------------------------------------------------------------
 * 6-step commutation for one motor.
 * Call from SysTick ISR only (not main loop).
 *
 * commut_offset rotates which table entry is used for each hall state.
 * offset=0: table[hall] (default, no rotation)
 * offset=N: table[HALL_ORDER[(pos_of_hall_in_HALL_ORDER + N) % 6]]
 * This lets you shift the energisation pattern forward or backward
 * relative to the rotor position to find the correct alignment.
 * -------------------------------------------------------------------------
 */
void Motor_Commutate(uint8_t mid)
{
    if (mid >= MOTOR_COUNT) return;

    MotorState_t    *ms = &g_motor[mid];
    const MotorHW_t *hw = &MOTOR_HW[mid];

    uint8_t new_hall = Motor_ReadHall(mid);

    uint8_t prev_hall = ms->hall_state;

    /* Record every genuine Hall transition into the ring buffer. */
    if (new_hall != ms->hall_state && new_hall != 0U && new_hall != 7U) {
        HallRing_t *r = &ms->hall_ring;
        r->buf[r->head & HALL_RING_MASK] = new_hall;
        r->head++;
        if (ms->dir == DIR_FORWARD)
            ms->hall_ticks++;
        else
            ms->hall_ticks--;
        ms->estimated_ticks = ms->hall_ticks;
        uint32_t now = HAL_GetTick();
        uint32_t interval = now - ms->last_hall_time_ms;
        if (ms->last_hall_time_ms != 0U) {
            ms->tick_times[ms->tick_time_idx % 6] = interval;
            ms->tick_time_idx++;
            ms->hall_period_ms = (uint16_t)interval;
        }
        ms->last_hall_time_ms = now;
        ms->hall_state = new_hall;
    }

    if (!ms->enabled) {
        if (ms->was_enabled) {
            all_off(mid);
            ms->was_enabled = 0;
        }
        return;
    }

    ms->was_enabled = 1;

    /* Keep MOE asserted on every tick while the motor is enabled.
     * Must come before the duty==0 guard: if duty is 0 we still want
     * MOE set so the next non-zero duty takes effect immediately without
     * a race against the SysTick reading duty before it is written. */
    if (hw->is_advanced)
        __HAL_TIM_MOE_ENABLE(hw->htim);

    if (ms->duty == 0) return;

    /* Forced startup stepping — duty-dependent timeout, exit on hall move */
    if (ms->force_steps > 0) {
        ms->force_step_ticks++;

        uint16_t step_timeout = 20 + (uint16_t)((DUTY_MAX - ms->force_duty) / 72);
        uint8_t hall_moved = (new_hall != prev_hall && new_hall != 0U && new_hall != 7U);
        uint8_t timed_out  = (ms->force_step_ticks > step_timeout);

        if (hall_moved) {
            ms->force_step_ticks = 0;
            ms->force_steps      = 0;
        } else if (timed_out) {
            ms->force_step_ticks = 0;
            ms->force_steps--;
            if (ms->dir == DIR_FORWARD) {
                ms->force_step_idx = (ms->force_step_idx + 1) % 6;
                ms->estimated_ticks++;
            } else {
                ms->force_step_idx = (ms->force_step_idx + 5) % 6;
                ms->estimated_ticks--;
            }
        }
        MotorDir_t eff = (mid == 1 || mid == 2) ? (ms->dir == DIR_FORWARD ? DIR_REVERSE : DIR_FORWARD) : ms->dir;
        const CommutStep_t *tbl = (eff == DIR_FORWARD) ? COMMUT_FWD : COMMUT_REV;
        uint8_t forced_hall = HALL_ORDER[ms->force_step_idx];
        CommutStep_t step   = tbl[forced_hall];
        uint8_t f_high      = ms->phase_map[step.high];
        uint8_t f_low       = ms->phase_map[step.low];
        for (uint8_t ch = 0; ch < 3; ch++) {
            __HAL_TIM_SET_COMPARE(hw->htim, TIM_CH[ch], 0);
            if (!hw->is_advanced && hw->ls_port[ch] != NULL)
                HAL_GPIO_WritePin(hw->ls_port[ch], hw->ls_pin[ch], GPIO_PIN_RESET);
        }
        if (hw->is_advanced) {
            hw->htim->Instance->CCER &= ~(TIM1_CCER_CC1E|TIM1_CCER_CC1NE|
                                          TIM1_CCER_CC2E|TIM1_CCER_CC2NE|
                                          TIM1_CCER_CC3E|TIM1_CCER_CC3NE);
            hw->htim->Instance->CCER |= TIM1_CCER_CCE[f_high] | TIM1_CCER_CCNE[f_low];
        }
        __HAL_TIM_SET_COMPARE(hw->htim, TIM_CH[f_high], ms->force_duty);
        if (!hw->is_advanced)
            HAL_GPIO_WritePin(hw->ls_port[f_low], hw->ls_pin[f_low], GPIO_PIN_SET);
        return;
    }

    MotorDir_t effective_dir = ms->dir;
    if (mid == 1 || mid == 2) effective_dir = (ms->dir == DIR_FORWARD) ? DIR_REVERSE : DIR_FORWARD;
    const CommutStep_t *table = (effective_dir == DIR_FORWARD) ? COMMUT_FWD : COMMUT_REV;

    /* Apply commutation offset: find current hall state in HALL_ORDER,
     * advance by commut_offset steps, use the resulting hall state as
     * the table lookup key.  Falls back to direct lookup for invalid
     * states (0 and 7) regardless of offset. */
    uint8_t lookup_hall = new_hall;
    if (ms->commut_offset != 0 && new_hall != 0U && new_hall != 7U) {
        uint8_t pos = 0;
        for (uint8_t i = 0; i < 6; i++) {
            if (HALL_ORDER[i] == new_hall) { pos = i; break; }
        }
        uint8_t effective_pos;
        if (effective_dir == DIR_FORWARD) {
            effective_pos = (pos + ms->commut_offset) % 6;
        } else {
            effective_pos = (pos + 6 - ms->commut_offset) % 6;
        }
        lookup_hall = HALL_ORDER[effective_pos];
    }

    CommutStep_t step = table[lookup_hall];

    if (step.high == 0xFF) {
        all_off(mid);
        ms->was_enabled = 0;
        return;
    }

    uint8_t phys_high = ms->phase_map[step.high];
    uint8_t phys_low  = ms->phase_map[step.low];

    /* Zero all CCRs and GPIO low-sides first */
    for (uint8_t ch = 0; ch < 3; ch++) {
        __HAL_TIM_SET_COMPARE(hw->htim, TIM_CH[ch], 0);
        if (!hw->is_advanced && hw->ls_port[ch] != NULL) {
            HAL_GPIO_WritePin(hw->ls_port[ch], hw->ls_pin[ch], GPIO_PIN_RESET);
        }
    }

    if (hw->is_advanced) {
        /* Clear ALL CCxE/CCxNE bits, then enable only the active pair */
        hw->htim->Instance->CCER &= ~(TIM1_CCER_CC1E  | TIM1_CCER_CC1NE |
                                      TIM1_CCER_CC2E  | TIM1_CCER_CC2NE |
                                      TIM1_CCER_CC3E  | TIM1_CCER_CC3NE);
        hw->htim->Instance->CCER |= TIM1_CCER_CCE[phys_high] | TIM1_CCER_CCNE[phys_low];
    }

    uint16_t effective_duty = ms->duty;
    if (!hw->is_advanced && effective_duty > (DUTY_MAX * 85 / 100))
        effective_duty = DUTY_MAX * 85 / 100;
    __HAL_TIM_SET_COMPARE(hw->htim, TIM_CH[phys_high], effective_duty);

    if (!hw->is_advanced) {
        HAL_GPIO_WritePin(hw->ls_port[phys_low], hw->ls_pin[phys_low], GPIO_PIN_SET);
    }

    ms->commut_step = step.high;
}

void Motor_CommutateAll(void)
{
    for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
        Motor_Commutate(m);
    }
}

void Motor_CheckStall(uint8_t mid, uint32_t now_ms)
{
    if (mid >= MOTOR_COUNT) return;
    MotorState_t *ms = &g_motor[mid];
    if (!ms->enabled || ms->duty == 0) return;
    if (ms->force_steps > 0) {
        ms->last_tick_snapshot = ms->hall_ticks;
        ms->last_tick_time     = now_ms;
        return;
    }

    if (ms->hall_ticks != ms->last_tick_snapshot) {
        ms->last_tick_snapshot = ms->hall_ticks;
        ms->last_tick_time     = now_ms;
        ms->stall_count        = 0;
        ms->in_stall_recovery  = 0;
        return;
    }

    uint32_t stall_ms = now_ms - ms->last_tick_time;
    if (stall_ms > 80) {
        ms->stall_count++;
        if (ms->stall_count <= 6) {
            ms->in_stall_recovery = 1;
            uint8_t current_pos = 0;
            for (uint8_t i = 0; i < 6; i++) {
                if (HALL_ORDER[i] == ms->hall_state) { current_pos = i; break; }
            }
            if (ms->dir == DIR_FORWARD)
                ms->force_step_idx = (current_pos + 1) % 6;
            else
                ms->force_step_idx = (current_pos + 5) % 6;

            ms->force_steps      = 15;
            ms->force_step_ticks = 0;
            ms->force_duty       = ms->duty < 700 ? 700 : ms->duty;
            ms->last_tick_time   = now_ms;
        } else {
            Motor_Disable(mid);
        }
    }
}

uint8_t Motor_GetTimingStats(uint8_t mid,
                              uint32_t *out_min,
                              uint32_t *out_max,
                              uint32_t *out_mean,
                              uint32_t *out_ripple_pct)
{
    if (mid >= MOTOR_COUNT) return 0;
    MotorState_t *ms = &g_motor[mid];
    if (ms->tick_time_idx < 6) return 0;

    uint32_t mn = 0xFFFFFFFF, mx = 0, sum = 0;
    for (uint8_t i = 0; i < 6; i++) {
        uint32_t v = ms->tick_times[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
    }
    uint32_t mean = sum / 6;
    uint32_t ripple = (mean > 0) ? ((mx - mn) * 100 / mean) : 0;

    *out_min        = mn;
    *out_max        = mx;
    *out_mean       = mean;
    *out_ripple_pct = ripple;
    return 1;
}
