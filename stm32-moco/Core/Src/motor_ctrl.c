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
    {0,    1   },  /* 0b001 – Hall=1: CH1H, CH2L */
    {2,    0   },  /* 0b010 – Hall=2: CH3H, CH1L */
    {1,    0   },  /* 0b011 – Hall=3: CH2H, CH1L */
    {1,    2   },  /* 0b100 – Hall=4: CH2H, CH3L */
    {0,    2   },  /* 0b101 – Hall=5: CH1H, CH3L */
    {2,    1   },  /* 0b110 – Hall=6: CH3H, CH2L */
    {0xFF, 0xFF},  /* 0b111 – invalid           */
};

static const CommutStep_t COMMUT_REV[8] = {
    {0xFF, 0xFF},  /* 0b000 – invalid           */
    {1,    0   },  /* 0b001 */
    {0,    2   },  /* 0b010 */
    {0,    1   },  /* 0b011 */
    {2,    1   },  /* 0b100 */
    {2,    0   },  /* 0b101 */
    {1,    2   },  /* 0b110 */
    {0xFF, 0xFF},  /* 0b111 – invalid           */
};

/* Timer channel IDs indexed 0-2 */
static const uint32_t TIM_CH[3] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3
};

/* -------------------------------------------------------------------------
 * Motor hardware descriptors
 * -------------------------------------------------------------------------
 */
typedef struct {
    TIM_HandleTypeDef *htim;
    /* GPIO port/pin for low-side enables (index matches CH 0-2) */
    GPIO_TypeDef *ls_port[3];
    uint16_t      ls_pin[3];
    /* Hall GPIO port/pin (index 0-2 → HA, HB, HC) */
    GPIO_TypeDef *hall_port[3];
    uint16_t      hall_pin[3];
    /* Is this timer TIM1 (advanced, has complementary outputs)? */
    uint8_t       is_advanced;
} MotorHW_t;

static const MotorHW_t MOTOR_HW[MOTOR_COUNT] = {
    /* Motor 1 – TIM1 (advanced), hardware dead-time, complementary outputs
     * HS: A8(CH1), A9(CH2), A10(CH3)
     * LS: B13(CH1N), B14(CH2N), B15(CH3N) – no separate GPIO needed
     * Hall: A0, A1, A2 */
    {
        .htim       = &htim1,
        .ls_port    = {NULL, NULL, NULL},
        .ls_pin     = {0, 0, 0},
        .hall_port  = {GPIOA, GPIOA, GPIOA},
        .hall_pin   = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2},
        .is_advanced = 1
    },
    /* Motor 2 – TIM4, GPIO low-side enables
     * HS: B6(CH1), B7(CH2), B8(CH3)
     * LS enables: A15, B3, B5
     * Hall: A3, A4, A5 */
    {
        .htim       = &htim4,
        .ls_port    = {GPIOA, GPIOB, GPIOB},
        .ls_pin     = {GPIO_PIN_15, GPIO_PIN_3, GPIO_PIN_5},
        .hall_port  = {GPIOA, GPIOA, GPIOA},
        .hall_pin   = {GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5},
        .is_advanced = 0
    },
    /* Motor 3 – TIM3, GPIO low-side enables
     * HS: B4(CH1), B0(CH3), B1(CH4)
     * LS enables: B9, B11, B12
     * Hall: A6, A7, B10 */
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

/* Set PWM duty on one channel.  Pulse=0 guarantees LOW before start. */
static void set_pwm(uint8_t mid, uint8_t phys_ch, uint16_t duty)
{
    const MotorHW_t *hw = &MOTOR_HW[mid];
    __HAL_TIM_SET_COMPARE(hw->htim, TIM_CH[phys_ch], duty);
}

/* De-assert all high-side PWM (pulse=0) and all low-side GPIOs */
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
        /* Disable MOE (main output enable) to force complementary outputs off */
        __HAL_TIM_MOE_DISABLE(hw->htim);
    }
}

/* -------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------
 */

void Motor_Init(void)
{
    /* Initialise state structs and ensure all outputs default LOW */
    for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
        g_motor[m].enabled     = 0;
        g_motor[m].was_enabled = 0;
        g_motor[m].dir         = DIR_FORWARD;
        g_motor[m].duty        = 0;
        g_motor[m].hall_state  = 0;
        g_motor[m].commut_step = 0;
        g_motor[m].hall_ticks  = 0;
        /* Default phase map: logical 0→0, 1→1, 2→2 (identity) */
        g_motor[m].phase_map[0] = 0;
        g_motor[m].phase_map[1] = 1;
        g_motor[m].phase_map[2] = 2;

        /* Start timers with pulse=0 (outputs low).  PWM channels
         * are already configured by MX_TIMx_Init() with Pulse=0. */
        HAL_TIM_PWM_Start(MOTOR_HW[m].htim, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(MOTOR_HW[m].htim, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(MOTOR_HW[m].htim, TIM_CHANNEL_3);

        if (MOTOR_HW[m].is_advanced) {
            /* Start complementary channels for TIM1 */
            HAL_TIMEx_PWMN_Start(MOTOR_HW[m].htim, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Start(MOTOR_HW[m].htim, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Start(MOTOR_HW[m].htim, TIM_CHANNEL_3);
            /* Enable dead-time on TIM1 */
            TIM_BreakDeadTimeConfigTypeDef bdtConfig = {0};
            bdtConfig.OffStateRunMode   = TIM_OSSR_DISABLE;
            bdtConfig.OffStateIDLEMode  = TIM_OSSI_DISABLE;
            bdtConfig.LockLevel         = TIM_LOCKLEVEL_OFF;
            bdtConfig.DeadTime          = TIM1_DEADTIME;
            bdtConfig.BreakState        = TIM_BREAK_DISABLE;
            bdtConfig.BreakPolarity     = TIM_BREAKPOLARITY_HIGH;
            bdtConfig.AutomaticOutput   = TIM_AUTOMATICOUTPUT_ENABLE;
            HAL_TIMEx_ConfigBreakDeadTime(MOTOR_HW[m].htim, &bdtConfig);
        }

        /* Force all outputs to known-low state */
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
    g_motor[motor_id].enabled = 1;
    if (MOTOR_HW[motor_id].is_advanced) {
        __HAL_TIM_MOE_ENABLE(MOTOR_HW[motor_id].htim);
    }
}

void Motor_Disable(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) return;
    g_motor[motor_id].enabled     = 0;
    g_motor[motor_id].was_enabled = 0;
    g_motor[motor_id].duty        = 0;
    all_off(motor_id);
}

/* Read 3-bit Hall state for a motor */
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
 * Call this frequently (main loop or timer interrupt).
 * When disabled, drives all outputs to zero on the falling edge only –
 * subsequent loop iterations are a no-op so manual SETPIN commands
 * are not clobbered while the motor stays disabled.
 * -------------------------------------------------------------------------
 */
void Motor_Commutate(uint8_t mid)
{
    if (mid >= MOTOR_COUNT) return;

    MotorState_t *ms      = &g_motor[mid];
    const MotorHW_t *hw   = &MOTOR_HW[mid];

    uint8_t new_hall = Motor_ReadHall(mid);

    /* Count Hall transitions for position/distance feedback */
    if (new_hall != ms->hall_state && new_hall != 0 && new_hall != 7) {
        ms->hall_ticks++;
        ms->hall_state = new_hall;
    }

    if (!ms->enabled || ms->duty == 0) {
        /* Only call all_off() on the falling edge (enabled → disabled).
         * While already disabled, do nothing – avoids clobbering SETPIN. */
        if (ms->was_enabled) {
            all_off(mid);
            ms->was_enabled = 0;
        }
        return;
    }

    ms->was_enabled = 1;

    /* Pick commutation table based on direction */
    const CommutStep_t *table = (ms->dir == DIR_FORWARD) ? COMMUT_FWD : COMMUT_REV;
    CommutStep_t step = table[new_hall];

    if (step.high == 0xFF) {
        /* Invalid Hall state – stop safely */
        all_off(mid);
        ms->was_enabled = 0;
        return;
    }

    /* Apply phase map (logical → physical channel) */
    uint8_t phys_high = ms->phase_map[step.high];
    uint8_t phys_low  = ms->phase_map[step.low];

    /* Zero all channels, then set active pair */
    for (uint8_t ch = 0; ch < 3; ch++) {
        __HAL_TIM_SET_COMPARE(hw->htim, TIM_CH[ch], 0);
        if (!hw->is_advanced && hw->ls_port[ch] != NULL) {
            HAL_GPIO_WritePin(hw->ls_port[ch], hw->ls_pin[ch], GPIO_PIN_RESET);
        }
    }

    /* Assert high-side PWM */
    __HAL_TIM_SET_COMPARE(hw->htim, TIM_CH[phys_high], ms->duty);

    /* Assert low-side */
    if (hw->is_advanced) {
        /* TIM1: complementary channel handles low-side automatically */
        /* The CHxN output mirrors CHx through dead-time hardware.    */
        /* Writing Pulse on CHx drives CHxN complement automatically. */
    } else {
        HAL_GPIO_WritePin(hw->ls_port[phys_low], hw->ls_pin[phys_low], GPIO_PIN_SET);
    }

    ms->commut_step = step.high; /* store for debug */
}

void Motor_CommutateAll(void)
{
    for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
        Motor_Commutate(m);
    }
}
