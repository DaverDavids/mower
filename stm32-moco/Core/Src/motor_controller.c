/* -----------------------------------------------------------------------
 * motor_controller.c
 * 3-channel sensorless-capable BLDC motor controller
 * Uses Hall sensors for 6-step commutation and odometry.
 *
 * Safe defaults: all motors disabled at init, PWM pulse = 0.
 * ----------------------------------------------------------------------- */

#include "motor_controller.h"
#include <string.h>

/* ---- Private timer handles ---- */
static TIM_HandleTypeDef *g_tim1;  /* Motor 0 – TIM1 CH1/1N */
static TIM_HandleTypeDef *g_tim2;  /* Motor 1 – TIM1 CH2/2N (same TIM1, alias) */
static TIM_HandleTypeDef *g_tim3;  /* Motor 2 – TIM1 CH3/3N (same TIM1, alias) */
/* Note: all three motor phases share TIM1 on this board.
   g_tim2 / g_tim3 are kept for future re-mapping flexibility. */

/* ---- Motor state array ---- */
static MotorState_t g_motors[MOTOR_COUNT];

/* ---- Pin definitions ---- */
typedef struct {
    GPIO_TypeDef *hall_port[3];
    uint16_t      hall_pin[3];
    GPIO_TypeDef *en_port;
    uint16_t      en_pin;
    uint32_t      tim_ch_hi;   /* TIM1 high-side channel */
    uint32_t      tim_ch_lo;   /* TIM1 complementary (low-side) handled by TIM1N */
} MotorPins_t;

static const MotorPins_t g_pins[MOTOR_COUNT] = {
    /* Motor 0 – Left Drive */
    {
        .hall_port = { GPIOA, GPIOA, GPIOA },
        .hall_pin  = { GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5 },
        .en_port   = GPIOB,
        .en_pin    = GPIO_PIN_3,
        .tim_ch_hi = TIM_CHANNEL_1,
        .tim_ch_lo = TIM_CHANNEL_1   /* CH1N is complementary, handled automatically */
    },
    /* Motor 1 – Right Drive */
    {
        .hall_port = { GPIOB, GPIOB, GPIOB },
        .hall_pin  = { GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12 },
        .en_port   = GPIOB,
        .en_pin    = GPIO_PIN_6,
        .tim_ch_hi = TIM_CHANNEL_2,
        .tim_ch_lo = TIM_CHANNEL_2
    },
    /* Motor 2 – Blade */
    {
        .hall_port = { GPIOB, GPIOB, GPIOB },  /* Only PB9 used; PB4/PB5 spare */
        .hall_pin  = { GPIO_PIN_9, GPIO_PIN_9, GPIO_PIN_9 },
        .en_port   = GPIOB,
        .en_pin    = GPIO_PIN_7,
        .tim_ch_hi = TIM_CHANNEL_3,
        .tim_ch_lo = TIM_CHANNEL_3
    }
};

/* ---- 6-step commutation table [hall_state][step] ----
 * For a standard 3-phase BLDC with 120-degree Hall sensors.
 * hall_state is the 3-bit value from (H3<<2 | H2<<1 | H1).
 * 0 and 7 are invalid states.
 * Each entry: {high_phase, low_phase} – simplified for single PWM channel
 * implementation; full bridge switching is done via CH/CHN pairs in TIM1.
 */
static const uint8_t g_commut_table[HALL_STATES] = {
    0xFF,  /* 0b000 – invalid */
    0,     /* 0b001 – step 0 */
    1,     /* 0b010 – step 1 */
    2,     /* 0b011 – step 2 */
    3,     /* 0b100 – step 3 */
    4,     /* 0b101 – step 4 */
    5,     /* 0b110 – step 5 */
    0xFF   /* 0b111 – invalid */
};

/* ---- Helper: set PWM duty on a TIM1 channel ---- */
static void SetChannelDuty(uint32_t channel, uint32_t duty)
{
    __HAL_TIM_SET_COMPARE(g_tim1, channel, duty);
}

/* ---- Helper: read 3-bit Hall state for a motor ---- */
static uint8_t ReadHall(uint8_t idx)
{
    uint8_t h = 0;
    h |= (HAL_GPIO_ReadPin(g_pins[idx].hall_port[0], g_pins[idx].hall_pin[0]) ? 1U : 0U);
    h |= (HAL_GPIO_ReadPin(g_pins[idx].hall_port[1], g_pins[idx].hall_pin[1]) ? 2U : 0U);
    h |= (HAL_GPIO_ReadPin(g_pins[idx].hall_port[2], g_pins[idx].hall_pin[2]) ? 4U : 0U);
    return h;
}

/* ---- Helper: apply enable pin ---- */
static void SetEnable(uint8_t idx, GPIO_PinState state)
{
    HAL_GPIO_WritePin(g_pins[idx].en_port, g_pins[idx].en_pin, state);
}

/* ======================================================================
 * MC_Init – call once after MX_TIMx_Init()
 * All motors start DISABLED, duty = 0. Safe default.
 * ====================================================================== */
void MC_Init(TIM_HandleTypeDef *tim1,
             TIM_HandleTypeDef *tim2,
             TIM_HandleTypeDef *tim3)
{
    g_tim1 = tim1;
    g_tim2 = tim2;
    g_tim3 = tim3;

    memset(g_motors, 0, sizeof(g_motors));

    /* Ensure all enable pins are LOW (motors off) */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        SetEnable(i, GPIO_PIN_RESET);
        SetChannelDuty(g_pins[i].tim_ch_hi, 0);
    }

    /* Start PWM outputs (pulse = 0, so no current flows) */
    HAL_TIM_PWM_Start(g_tim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(g_tim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(g_tim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(g_tim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(g_tim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(g_tim1, TIM_CHANNEL_3);
}

/* ---- Enable a motor (gates its EN pin high) ---- */
void MC_Enable(uint8_t idx)
{
    if (idx >= MOTOR_COUNT) return;
    if (g_motors[idx].fault) return;   /* Don't enable a faulted motor */
    g_motors[idx].enabled = 1;
    SetEnable(idx, GPIO_PIN_SET);
}

/* ---- Disable a motor (EN low, duty to 0) ---- */
void MC_Disable(uint8_t idx)
{
    if (idx >= MOTOR_COUNT) return;
    g_motors[idx].enabled = 0;
    g_motors[idx].duty    = 0;
    SetChannelDuty(g_pins[idx].tim_ch_hi, 0);
    SetEnable(idx, GPIO_PIN_RESET);
}

/* ---- Emergency stop all motors ---- */
void MC_DisableAll(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        MC_Disable(i);
    }
}

/* ---- Set PWM duty (0 – PWM_PERIOD) ---- */
void MC_SetDuty(uint8_t idx, uint32_t duty)
{
    if (idx >= MOTOR_COUNT) return;
    if (duty > PWM_MAX) duty = PWM_MAX;
    g_motors[idx].duty = duty;
    if (g_motors[idx].enabled) {
        SetChannelDuty(g_pins[idx].tim_ch_hi, duty);
    }
}

/* ---- Set direction ---- */
void MC_SetDirection(uint8_t idx, MotorDir_t dir)
{
    if (idx >= MOTOR_COUNT) return;
    g_motors[idx].direction = dir;
}

/* ======================================================================
 * MC_CommutationTick
 * Poll Hall sensors for all enabled motors.
 * On a state change: increment commut_count (odometry), log errors.
 * Call this every main loop iteration (~10-20 kHz effective).
 * ====================================================================== */
void MC_CommutationTick(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (!g_motors[i].enabled) continue;

        uint8_t hall = ReadHall(i);
        uint8_t step = g_commut_table[hall & 0x07];

        if (step == 0xFF) {
            /* Invalid Hall state */
            g_motors[i].hall_errors++;
            /* If too many errors, latch fault and kill motor */
            if (g_motors[i].hall_errors > 100) {
                g_motors[i].fault = 1;
                MC_Disable(i);
            }
            continue;
        }

        if (hall != g_motors[i].last_hall) {
            /* Valid transition – count commutation edge */
            g_motors[i].commut_count++;
            g_motors[i].last_hall = hall;
        }
    }
}

/* ---- Return a snapshot of motor state ---- */
MotorState_t MC_GetState(uint8_t idx)
{
    if (idx >= MOTOR_COUNT) {
        MotorState_t empty = {0};
        return empty;
    }
    return g_motors[idx];
}

/* ---- Clear a latched fault and re-allow enable ---- */
void MC_ClearFault(uint8_t idx)
{
    if (idx >= MOTOR_COUNT) return;
    g_motors[idx].fault       = 0;
    g_motors[idx].hall_errors = 0;
}

/* ---- Get raw commutation count (odometry) ---- */
uint32_t MC_GetCommutCount(uint8_t idx)
{
    if (idx >= MOTOR_COUNT) return 0;
    return g_motors[idx].commut_count;
}

/* ---- Reset odometry counter ---- */
void MC_ResetCommutCount(uint8_t idx)
{
    if (idx >= MOTOR_COUNT) return;
    g_motors[idx].commut_count = 0;
}
