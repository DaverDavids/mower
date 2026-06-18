/**
 * motor_controller.c
 * 3-axis BLDC commutation for STM32F103C8T6 Blue Pill
 *
 * Safe default: all PWM outputs at 0, all enable lines LOW on init.
 * Commutation is driven by Hall sensor polling in MC_CommutationTick().
 */

#include "motor_controller.h"

/* ============================================================
 * Internal handles
 * ============================================================ */
static TIM_HandleTypeDef *g_htim1 = NULL; /* Motor 1 - advanced timer */
static TIM_HandleTypeDef *g_htim3 = NULL; /* Motor 3 */
static TIM_HandleTypeDef *g_htim4 = NULL; /* Motor 2 */

static MotorState_t g_motors[MOTOR_COUNT];

/* ============================================================
 * 6-step commutation tables
 *
 * Index = Hall state (1..6). Hall state 0 and 7 are invalid.
 * Each entry: { hi_ch_on, lo_ch_on } where bits represent channels.
 * For Motor 1 (TIM1): high-side = CH1/CH2/CH3, low-side = CH1N/CH2N/CH3N.
 * For M2/M3 (TIM4/TIM3): high-side channels, low-side = GPIO enable pins.
 *
 * Table format per state [hall_1..6]:
 *   [0] = phase A high, phase B low  (forward step 1)
 *   [1] = phase A high, phase C low  (forward step 2)
 *   ... standard 6-step sequence
 *
 * Bit mask for channels: bit0=CH1, bit1=CH2, bit2=CH3
 * ============================================================ */

typedef struct {
    uint8_t pwm_ch;   /* bitmask: which high-side PWM channel is active */
    uint8_t en_ch;    /* bitmask: which low-side enable/CHN is active   */
} CommutStep_t;

/* Standard 6-step BLDC forward commutation table indexed by Hall[2:0]
 * Hall states 0 and 7 are fault states - all off.
 * Adjust phase mapping to match your specific motor wiring. */
static const CommutStep_t commut_fwd[8] = {
    /* Hall 0 - INVALID */ { 0x00, 0x00 },
    /* Hall 1 (001)     */ { 0x01, 0x02 }, /* A_hi, B_lo */
    /* Hall 2 (010)     */ { 0x04, 0x01 }, /* C_hi, A_lo */
    /* Hall 3 (011)     */ { 0x04, 0x02 }, /* C_hi, B_lo */
    /* Hall 4 (100)     */ { 0x02, 0x04 }, /* B_hi, C_lo */
    /* Hall 5 (101)     */ { 0x01, 0x04 }, /* A_hi, C_lo */
    /* Hall 6 (110)     */ { 0x02, 0x01 }, /* B_hi, A_lo */
    /* Hall 7 - INVALID */ { 0x00, 0x00 },
};

/* Reverse table: mirror of forward */
static const CommutStep_t commut_rev[8] = {
    /* Hall 0 - INVALID */ { 0x00, 0x00 },
    /* Hall 1 (001)     */ { 0x02, 0x01 },
    /* Hall 2 (010)     */ { 0x01, 0x04 },
    /* Hall 3 (011)     */ { 0x02, 0x04 },
    /* Hall 4 (100)     */ { 0x04, 0x02 },
    /* Hall 5 (101)     */ { 0x04, 0x01 },
    /* Hall 6 (110)     */ { 0x01, 0x02 },
    /* Hall 7 - INVALID */ { 0x00, 0x00 },
};

/* ============================================================
 * GPIO pin definitions for enable lines and Hall sensors
 * ============================================================ */

/* Motor 2 enable lines (low-side) */
#define M2_EN_A_PORT    GPIOA
#define M2_EN_A_PIN     GPIO_PIN_15   /* A15 */
#define M2_EN_B_PORT    GPIOB
#define M2_EN_B_PIN     GPIO_PIN_3    /* B3  */
#define M2_EN_C_PORT    GPIOB
#define M2_EN_C_PIN     GPIO_PIN_5    /* B5  */

/* Motor 3 enable lines (low-side) */
#define M3_EN_A_PORT    GPIOB
#define M3_EN_A_PIN     GPIO_PIN_9    /* B9  */
#define M3_EN_B_PORT    GPIOB
#define M3_EN_B_PIN     GPIO_PIN_11   /* B11 */
#define M3_EN_C_PORT    GPIOB
#define M3_EN_C_PIN     GPIO_PIN_12   /* B12 */

/* Hall sensor pins */
#define M1_HALL_A_PORT  GPIOA
#define M1_HALL_A_PIN   GPIO_PIN_0
#define M1_HALL_B_PORT  GPIOA
#define M1_HALL_B_PIN   GPIO_PIN_1
#define M1_HALL_C_PORT  GPIOA
#define M1_HALL_C_PIN   GPIO_PIN_2

#define M2_HALL_A_PORT  GPIOA
#define M2_HALL_A_PIN   GPIO_PIN_3
#define M2_HALL_B_PORT  GPIOA
#define M2_HALL_B_PIN   GPIO_PIN_4
#define M2_HALL_C_PORT  GPIOA
#define M2_HALL_C_PIN   GPIO_PIN_5

#define M3_HALL_A_PORT  GPIOA
#define M3_HALL_A_PIN   GPIO_PIN_6
#define M3_HALL_B_PORT  GPIOA
#define M3_HALL_B_PIN   GPIO_PIN_7
#define M3_HALL_C_PORT  GPIOB
#define M3_HALL_C_PIN   GPIO_PIN_10

/* ============================================================
 * Private helpers
 * ============================================================ */

static uint8_t read_hall(uint8_t motor)
{
    uint8_t state = 0;
    switch (motor) {
        case MOTOR_1:
            if (HAL_GPIO_ReadPin(M1_HALL_A_PORT, M1_HALL_A_PIN)) state |= 0x01;
            if (HAL_GPIO_ReadPin(M1_HALL_B_PORT, M1_HALL_B_PIN)) state |= 0x02;
            if (HAL_GPIO_ReadPin(M1_HALL_C_PORT, M1_HALL_C_PIN)) state |= 0x04;
            break;
        case MOTOR_2:
            if (HAL_GPIO_ReadPin(M2_HALL_A_PORT, M2_HALL_A_PIN)) state |= 0x01;
            if (HAL_GPIO_ReadPin(M2_HALL_B_PORT, M2_HALL_B_PIN)) state |= 0x02;
            if (HAL_GPIO_ReadPin(M2_HALL_C_PORT, M2_HALL_C_PIN)) state |= 0x04;
            break;
        case MOTOR_3:
            if (HAL_GPIO_ReadPin(M3_HALL_A_PORT, M3_HALL_A_PIN)) state |= 0x01;
            if (HAL_GPIO_ReadPin(M3_HALL_B_PORT, M3_HALL_B_PIN)) state |= 0x02;
            if (HAL_GPIO_ReadPin(M3_HALL_C_PORT, M3_HALL_C_PIN)) state |= 0x04;
            break;
        default:
            break;
    }
    return state;
}

/* Set Motor 1 commutation step via TIM1 complementary PWM */
static void apply_commut_m1(const CommutStep_t *step, uint32_t duty)
{
    /* Stop all channels first */
    __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_3, 0);

    if (step->pwm_ch == 0 && step->en_ch == 0) return; /* fault state */

    /* Apply high-side PWM duty to the active channel */
    if (step->pwm_ch & 0x01) __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_1, duty);
    if (step->pwm_ch & 0x02) __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_2, duty);
    if (step->pwm_ch & 0x04) __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_3, duty);

    /* Low-side CHN channels are complementary - driven by TIM1 hardware
     * The complementary output is always active when main channel is off.
     * We just need the correct channel routing which the PWM mode handles. */
}

/* Set Motor 2 commutation step via TIM4 PWM + GPIO enables */
static void apply_commut_m2(const CommutStep_t *step, uint32_t duty)
{
    /* Disable all enables first */
    HAL_GPIO_WritePin(M2_EN_A_PORT, M2_EN_A_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_EN_B_PORT, M2_EN_B_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_EN_C_PORT, M2_EN_C_PIN, GPIO_PIN_RESET);

    /* Zero all PWM */
    __HAL_TIM_SET_COMPARE(g_htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(g_htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(g_htim4, TIM_CHANNEL_3, 0);

    if (step->pwm_ch == 0 && step->en_ch == 0) return;

    if (step->pwm_ch & 0x01) __HAL_TIM_SET_COMPARE(g_htim4, TIM_CHANNEL_1, duty);
    if (step->pwm_ch & 0x02) __HAL_TIM_SET_COMPARE(g_htim4, TIM_CHANNEL_2, duty);
    if (step->pwm_ch & 0x04) __HAL_TIM_SET_COMPARE(g_htim4, TIM_CHANNEL_3, duty);

    /* Apply enable lines after PWM is set to avoid shoot-through */
    if (step->en_ch & 0x01) HAL_GPIO_WritePin(M2_EN_A_PORT, M2_EN_A_PIN, GPIO_PIN_SET);
    if (step->en_ch & 0x02) HAL_GPIO_WritePin(M2_EN_B_PORT, M2_EN_B_PIN, GPIO_PIN_SET);
    if (step->en_ch & 0x04) HAL_GPIO_WritePin(M2_EN_C_PORT, M2_EN_C_PIN, GPIO_PIN_SET);
}

/* Set Motor 3 commutation step via TIM3 PWM + GPIO enables */
static void apply_commut_m3(const CommutStep_t *step, uint32_t duty)
{
    HAL_GPIO_WritePin(M3_EN_A_PORT, M3_EN_A_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_EN_B_PORT, M3_EN_B_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_EN_C_PORT, M3_EN_C_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(g_htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(g_htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(g_htim3, TIM_CHANNEL_4, 0);

    if (step->pwm_ch == 0 && step->en_ch == 0) return;

    /* Motor 3 uses TIM3 CH1/CH3/CH4 (B4/B0/B1) */
    if (step->pwm_ch & 0x01) __HAL_TIM_SET_COMPARE(g_htim3, TIM_CHANNEL_1, duty);
    if (step->pwm_ch & 0x02) __HAL_TIM_SET_COMPARE(g_htim3, TIM_CHANNEL_3, duty);
    if (step->pwm_ch & 0x04) __HAL_TIM_SET_COMPARE(g_htim3, TIM_CHANNEL_4, duty);

    if (step->en_ch & 0x01) HAL_GPIO_WritePin(M3_EN_A_PORT, M3_EN_A_PIN, GPIO_PIN_SET);
    if (step->en_ch & 0x02) HAL_GPIO_WritePin(M3_EN_B_PORT, M3_EN_B_PIN, GPIO_PIN_SET);
    if (step->en_ch & 0x04) HAL_GPIO_WritePin(M3_EN_C_PORT, M3_EN_C_PIN, GPIO_PIN_SET);
}

static void motor_all_off(uint8_t motor)
{
    CommutStep_t off = { 0x00, 0x00 };
    switch (motor) {
        case MOTOR_1: apply_commut_m1(&off, 0); break;
        case MOTOR_2: apply_commut_m2(&off, 0); break;
        case MOTOR_3: apply_commut_m3(&off, 0); break;
        default: break;
    }
}

/* ============================================================
 * Public API
 * ============================================================ */

void MC_Init(TIM_HandleTypeDef *htim1_h,
             TIM_HandleTypeDef *htim3_h,
             TIM_HandleTypeDef *htim4_h)
{
    g_htim1 = htim1_h;
    g_htim3 = htim3_h;
    g_htim4 = htim4_h;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_motors[i].enabled       = 0;
        g_motors[i].direction     = DIR_FORWARD;
        g_motors[i].duty          = DEFAULT_DUTY;
        g_motors[i].hall_state    = 0xFF; /* force first commutation */
        g_motors[i].commut_count  = 0;
        g_motors[i].hall_errors   = 0;
    }

    /* Safe default: all outputs off */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_all_off(i);
    }

    /* Start PWM timers (outputs stay at 0 duty until commanded) */
    HAL_TIM_PWM_Start(g_htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(g_htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(g_htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(g_htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(g_htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(g_htim1, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(g_htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(g_htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(g_htim4, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(g_htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(g_htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(g_htim3, TIM_CHANNEL_4);
}

void MC_Enable(uint8_t motor)
{
    if (motor >= MOTOR_COUNT) return;
    g_motors[motor].enabled = 1;
}

void MC_Disable(uint8_t motor)
{
    if (motor >= MOTOR_COUNT) return;
    g_motors[motor].enabled = 0;
    g_motors[motor].duty    = 0;
    motor_all_off(motor);
}

void MC_DisableAll(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        MC_Disable(i);
    }
}

void MC_SetDuty(uint8_t motor, uint32_t duty)
{
    if (motor >= MOTOR_COUNT) return;
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
    g_motors[motor].duty = duty;
}

void MC_SetDirection(uint8_t motor, MotorDir_t dir)
{
    if (motor >= MOTOR_COUNT) return;
    g_motors[motor].direction = dir;
}

void MC_CommutationTick(void)
{
    for (int m = 0; m < MOTOR_COUNT; m++) {
        uint8_t hall = read_hall(m);

        /* Detect invalid Hall state */
        if (hall == 0 || hall == 7) {
            g_motors[m].hall_errors++;
            motor_all_off(m);
            continue;
        }

        /* Only commutate if Hall state changed */
        if (hall == g_motors[m].hall_state) continue;
        g_motors[m].hall_state = hall;

        if (!g_motors[m].enabled) {
            motor_all_off(m);
            continue;
        }

        g_motors[m].commut_count++;

        const CommutStep_t *step = (g_motors[m].direction == DIR_FORWARD)
                                    ? &commut_fwd[hall]
                                    : &commut_rev[hall];

        switch (m) {
            case MOTOR_1: apply_commut_m1(step, g_motors[m].duty); break;
            case MOTOR_2: apply_commut_m2(step, g_motors[m].duty); break;
            case MOTOR_3: apply_commut_m3(step, g_motors[m].duty); break;
            default: break;
        }
    }
}

uint32_t MC_GetCommutCount(uint8_t motor)
{
    if (motor >= MOTOR_COUNT) return 0;
    return g_motors[motor].commut_count;
}

void MC_ResetCommutCount(uint8_t motor)
{
    if (motor >= MOTOR_COUNT) return;
    g_motors[motor].commut_count = 0;
}

MotorState_t MC_GetState(uint8_t motor)
{
    MotorState_t empty = {0};
    if (motor >= MOTOR_COUNT) return empty;
    return g_motors[motor];
}
