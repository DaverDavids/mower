#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ============================================================
 * Motor Controller - 3-axis BLDC, STM32F103C8T6 Blue Pill
 * ============================================================
 * Motor 1: TIM1 CH1/2/3 (high-side A8/A9/A10)
 *          TIM1 CH1N/2N/3N (low-side  B13/B14/B15) - hardware dead-time
 * Motor 2: TIM4 CH1/2/3 (PWM B6/B7/B8)
 *          GPIO enables   A15, B3, B5
 * Motor 3: TIM3 CH1/3/4 (PWM B4/B0/B1)
 *          GPIO enables   B9, B11, B12
 *
 * Hall Sensors:
 *   M1: A0, A1, A2
 *   M2: A3, A4, A5
 *   M3: A6, A7, B10
 * ============================================================ */

/* --- Timer period (ARR) for 20kHz @ 72MHz sysclk --- */
#define PWM_PERIOD          3599U
#define PWM_MAX_DUTY        PWM_PERIOD
#define PWM_MIN_DUTY        0U

/* --- Motor indices --- */
#define MOTOR_1             0
#define MOTOR_2             1
#define MOTOR_3             2
#define MOTOR_COUNT         3

/* --- Safe default speed (0 = off) --- */
#define DEFAULT_DUTY        0U

/* --- Hall state count per electrical cycle --- */
#define HALL_STATES         6

/* --- Direction --- */
typedef enum {
    DIR_FORWARD = 0,
    DIR_REVERSE = 1
} MotorDir_t;

/* --- Per-motor state --- */
typedef struct {
    uint8_t     enabled;        /* 0 = disabled (safe), 1 = running  */
    MotorDir_t  direction;
    uint32_t    duty;           /* 0 .. PWM_PERIOD                   */
    uint8_t     hall_state;     /* last read 3-bit Hall value         */
    uint32_t    commut_count;   /* total commutation steps (odometry) */
    uint32_t    hall_errors;    /* invalid Hall states seen           */
} MotorState_t;

/* --- Public API --- */

/**
 * @brief  Initialise motor controller. All outputs go to safe (off).
 *         Call after HAL_Init and peripheral inits, before main loop.
 */
void MC_Init(TIM_HandleTypeDef *htim1_h,
             TIM_HandleTypeDef *htim3_h,
             TIM_HandleTypeDef *htim4_h);

/**
 * @brief  Enable a motor. Does NOT start PWM until MC_SetDuty is called.
 */
void MC_Enable(uint8_t motor);

/**
 * @brief  Disable a motor immediately - PWM off, enables low.
 */
void MC_Disable(uint8_t motor);

/**
 * @brief  Disable all motors. Call on any fault.
 */
void MC_DisableAll(void);

/**
 * @brief  Set duty cycle (0 .. PWM_PERIOD). Motor must be enabled.
 */
void MC_SetDuty(uint8_t motor, uint32_t duty);

/**
 * @brief  Set direction for a motor.
 */
void MC_SetDirection(uint8_t motor, MotorDir_t dir);

/**
 * @brief  Read Hall sensors and perform one commutation step if state changed.
 *         Call this every main loop tick (or from a timer ISR).
 */
void MC_CommutationTick(void);

/**
 * @brief  Get commutation count (usable as distance/position feedback).
 * @param  motor  MOTOR_1 / MOTOR_2 / MOTOR_3
 * @return Number of commutation steps since init or last reset.
 */
uint32_t MC_GetCommutCount(uint8_t motor);

/**
 * @brief  Reset commutation counter for a motor.
 */
void MC_ResetCommutCount(uint8_t motor);

/**
 * @brief  Get current motor state struct (read-only snapshot).
 */
MotorState_t MC_GetState(uint8_t motor);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROLLER_H */
