#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "main.h"
#include <stdint.h>

/* -----------------------------------------------------------------------
 * motor_controller.h
 * 3-channel BLDC motor controller for STM32F103C6 Blue Pill
 *
 * Motor mapping (from project-and-pins.txt):
 *   Motor 0 (Left Drive)  : TIM1 CH1/CH1N (PA8/PB13), Hall: PA3/PA4/PA5
 *                           Enable: PB3,  Fault-in: PB4
 *   Motor 1 (Right Drive) : TIM1 CH2/CH2N (PA9/PB14), Hall: PB10/PB11/PB12
 *                           Enable: PB6
 *   Motor 2 (Blade)       : TIM1 CH3/CH3N (PA10/PB15), Hall: PB9
 *                           Enable: PB7,  Fault-in: PB5
 *   Extra EN pin          : PA15
 * ----------------------------------------------------------------------- */

#define MOTOR_COUNT         3U
#define PWM_PERIOD          3599U   /* TIM1 ARR = 3599 → 20 kHz @ 72 MHz */
#define PWM_MAX             PWM_PERIOD
#define PWM_SAFE_DEFAULT    0U      /* All motors OFF at boot */

/* Hall sensor state → 6-step commutation table index (0=invalid) */
#define HALL_STATES         8U

typedef enum {
    DIR_FORWARD = 0,
    DIR_REVERSE = 1
} MotorDir_t;

typedef struct {
    uint8_t  enabled;        /* 1 = running, 0 = stopped/disabled */
    uint32_t duty;           /* 0 – PWM_PERIOD */
    MotorDir_t direction;
    uint32_t commut_count;   /* Hall edge commutation count (odometry) */
    uint32_t hall_errors;    /* Invalid Hall states seen */
    uint8_t  last_hall;      /* Last valid 3-bit Hall reading */
    uint8_t  fault;          /* Latched fault flag */
} MotorState_t;

/* ---- Public API ---- */
void         MC_Init(TIM_HandleTypeDef *tim1,
                     TIM_HandleTypeDef *tim2,
                     TIM_HandleTypeDef *tim3);
void         MC_Enable(uint8_t motor_idx);
void         MC_Disable(uint8_t motor_idx);
void         MC_DisableAll(void);
void         MC_SetDuty(uint8_t motor_idx, uint32_t duty);
void         MC_SetDirection(uint8_t motor_idx, MotorDir_t dir);
void         MC_CommutationTick(void);   /* Call from main loop */
MotorState_t MC_GetState(uint8_t motor_idx);
void         MC_ClearFault(uint8_t motor_idx);
uint32_t     MC_GetCommutCount(uint8_t motor_idx);
void         MC_ResetCommutCount(uint8_t motor_idx);

#endif /* MOTOR_CONTROLLER_H */
