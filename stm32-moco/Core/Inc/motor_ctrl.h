#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "main.h"
#include <stdint.h>

/* -------------------------------------------------------------------------
 * Motor Controller – Public API
 *
 * Three BLDC motors driven by 6-step Hall commutation.
 * All outputs default LOW at boot (safe state).
 * Communication: USB CDC (Virtual COM Port) ASCII protocol.
 * Debug shell: accessible on same USB port.
 * -------------------------------------------------------------------------
 */

/* Number of supported motors */
#define MOTOR_COUNT  3U

/* PWM resolution – must match ARR in timer init (ARR = 1439) */
#define PWM_PERIOD   1440U

/* Maximum duty cycle (100% of PWM_PERIOD) */
#define DUTY_MAX     PWM_PERIOD

/* Dead-time register value for TIM1 (~700 ns at 72 MHz, DTG=0x32) */
#define TIM1_DEADTIME  0x32U

/* Motor indices */
#define MOTOR_1  0U
#define MOTOR_2  1U
#define MOTOR_3  2U

/* Motor direction */
typedef enum {
    DIR_FORWARD  = 0,
    DIR_REVERSE  = 1
} MotorDir_t;

/* Per-motor runtime state */
typedef struct {
    uint8_t   enabled;          /* 0 = stopped, 1 = running              */
    MotorDir_t dir;             /* Rotation direction                    */
    uint16_t  duty;             /* 0 – PWM_PERIOD                        */
    uint8_t   hall_state;       /* Last raw 3-bit Hall reading (0-7)     */
    uint8_t   commut_step;      /* Active commutation step (0-5)         */
    int32_t   hall_ticks;       /* Cumulative Hall transition counter    */
    /* For pin-swap debug: logical→physical channel remap (0,1,2)        */
    uint8_t   phase_map[3];     /* phase_map[logical] = physical_channel */
} MotorState_t;

/* Global state array – accessible from usb_cmd.c for debug reads */
extern MotorState_t g_motor[MOTOR_COUNT];

/* ----- Init / Lifecycle ------------------------------------------------ */
void Motor_Init(void);          /* Call after all MX_xxx_Init() calls    */
void Motor_SafeAll(void);       /* Drive all outputs LOW immediately     */

/* ----- Per-motor control ----------------------------------------------- */
void Motor_SetDuty(uint8_t motor_id, uint16_t duty);
void Motor_SetDir(uint8_t motor_id, MotorDir_t dir);
void Motor_Enable(uint8_t motor_id);
void Motor_Disable(uint8_t motor_id);

/* ----- Commutation (call from main loop / TIM ISR) --------------------- */
void Motor_Commutate(uint8_t motor_id);
void Motor_CommutateAll(void);

/* ----- Hall feedback --------------------------------------------------- */
uint8_t Motor_ReadHall(uint8_t motor_id);   /* Returns 3-bit Hall state  */
int32_t Motor_GetTicks(uint8_t motor_id);   /* Signed tick count         */
void    Motor_ResetTicks(uint8_t motor_id);

/* ----- Phase-map swap (debug) ----------------------------------------- */
void Motor_SetPhaseMap(uint8_t motor_id, uint8_t p0, uint8_t p1, uint8_t p2);

#endif /* MOTOR_CTRL_H */
