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

/* Firmware version – displayed in TUI title bar and VERSION command */
#define FIRMWARE_VERSION  "moco v1.0"

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

/* Hall sequence ring – captured in Motor_Commutate(), read by TUI.
 * 256 entries per motor (was 64 – increased so fast spins don't
 * overwrite the ring before the 100 ms TUI frame can read it).
 * Must remain a power-of-2 for the bitmask wrap to work.
 *
 * Both fields are volatile: Motor_Commutate() runs from SysTick ISR
 * and writes buf[]/head while the HALLMONITOR spin-loop (main context)
 * reads them.  Without volatile the compiler can hoist head into a
 * register and the monitor loop never sees new transitions. */
#define HALL_RING_LEN  256U
#define HALL_RING_MASK (HALL_RING_LEN - 1U)

typedef struct {
    volatile uint8_t  buf[HALL_RING_LEN];  /* circular buffer of hall values     */
    volatile uint32_t head;                /* monotonically increasing write count */
} HallRing_t;

/* Per-motor runtime state */
typedef struct {
    uint8_t    enabled;          /* 0 = stopped, 1 = running              */
    uint8_t    was_enabled;      /* tracks falling edge for all_off()     */
    MotorDir_t dir;              /* Rotation direction                    */
    uint16_t   duty;             /* 0 – PWM_PERIOD                        */
    uint16_t   target_duty;      /* requested duty (for startup ramp)     */
    uint8_t    ramping;          /* 1 = startup ramp in progress          */
    uint8_t    hall_state;       /* Last raw 3-bit Hall reading (0-7)     */
    uint8_t    commut_step;      /* Active commutation step (0-5)         */
    uint8_t    commut_offset;    /* Table rotation offset 0-5 (default 0) */
    int32_t    hall_ticks;       /* Cumulative Hall transition counter    */
    uint8_t    phase_map[3];     /* phase_map[logical] = physical_channel */
    HallRing_t hall_ring;        /* ISR-speed transition log              */
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

/* ----- Commutation (call from SysTick ISR only, NOT main loop) --------- */
void Motor_Commutate(uint8_t motor_id);
void Motor_CommutateAll(void);

/* ----- Hall feedback --------------------------------------------------- */
uint8_t Motor_ReadHall(uint8_t motor_id);   /* Returns 3-bit Hall state  */
int32_t Motor_GetTicks(uint8_t motor_id);   /* Signed tick count         */
void    Motor_ResetTicks(uint8_t motor_id);
void    Motor_ClearHallRing(uint8_t motor_id);

/* ----- Phase-map swap (debug) ----------------------------------------- */
void Motor_SetPhaseMap(uint8_t motor_id, uint8_t p0, uint8_t p1, uint8_t p2);

/* ----- Commutation table offset (0-5, rotates lookup by N steps) ------- */
void Motor_SetCommutOffset(uint8_t motor_id, uint8_t offset);

#endif /* MOTOR_CTRL_H */
