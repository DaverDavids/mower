#pragma once
#include <stdint.h>

// Motor channel identifiers
typedef enum { MOTOR_LEFT = 0, MOTOR_RIGHT = 1 } MotorID;

// Motor state tracking
typedef struct {
    int16_t  target_pwm;   // -255 to 255 (sign = direction)
    int16_t  current_pwm;  // current ramped value
    uint32_t stall_timer;  // for future stall detection
} MotorState;

void motor_init(void);
void motor_set(MotorID id, int16_t pwm);  // -255..255, 0=brake
void motor_stop_all(void);                // coast/brake both motors immediately
void motor_ramp_tick(void);               // call every RAMP_INTERVAL_MS
int16_t motor_get_pwm(MotorID id);
