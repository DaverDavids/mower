// ============================================================
// motor.cpp - Dual DC motor control with PWM + direction
// Uses STM32 HAL TIM2 for PWM (CH1=left, CH2=right)
// ============================================================
#include "motor.h"
#include "moco_config.h"
#include <Arduino.h>

static MotorState motors[2];

void motor_init(void) {
    // Set up direction pins
    pinMode(PIN_LEFT_IN1,  OUTPUT);
    pinMode(PIN_LEFT_IN2,  OUTPUT);
    pinMode(PIN_RIGHT_IN1, OUTPUT);
    pinMode(PIN_RIGHT_IN2, OUTPUT);

    // PWM output pins (Arduino framework handles timer setup)
    pinMode(PIN_LEFT_PWM,  OUTPUT);
    pinMode(PIN_RIGHT_PWM, OUTPUT);

    motor_stop_all();
}

static void apply_motor(MotorID id, int16_t pwm) {
    // Apply deadband
    if (pwm > -PWM_DEADBAND && pwm < PWM_DEADBAND) pwm = 0;
    // Clamp
    if (pwm >  PWM_MAX) pwm =  PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;

    uint8_t speed = (uint8_t)(pwm < 0 ? -pwm : pwm);

    if (id == MOTOR_LEFT) {
        if (pwm > 0) {
            digitalWrite(PIN_LEFT_IN1, HIGH);
            digitalWrite(PIN_LEFT_IN2, LOW);
        } else if (pwm < 0) {
            digitalWrite(PIN_LEFT_IN1, LOW);
            digitalWrite(PIN_LEFT_IN2, HIGH);
        } else {
            // Active brake
            digitalWrite(PIN_LEFT_IN1, LOW);
            digitalWrite(PIN_LEFT_IN2, LOW);
        }
        analogWrite(PIN_LEFT_PWM, speed);
    } else {
        if (pwm > 0) {
            digitalWrite(PIN_RIGHT_IN1, HIGH);
            digitalWrite(PIN_RIGHT_IN2, LOW);
        } else if (pwm < 0) {
            digitalWrite(PIN_RIGHT_IN1, LOW);
            digitalWrite(PIN_RIGHT_IN2, HIGH);
        } else {
            digitalWrite(PIN_RIGHT_IN1, LOW);
            digitalWrite(PIN_RIGHT_IN2, LOW);
        }
        analogWrite(PIN_RIGHT_PWM, speed);
    }
}

void motor_set(MotorID id, int16_t pwm) {
    motors[id].target_pwm = pwm;
#if RAMP_STEP == 0
    motors[id].current_pwm = pwm;
    apply_motor(id, pwm);
#endif
}

void motor_stop_all(void) {
    motors[MOTOR_LEFT].target_pwm  = 0;
    motors[MOTOR_LEFT].current_pwm = 0;
    motors[MOTOR_RIGHT].target_pwm  = 0;
    motors[MOTOR_RIGHT].current_pwm = 0;
    apply_motor(MOTOR_LEFT,  0);
    apply_motor(MOTOR_RIGHT, 0);
}

void motor_ramp_tick(void) {
#if RAMP_STEP > 0
    for (int i = 0; i < 2; i++) {
        int16_t diff = motors[i].target_pwm - motors[i].current_pwm;
        if (diff == 0) continue;
        if (diff >  RAMP_STEP) diff =  RAMP_STEP;
        if (diff < -RAMP_STEP) diff = -RAMP_STEP;
        motors[i].current_pwm += diff;
        apply_motor((MotorID)i, motors[i].current_pwm);
    }
#endif
}

int16_t motor_get_pwm(MotorID id) {
    return motors[id].current_pwm;
}
