#pragma once
// ============================================================
// moco_config.h - Compile-time configuration for stm32-moco
// Tune these values to match your physical hardware.
// ============================================================

// --- Serial / Communication ---
#define BAUD_RATE           115200
#define WATCHDOG_MS         500     // ms without a valid command before ESTOP
#define FEEDBACK_HZ         10      // encoder feedback report rate
#define RX_BUFFER_SIZE      64

// --- PWM ---
#define PWM_FREQ_HZ         20000   // 20 kHz (above audible range)
#define PWM_MAX             255
#define PWM_DEADBAND        15      // values below this map to 0 (motor won't creep)

// --- Encoder / Distance ---
#define ENCODER_PPR         20      // single-channel pulses per wheel revolution
#define ENCODER_PPR_QUAD    (ENCODER_PPR * 4)   // quadrature x4 = 80
#define WHEEL_DIAMETER_MM   200.0f  // MEASURE YOUR ACTUAL WHEEL
#define MM_PER_TICK         (3.14159265f * WHEEL_DIAMETER_MM / ENCODER_PPR_QUAD)

// --- Soft Ramp ---
#define RAMP_STEP           8       // PWM counts per ramp tick (0 = disabled)
#define RAMP_INTERVAL_MS    10      // ms between ramp steps

// --- Pin Definitions (STM32F103 Blue Pill) ---
// Left Motor
#define PIN_LEFT_PWM        PA0     // TIM2_CH1
#define PIN_LEFT_IN1        PB0
#define PIN_LEFT_IN2        PB1
// Right Motor
#define PIN_RIGHT_PWM       PA1     // TIM2_CH2
#define PIN_RIGHT_IN1       PB10
#define PIN_RIGHT_IN2       PB11
// Left Encoder (TIM3 encoder mode)
#define PIN_ENC_L_A         PA6     // TIM3_CH1
#define PIN_ENC_L_B         PA7     // TIM3_CH2
// Right Encoder (TIM4 encoder mode)
#define PIN_ENC_R_A         PB6     // TIM4_CH1
#define PIN_ENC_R_B         PB7     // TIM4_CH2
// Status
#define PIN_LED_HB          PC13    // Heartbeat (active LOW, onboard)
#define PIN_LED_FAULT       PB12    // Fault indicator (active HIGH)
