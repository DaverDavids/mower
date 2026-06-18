// ============================================================
// encoder.cpp - Quadrature encoder reading via hardware timers
// Left:  TIM3 (PA6=CH1, PA7=CH2) - encoder mode
// Right: TIM4 (PB6=CH1, PB7=CH2) - encoder mode
//
// NOTE: STM32duino / HardwareTimer is used here.
// If using STM32CubeIDE HAL directly, replace with TIM_Encoder_Init.
// ============================================================
#include "encoder.h"
#include "moco_config.h"
#include <Arduino.h>

// For STM32duino we read the timer counter registers directly.
// TIM3 base: 0x40000400, TIM4 base: 0x40000800
// Counter register is at offset 0x24 in STM32F1 peripheral map.

// Store baseline to allow reset
static int32_t enc_offset[2] = {0, 0};

// Timer counter access via direct register (STM32F103)
// TIM3->CNT and TIM4->CNT are 16-bit, we cast to int16_t for signed wrap
extern "C" {
    // Defined by STM32 CMSIS headers:
    //   TIM3 at 0x40000400
    //   TIM4 at 0x40000800
    // We use volatile pointer to avoid optimizer issues.
}

#define TIM3_CNT  (*(volatile uint16_t*)0x40000424)
#define TIM4_CNT  (*(volatile uint16_t*)0x40000824)

// Configure TIM3 and TIM4 as encoder interfaces
static void setup_encoder_timer(uint32_t tim_base) {
    volatile uint32_t *CR1  = (volatile uint32_t*)(tim_base + 0x00);
    volatile uint32_t *SMCR = (volatile uint32_t*)(tim_base + 0x08);
    volatile uint32_t *CCMR1= (volatile uint32_t*)(tim_base + 0x18);
    volatile uint32_t *CCER = (volatile uint32_t*)(tim_base + 0x20);
    volatile uint32_t *ARR  = (volatile uint32_t*)(tim_base + 0x2C);

    // Enable timer clock via RCC - done by Arduino framework init
    *ARR   = 0xFFFF;           // max count (16-bit)
    *CCMR1 = (0x01) | (0x01 << 8); // CC1S=01 (TI1), CC2S=01 (TI2)
    *CCER  = 0;                // rising edge for both
    *SMCR  = 0x03;             // SMS=011 = Encoder mode 3 (both edges)
    *CR1   = 0x01;             // CEN = enable
}

void encoder_init(void) {
    // Set encoder pins as input with pull-up
    pinMode(PIN_ENC_L_A, INPUT_PULLUP);
    pinMode(PIN_ENC_L_B, INPUT_PULLUP);
    pinMode(PIN_ENC_R_A, INPUT_PULLUP);
    pinMode(PIN_ENC_R_B, INPUT_PULLUP);

    // Enable RCC clocks for TIM3 and TIM4
    // RCC_APB1ENR: bit 1 = TIM3EN, bit 2 = TIM4EN
    volatile uint32_t *RCC_APB1ENR = (volatile uint32_t*)0x40021018;
    *RCC_APB1ENR |= (1 << 1) | (1 << 2);

    // Remap TIM4 CH1/CH2 to PB6/PB7 (default, no remap needed on F103)
    // TIM3 default: PA6/PA7 (CH1/CH2) - no remap needed for basic use
    setup_encoder_timer(0x40000400); // TIM3
    setup_encoder_timer(0x40000800); // TIM4

    encoder_reset_all();
}

int32_t encoder_get_ticks(EncID id) {
    int16_t raw = (id == ENC_LEFT) ? (int16_t)TIM3_CNT : (int16_t)TIM4_CNT;
    return (int32_t)raw - enc_offset[id];
}

float encoder_get_mm(EncID id) {
    return (float)encoder_get_ticks(id) * MM_PER_TICK;
}

void encoder_reset(EncID id) {
    int16_t raw = (id == ENC_LEFT) ? (int16_t)TIM3_CNT : (int16_t)TIM4_CNT;
    enc_offset[id] = (int32_t)raw;
}

void encoder_reset_all(void) {
    encoder_reset(ENC_LEFT);
    encoder_reset(ENC_RIGHT);
}
