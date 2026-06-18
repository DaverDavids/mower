// ============================================================
// moco_main.ino - STM32 Motor Controller Main
// Target: STM32F103C8T6 (Blue Pill)
// Framework: STM32duino (Arduino for STM32)
//
// SAFETY DEFAULTS:
//   - Motors are OFF at startup
//   - Comms watchdog: motors stop if no command within WATCHDOG_MS
//   - Hardware IWDG enabled (resets MCU on firmware lockup)
//   - ESTOP command = immediate motor stop, fault LED on
//
// DEBUG:
//   Send "DEBUG:ON\n" over serial to enable verbose DBG: messages
//   Send "PING\n" to confirm connectivity before enabling motors
// ============================================================

#include "moco_config.h"
#include "motor.h"
#include "encoder.h"
#include "comms.h"
#include "watchdog.h"
#include <Arduino.h>

// ---- State machine ----
typedef enum {
    STATE_SAFE = 0,   // startup / watchdog / estop
    STATE_RUN,        // normal operation
    STATE_FAULT       // unrecoverable (future use)
} SystemState;

static SystemState state = STATE_SAFE;

// ---- Timing ----
static uint32_t last_feedback_ms  = 0;
static uint32_t last_ramp_ms      = 0;
static uint32_t last_heartbeat_ms = 0;
static const uint32_t FEEDBACK_INTERVAL_MS = 1000 / FEEDBACK_HZ;

// ---- IWDG (hardware watchdog) ----
static void iwdg_init(void) {
    // IWDG: ~1 second timeout using LSI ~40kHz
    // KR=0xCCCC start, KR=0x5555 unlock, PR=0x06 (prescaler/256), RLR=0x09C3 (~1s)
    volatile uint32_t *IWDG_KR  = (volatile uint32_t*)0x40003000;
    volatile uint32_t *IWDG_PR  = (volatile uint32_t*)0x40003004;
    volatile uint32_t *IWDG_RLR = (volatile uint32_t*)0x40003008;
    *IWDG_KR  = 0xCCCC; // Start
    *IWDG_KR  = 0x5555; // Unlock
    *IWDG_PR  = 0x06;   // Prescaler /256 -> ~156 Hz tick
    *IWDG_RLR = 0x09C3; // Reload -> ~2500 ticks / 156 Hz ~= 2.5s
    *IWDG_KR  = 0xAAAA; // Reload
    *IWDG_KR  = 0xCCCC; // Enable IWDG
}

static void iwdg_feed(void) {
    volatile uint32_t *IWDG_KR = (volatile uint32_t*)0x40003000;
    *IWDG_KR = 0xAAAA;
}

// ---- Enter safe/estop state ----
static void enter_safe(const char *reason) {
    motor_stop_all();
    state = STATE_SAFE;
    digitalWrite(PIN_LED_FAULT, HIGH);
    comms_send_debug(reason);
}

void setup() {
    // Status LEDs first
    pinMode(PIN_LED_HB,    OUTPUT);
    pinMode(PIN_LED_FAULT, OUTPUT);
    digitalWrite(PIN_LED_HB,    HIGH); // LED off (active LOW)
    digitalWrite(PIN_LED_FAULT, LOW);  // No fault yet

    // Init subsystems
    comms_init(BAUD_RATE);
    motor_init();
    encoder_init();
    watchdog_init(WATCHDOG_MS);
    iwdg_init();

    // Start in SAFE state - motors already stopped by motor_init
    state = STATE_SAFE;

    // Startup message
    delay(100); // Let serial settle
    Serial.println("MOCO:READY");
    Serial.println("DBG:State=SAFE. Send PING to confirm, then L:0,R:0 to arm.");

    last_feedback_ms  = millis();
    last_ramp_ms      = millis();
    last_heartbeat_ms = millis();
}

void loop() {
    uint32_t now = millis();

    // ---- Feed hardware watchdog (prevent MCU reset if loop is running) ----
    iwdg_feed();

    // ---- Parse incoming serial commands ----
    Command cmd = comms_poll();

    switch (cmd.type) {
        case CMD_DRIVE:
            if (state == STATE_FAULT) {
                comms_send_error("In FAULT state. Restart MCU.");
            } else {
                // Any drive command clears SAFE state
                if (state == STATE_SAFE) {
                    state = STATE_RUN;
                    digitalWrite(PIN_LED_FAULT, LOW);
                    comms_send_debug("State=RUN");
                }
                motor_set(MOTOR_LEFT,  cmd.left_pwm);
                motor_set(MOTOR_RIGHT, cmd.right_pwm);
                watchdog_feed();
                if (comms_debug_enabled()) {
                    char dbg[48];
                    snprintf(dbg, sizeof(dbg), "Drive L:%d R:%d", cmd.left_pwm, cmd.right_pwm);
                    comms_send_debug(dbg);
                }
            }
            break;

        case CMD_ESTOP:
            enter_safe("ESTOP received");
            comms_send_error("ESTOP");
            break;

        case CMD_PING:
            comms_send_pong();
            watchdog_feed();
            break;

        case CMD_DEBUG_ON:
            comms_send_debug("Debug ON");
            watchdog_feed();
            break;

        case CMD_DEBUG_OFF:
            // Already handled inside comms.cpp
            watchdog_feed();
            break;

        case CMD_RESET_ENC:
            encoder_reset_all();
            watchdog_feed();
            comms_send_debug("Encoders reset");
            break;

        case CMD_STATUS: {
            // Force immediate feedback report
            int32_t lt = encoder_get_ticks(ENC_LEFT);
            int32_t rt = encoder_get_ticks(ENC_RIGHT);
            float   lm = encoder_get_mm(ENC_LEFT);
            float   rm = encoder_get_mm(ENC_RIGHT);
            comms_send_feedback(lt, rt, lm, rm);
            watchdog_feed();
            break;
        }

        case CMD_UNKNOWN:
            comms_send_error("Unknown command");
            break;

        case CMD_NONE:
        default:
            break;
    }

    // ---- Comms watchdog check ----
    if (state == STATE_RUN && watchdog_expired()) {
        enter_safe("Watchdog: no command received");
    }

    // ---- Soft ramp tick ----
#if RAMP_STEP > 0
    if (now - last_ramp_ms >= RAMP_INTERVAL_MS) {
        motor_ramp_tick();
        last_ramp_ms = now;
    }
#endif

    // ---- Periodic encoder feedback ----
    if (now - last_feedback_ms >= FEEDBACK_INTERVAL_MS) {
        last_feedback_ms = now;
        int32_t lt = encoder_get_ticks(ENC_LEFT);
        int32_t rt = encoder_get_ticks(ENC_RIGHT);
        float   lm = encoder_get_mm(ENC_LEFT);
        float   rm = encoder_get_mm(ENC_RIGHT);
        comms_send_feedback(lt, rt, lm, rm);
    }

    // ---- Heartbeat LED (PC13, active LOW) ----
    // Fast blink = RUN, slow blink = SAFE, solid fault LED = FAULT
    uint32_t hb_period = (state == STATE_RUN) ? 200 : 1000;
    if (now - last_heartbeat_ms >= hb_period) {
        last_heartbeat_ms = now;
        digitalWrite(PIN_LED_HB, !digitalRead(PIN_LED_HB));
    }
}
