#pragma once
#include <stdint.h>
#include <stdbool.h>

// Command types parsed from serial input
typedef enum {
    CMD_NONE = 0,
    CMD_DRIVE,    // L:<val>,R:<val>
    CMD_ESTOP,    // ESTOP
    CMD_PING,     // PING
    CMD_DEBUG_ON, // DEBUG:ON
    CMD_DEBUG_OFF,// DEBUG:OFF
    CMD_RESET_ENC,// RESET_ENC
    CMD_STATUS,   // STATUS  (force immediate feedback)
    CMD_UNKNOWN
} CmdType;

typedef struct {
    CmdType type;
    int16_t left_pwm;   // only valid for CMD_DRIVE
    int16_t right_pwm;  // only valid for CMD_DRIVE
} Command;

void    comms_init(uint32_t baud);
Command comms_poll(void);              // call from main loop, returns CMD_NONE if no data
void    comms_send_feedback(int32_t l_ticks, int32_t r_ticks, float l_mm, float r_mm);
void    comms_send_debug(const char *msg);
void    comms_send_error(const char *msg);
void    comms_send_pong(void);
bool    comms_debug_enabled(void);
