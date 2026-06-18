// ============================================================
// comms.cpp - Serial command parsing and feedback output
// Protocol:
//   IN:  "L:<val>,R:<val>\n"  drive command (-255 to 255)
//   IN:  "ESTOP\n"
//   IN:  "PING\n"
//   IN:  "DEBUG:ON\n" / "DEBUG:OFF\n"
//   IN:  "RESET_ENC\n"
//   IN:  "STATUS\n"
//   OUT: "ENC:L<ticks>,R<ticks>,DL<mm>,DR<mm>\n"
//   OUT: "DBG:<msg>\n"
//   OUT: "ERR:<msg>\n"
//   OUT: "PONG\n"
// ============================================================
#include "comms.h"
#include "moco_config.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static char rx_buf[RX_BUFFER_SIZE];
static uint8_t rx_pos = 0;
static bool debug_mode = false;

void comms_init(uint32_t baud) {
    Serial.begin(baud);
    // Flush any startup noise
    while (Serial.available()) Serial.read();
}

bool comms_debug_enabled(void) { return debug_mode; }

static Command parse_line(char *line) {
    Command cmd = {CMD_NONE, 0, 0};

    // Strip trailing \r if present
    size_t len = strlen(line);
    if (len > 0 && line[len-1] == '\r') line[--len] = '\0';

    if (len == 0) return cmd;

    if (strncmp(line, "L:", 2) == 0) {
        // Format: L:<val>,R:<val>
        char *comma = strchr(line, ',');
        if (comma && strncmp(comma+1, "R:", 2) == 0) {
            cmd.type      = CMD_DRIVE;
            cmd.left_pwm  = (int16_t)atoi(line + 2);
            cmd.right_pwm = (int16_t)atoi(comma + 3);
            // Clamp
            if (cmd.left_pwm  >  255) cmd.left_pwm  =  255;
            if (cmd.left_pwm  < -255) cmd.left_pwm  = -255;
            if (cmd.right_pwm >  255) cmd.right_pwm =  255;
            if (cmd.right_pwm < -255) cmd.right_pwm = -255;
        }
    } else if (strcmp(line, "ESTOP") == 0) {
        cmd.type = CMD_ESTOP;
    } else if (strcmp(line, "PING") == 0) {
        cmd.type = CMD_PING;
    } else if (strcmp(line, "DEBUG:ON") == 0) {
        cmd.type  = CMD_DEBUG_ON;
        debug_mode = true;
    } else if (strcmp(line, "DEBUG:OFF") == 0) {
        cmd.type  = CMD_DEBUG_OFF;
        debug_mode = false;
    } else if (strcmp(line, "RESET_ENC") == 0) {
        cmd.type = CMD_RESET_ENC;
    } else if (strcmp(line, "STATUS") == 0) {
        cmd.type = CMD_STATUS;
    } else {
        cmd.type = CMD_UNKNOWN;
    }
    return cmd;
}

Command comms_poll(void) {
    Command cmd = {CMD_NONE, 0, 0};
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            rx_buf[rx_pos] = '\0';
            cmd = parse_line(rx_buf);
            rx_pos = 0;
            return cmd;
        } else {
            if (rx_pos < RX_BUFFER_SIZE - 1) {
                rx_buf[rx_pos++] = c;
            } else {
                // Overflow - discard and reset
                rx_pos = 0;
            }
        }
    }
    return cmd;
}

void comms_send_feedback(int32_t l_ticks, int32_t r_ticks, float l_mm, float r_mm) {
    char buf[64];
    // Format: ENC:L<ticks>,R<ticks>,DL<mm>,DR<mm>
    snprintf(buf, sizeof(buf), "ENC:L%ld,R%ld,DL%.1f,DR%.1f\n",
             (long)l_ticks, (long)r_ticks, (double)l_mm, (double)r_mm);
    Serial.print(buf);
}

void comms_send_debug(const char *msg) {
    if (!debug_mode) return;
    Serial.print("DBG:");
    Serial.println(msg);
}

void comms_send_error(const char *msg) {
    Serial.print("ERR:");
    Serial.println(msg);
}

void comms_send_pong(void) {
    Serial.println("PONG");
}
