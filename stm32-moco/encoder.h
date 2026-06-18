#pragma once
#include <stdint.h>

// Encoder channel identifiers
typedef enum { ENC_LEFT = 0, ENC_RIGHT = 1 } EncID;

void    encoder_init(void);
void    encoder_reset(EncID id);
int32_t encoder_get_ticks(EncID id);     // signed cumulative ticks
float   encoder_get_mm(EncID id);        // distance in mm from last reset
void    encoder_reset_all(void);
