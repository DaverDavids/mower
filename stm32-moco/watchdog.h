#pragma once
#include <stdint.h>
#include <stdbool.h>

void watchdog_init(uint32_t timeout_ms);
void watchdog_feed(void);      // call on every valid command received
bool watchdog_expired(void);   // returns true if timeout elapsed
void watchdog_reset(void);
