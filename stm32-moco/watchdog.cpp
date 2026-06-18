// ============================================================
// watchdog.cpp - Software comms watchdog
// If no valid command arrives within timeout_ms, flag as expired.
// Hardware IWDG (independent watchdog) is also enabled in main
// as a backup for complete firmware lockup.
// ============================================================
#include "watchdog.h"
#include <Arduino.h>

static uint32_t _timeout_ms = 500;
static uint32_t _last_feed  = 0;
static bool     _expired    = false;

void watchdog_init(uint32_t timeout_ms) {
    _timeout_ms = timeout_ms;
    _last_feed  = millis();
    _expired    = false;
}

void watchdog_feed(void) {
    _last_feed = millis();
    _expired   = false;
}

bool watchdog_expired(void) {
    if (!_expired && (millis() - _last_feed) >= _timeout_ms) {
        _expired = true;
    }
    return _expired;
}

void watchdog_reset(void) {
    _last_feed = millis();
    _expired   = false;
}
