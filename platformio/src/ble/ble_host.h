#pragma once

// #include "acquisition/analyzer.h"
#include <stdint.h>

namespace ble_host {

void setup();

// If state notification is enabled, send a notification with
// this state.
// void notify_state_if_enabled(const analyzer::State& state);

// Returns true if a host is connected. Used also to check
// connection WDT expriation.
// bool is_connected();


}  // namespace ble_host