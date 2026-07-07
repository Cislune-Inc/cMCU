#pragma once

#include "system_state.h"

void setup_host_link();
void update_host_link(SystemState& state, uint32_t now_ms);
void send_telemetry(SystemState& state, uint32_t now_ms);
