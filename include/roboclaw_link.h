#pragma once

#include "system_state.h"

void setup_roboclaw_link(SystemState& state);
void update_roboclaw_telemetry(SystemState& state, uint32_t now_ms);
void apply_motor_outputs(SystemState& state, uint32_t now_ms);
