#pragma once

#include "system_state.h"

namespace control_logic {

float clamp_norm(float value);
float apply_deadband(float value);
MixedDrive mix_drive(float throttle, float turn);
WheelOutputs build_wheel_outputs(const MixedDrive& drive, DriveMode mode);
void update_state(SystemState& state, uint32_t now_ms);

}  // namespace control_logic
