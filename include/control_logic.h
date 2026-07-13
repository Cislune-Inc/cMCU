#pragma once

#include "system_state.h"

namespace control_logic {

WheelOutputs body_velocity_targets(const MotionCommand& command);
WheelOutputs slew_velocity_targets(const WheelOutputs& current,
                                    const WheelOutputs& desired,
                                    uint32_t elapsed_ms);
WheelOutputs duty_test_targets(const DutyTestCommand& command);
void update_state(SystemState& state, uint32_t now_ms);

}  // namespace control_logic
