#include "control_logic.h"

#include <math.h>

#include "config.h"

namespace control_logic {
namespace {

float clamp(float value, float limit) {
  if (value < -limit) {
    return -limit;
  }
  if (value > limit) {
    return limit;
  }
  return value;
}

int32_t clamp_qpps(int32_t value) {
  if (value < -config::kMaxQpps) {
    return -config::kMaxQpps;
  }
  if (value > config::kMaxQpps) {
    return config::kMaxQpps;
  }
  return value;
}

int32_t slew(int32_t current, int32_t desired, int32_t max_change) {
  if (desired > current + max_change) {
    return current + max_change;
  }
  if (desired < current - max_change) {
    return current - max_change;
  }
  return desired;
}

FaultCode compute_fault(const SystemState& state, uint32_t now_ms) {
  if ((now_ms - state.startup_ms) < config::kStartupLockoutMs) {
    return FaultCode::STARTUP;
  }
  if (!state.front_roboclaw.healthy || !state.rear_roboclaw.healthy) {
    return FaultCode::ROBOCLAW;
  }
  for (size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    if (!state.telemetry.encoder_valid[wheel] ||
        !state.telemetry.speed_valid[wheel]) {
      return FaultCode::FEEDBACK;
    }
  }
  if (!state.telemetry.battery_valid ||
      state.telemetry.battery_voltage < config::kBatteryCriticalVolts) {
    return FaultCode::BATTERY;
  }
  if (!state.host.handshake_complete) {
    return FaultCode::HOST_TIMEOUT;
  }

  if (!state.host.velocity_command.valid ||
      (now_ms - state.host.velocity_command.received_ms) >
          config::kHostTimeoutMs) {
    return FaultCode::HOST_TIMEOUT;
  }
  return FaultCode::OK;
}

}  // namespace

WheelOutputs body_velocity_targets(const MotionCommand& command) {
  const float linear = clamp(command.linear_mps, config::kMaxLinearMps);
  const float angular = clamp(command.angular_radps, config::kMaxAngularRadps);
  const float left_mps = linear - angular * config::kEffectiveTrackWidthMeters * 0.5f;
  const float right_mps = linear + angular * config::kEffectiveTrackWidthMeters * 0.5f;
  const float qpps_per_mps = config::kEncoderCountsPerWheelRevolution /
                             (2.0f * static_cast<float>(M_PI) *
                              config::kWheelRadiusMeters);

  float left_qpps = left_mps * qpps_per_mps;
  float right_qpps = right_mps * qpps_per_mps;
  const float largest = fmaxf(fabsf(left_qpps), fabsf(right_qpps));
  if (largest > static_cast<float>(config::kMaxQpps)) {
    const float scale = static_cast<float>(config::kMaxQpps) / largest;
    left_qpps *= scale;
    right_qpps *= scale;
  }

  WheelOutputs outputs = {};
  const int32_t left = clamp_qpps(static_cast<int32_t>(lroundf(left_qpps)));
  const int32_t right = clamp_qpps(static_cast<int32_t>(lroundf(right_qpps)));
  outputs.qpps[static_cast<size_t>(WheelId::FL)] = left;
  outputs.qpps[static_cast<size_t>(WheelId::FR)] = right;
  outputs.qpps[static_cast<size_t>(WheelId::RL)] = left;
  outputs.qpps[static_cast<size_t>(WheelId::RR)] = right;
  return outputs;
}

WheelOutputs slew_velocity_targets(const WheelOutputs& current,
                                    const WheelOutputs& desired,
                                    uint32_t elapsed_ms) {
  WheelOutputs outputs = desired;
  int32_t max_change = static_cast<int32_t>(
      (static_cast<int64_t>(config::kMaxQppsChangePerSecond) * elapsed_ms) / 1000);
  if (max_change < 1) {
    max_change = 1;
  }
  for (size_t i = 0; i < kWheelCount; ++i) {
    outputs.qpps[i] = slew(current.qpps[i], desired.qpps[i], max_change);
  }
  return outputs;
}

void update_state(SystemState& state, uint32_t now_ms) {
  state.fault = compute_fault(state, now_ms);
  if (state.fault != FaultCode::OK) {
    state.mode = OperatingMode::FAULT;
    state.wheel_outputs = {};
    state.last_control_update_ms = now_ms;
    return;
  }

  const uint32_t elapsed_ms = state.last_control_update_ms == 0
                                  ? config::kMotorUpdatePeriodMs
                                  : now_ms - state.last_control_update_ms;
  state.last_control_update_ms = now_ms;

  state.mode = OperatingMode::BODY_VELOCITY;
  const WheelOutputs desired = body_velocity_targets(state.host.velocity_command);
  state.wheel_outputs = slew_velocity_targets(state.wheel_outputs, desired, elapsed_ms);
}

}  // namespace control_logic
