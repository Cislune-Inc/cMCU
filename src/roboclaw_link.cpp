#include "roboclaw_link.h"

#include <Arduino.h>

#include "config.h"

namespace {

struct RoboclawCommandLog {
  int16_t front_duty_m1 = 0;
  int16_t front_duty_m2 = 0;
  int16_t rear_duty_m1 = 0;
  int16_t rear_duty_m2 = 0;
  int32_t front_qpps_m1 = 0;
  int32_t front_qpps_m2 = 0;
  int32_t rear_qpps_m1 = 0;
  int32_t rear_qpps_m2 = 0;
};

RoboclawCommandLog g_last_commands = {};

int16_t apply_direction(int16_t value, int8_t direction) {
  return static_cast<int16_t>(value * direction);
}

int32_t apply_direction(int32_t value, int8_t direction) {
  return value * direction;
}

}  // namespace

void setup_roboclaw_link(SystemState& state) {
  Serial2.begin(config::kRoboclawBaud);
  state.roboclaw_ok = true;
}

void update_roboclaw_telemetry(SystemState& state, uint32_t now_ms) {
  (void)now_ms;

  if (!state.telemetry.battery_valid) {
    state.telemetry.battery_voltage = 24.0f;
    state.telemetry.battery_valid = true;
  }

  for (size_t i = 0; i < kWheelCount; ++i) {
    state.telemetry.encoder_valid[i] = true;
  }
}

void apply_motor_outputs(SystemState& state, uint32_t now_ms) {
  if ((now_ms - state.last_motor_update_ms) < config::kMotorUpdatePeriodMs) {
    return;
  }
  state.last_motor_update_ms = now_ms;

  const int16_t fl_duty = apply_direction(
      state.wheel_outputs.duty[static_cast<size_t>(WheelId::FL)], kMotorMap[0].direction);
  const int16_t fr_duty = apply_direction(
      state.wheel_outputs.duty[static_cast<size_t>(WheelId::FR)], kMotorMap[1].direction);
  const int16_t rl_duty = apply_direction(
      state.wheel_outputs.duty[static_cast<size_t>(WheelId::RL)], kMotorMap[2].direction);
  const int16_t rr_duty = apply_direction(
      state.wheel_outputs.duty[static_cast<size_t>(WheelId::RR)], kMotorMap[3].direction);

  const int32_t fl_qpps = apply_direction(
      state.wheel_outputs.qpps[static_cast<size_t>(WheelId::FL)], kMotorMap[0].direction);
  const int32_t fr_qpps = apply_direction(
      state.wheel_outputs.qpps[static_cast<size_t>(WheelId::FR)], kMotorMap[1].direction);
  const int32_t rl_qpps = apply_direction(
      state.wheel_outputs.qpps[static_cast<size_t>(WheelId::RL)], kMotorMap[2].direction);
  const int32_t rr_qpps = apply_direction(
      state.wheel_outputs.qpps[static_cast<size_t>(WheelId::RR)], kMotorMap[3].direction);

  g_last_commands.front_duty_m1 = fl_duty;
  g_last_commands.front_duty_m2 = fr_duty;
  g_last_commands.rear_duty_m1 = rl_duty;
  g_last_commands.rear_duty_m2 = rr_duty;
  g_last_commands.front_qpps_m1 = fl_qpps;
  g_last_commands.front_qpps_m2 = fr_qpps;
  g_last_commands.rear_qpps_m1 = rl_qpps;
  g_last_commands.rear_qpps_m2 = rr_qpps;

  state.telemetry.encoder_counts[0] += fl_qpps / 50;
  state.telemetry.encoder_counts[1] += fr_qpps / 50;
  state.telemetry.encoder_counts[2] += rl_qpps / 50;
  state.telemetry.encoder_counts[3] += rr_qpps / 50;
}
