#include "roboclaw_link.h"

#include <Arduino.h>

#include "RoboClaw.h"
#include "board_pins.h"
#include "config.h"

namespace {

RoboClaw g_front(&board::front_roboclaw_uart, config::kRoboclawTimeoutUs);
RoboClaw g_rear(&board::rear_roboclaw_uart, config::kRoboclawTimeoutUs);
uint32_t g_last_telemetry_ms = 0;

int32_t apply_direction(int32_t value, int8_t direction) {
  return value * direction;
}

bool probe(RoboClaw& roboclaw, uint8_t address) {
  char version[64] = {};
  return roboclaw.ReadVersion(address, version);
}

bool read_controller(RoboClaw& roboclaw, uint8_t address, size_t first_wheel,
                     RoboclawState& controller, TelemetryState& telemetry,
                     uint32_t now_ms) {
  bool encoder_valid[2] = {};
  bool speed_valid[2] = {};
  uint8_t status = 0;
  const uint32_t encoder_m1 = roboclaw.ReadEncM1(address, &status, &encoder_valid[0]);
  const uint32_t encoder_m2 = roboclaw.ReadEncM2(address, &status, &encoder_valid[1]);
  const uint32_t speed_m1 = roboclaw.ReadISpeedM1(address, &status, &speed_valid[0]);
  const uint32_t speed_m2 = roboclaw.ReadISpeedM2(address, &status, &speed_valid[1]);

  const uint32_t encoders[2] = {encoder_m1, encoder_m2};
  const uint32_t speeds[2] = {speed_m1, speed_m2};
  for (size_t channel = 0; channel < 2; ++channel) {
    const size_t wheel = first_wheel + channel;
    telemetry.encoder_valid[wheel] = encoder_valid[channel];
    telemetry.speed_valid[wheel] = speed_valid[channel];
    if (encoder_valid[channel]) {
      telemetry.encoder_counts[wheel] =
          apply_direction(static_cast<int32_t>(encoders[channel]),
                          kMotorMap[wheel].encoder_direction);
    }
    if (speed_valid[channel]) {
      telemetry.measured_qpps[wheel] =
          apply_direction(static_cast<int32_t>(speeds[channel]),
                          kMotorMap[wheel].encoder_direction);
    }
  }

  controller.error_status = roboclaw.ReadError(address, &controller.error_valid);
  const bool feedback_complete = encoder_valid[0] && encoder_valid[1] &&
                                 speed_valid[0] && speed_valid[1];
  if (controller.error_valid) {
    controller.last_response_ms = now_ms;
  }
  controller.healthy =
      controller.last_response_ms != 0 &&
      (now_ms - controller.last_response_ms) <= config::kRoboclawHealthTimeoutMs &&
      (controller.error_status & config::kRoboclawCriticalErrorMask) == 0;
  return feedback_complete && controller.error_valid;
}

}  // namespace

void setup_roboclaw_link(SystemState& state) {
  g_front.begin(config::kRoboclawBaud);
  g_rear.begin(config::kRoboclawBaud);
  delay(config::kRoboclawSetupDelayMs);

  state.front_roboclaw.healthy = probe(g_front, config::kFrontRoboclawAddress);
  state.rear_roboclaw.healthy = probe(g_rear, config::kRearRoboclawAddress);
  const uint32_t now_ms = millis();
  if (state.front_roboclaw.healthy) {
    state.front_roboclaw.last_response_ms = now_ms;
  }
  if (state.rear_roboclaw.healthy) {
    state.rear_roboclaw.last_response_ms = now_ms;
  }
}

void update_roboclaw_telemetry(SystemState& state, uint32_t now_ms) {
  if ((now_ms - g_last_telemetry_ms) < config::kMotorUpdatePeriodMs) {
    return;
  }
  g_last_telemetry_ms = now_ms;

  read_controller(g_front, config::kFrontRoboclawAddress, 0,
                  state.front_roboclaw, state.telemetry, now_ms);
  read_controller(g_rear, config::kRearRoboclawAddress, 2,
                  state.rear_roboclaw, state.telemetry, now_ms);

  bool battery_valid = false;
  const uint16_t raw_tenths =
      g_rear.ReadMainBatteryVoltage(config::kRearRoboclawAddress, &battery_valid);
  state.telemetry.battery_valid = battery_valid;
  if (battery_valid) {
    state.telemetry.battery_voltage = static_cast<float>(raw_tenths) / 10.0f;
  }
}

void apply_motor_outputs(SystemState& state, uint32_t now_ms) {
  if ((now_ms - state.last_motor_update_ms) < config::kMotorUpdatePeriodMs) {
    return;
  }
  state.last_motor_update_ms = now_ms;

  const int32_t fl_qpps = apply_direction(
      state.wheel_outputs.qpps[0], kMotorMap[0].command_direction);
  const int32_t fr_qpps = apply_direction(
      state.wheel_outputs.qpps[1], kMotorMap[1].command_direction);
  const int32_t rl_qpps = apply_direction(
      state.wheel_outputs.qpps[2], kMotorMap[2].command_direction);
  const int32_t rr_qpps = apply_direction(
      state.wheel_outputs.qpps[3], kMotorMap[3].command_direction);

  const bool front_ok = g_front.SpeedM1M2(
      config::kFrontRoboclawAddress, static_cast<uint32_t>(fl_qpps),
      static_cast<uint32_t>(fr_qpps));
  const bool rear_ok = g_rear.SpeedM1M2(
      config::kRearRoboclawAddress, static_cast<uint32_t>(rl_qpps),
      static_cast<uint32_t>(rr_qpps));

  if (!front_ok) {
    state.front_roboclaw.healthy = false;
  }
  if (!rear_ok) {
    state.rear_roboclaw.healthy = false;
  }
}
