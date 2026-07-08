#include "roboclaw_link.h"

#include <Arduino.h>

#include "RoboClaw.h"
#include "config.h"

namespace {

RoboClaw g_roboclaw(&Serial2, config::kRoboclawTimeoutUs);
uint32_t g_last_telemetry_ms = 0;
uint32_t g_last_debug_ms = 0;

int16_t apply_direction(int16_t value, int8_t direction) {
  return static_cast<int16_t>(value * direction);
}

int32_t apply_direction(int32_t value, int8_t direction) {
  return value * direction;
}

bool probe_address(uint8_t address) {
  char version[64] = {0};
  return g_roboclaw.ReadVersion(address, version);
}

}  // namespace

void setup_roboclaw_link(SystemState& state) {
  Serial2.begin(config::kRoboclawBaud);
  delay(config::kRoboclawSetupDelayMs);

  g_roboclaw.begin(config::kRoboclawBaud);
  delay(config::kRoboclawSetupDelayMs);

  g_last_telemetry_ms = 0;
  const bool front_ok = probe_address(config::kFrontRoboclawAddress);
  const bool rear_ok = probe_address(config::kRearRoboclawAddress);
  const bool battery_ok =
      config::kBatteryRoboclawAddress == config::kFrontRoboclawAddress ||
              config::kBatteryRoboclawAddress == config::kRearRoboclawAddress
          ? true
          : probe_address(config::kBatteryRoboclawAddress);
  state.roboclaw_ok = front_ok && rear_ok;
  state.telemetry.battery_valid = false;

  Serial1.println("DBG,RC,INIT");
  Serial1.print("DBG,RC,PROBE,0x80,");
  Serial1.println(front_ok ? "OK" : "FAIL");
  Serial1.print("DBG,RC,PROBE,0x81,");
  Serial1.println(rear_ok ? "OK" : "FAIL");
  Serial1.print("DBG,RC,PROBE,0x82,");
  Serial1.println(battery_ok ? "OK" : "FAIL");

  for (size_t i = 0; i < kWheelCount; ++i) {
    state.telemetry.encoder_valid[i] = false;
  }
}

void update_roboclaw_telemetry(SystemState& state, uint32_t now_ms) {
  if ((now_ms - g_last_telemetry_ms) < config::kMotorUpdatePeriodMs) {
    return;
  }
  g_last_telemetry_ms = now_ms;

  bool any_encoder_valid = false;
  for (size_t i = 0; i < kWheelCount; ++i) {
    uint8_t status = 0;
    bool valid = false;
    uint32_t raw = 0;

    if (kMotorMap[i].channel_m2) {
      raw = g_roboclaw.ReadEncM2(kMotorMap[i].address, &status, &valid);
    } else {
      raw = g_roboclaw.ReadEncM1(kMotorMap[i].address, &status, &valid);
    }

    state.telemetry.encoder_valid[i] = valid;
    if (valid) {
      state.telemetry.encoder_counts[i] = static_cast<int32_t>(raw);
      any_encoder_valid = true;
    }
  }

  bool battery_valid = false;
  uint16_t raw_tenths =
      g_roboclaw.ReadMainBatteryVoltage(config::kBatteryRoboclawAddress, &battery_valid);
  if (!battery_valid) {
    raw_tenths = g_roboclaw.ReadMainBatteryVoltage(config::kFrontRoboclawAddress, &battery_valid);
  }
  if (!battery_valid) {
    raw_tenths = g_roboclaw.ReadMainBatteryVoltage(config::kRearRoboclawAddress, &battery_valid);
  }
  if (battery_valid) {
    state.telemetry.battery_voltage = static_cast<float>(raw_tenths) / 10.0f;
    state.telemetry.battery_valid = true;
  }

  if (any_encoder_valid) {
    state.roboclaw_ok = true;
  }

  if ((now_ms - g_last_debug_ms) >= 1000U) {
    g_last_debug_ms = now_ms;
    Serial1.print("DBG,RC,TELEM,enc=");
    Serial1.print(any_encoder_valid ? "1" : "0");
    Serial1.print(",batt=");
    Serial1.print(battery_valid ? "1" : "0");
    Serial1.print(",raw=");
    Serial1.println(raw_tenths);
  }
}

void apply_motor_outputs(SystemState& state, uint32_t now_ms) {
  if ((now_ms - state.last_motor_update_ms) < config::kMotorUpdatePeriodMs) {
    return;
  }
  state.last_motor_update_ms = now_ms;

  int16_t fl_duty = apply_direction(
      state.wheel_outputs.duty[static_cast<size_t>(WheelId::FL)], kMotorMap[0].direction);
  int16_t fr_duty = apply_direction(
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

  bool front_ok = false;
  bool rear_ok = false;
  if (state.drive_mode == DriveMode::VELOCITY) {
    front_ok = g_roboclaw.SpeedM1M2(
        config::kFrontRoboclawAddress,
        static_cast<uint32_t>(fl_qpps),
        static_cast<uint32_t>(fr_qpps));
    rear_ok = g_roboclaw.SpeedM1M2(
        config::kRearRoboclawAddress,
        static_cast<uint32_t>(rl_qpps),
        static_cast<uint32_t>(rr_qpps));
  } else {
    front_ok = g_roboclaw.DutyM1M2(
        config::kFrontRoboclawAddress,
        static_cast<uint16_t>(fl_duty),
        static_cast<uint16_t>(fr_duty));
    rear_ok = g_roboclaw.DutyM1M2(
        config::kRearRoboclawAddress,
        static_cast<uint16_t>(rl_duty),
        static_cast<uint16_t>(rr_duty));
  }

  if (!front_ok || !rear_ok) {
    state.roboclaw_ok = false;
  }
}
