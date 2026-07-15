#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum class OperatingMode : uint8_t {
  DISABLED = 'D',
  BODY_VELOCITY = 'V',
  FAULT = 'F',
};

enum class FaultCode : uint8_t {
  OK = '0',
  STARTUP = 'B',
  HOST_TIMEOUT = 'H',
  BATTERY = 'T',
  ROBOCLAW = 'C',
  FEEDBACK = 'E',
};

enum class WheelId : uint8_t {
  FL = 0,
  FR = 1,
  RL = 2,
  RR = 3,
};

struct MotionCommand {
  float linear_mps = 0.0f;
  float angular_radps = 0.0f;
  uint32_t sequence = 0;
  uint32_t received_ms = 0;
  bool valid = false;
};

struct HostState {
  MotionCommand velocity_command = {};
  uint32_t last_accepted_sequence = 0;
  bool has_accepted_sequence = false;
  bool handshake_complete = false;
};

struct WheelOutputs {
  int32_t qpps[4] = {0, 0, 0, 0};
};

struct TelemetryState {
  float battery_voltage = 0.0f;
  bool battery_valid = false;
  int32_t encoder_counts[4] = {0, 0, 0, 0};
  int32_t measured_qpps[4] = {0, 0, 0, 0};
  bool encoder_valid[4] = {false, false, false, false};
  bool speed_valid[4] = {false, false, false, false};
};

struct RoboclawState {
  bool healthy = false;
  bool error_valid = false;
  uint32_t error_status = 0;
  uint32_t last_response_ms = 0;
};

struct SystemState {
  OperatingMode mode = OperatingMode::DISABLED;
  FaultCode fault = FaultCode::STARTUP;

  HostState host = {};
  WheelOutputs wheel_outputs = {};
  TelemetryState telemetry = {};
  RoboclawState front_roboclaw = {};
  RoboclawState rear_roboclaw = {};

  uint32_t startup_ms = 0;
  uint32_t last_telemetry_ms = 0;
  uint32_t last_motor_update_ms = 0;
  uint32_t last_control_update_ms = 0;
  uint32_t telemetry_sequence = 0;
};

struct MotorChannel {
  int8_t command_direction;
  int8_t encoder_direction;
};

constexpr size_t kWheelCount = 4;

constexpr MotorChannel kMotorMap[kWheelCount] = {
    {1, 1},  // FL
    {1, 1},  // FR
    {1, 1},  // RL
    {1, 1},  // RR
};
